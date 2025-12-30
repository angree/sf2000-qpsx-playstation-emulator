/***************************************************************************
 *   Copyright (C) 2007 Ryan Schultz, PCSX-df Team, PCSX team              *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02111-1307 USA.           *
 ***************************************************************************/

/*
 * PSX assembly interpreter.
 */

#include "psxcommon.h"
#include "r3000a.h"
#include "gte.h"
#include "psxhle.h"

/* SF2000 xlog debugging */
#ifdef SF2000
extern "C" {
    extern void xlog(const char *fmt, ...);
}
/* QPSX_045: DISABLED - was logging every instruction, way too slow! */
#define INT_LOG(fmt, ...) do {} while(0)
#else
#define INT_LOG(fmt, ...) do {} while(0)
#endif

static int exec_count = 0;

static int branch = 0;
static int branch2 = 0;
static u32 branchPC;

// These macros are used to assemble the repassembler functions

#ifdef PSXCPU_LOG
#define debugI() PSXCPU_LOG("%s\n", disR3000AF(psxRegs.code, psxRegs.pc));
#else
#define debugI()
#endif

void execI();

static void psxJumpTest(void) {
	if (!Config.HLE) {
		u32 call = psxRegs.GPR.n.t1 & 0xff;
		switch (psxRegs.pc & 0x1fffff) {
			case 0xa0:
				if (biosA0[call])
					biosA0[call]();
				break;
			case 0xb0:
				if (biosB0[call])
					biosB0[call]();
				break;
			case 0xc0:
				if (biosC0[call])
					biosC0[call]();
				break;
		}
	}
}

// Subsets
extern void (*psxBSC[64])(void);
extern void (*psxSPC[64])(void);
extern void (*psxREG[32])(void);
extern void (*psxCP0[32])(void);
extern void (*psxCP2[64])(void);
extern void (*psxCP2BSC[32])(void);

static void delayRead(int reg, u32 bpc) {
	u32 rold, rnew;

//	printf("delayRead at %x!\n", psxRegs.pc);

	rold = psxRegs.GPR.r[reg];
	psxBSC[psxRegs.code >> 26](); // branch delay load
	rnew = psxRegs.GPR.r[reg];

	psxRegs.pc = bpc;

	branch = 0;

	psxRegs.GPR.r[reg] = rold;
	execI(); // first branch opcode
	psxRegs.GPR.r[reg] = rnew;

	psxBranchTest();
}

static void delayWrite(int reg, u32 bpc) {

/*	printf("delayWrite at %x!\n", psxRegs.pc);

	printf("%s\n", disR3000AF(psxRegs.code, psxRegs.pc-4));
	printf("%s\n", disR3000AF(PSXMu32(bpc), bpc));*/

	// no changes from normal behavior

	psxBSC[psxRegs.code >> 26]();

	branch = 0;
	psxRegs.pc = bpc;

	psxBranchTest();
}

static void delayReadWrite(int reg, u32 bpc) {

//	printf("delayReadWrite at %x!\n", psxRegs.pc);

	// the branch delay load is skipped

	branch = 0;
	psxRegs.pc = bpc;

	psxBranchTest();
}

// this defines shall be used with the tmp
// of the next func (instead of _Funct_...)
#define _tFunct_  ((tmp      ) & 0x3F)  // The funct part of the instruction register
#define _tRd_     ((tmp >> 11) & 0x1F)  // The rd part of the instruction register
#define _tRt_     ((tmp >> 16) & 0x1F)  // The rt part of the instruction register
#define _tRs_     ((tmp >> 21) & 0x1F)  // The rs part of the instruction register
#define _tSa_     ((tmp >>  6) & 0x1F)  // The sa part of the instruction register

int psxTestLoadDelay(int reg, u32 tmp) {
	if (tmp == 0) return 0; // NOP
	switch (tmp >> 26) {
		case 0x00: // SPECIAL
			switch (_tFunct_) {
				case 0x00: // SLL
				case 0x02: case 0x03: // SRL/SRA
					if (_tRd_ == reg && _tRt_ == reg) return 1; else
					if (_tRt_ == reg) return 2; else
					if (_tRd_ == reg) return 3;
					break;

				case 0x08: // JR
					if (_tRs_ == reg) return 2;
					break;
				case 0x09: // JALR
					if (_tRd_ == reg && _tRs_ == reg) return 1; else
					if (_tRs_ == reg) return 2; else
					if (_tRd_ == reg) return 3;
					break;

				// SYSCALL/BREAK just a break;

				case 0x20: case 0x21: case 0x22: case 0x23:
				case 0x24: case 0x25: case 0x26: case 0x27:
				case 0x2a: case 0x2b: // ADD/ADDU...
				case 0x04: case 0x06: case 0x07: // SLLV...
					if (_tRd_ == reg && (_tRt_ == reg || _tRs_ == reg)) return 1; else
					if (_tRt_ == reg || _tRs_ == reg) return 2; else
					if (_tRd_ == reg) return 3;
					break;

				case 0x10: case 0x12: // MFHI/MFLO
					if (_tRd_ == reg) return 3;
					break;
				case 0x11: case 0x13: // MTHI/MTLO
					if (_tRs_ == reg) return 2;
					break;

				case 0x18: case 0x19:
				case 0x1a: case 0x1b: // MULT/DIV...
					if (_tRt_ == reg || _tRs_ == reg) return 2;
					break;
			}
			break;

		case 0x01: // REGIMM
			switch (_tRt_) {
				case 0x00: case 0x01:
				case 0x10: case 0x11: // BLTZ/BGEZ...
					// Xenogears - lbu v0 / beq v0
					// - no load delay (fixes battle loading)
					break;

					if (_tRs_ == reg) return 2;
					break;
			}
			break;

		// J would be just a break;
		case 0x03: // JAL
			if (31 == reg) return 3;
			break;

		case 0x04: case 0x05: // BEQ/BNE
			// Xenogears - lbu v0 / beq v0
			// - no load delay (fixes battle loading)
			break;

			if (_tRs_ == reg || _tRt_ == reg) return 2;
			break;

		case 0x06: case 0x07: // BLEZ/BGTZ
			// Xenogears - lbu v0 / beq v0
			// - no load delay (fixes battle loading)
			break;

			if (_tRs_ == reg) return 2;
			break;

		case 0x08: case 0x09: case 0x0a: case 0x0b:
		case 0x0c: case 0x0d: case 0x0e: // ADDI/ADDIU...
			if (_tRt_ == reg && _tRs_ == reg) return 1; else
			if (_tRs_ == reg) return 2; else
			if (_tRt_ == reg) return 3;
			break;

		case 0x0f: // LUI
			if (_tRt_ == reg) return 3;
			break;

		case 0x10: // COP0
			switch (_tFunct_) {
				case 0x00: // MFC0
					if (_tRt_ == reg) return 3;
					break;
				case 0x02: // CFC0
					if (_tRt_ == reg) return 3;
					break;
				case 0x04: // MTC0
					if (_tRt_ == reg) return 2;
					break;
				case 0x06: // CTC0
					if (_tRt_ == reg) return 2;
					break;
				// RFE just a break;
			}
			break;

		case 0x12: // COP2
			switch (_tFunct_) {
				case 0x00:
					switch (_tRs_) {
						case 0x00: // MFC2
							if (_tRt_ == reg) return 3;
							break;
						case 0x02: // CFC2
							if (_tRt_ == reg) return 3;
							break;
						case 0x04: // MTC2
							if (_tRt_ == reg) return 2;
							break;
						case 0x06: // CTC2
							if (_tRt_ == reg) return 2;
							break;
					}
					break;
				// RTPS... break;
			}
			break;

		case 0x22: case 0x26: // LWL/LWR
			if (_tRt_ == reg) return 3; else
			if (_tRs_ == reg) return 2;
			break;

		case 0x20: case 0x21: case 0x23:
		case 0x24: case 0x25: // LB/LH/LW/LBU/LHU
			if (_tRt_ == reg && _tRs_ == reg) return 1; else
			if (_tRs_ == reg) return 2; else
			if (_tRt_ == reg) return 3;
			break;

		case 0x28: case 0x29: case 0x2a:
		case 0x2b: case 0x2e: // SB/SH/SWL/SW/SWR
			if (_tRt_ == reg || _tRs_ == reg) return 2;
			break;

		case 0x32: case 0x3a: // LWC2/SWC2
			if (_tRs_ == reg) return 2;
			break;
	}

	return 0;
}

void psxDelayTest(int reg, u32 bpc) {
	u32 *code;
	u32 tmp;

	code = (u32 *)PSXM(bpc);
	tmp = ((code == NULL) ? 0 : SWAP32(*code));
	branch = 1;

	switch (psxTestLoadDelay(reg, tmp)) {
		case 1:
			delayReadWrite(reg, bpc); return;
		case 2:
			delayRead(reg, bpc); return;
		case 3:
			delayWrite(reg, bpc); return;
	}
	psxBSC[psxRegs.code >> 26]();

	branch = 0;
	psxRegs.pc = bpc;

	psxBranchTest();
}

static u32 psxBranchNoDelay(void) {
	u32 *code;
	u32 temp;

	code = (u32 *)PSXM(psxRegs.pc);
	psxRegs.code = ((code == NULL) ? 0 : SWAP32(*code));
	switch (_Op_) {
		case 0x00: // SPECIAL
			switch (_Funct_) {
				case 0x08: // JR
					return _u32(_rRs_);
				case 0x09: // JALR
					temp = _u32(_rRs_);
					if (_Rd_) { _SetLink(_Rd_); }
					return temp;
			}
			break;
		case 0x01: // REGIMM
			switch (_Rt_) {
				case 0x00: // BLTZ
					if (_i32(_rRs_) < 0)
						return _BranchTarget_;
					break;
				case 0x01: // BGEZ
					if (_i32(_rRs_) >= 0)
						return _BranchTarget_;
					break;
				case 0x08: // BLTZAL
					if (_i32(_rRs_) < 0) {
						_SetLink(31);
						return _BranchTarget_;
					}
					break;
				case 0x09: // BGEZAL
					if (_i32(_rRs_) >= 0) {
						_SetLink(31);
						return _BranchTarget_;
					}
					break;
			}
			break;
		case 0x02: // J
			return _JumpTarget_;
		case 0x03: // JAL
			_SetLink(31);
			return _JumpTarget_;
		case 0x04: // BEQ
			if (_i32(_rRs_) == _i32(_rRt_))
				return _BranchTarget_;
			break;
		case 0x05: // BNE
			if (_i32(_rRs_) != _i32(_rRt_))
				return _BranchTarget_;
			break;
		case 0x06: // BLEZ
			if (_i32(_rRs_) <= 0)
				return _BranchTarget_;
			break;
		case 0x07: // BGTZ
			if (_i32(_rRs_) > 0)
				return _BranchTarget_;
			break;
	}

	return (u32)-1;
}

static int psxDelayBranchExec(u32 tar) {
	execI();

	branch = 0;
	psxRegs.pc = tar;
	psxRegs.cycle += BIAS;
	psxBranchTest();
	return 1;
}

static int psxDelayBranchTest(u32 tar1) {
	u32 tar2, tmp1, tmp2;

	tar2 = psxBranchNoDelay();
	if (tar2 == (u32)-1)
		return 0;

	debugI();

	/*
	 * Branch in delay slot:
	 * - execute 1 instruction at tar1
	 * - jump to tar2 (target of branch in delay slot; this branch
	 *   has no normal delay slot, instruction at tar1 was fetched instead)
	 */
	psxRegs.pc = tar1;
	tmp1 = psxBranchNoDelay();
	if (tmp1 == (u32)-1) {
		return psxDelayBranchExec(tar2);
	}
	debugI();
	psxRegs.cycle += BIAS;

	/*
	 * Got a branch at tar1:
	 * - execute 1 instruction at tar2
	 * - jump to target of that branch (tmp1)
	 */
	psxRegs.pc = tar2;
	tmp2 = psxBranchNoDelay();
	if (tmp2 == (u32)-1) {
		return psxDelayBranchExec(tmp1);
	}
	debugI();
	psxRegs.cycle += BIAS;

	/*
	 * Got a branch at tar2:
	 * - execute 1 instruction at tmp1
	 * - jump to target of that branch (tmp2)
	 */
	psxRegs.pc = tmp1;
	return psxDelayBranchExec(tmp2);
}

static void doBranch(u32 tar) {
	u32 *code;
	u32 tmp;

	branch2 = branch = 1;
	branchPC = tar;

	// check for branch in delay slot
	if (psxDelayBranchTest(tar))
		return;

	code = (u32 *)PSXM(psxRegs.pc);
	psxRegs.code = ((code == NULL) ? 0 : SWAP32(*code));

	debugI();

	psxRegs.pc += 4;
	psxRegs.cycle += BIAS;

	// check for load delay
	tmp = psxRegs.code >> 26;
	switch (tmp) {
		case 0x10: // COP0
			switch (_Rs_) {
				case 0x00: // MFC0
				case 0x02: // CFC0
					psxDelayTest(_Rt_, branchPC);
					return;
			}
			break;
		case 0x12: // COP2
			switch (_Funct_) {
				case 0x00:
					switch (_Rs_) {
						case 0x00: // MFC2
						case 0x02: // CFC2
							psxDelayTest(_Rt_, branchPC);
							return;
					}
					break;
			}
			break;
		case 0x32: // LWC2
			psxDelayTest(_Rt_, branchPC);
			return;
		default:
			if (tmp >= 0x20 && tmp <= 0x26) { // LB/LH/LWL/LW/LBU/LHU/LWR
				psxDelayTest(_Rt_, branchPC);
				return;
			}
			break;
	}

	psxBSC[psxRegs.code >> 26]();

	branch = 0;
	psxRegs.pc = branchPC;

	psxBranchTest();
}

/*********************************************************
* Arithmetic with immediate operand                      *
* Format:  OP rt, rs, immediate                          *
*********************************************************/
void psxADDI(void) 	{ if (!_Rt_) return; _rRt_ = _u32(_rRs_) + _Imm_ ; }		// Rt = Rs + Im 	(Exception on Integer Overflow)
void psxADDIU(void) {
	if (exec_count < 20) INT_LOG("ADDIU 1");
	int rt = _Rt_;
	if (exec_count < 20) INT_LOG("ADDIU 2 rt=%d", rt);
	if (!rt) return;
	if (exec_count < 20) INT_LOG("ADDIU 3");
	int rs = _Rs_;
	if (exec_count < 20) INT_LOG("ADDIU 4 rs=%d", rs);
	u32 rs_val = psxRegs.GPR.r[rs];
	if (exec_count < 20) INT_LOG("ADDIU 5 rs_val=0x%08x", rs_val);
	s32 imm = _Imm_;
	if (exec_count < 20) INT_LOG("ADDIU 6 imm=%d", imm);
	u32 result = rs_val + imm;
	if (exec_count < 20) INT_LOG("ADDIU 7 result=0x%08x", result);
	psxRegs.GPR.r[rt] = result;
	if (exec_count < 20) INT_LOG("ADDIU 8 done");
}
void psxANDI(void) 	{ if (!_Rt_) return; _rRt_ = _u32(_rRs_) & _ImmU_; }		// Rt = Rs And Im
void psxORI(void) 	{ if (!_Rt_) return; _rRt_ = _u32(_rRs_) | _ImmU_; }		// Rt = Rs Or  Im
void psxXORI(void) 	{ if (!_Rt_) return; _rRt_ = _u32(_rRs_) ^ _ImmU_; }		// Rt = Rs Xor Im
void psxSLTI(void) 	{ if (!_Rt_) return; _rRt_ = _i32(_rRs_) < _Imm_ ; }		// Rt = Rs < Im		(Signed)
void psxSLTIU(void) { if (!_Rt_) return; _rRt_ = _u32(_rRs_) < ((u32)_Imm_); }		// Rt = Rs < Im		(Unsigned)

/*********************************************************
* Register arithmetic                                    *
* Format:  OP rd, rs, rt                                 *
*********************************************************/
void psxADD(void)	{ if (!_Rd_) return; _rRd_ = _u32(_rRs_) + _u32(_rRt_); }	// Rd = Rs + Rt		(Exception on Integer Overflow)
void psxADDU(void) 	{ if (!_Rd_) return; _rRd_ = _u32(_rRs_) + _u32(_rRt_); }	// Rd = Rs + Rt
void psxSUB(void) 	{ if (!_Rd_) return; _rRd_ = _u32(_rRs_) - _u32(_rRt_); }	// Rd = Rs - Rt		(Exception on Integer Overflow)
void psxSUBU(void) 	{ if (!_Rd_) return; _rRd_ = _u32(_rRs_) - _u32(_rRt_); }	// Rd = Rs - Rt
void psxAND(void) 	{ if (!_Rd_) return; _rRd_ = _u32(_rRs_) & _u32(_rRt_); }	// Rd = Rs And Rt
void psxOR(void) 	{ if (!_Rd_) return; _rRd_ = _u32(_rRs_) | _u32(_rRt_); }	// Rd = Rs Or  Rt
void psxXOR(void) 	{ if (!_Rd_) return; _rRd_ = _u32(_rRs_) ^ _u32(_rRt_); }	// Rd = Rs Xor Rt
void psxNOR(void) 	{ if (!_Rd_) return; _rRd_ =~(_u32(_rRs_) | _u32(_rRt_)); }// Rd = Rs Nor Rt
void psxSLT(void) 	{ if (!_Rd_) return; _rRd_ = _i32(_rRs_) < _i32(_rRt_); }	// Rd = Rs < Rt		(Signed)
void psxSLTU(void) { if (!_Rd_) return; _rRd_ = _u32(_rRs_) < _u32(_rRt_); }

/*********************************************************
* Register mult/div & Register trap logic                *
* Format:  OP rs, rt                                     *
*********************************************************/
void psxDIV(void) {
	if (_i32(_rRt_) != 0) {
		_i32(_rLo_) = _i32(_rRs_) / _i32(_rRt_);
		_i32(_rHi_) = _i32(_rRs_) % _i32(_rRt_);
	}
	else {
		_i32(_rLo_) = _i32(_rRs_) >= 0 ? 0xffffffff : 1;
		_i32(_rHi_) = _i32(_rRs_);
	}
}

void psxDIVU(void) {
	if (_rRt_ != 0) {
		_rLo_ = _rRs_ / _rRt_;
		_rHi_ = _rRs_ % _rRt_;
	}
	else {
		_i32(_rLo_) = 0xffffffff;
		_i32(_rHi_) = _i32(_rRs_);
	}
}

void psxMULT(void) {
	u64 res = (s64)((s64)_i32(_rRs_) * (s64)_i32(_rRt_));

	psxRegs.GPR.n.lo = (u32)(res & 0xffffffff);
	psxRegs.GPR.n.hi = (u32)((res >> 32) & 0xffffffff);
}

void psxMULTU(void) {
	u64 res = (u64)((u64)_u32(_rRs_) * (u64)_u32(_rRt_));

	psxRegs.GPR.n.lo = (u32)(res & 0xffffffff);
	psxRegs.GPR.n.hi = (u32)((res >> 32) & 0xffffffff);
}

/*********************************************************
* Register branch logic                                  *
* Format:  OP rs, offset                                 *
*********************************************************/
#define RepZBranchi32(op)      if(_i32(_rRs_) op 0) doBranch(_BranchTarget_);
#define RepZBranchLinki32(op)  if(_i32(_rRs_) op 0) { _SetLink(31); doBranch(_BranchTarget_); }

void psxBGEZ(void)   { RepZBranchi32(>=) }      // Branch if Rs >= 0
void psxBGEZAL(void) { RepZBranchLinki32(>=) }  // Branch if Rs >= 0 and link
void psxBGTZ(void)   { RepZBranchi32(>) }       // Branch if Rs >  0
void psxBLEZ(void)   { RepZBranchi32(<=) }      // Branch if Rs <= 0
void psxBLTZ(void)   { RepZBranchi32(<) }       // Branch if Rs <  0
void psxBLTZAL(void) { RepZBranchLinki32(<) }   // Branch if Rs <  0 and link

/*********************************************************
* Shift arithmetic with constant shift                   *
* Format:  OP rd, rt, sa                                 *
*********************************************************/
void psxSLL(void) { if (!_Rd_) return; _u32(_rRd_) = _u32(_rRt_) << _Sa_; } // Rd = Rt << sa
void psxSRA(void) { if (!_Rd_) return; _i32(_rRd_) = _i32(_rRt_) >> _Sa_; } // Rd = Rt >> sa (arithmetic)
void psxSRL(void) { if (!_Rd_) return; _u32(_rRd_) = _u32(_rRt_) >> _Sa_; } // Rd = Rt >> sa (logical)

/*********************************************************
* Shift arithmetic with variant register shift           *
* Format:  OP rd, rt, rs                                 *
*********************************************************/
void psxSLLV(void) { if (!_Rd_) return; _u32(_rRd_) = _u32(_rRt_) << _u32(_rRs_); } // Rd = Rt << rs
void psxSRAV(void) { if (!_Rd_) return; _i32(_rRd_) = _i32(_rRt_) >> _u32(_rRs_); } // Rd = Rt >> rs (arithmetic)
void psxSRLV(void) { if (!_Rd_) return; _u32(_rRd_) = _u32(_rRt_) >> _u32(_rRs_); } // Rd = Rt >> rs (logical)

/*********************************************************
* Load higher 16 bits of the first word in GPR with imm  *
* Format:  OP rt, immediate                              *
*********************************************************/
void psxLUI(void) { if (!_Rt_) return; _u32(_rRt_) = psxRegs.code << 16; } // Upper halfword of Rt = Im

/*********************************************************
* Move from HI/LO to GPR                                 *
* Format:  OP rd                                         *
*********************************************************/
void psxMFHI(void) { if (!_Rd_) return; _rRd_ = _rHi_; } // Rd = Hi
void psxMFLO(void) { if (!_Rd_) return; _rRd_ = _rLo_; } // Rd = Lo

/*********************************************************
* Move to GPR to HI/LO & Register jump                   *
* Format:  OP rs                                         *
*********************************************************/
void psxMTHI(void) { _rHi_ = _rRs_; } // Hi = Rs
void psxMTLO(void) { _rLo_ = _rRs_; } // Lo = Rs

/*********************************************************
* Special purpose instructions                           *
* Format:  OP                                            *
*********************************************************/
void psxBREAK(void) {
	// Break exception - psx rom doens't handles this
}

void psxSYSCALL(void) {
	psxRegs.pc -= 4;
	psxException(0x20, branch);
}

void psxRFE(void) {
//	printf("psxRFE\n");
	psxRegs.CP0.n.Status = (psxRegs.CP0.n.Status & 0xfffffff0) |
						  ((psxRegs.CP0.n.Status & 0x3c) >> 2);
}

/*********************************************************
* Register branch logic                                  *
* Format:  OP rs, rt, offset                             *
*********************************************************/
#define RepBranchi32(op)      if(_i32(_rRs_) op _i32(_rRt_)) doBranch(_BranchTarget_);

void psxBEQ(void) {	RepBranchi32(==) }  // Branch if Rs == Rt
void psxBNE(void) { RepBranchi32(!=) }

/*********************************************************
* Jump to target                                         *
* Format:  OP target                                     *
*********************************************************/
void psxJ(void)   {               doBranch(_JumpTarget_); }
void psxJAL(void) {	_SetLink(31); doBranch(_JumpTarget_); }

/*********************************************************
* Register jump                                          *
* Format:  OP rs, rd                                     *
*********************************************************/
void psxJR(void)   {
	doBranch(_u32(_rRs_));
	psxJumpTest();
}

void psxJALR(void) {
	u32 temp = _u32(_rRs_);
	if (_Rd_) { _SetLink(_Rd_); }
	doBranch(temp);
}

/*********************************************************
* Load and store for GPR                                 *
* Format:  OP rt, offset(base)                           *
*********************************************************/

#define _oB_ (_u32(_rRs_) + _Imm_)

void psxLB(void) {
	if (_Rt_) {
		_i32(_rRt_) = (signed char)psxMemRead8(_oB_);
	} else {
		psxMemRead8(_oB_);
	}
}

void psxLBU(void) {
	if (_Rt_) {
		_u32(_rRt_) = psxMemRead8(_oB_);
	} else {
		psxMemRead8(_oB_);
	}
}

void psxLH(void) {
	if (_Rt_) {
		_i32(_rRt_) = (short)psxMemRead16(_oB_);
	} else {
		psxMemRead16(_oB_);
	}
}

void psxLHU(void) {
	if (_Rt_) {
		_u32(_rRt_) = psxMemRead16(_oB_);
	} else {
		psxMemRead16(_oB_);
	}
}

void psxLW(void) {
	if (_Rt_) {
		_u32(_rRt_) = psxMemRead32(_oB_);
	} else {
		psxMemRead32(_oB_);
	}
}

u32 LWL_MASK[4] = { 0xffffff, 0xffff, 0xff, 0 };
u32 LWL_SHIFT[4] = { 24, 16, 8, 0 };

void psxLWL(void) {
	u32 addr = _oB_;
	u32 shift = addr & 3;
	u32 mem = psxMemRead32(addr & ~3);

	if (!_Rt_) return;
	_u32(_rRt_) =	( _u32(_rRt_) & LWL_MASK[shift]) |
					( mem << LWL_SHIFT[shift]);

	/*
	Mem = 1234.  Reg = abcd

	0   4bcd   (mem << 24) | (reg & 0x00ffffff)
	1   34cd   (mem << 16) | (reg & 0x0000ffff)
	2   234d   (mem <<  8) | (reg & 0x000000ff)
	3   1234   (mem      ) | (reg & 0x00000000)
	*/
}

u32 LWR_MASK[4] = { 0, 0xff000000, 0xffff0000, 0xffffff00 };
u32 LWR_SHIFT[4] = { 0, 8, 16, 24 };

void psxLWR(void) {
	u32 addr = _oB_;
	u32 shift = addr & 3;
	u32 mem = psxMemRead32(addr & ~3);

	if (!_Rt_) return;
	_u32(_rRt_) =	( _u32(_rRt_) & LWR_MASK[shift]) |
					( mem >> LWR_SHIFT[shift]);

	/*
	Mem = 1234.  Reg = abcd

	0   1234   (mem      ) | (reg & 0x00000000)
	1   a123   (mem >>  8) | (reg & 0xff000000)
	2   ab12   (mem >> 16) | (reg & 0xffff0000)
	3   abc1   (mem >> 24) | (reg & 0xffffff00)
	*/
}

void psxSB(void) { psxMemWrite8 (_oB_, _rRt_ &   0xff); }
void psxSH(void) { psxMemWrite16(_oB_, _rRt_ & 0xffff); }
void psxSW(void) {
	if (exec_count < 20) INT_LOG("SW 1");
	/* Expand _oB_ manually: (_u32(_rRs_) + _Imm_) */
	int rs = _Rs_;
	if (exec_count < 20) INT_LOG("SW 1a rs=%d", rs);
	u32 base = psxRegs.GPR.r[rs];
	if (exec_count < 20) INT_LOG("SW 1b base=0x%08x", base);
	if (exec_count < 20) INT_LOG("SW 1b2 code=0x%08x", psxRegs.code);
	s16 imm_raw = (s16)psxRegs.code;
	if (exec_count < 20) INT_LOG("SW 1b3 imm_raw=%d", imm_raw);
	s32 offset = imm_raw;
	if (exec_count < 20) INT_LOG("SW 1c offset=%d", offset);
	u32 addr = base + offset;
	if (exec_count < 20) INT_LOG("SW 2 addr=0x%08x", addr);
	u32 value = _rRt_;
	if (exec_count < 20) INT_LOG("SW 3 value=0x%08x", value);
	u32 t = addr >> 16;
	u32 m = addr & 0xffff;
	if (exec_count < 20) INT_LOG("SW 4 t=0x%04x m=0x%04x", t, m);

	/* Skip hardware registers for now - just handle RAM */
	if (t == 0x1f80 || t == 0x9f80 || t == 0xbf80) {
		if (exec_count < 20) INT_LOG("SW HW reg - calling psxMemWrite32");
		psxMemWrite32(addr, value);
		if (exec_count < 20) INT_LOG("SW HW done");
		return;
	}

	if (exec_count < 20) INT_LOG("SW 5 looking up WLUT[%d]", t);
	u8 *p = (u8*)(psxMemWLUT[t]);
	if (exec_count < 20) INT_LOG("SW 6 p=0x%08x", (u32)p);
	if (p != NULL) {
		/* SF2000: Byte-by-byte write to avoid unaligned access crash */
		u8 *dst = p + m;
		if (exec_count < 20) INT_LOG("SW 7 dst=0x%08x", (u32)dst);
		u32 swapped = SWAPu32(value);
		if (exec_count < 20) INT_LOG("SW 8 swapped=0x%08x", swapped);
		dst[0] = swapped & 0xff;
		if (exec_count < 20) INT_LOG("SW 9 byte0");
		dst[1] = (swapped >> 8) & 0xff;
		if (exec_count < 20) INT_LOG("SW 10 byte1");
		dst[2] = (swapped >> 16) & 0xff;
		if (exec_count < 20) INT_LOG("SW 11 byte2");
		dst[3] = (swapped >> 24) & 0xff;
		if (exec_count < 20) INT_LOG("SW 12 done");
	} else {
		if (exec_count < 20) INT_LOG("SW NULL ptr!");
	}
}

u32 SWL_MASK[4] = { 0xffffff00, 0xffff0000, 0xff000000, 0 };
u32 SWL_SHIFT[4] = { 24, 16, 8, 0 };

void psxSWL(void) {
	u32 addr = _oB_;
	u32 shift = addr & 3;
	u32 mem = psxMemRead32(addr & ~3);

	psxMemWrite32(addr & ~3,  (_u32(_rRt_) >> SWL_SHIFT[shift]) |
			     (  mem & SWL_MASK[shift]) );
	/*
	Mem = 1234.  Reg = abcd

	0   123a   (reg >> 24) | (mem & 0xffffff00)
	1   12ab   (reg >> 16) | (mem & 0xffff0000)
	2   1abc   (reg >>  8) | (mem & 0xff000000)
	3   abcd   (reg      ) | (mem & 0x00000000)
	*/
}

u32 SWR_MASK[4] = { 0, 0xff, 0xffff, 0xffffff };
u32 SWR_SHIFT[4] = { 0, 8, 16, 24 };

void psxSWR(void) {
	u32 addr = _oB_;
	u32 shift = addr & 3;
	u32 mem = psxMemRead32(addr & ~3);

	psxMemWrite32(addr & ~3,  (_u32(_rRt_) << SWR_SHIFT[shift]) |
			     (  mem & SWR_MASK[shift]) );

	/*
	Mem = 1234.  Reg = abcd

	0   abcd   (reg      ) | (mem & 0x00000000)
	1   bcd4   (reg <<  8) | (mem & 0x000000ff)
	2   cd34   (reg << 16) | (mem & 0x0000ffff)
	3   d234   (reg << 24) | (mem & 0x00ffffff)
	*/
}

/*********************************************************
* Moves between GPR and COPx                             *
* Format:  OP rt, fs                                     *
*********************************************************/
void psxMFC0(void) { if (!_Rt_) return; _i32(_rRt_) = (int)_rFs_; }
void psxCFC0(void) { if (!_Rt_) return; _i32(_rRt_) = (int)_rFs_; }

void psxTestSWInts(void) {
	if (psxRegs.CP0.n.Cause & psxRegs.CP0.n.Status & 0x0300 &&
	   psxRegs.CP0.n.Status & 0x1) {
		psxRegs.CP0.n.Cause &= ~0x7c;
		psxException(psxRegs.CP0.n.Cause, branch);
	}
}

void MTC0(int reg, u32 val) {
//	printf("MTC0 %d: %x\n", reg, val);
	switch (reg) {
		case 12: // Status
			psxRegs.CP0.r[12] = val;
			psxTestSWInts();
			break;

		case 13: // Cause
			psxRegs.CP0.n.Cause &= ~0x0300;
			psxRegs.CP0.n.Cause |= val & 0x0300;
			psxTestSWInts();
			break;

		default:
			psxRegs.CP0.r[reg] = val;
			break;
	}
}

void psxMTC0(void) { MTC0(_Rd_, _u32(_rRt_)); }
void psxCTC0(void) { MTC0(_Rd_, _u32(_rRt_)); }

/*********************************************************
* Unknow instruction (would generate an exception)       *
* Format:  ?                                             *
*********************************************************/
void psxNULL(void) {
#ifdef PSXCPU_LOG
	PSXCPU_LOG("psx: Unimplemented op %x\n", psxRegs.code);
#endif
}

void psxSPECIAL(void) {
	psxSPC[_Funct_]();
}

void psxREGIMM(void) {
	psxREG[_Rt_]();
}

void psxCOP0(void) {
	psxCP0[_Rs_]();
}

void psxCOP2(void) {
	psxCP2[_Funct_]();
}

void psxBASIC(void) {
	psxCP2BSC[_Rs_]();
}

void psxHLE(void) {
//	psxHLEt[psxRegs.code & 0xffff]();
	psxHLEt[psxRegs.code & 0x07]();		// HDHOSHY experimental patch
}

void (*psxBSC[64])(void) = {
	psxSPECIAL, psxREGIMM, psxJ   , psxJAL  , psxBEQ , psxBNE , psxBLEZ, psxBGTZ,
	psxADDI   , psxADDIU , psxSLTI, psxSLTIU, psxANDI, psxORI , psxXORI, psxLUI ,
	psxCOP0   , psxNULL  , psxCOP2, psxNULL , psxNULL, psxNULL, psxNULL, psxNULL,
	psxNULL   , psxNULL  , psxNULL, psxNULL , psxNULL, psxNULL, psxNULL, psxNULL,
	psxLB     , psxLH    , psxLWL , psxLW   , psxLBU , psxLHU , psxLWR , psxNULL,
	psxSB     , psxSH    , psxSWL , psxSW   , psxNULL, psxNULL, psxSWR , psxNULL,
	psxNULL   , psxNULL  , gteLWC2, psxNULL , psxNULL, psxNULL, psxNULL, psxNULL,
	psxNULL   , psxNULL  , gteSWC2, psxHLE  , psxNULL, psxNULL, psxNULL, psxNULL
};


void (*psxSPC[64])(void) = {
	psxSLL , psxNULL , psxSRL , psxSRA , psxSLLV   , psxNULL , psxSRLV, psxSRAV,
	psxJR  , psxJALR , psxNULL, psxNULL, psxSYSCALL, psxBREAK, psxNULL, psxNULL,
	psxMFHI, psxMTHI , psxMFLO, psxMTLO, psxNULL   , psxNULL , psxNULL, psxNULL,
	psxMULT, psxMULTU, psxDIV , psxDIVU, psxNULL   , psxNULL , psxNULL, psxNULL,
	psxADD , psxADDU , psxSUB , psxSUBU, psxAND    , psxOR   , psxXOR , psxNOR ,
	psxNULL, psxNULL , psxSLT , psxSLTU, psxNULL   , psxNULL , psxNULL, psxNULL,
	psxNULL, psxNULL , psxNULL, psxNULL, psxNULL   , psxNULL , psxNULL, psxNULL,
	psxNULL, psxNULL , psxNULL, psxNULL, psxNULL   , psxNULL , psxNULL, psxNULL
};

void (*psxREG[32])(void) = {
	psxBLTZ  , psxBGEZ  , psxNULL, psxNULL, psxNULL, psxNULL, psxNULL, psxNULL,
	psxNULL  , psxNULL  , psxNULL, psxNULL, psxNULL, psxNULL, psxNULL, psxNULL,
	psxBLTZAL, psxBGEZAL, psxNULL, psxNULL, psxNULL, psxNULL, psxNULL, psxNULL,
	psxNULL  , psxNULL  , psxNULL, psxNULL, psxNULL, psxNULL, psxNULL, psxNULL
};

void (*psxCP0[32])(void) = {
	psxMFC0, psxNULL, psxCFC0, psxNULL, psxMTC0, psxNULL, psxCTC0, psxNULL,
	psxNULL, psxNULL, psxNULL, psxNULL, psxNULL, psxNULL, psxNULL, psxNULL,
	psxRFE , psxNULL, psxNULL, psxNULL, psxNULL, psxNULL, psxNULL, psxNULL,
	psxNULL, psxNULL, psxNULL, psxNULL, psxNULL, psxNULL, psxNULL, psxNULL
};

/* Certain GTE (CP2) functions take an argument, which is the 32-bit CP2 opcode
 *  shifted right 10 places, encoding various parameters. This allows dynarecs
 *  to emit faster calls. However, the interpreter needs some local wrapper
 *  functions to help it pass the argument.
 */
#define GTE_FUNC_1_ARG_WRAPPER(gf) \
static void w_##gf()               \
{                                  \
    gf(psxRegs.code >> 10);        \
}
GTE_FUNC_1_ARG_WRAPPER(gteOP)
GTE_FUNC_1_ARG_WRAPPER(gteDPCS)
GTE_FUNC_1_ARG_WRAPPER(gteINTPL)
GTE_FUNC_1_ARG_WRAPPER(gteMVMVA)
GTE_FUNC_1_ARG_WRAPPER(gteSQR)
GTE_FUNC_1_ARG_WRAPPER(gteDCPL)
GTE_FUNC_1_ARG_WRAPPER(gteGPF)
GTE_FUNC_1_ARG_WRAPPER(gteGPL)

/* Anything beginning with 'w_' is a local wrapper func, see note above. */
void (*psxCP2[64])(void) = {
	psxBASIC  , gteRTPS   , psxNULL   , psxNULL   , psxNULL   , psxNULL   , gteNCLIP  , psxNULL   , // 00
	psxNULL   , psxNULL   , psxNULL   , psxNULL   , w_gteOP   , psxNULL   , psxNULL   , psxNULL   , // 08
	w_gteDPCS , w_gteINTPL, w_gteMVMVA, gteNCDS   , gteCDP    , psxNULL   , gteNCDT   , psxNULL   , // 10
	psxNULL   , psxNULL   , psxNULL   , gteNCCS   , gteCC     , psxNULL   , gteNCS    , psxNULL   , // 18
	gteNCT    , psxNULL   , psxNULL   , psxNULL   , psxNULL   , psxNULL   , psxNULL   , psxNULL   , // 20
	w_gteSQR  , w_gteDCPL , gteDPCT   , psxNULL   , psxNULL   , gteAVSZ3  , gteAVSZ4  , psxNULL   , // 28
	gteRTPT   , psxNULL   , psxNULL   , psxNULL   , psxNULL   , psxNULL   , psxNULL   , psxNULL   , // 30
	psxNULL   , psxNULL   , psxNULL   , psxNULL   , psxNULL   , w_gteGPF  , w_gteGPL  , gteNCCT     // 38
};

void (*psxCP2BSC[32])(void) = {
	gteMFC2, psxNULL, gteCFC2, psxNULL, gteMTC2, psxNULL, gteCTC2, psxNULL,
	psxNULL, psxNULL, psxNULL, psxNULL, psxNULL, psxNULL, psxNULL, psxNULL,
	psxNULL, psxNULL, psxNULL, psxNULL, psxNULL, psxNULL, psxNULL, psxNULL,
	psxNULL, psxNULL, psxNULL, psxNULL, psxNULL, psxNULL, psxNULL, psxNULL
};


///////////////////////////////////////////

static int intInit(void) {
	return 0;
}

static void intReset(void) {
}

static int intExecute_call_count = 0;

static void intExecute(void) {
	/* QPSX_033: Debug logging to diagnose why Execute() doesn't return */
	extern volatile int emu_frame_complete;

	/* Log first few calls to confirm interpreter is used */
	if (intExecute_call_count < 3) {
		INT_LOG(">>> intExecute() ENTERED call #%d - INTERPRETER MODE", intExecute_call_count);
		intExecute_call_count++;
	}

	emu_frame_complete = 0;

	for (;;) {
		execI();
		if (emu_frame_complete) {
			/* Log every return to confirm we're exiting */
			if (intExecute_call_count < 10) {
				INT_LOG("<<< intExecute() RETURNING - emu_frame_complete was 1");
			}
			emu_frame_complete = 0;
			return;
		}
	}
}

static void intExecuteBlock(unsigned target_pc) {
	branch2 = 0;
	do{ execI(); }while(psxRegs.pc!=target_pc);
}

static void intClear(u32 Addr, u32 Size) {
}

/* Custom function added to notify dynarecs about icache-related events.
 *  Could be used here to implement a cleaner version of Shalma's icache
 *  emulation, which hasn't yet been backported to our interpreter.
 */
static void intNotify(int note, void *data) {
}

static void intShutdown(void) {
}

// interpreter execution
void execI(void) {
	/* SF2000: Log first 10 + progress every 100K */
	if (exec_count < 10) {
		INT_LOG("EXEC #%d pc=0x%08x", exec_count, psxRegs.pc);
	} else if ((exec_count % 100000) == 0) {
		INT_LOG("PROGRESS: %d instructions, pc=0x%08x", exec_count, psxRegs.pc);
	}

	u32 *code = (u32 *)PSXM(psxRegs.pc);
	psxRegs.code = ((code == NULL) ? 0 : SWAP32(*code));

	if (exec_count < 10) {
		INT_LOG("  op=0x%08x h=%d", psxRegs.code, psxRegs.code >> 26);
	}

	debugI();

	psxRegs.pc += 4;
	psxRegs.cycle += BIAS;

	psxBSC[psxRegs.code >> 26]();

	if (exec_count < 10) {
		INT_LOG("  done");
	}
	exec_count++;
}

R3000Acpu psxInt = {
	intInit,
	intReset,
	intExecute,
	intExecuteBlock,
	intClear,
	intNotify,
	intShutdown
};
