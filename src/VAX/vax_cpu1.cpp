/* vax_cpu1.c: VAX complex instructions

   Copyright (c) 1998-2011, Robert M Supnik

   Permission is hereby granted, free of charge, to any person obtaining a
   copy of this software and associated documentation files (the "Software"),
   to deal in the Software without restriction, including without limitation
   the rights to use, copy, modify, merge, publish, distribute, sublicense,
   and/or sell copies of the Software, and to permit persons to whom the
   Software is furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
   ROBERT M SUPNIK BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
   IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
   CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

   Except as contained in this notice, the name of Robert M Supnik shall not be
   used in advertising or otherwise to promote the sale, use or other dealings
   in this Software without prior written authorization from Robert M Supnik.

   25-Nov-11    RMS     Added VEC_QBUS test in interrupt handler
   23-Mar-11    RMS     Revised idle design (from Mark Pizzolato)
   28-May-08    RMS     Inlined physical memory routines
   29-Apr-07    RMS     Separated base register access checks for 11/780
   10-May-06    RMS     Added access check on system PTE for 11/780
                        Added mbz check in LDPCTX for 11/780
   22-Sep-06    RMS     Fixed declarations (from Sterling Garwood)
   30-Sep-04    RMS     Added conditionals for full VAX
                        Moved emulation to vax_cis.c
                        Moved model-specific IPRs to system module
   27-Jan-04    RMS     Added device logging support
                        Fixed EXTxV, INSV double register PC reference fault
   30-Apr-02    RMS     Fixed interrupt/exception handler to clear traps
   17-Apr-02    RMS     Fixed pos > 31 test in bit fields (should be unsigned)
   14-Apr-02    RMS     Fixed prv_mode handling for interrupts (found by Tim Stark)
                        Fixed PROBEx to mask mode to 2b (found by Kevin Handy)

   This module contains the instruction simulators for

   Field instructions:
        - BBS, BBC, BBSSI, BBCCI
        - BBSC, BBCC, BBCS, BBSS
        - EXTV, EXTZV, CMPV, CMPZV
        - FFS, FFC, INSV

   Call/return and push/pop instructions:
        - CALLS, CALLG, RET
        - PUSHR, POPR

   Queue instructions:
        - INSQUE, REMQUE
        - INSQHI, INSQTI, REMQHI, REMQTI

   String instructions:
        - MOVC3, MOVC5, CMPC3, CMPC5
        - LOCC, SKPC, SCANC, SPANC

   Operating system interface instructions:
        - CHMK, CHME, CHMS, CHMU
        - PROBER, PROBEW, REI
        - MTPR, MFPR
        - LDPCTX, SVPCTX
        - (interrupt and exception routine) 
*/

#include "sim_defs.h"
#include "vax_defs.h"

#include "vax_cpu.h"

static int32 op_insqti_native (RUN_DECL, int32 *opnd, int32 acc);
static int32 op_insqti_portable (RUN_DECL, int32 *opnd, int32 acc);
static int32 op_remqti_native (RUN_DECL, int32 *opnd, int32 acc);
static int32 op_remqti_portable (RUN_DECL, int32 *opnd, int32 acc);
static int32 op_remqhi_native (RUN_DECL, int32 *opnd, int32 acc);
static int32 op_remqhi_portable (RUN_DECL, int32 *opnd, int32 acc);
static int32 op_remqhi_portable_2 (RUN_DECL, int32 *opnd, int32 acc, sim_try_volatile InterlockedOpLock* iop);
static int32 op_insqhi_native (RUN_DECL, int32 *opnd, int32 acc);
static int32 op_insqhi_portable (RUN_DECL, int32 *opnd, int32 acc);
static int32 op_insqhi_portable_2 (RUN_DECL, int32 *opnd, int32 acc, sim_try_volatile InterlockedOpLock* iop);


static const uint8 rcnt[128] = {
 0, 4, 4, 8, 4, 8, 8,12, 4, 8, 8,12, 8,12,12,16,        /* 00 - 0F */
 4, 8, 8,12, 8,12,12,16, 8,12,12,16,12,16,16,20,        /* 10 - 1F */
 4, 8, 8,12, 8,12,12,16, 8,12,12,16,12,16,16,20,        /* 20 - 2F */
 8,12,12,16,12,16,16,20,12,16,16,20,16,20,20,24,        /* 30 - 3F */
 4, 8, 8,12, 8,12,12,16, 8,12,12,16,12,16,16,20,        /* 40 - 4F */
 8,12,12,16,12,16,16,20,12,16,16,20,16,20,20,24,        /* 50 - 5F */
 8,12,12,16,12,16,16,20,12,16,16,20,16,20,20,24,        /* 60 - 6F */
12,16,16,20,16,20,20,24,16,20,20,24,20,24,24,28         /* 70 - 7F */
};

// int32 last_chm = 0;

extern const uint32 byte_mask[33];
extern SMP_FILE *sim_deb;
extern DEVICE cpu_dev;

/* Branch on bit and no modify
   Branch on bit and modify

        opnd[0] =       position (pos.rl)
        opnd[1] =       register number/memory flag
        opnd[2] =       memory address, if memory
   Returns bit to be tested
*/

int32 op_bb_n (RUN_DECL, int32 *opnd, int32 acc)
{
int32 pos = opnd[0];
int32 rn = opnd[1];
int32 ea, by;

if (rn >= 0) {                                          /* register? */
    if (((uint32) pos) > 31)                            /* pos > 31? fault */
        RSVD_OPND_FAULT;
    return (R[rn] >> pos) & 1;                          /* get bit */
    }
ea = opnd[2] + (pos >> 3);                              /* base byte addr */
pos = pos & 07;                                         /* pos in byte */
by = Read (RUN_PASS, ea, L_BYTE, RA);                   /* read byte */
return ((by >> pos) & 1);                               /* get bit */
}

int32 op_bb_x (RUN_DECL, int32 *opnd, int32 newb, int32 acc, t_bool interlocked)
{
    // t_bool entering_ilk = FALSE;

    if (interlocked && use_native_interlocked && opnd[1] < 0)
    {
        int32 pos = opnd[0];
        int32 ea = opnd[2] + (pos >> 3);
        int32 pa = TestMark (RUN_PASS, ea, WA, NULL);
        if (ADDR_IS_MEM(pa))
        {
            /* enter ILK synchronization window for BBSSI */
            if (newb && (syncw.on & SYNCW_ILK) && !(cpu_unit->syncw_active & SYNCW_ILK) && 
                PSL_GETIPL(PSL) >= syncw.ipl_resched)
            {
                syncw_enter_ilk(RUN_PASS);
                // entering_ilk = TRUE;
            }

            /* change the bit */
            int32 bit = smp_native_bb(M, pa, pos & 7, (t_bool) newb);

            /* if the bit was already set, pause in presumed BBSSI spinlock acquisition spin-loop... */
            if (newb && bit)
            {
                smp_cpu_relax();
                /*
                 * It is tempting to back out of ILK if BBSSI does not actually flip the bit.
                 * However this would limit the coverage of interaction cases protected by ILK window.
                 * In addition, it would also cause thrashing when spinning on OS-level spinlock: 
                 * for each spin cycle, VCPU will be entering and exiting syncw, having to acquire 
                 * cpu_database_lock twice. Therefore avoid it.
                 */
                // if (FALSE && entering_ilk)
                //     syncw_leave_ilk(RUN_PASS);
            }

            return bit;
        }
        /*
         * BBSSI/BBCCI to a non-memory location is not supported by VAX architecture.
         * Could signal an exception, but just fall through to the original SIMH code.
         */
    }

    sim_try_volatile InterlockedOpLock iop(RUN_PASS);

    int32 pos = opnd[0];
    int32 rn = opnd[1];
    int32 ea, by, bit;

    t_bool b_sys_mask = FALSE;
    uint32 old_sys_mask = 0;                                /* initialize to suppress false GCC warning */
    int32 sys_pos = 0;                                      /* ... */

    if (rn >= 0)                                            /* register? */
    {
        if (((uint32) pos) > 31)                            /* pos > 31? fault */
            RSVD_OPND_FAULT;
        bit = (R[rn] >> pos) & 1;                           /* get bit */
        R[rn] = newb? (R[rn] | (1u << pos)): (R[rn] & ~(1u << pos));
        return bit;
    }

    /* Trap access to VMS CPU idle mask */
    if (unlikely(opnd[2] == sys_idle_cpu_mask_va) &&
        sys_idle_cpu_mask_va && mapen &&
        (PSL & PSL_CUR) == 0 && is_os_running(RUN_PASS) &&
        pos <= 31)
    {
        /* in VMS, only BBSC, CLRL and BICL2 will actually clear bits in the idle mask */
        b_sys_mask = TRUE;
        old_sys_mask = Read (RUN_PASS, sys_idle_cpu_mask_va, L_LONG, RA);
        sys_pos = pos;
    }

    ea = opnd[2] + (pos >> 3);                              /* base byte addr */
    pos = pos & 07;                                         /* pos in byte */
    if (interlocked)  iop.virt_lock(ea, acc);               /* base longword will be interlocked by iop.lock(), not target byte */
    by = Read (RUN_PASS, ea, L_BYTE, WA);                   /* read byte */
    bit = (by >> pos) & 1;                                  /* get bit */

    /* enter ILK synchronization window for BBSSI, but will stay there only if BBSSI actually flips the bit */
    if (interlocked && newb && (syncw.on & SYNCW_ILK) && !(cpu_unit->syncw_active & SYNCW_ILK))
    {
        syncw_enter_ilk(RUN_PASS);
        // entering_ilk = TRUE;
    }

    by = newb ? (by | (1u << pos)) : (by & ~(1u << pos));   /* change bit */
    iop.wmb = TRUE;
    Write (RUN_PASS, ea, by, L_BYTE, WA);                   /* rewrite byte */

    iop.unlock();

    /* If bits in VMS CPU idle mask were cleared, wake up corresponding CPUs */
    if (unlikely(b_sys_mask))
    {
        uint32 new_sys_mask = newb ? (old_sys_mask | (1u << sys_pos)) : 
                                     (old_sys_mask & ~(1u << sys_pos));
        wakeup_cpus(RUN_PASS, old_sys_mask, new_sys_mask);
    }

    /* if the bit was already set, pause in presumed BBSSI spinlock acquisition spin-loop... */
    if (interlocked && newb && bit)
    {
        smp_cpu_relax();
        /* Do NOT back out of ILK (see comment above) */
        // if (FALSE && entering_ilk)
        //     syncw_leave_ilk(RUN_PASS);
    }

    return bit;
}

/* Extract field

        opnd[0] =       position (pos.rl)
        opnd[1] =       size (size.rb)
        opnd[2] =       register number/memory flag
        opnd[3] =       register content/memory address

   If the field is in a register, rn + 1 is in vfldrp1
*/

int32 op_extv (RUN_DECL, int32 *opnd, int32 vfldrp1, int32 acc)
{
int32 pos = opnd[0];
int32 size = opnd[1];
int32 rn = opnd[2];
uint32 wd = opnd[3];
int32 ba, wd1 = 0;

if (size == 0)                                          /* size 0? field = 0 */
    return 0;
if (size > 32)                                          /* size > 32? fault */
    RSVD_OPND_FAULT;
if (rn >= 0) {                                          /* register? */
    if (((uint32) pos) > 31)                            /* pos > 31? fault */
        RSVD_OPND_FAULT;
    if (((pos + size) > 32) && (rn >= nSP))             /* span 2 reg, PC? */
        RSVD_ADDR_FAULT;                                /* fault */
    if (pos)
        wd = (wd >> pos) | (((uint32) vfldrp1) << (32 - pos));
    }
else {
    ba = wd + (pos >> 3);                               /* base byte addr */
    pos = (pos & 07) | ((ba & 03) << 3);                /* bit offset */
    ba = ba & ~03;                                      /* lw align base */
    wd = Read (RUN_PASS, ba, L_LONG, RA);               /* read field */
    if ((size + pos) > 32)
        wd1 = Read (RUN_PASS, ba + 4, L_LONG, RA);
    if (pos)
        wd = (wd >> pos) | (((uint32) wd1) << (32 - pos));
    }
return wd & byte_mask[size];
}

/* Insert field

        opnd[0] =       field (src.rl)
        opnd[1] =       position (pos.rl)
        opnd[2] =       size (size.rb)
        opnd[3] =       register number/memory flag
        opnd[4] =       register content/memory address

   If the field is in a register, rn + 1 is in vfldrp1
*/

void op_insv (RUN_DECL, int32 *opnd, int32 vfldrp1, int32 acc)
{
uint32 ins = opnd[0];
int32 pos = opnd[1];
int32 size = opnd[2];
int32 rn = opnd[3];
int32 val, mask, ba, wd, wd1;

if (size == 0)                                          /* size = 0? done */
    return;
if (size > 32)                                          /* size > 32? fault */
    RSVD_OPND_FAULT;
if (rn >= 0) {                                          /* in registers? */
    if (((uint32) pos) > 31)                            /* pos > 31? fault */
        RSVD_OPND_FAULT;
    if ((pos + size) > 32) {                            /* span two reg? */
        if (rn >= nSP)                                  /* if PC, fault */
            RSVD_ADDR_FAULT;
        mask = byte_mask[pos + size - 32];              /* insert fragment */
        val = ins >> (32 - pos);
        R[rn + 1] = (vfldrp1 & ~mask) | (val & mask);
        }
    mask = byte_mask[size] << pos;                      /* insert field */
    val = ins << pos;
    R[rn] = (R[rn] & ~mask) | (val & mask);
    }
else {
    ba = opnd[4] + (pos >> 3);                          /* base byte addr */
    pos = (pos & 07) | ((ba & 03) << 3);                /* bit offset */
    ba = ba & ~03;                                      /* lw align base */
    wd = Read (RUN_PASS, ba, L_LONG, WA);               /* read field */
    if ((size + pos) > 32) {                            /* field span lw? */
        wd1 = Read (RUN_PASS, ba + 4, L_LONG, WA);      /* read 2nd lw */
        mask = byte_mask[pos + size - 32];              /* insert fragment */
        val = ins >> (32 - pos);
        Write (RUN_PASS, ba + 4, (wd1 & ~mask) | (val & mask), L_LONG, WA);
        }
    mask = byte_mask[size] << pos;                      /* insert field */
    val = ins << pos;
    Write (RUN_PASS, ba, (wd & ~mask) | (val & mask), L_LONG, WA);
    }
return;
}

/* Find first */

int32 op_ffs (RUN_DECL, uint32 wd, int32 size)
{
int32 i;

for (i = 0; wd; i++, wd = wd >> 1) {
    if (wd & 1)
        return i;
    }
return size;
}

#define CALL_DV         0x8000                          /* DV set */
#define CALL_IV         0x4000                          /* IV set */
#define CALL_MBZ        0x3000                          /* MBZ */
#define CALL_MASK       0x0FFF                          /* mask */
#define CALL_V_SPA      30                              /* SPA */
#define CALL_M_SPA      03
#define CALL_V_S        29                              /* S flag */
#define CALL_S          (1 << CALL_V_S)
#define CALL_V_MASK     16
#define CALL_PUSH(n)    if ((mask >> (n)) & 1) { \
                            tsp = tsp - 4; \
                            Write (RUN_PASS, tsp, R[n], L_LONG, WA); \
                            }
#define CALL_GETSPA(x)  (((x) >> CALL_V_SPA) & CALL_M_SPA)
#define RET_POP(n)      if ((spamask >> (n + CALL_V_MASK)) & 1) { \
                            R[n] = Read (RUN_PASS, tsp, L_LONG, RA); \
                            tsp = tsp + 4; \
                            }
#define PUSHR_PUSH(n)   CALL_PUSH(n)
#define POPR_POP(n)     if ((mask >> (n)) & 1) { \
                            R[n] = Read (RUN_PASS, SP, L_LONG, RA); \
                            SP = SP + 4; \
                            }

/* CALLG, CALLS

        opnd[0]         =       argument (arg.rx)
        opnd[1]         =       procedure address (adr.ab)
        flg             =       CALLG (0), CALLS (1)    
        acc             =       access mask

        These instructions implement a generalized procedure call and return facility.
        The principal data structure involved is the stack frame.
        CALLS and CALLG build a stack frame in the following format:


        +---------------------------------------------------------------+
        |                 condition handler (initially 0)               |
        +---+-+-+-----------------------+--------------------+----------+       
        |SPA|S|0|   entry mask<11:0>    |   saved PSW<15:5>  | 0 0 0 0 0|
        +---+-+-+-----------------------+--------------------+----------+
        |                           saved AP                            |
        +---------------------------------------------------------------+
        |                           saved FP                            |
        +---------------------------------------------------------------+
        |                           saved PC                            |
        +---------------------------------------------------------------+
        |                           saved R0 (...)                      |
        +---------------------------------------------------------------+
                .                                               .       
                .       (according to entry mask<11:0>)         .       
                .                                               .       
        +---------------------------------------------------------------+
        |                           saved R11 (...)                     |
        +---------------+-----------------------------------------------+
        | #args (CALLS) |       (0-3 bytes needed to align stack)       |
        +---------------+-----------------------------------------------+
        |               |       0       0       0       (CALLS)         |
        +---------------+-----------------------------------------------+

        RET expects to find this structure based at the frame pointer (FP).

        For CALLG and CALLS, the entry mask specifies the new settings of
        DV and IV, and also which registers are to be saved on entry:

         15 14 13 12 11                               0
        +--+--+-----+----------------------------------+
        |DV|IV| MBZ |           register mask          |
        +--+--+-----+----------------------------------+

        CALLG/CALLS operation:

                read the procedure entry mask
                make sure that the stack frame will be accessible
                if CALLS, push the number of arguments onto the stack
                align the stack to the next lower longword boundary
                push the registers specified by the procedure entry mask
                push PC, AP, FP, saved SPA/S0/mask/PSW, condition handler
                update PC, SP, FP, AP
                update PSW traps, clear condition codes
*/

int32 op_call (RUN_DECL, int32 *opnd, t_bool gs, int32 acc)
{
int32 addr = opnd[1];
int32 mask, stklen, tsp, wd;

mask = Read (RUN_PASS, addr, L_WORD, RA);               /* get proc mask */
if (mask & CALL_MBZ)                                    /* test mbz */
    RSVD_OPND_FAULT;
stklen = rcnt[mask & 077] + rcnt[(mask >> 6) & 077] + (gs? 24: 20);
Read (RUN_PASS, SP - stklen, L_BYTE, WA);               /* wchk stk */
if (gs) {
    Write (RUN_PASS, SP - 4, opnd[0], L_LONG, WA);      /* if S, push #arg */
    SP = SP - 4;                                        /* stack is valid */
    }
tsp = SP & ~CALL_M_SPA;                                 /* lw align stack */
CALL_PUSH (11);                                         /* check mask bits, */
CALL_PUSH (10);                                         /* push sel reg */
CALL_PUSH (9);
CALL_PUSH (8);
CALL_PUSH (7);
CALL_PUSH (6);
CALL_PUSH (5);
CALL_PUSH (4);
CALL_PUSH (3);
CALL_PUSH (2);
CALL_PUSH (1);
CALL_PUSH (0);
Write (RUN_PASS, tsp - 4, PC, L_LONG, WA);              /* push PC */
Write (RUN_PASS, tsp - 8, FP, L_LONG, WA);              /* push AP */
Write (RUN_PASS, tsp - 12, AP, L_LONG, WA);             /* push FP */
wd = ((SP & CALL_M_SPA) << CALL_V_SPA) | (gs << CALL_V_S) |
    ((mask & CALL_MASK) << CALL_V_MASK) | (PSL & 0xFFE0);
Write (RUN_PASS, tsp - 16, wd, L_LONG, WA);                       /* push spa/s/mask/psw */
Write (RUN_PASS, tsp - 20, 0, L_LONG, WA);                        /* push cond hdlr */
if (gs)                                                 /* update AP */
    AP = SP;
else AP = opnd[0];
SP = FP = tsp - 20;                                     /* update FP, SP */
PSL = (PSL & ~(PSW_DV | PSW_FU | PSW_IV)) |             /* update PSW */
    ((mask & CALL_DV)? PSW_DV: 0) |
    ((mask & CALL_IV)? PSW_IV: 0);
JUMP (addr + 2);                                        /* new PC */
return 0;                                               /* new cc's */
}

int32 op_ret (RUN_DECL, int32 acc)
{
int32 spamask, stklen, newpc, nargs;
int32 tsp = FP;

spamask = Read (RUN_PASS, tsp + 4, L_LONG, RA);         /* spa/s/mask/psw */
if (spamask & PSW_MBZ)                                  /* test mbz */
    RSVD_OPND_FAULT;
stklen = rcnt[(spamask >> CALL_V_MASK) & 077] +
    rcnt[(spamask >> (CALL_V_MASK + 6)) & 077] + ((spamask & CALL_S)? 23: 19);
Read (RUN_PASS, tsp + stklen, L_BYTE, RA);              /* rchk stk end */
AP = Read (RUN_PASS, tsp + 8, L_LONG, RA);              /* restore AP */
FP = Read (RUN_PASS, tsp + 12, L_LONG, RA);             /* restore FP */
newpc = Read (RUN_PASS, tsp + 16, L_LONG, RA);          /* get new PC */
tsp = tsp + 20;                                         /* update stk ptr */
RET_POP (0);                                            /* chk mask bits, */
RET_POP (1);                                            /* pop sel regs */
RET_POP (2);
RET_POP (3);
RET_POP (4);
RET_POP (5);
RET_POP (6);
RET_POP (7);
RET_POP (8);
RET_POP (9);
RET_POP (10);
RET_POP (11);
SP = tsp + CALL_GETSPA (spamask);                       /* dealign stack */
if (spamask & CALL_S) {                                 /* CALLS? */
    nargs = Read (RUN_PASS, SP, L_LONG, RA);            /* read #args */
    SP = SP + 4 + ((nargs & BMASK) << 2);               /* pop arg list */
    }
PSL = (PSL & ~(PSW_DV | PSW_FU | PSW_IV | PSW_T)) |     /* reset PSW */
    (spamask & (PSW_DV | PSW_FU | PSW_IV | PSW_T));
JUMP (newpc);                                           /* set new PC */
return spamask & (CC_MASK);                             /* return cc's */
}

/* PUSHR and POPR */

void op_pushr (RUN_DECL, int32 *opnd, int32 acc)
{
int32 mask = opnd[0] & 0x7FFF;
int32 stklen, tsp;

if (mask == 0)
    return;
stklen = rcnt[(mask >> 7) & 0177] + rcnt[mask & 0177] +
    ((mask & 0x4000)? 4: 0);
Read (RUN_PASS, SP - stklen, L_BYTE, WA);               /* wchk stk end */
tsp = SP;                                               /* temp stk ptr */
PUSHR_PUSH (14);                                        /* check mask bits, */
PUSHR_PUSH (13);                                        /* push sel reg */
PUSHR_PUSH (12);
PUSHR_PUSH (11);
PUSHR_PUSH (10);
PUSHR_PUSH (9);
PUSHR_PUSH (8);
PUSHR_PUSH (7);
PUSHR_PUSH (6);
PUSHR_PUSH (5);
PUSHR_PUSH (4);
PUSHR_PUSH (3);
PUSHR_PUSH (2);
PUSHR_PUSH (1);
PUSHR_PUSH (0);
SP = tsp;                                               /* update stk ptr */
return;
}

void op_popr (RUN_DECL, int32 *opnd, int32 acc)
{
int32 mask = opnd[0] & 0x7FFF;
int32 stklen;

if (mask == 0)
    return;
stklen = rcnt[(mask >> 7) & 0177] + rcnt[mask & 0177] +
    ((mask & 0x4000)? 4: 0);
Read (RUN_PASS, SP + stklen - 1, L_BYTE, RA);           /* rchk stk end */
POPR_POP (0);                                           /* check mask bits, */
POPR_POP (1);                                           /* pop sel regs */
POPR_POP (2);
POPR_POP (3);
POPR_POP (4);
POPR_POP (5);
POPR_POP (6);
POPR_POP (7);
POPR_POP (8);
POPR_POP (9);
POPR_POP (10);
POPR_POP (11);
POPR_POP (12);
POPR_POP (13);
if (mask & 0x4000)                                      /* if pop SP, no inc */
    SP = Read (RUN_PASS, SP, L_LONG, RA);
return;
}


/* INSQUE

        opnd[0] =       entry address (ent.ab)
        opnd[1] =       predecessor address (pred.ab)

   Condition codes returned to caller on comparison of (ent):(ent+4).
   All writes must be checked before any writes are done.

   Pictorially:

        BEFORE                  AFTER

        P:      S               P:      E       W
        P+4:    (n/a)           P+4:    (n/a)

        E:      ---             E:      S       W
        E+4:    ---             E+4:    P       W

        S:      (n/a)           S:      (n/a)
        S+4:    P               S+4:    E       W

   s+4 must be tested with a read modify rather than a probe, as it
   might be misaligned.
*/

int32 op_insque (RUN_DECL, int32 *opnd, int32 acc)
{
int32 p = opnd[1];
int32 e = opnd[0];
int32 s, cc;

s = Read (RUN_PASS, p, L_LONG, WA);                     /* s <- (p), wchk */
Read (RUN_PASS, s + 4, L_LONG, WA);                     /* wchk s+4 */
Read (RUN_PASS, e + 4, L_LONG, WA);                     /* wchk e+4 */
Write (RUN_PASS, e, s, L_LONG, WA);                     /* (e) <- s */
Write (RUN_PASS, e + 4, p, L_LONG, WA);                 /* (e+4) <- p */
Write (RUN_PASS, s + 4, e, L_LONG, WA);                 /* (s+4) <- ent */
Write (RUN_PASS, p, e, L_LONG, WA);                     /* (p) <- e */
CC_CMP_L (s, p);                                        /* set cc's */
return cc;
}

/* REMQUE

        opnd[0] =       entry address (ent.ab)
        opnd[1:2] =     destination address (dst.wl)

   Condition codes returned to caller based on (ent):(ent+4).
   All writes must be checked before any writes are done.

   Pictorially:

        BEFORE                  AFTER

        P:      E               P:      S       W
        P+4:    (n/a)           P+4:    (n/a)

        E:      S       W       E:      S
        E+4:    P       W       E+4:    P

        S:      (n/a)           S:      (n/a)
        S+4:    E       W       S+4:    P

*/

int32 op_remque (RUN_DECL, int32 *opnd, int32 acc)
{
int32 e = opnd[0];
int32 s, p, cc;

s = Read (RUN_PASS, e, L_LONG, RA);                     /* s <- (e) */
p = Read (RUN_PASS, e + 4, L_LONG, RA);                 /* p <- (e+4) */
CC_CMP_L (s, p);                                        /* set cc's */
if (e != p) {                                           /* queue !empty? */
    Read (RUN_PASS, s + 4, L_LONG, WA);                 /* wchk (s+4) */
    if (opnd[1] < 0)                                    /* wchk dest */
        Read (RUN_PASS, opnd[2], L_LONG, WA);
    Write (RUN_PASS, p, s, L_LONG, WA);                 /* (p) <- s */
    Write (RUN_PASS, s + 4, p, L_LONG, WA);             /* (s+4) <- p */
    }
else cc = cc | CC_V;                                    /* else set v */
if (opnd[1] >= 0)                                       /* store result */
    R[opnd[1]] = e;
else Write (RUN_PASS, opnd[2], e, L_LONG, WA);
return cc;
}

/* Interlocked insert instructions

        opnd[0] =       entry (ent.ab)
        opnd[1] =       header (hdr.aq)

        Pictorially:

        BEFORE          AFTER INSQHI            AFTER INSQTI

        H:      A-H     H:      D-H     W       H:      A-H     W for interlock
        H+4:    C-H     H+4:    C-H             H+4:    D-H     W

        A:      B-A     A:      B-A             A:      B-A
        A+4:    H-A     A+4:    D-A     W       A+4:    H-A

        B:      C-B     B:      C-B             B:      C-B
        B+4:    A-B     B+4:    A-B             B+4:    A-B

        C:      H-C     C:      H-C             C:      D-C     W
        C+4:    B-C     C+4:    B-C             C+4:    B-C

        D:      ---     D:      A-D     W       D:      H-D     W
        D+4:    ---     D+4:    H-D     W       D+4:    C-D     W

        Note that the queue header, the entry to be inserted, and all
        the intermediate entries that are "touched" in any way must be
        QUADWORD aligned.  In addition, the header and  the entry must
        not be equal.
*/

int32 op_insqhi (RUN_DECL, int32 *opnd, int32 acc)
{
    if (use_native_interlocked)
        return op_insqhi_native(RUN_PASS, opnd, acc);
    else
        return op_insqhi_portable(RUN_PASS, opnd, acc);
}

static int32 op_insqhi_native (RUN_DECL, int32 *opnd, int32 acc)
{
    int32 pa_h1, pa_d1, pa_d2, pa_a2;
    int32 d = opnd[0];
    int32 h = opnd[1];
    int32 a;
    sim_try_volatile t_bool release = FALSE;
    sim_try_volatile InterlockedOpLock iop(RUN_PASS, IOP_ILK);

    /* check operands alignment and non-equality */
    if (h == d || ((h | d) & 07))
        RSVD_OPND_FAULT;

    /* 
     * get physical addresses; note that "d" is quadword-aligned, 
     * therefore it cannot cross page boundary and hence
     * pa_d2 is in the same page as pa_d1
     */
    pa_h1 = TestMark (RUN_PASS, h, WA, NULL);
    pa_d1 = TestMark (RUN_PASS, d, WA, NULL);
    pa_d2 = pa_d1 + 4;

    /* must be in memory (not IO space): do not need to test pa_d2 */
    if (! (ADDR_IS_MEM(pa_h1) && ADDR_IS_MEM(pa_d1)))
        RSVD_OPND_FAULT;

    sim_try
    {
        /* elevate thread priority if required */
        iop.prio_lock();

        /* acquire secondary interlock */
        smp_pre_interlocked_mb();
        if (smp_native_bb(M, pa_h1, 0, TRUE))
        {
            iop.qxi_busy();
            smp_cpu_relax();
            return CC_C;
        }

        release = TRUE;

        /* forward link from header */
        a = h + (ReadLP(RUN_PASS, pa_h1) & ~1);

        if (a & 06)                                         /* check quad align */
            RSVD_OPND_FAULT;

        pa_a2 = TestMark (RUN_PASS, a + 4, WA, NULL);       /* make sure it is valid */
        if (!ADDR_IS_MEM(pa_a2))
            RSVD_OPND_FAULT;

        WriteLP(RUN_PASS, pa_a2, d - a);
        WriteLP(RUN_PASS, pa_d1, a - d);
        WriteLP(RUN_PASS, pa_d2, h - d);

        /* store forward link and release secondary interlock */
        smp_interlocked_uint32* vpa_h1 = (smp_interlocked_uint32*) ((t_byte*) M + pa_h1);
        while (! smp_interlocked_cas_done(vpa_h1, weak_read(*vpa_h1), (uint32) (d - h)))  ;
        release = FALSE;
        smp_post_interlocked_mb();

        return (a == h) ? CC_Z : 0;                         /* Z = 1 if a = h */
    }
    sim_catch_all
    {
        /* release secondary interlock and re-throw */
        if (release)
        {
            smp_native_bb(M, pa_h1, 0, FALSE);
            smp_post_interlocked_mb();
        }
        sim_rethrow;
    }
    sim_end_try

    sim_noreturn_int32;
}

static int32 op_insqhi_portable (RUN_DECL, int32 *opnd, int32 acc)
{
    sim_try_volatile InterlockedOpLock iop(RUN_PASS, IOP_ILK);
    return op_insqhi_portable_2 (RUN_PASS, opnd, acc, &iop);
}

static int32 op_insqhi_portable_2 (RUN_DECL, int32 *opnd, int32 acc, sim_try_volatile InterlockedOpLock* iop)
{
    int32 h = opnd[1];
    int32 d = opnd[0];
    int32 a, t;

    if ((h == d) || ((h | d) & 07))                         /* h, d quad align? */
        RSVD_OPND_FAULT;
    Read (RUN_PASS, d, L_BYTE, WA);                         /* wchk ent */
    iop->virt_lock(h, acc);
    a = Read (RUN_PASS, h, L_LONG, WA);                     /* a <- (h), wchk */
    if (a & 06)                                             /* chk quad align */
        RSVD_OPND_FAULT;
    if (a & 01)                                             /* busy, cc = 0001 */
    {
        iop->qxi_busy();
        return CC_C;
    }
    iop->wmb = TRUE;
    Write (RUN_PASS, h, a | 1, L_LONG, WA);                 /* get interlock */
    a = a + h;                                              /* abs addr of a */
    if (Test (RUN_PASS, a, WA, &t) < 0)                     /* wtst a, rls if err */
        Write (RUN_PASS, h, a - h, L_LONG, WA);
    Write (RUN_PASS, a + 4, d - a, L_LONG, WA);             /* (a+4) <- d-a, flt ok */
    Write (RUN_PASS, d, a - d, L_LONG, WA);                 /* (d) <- a-d */
    Write (RUN_PASS, d + 4, h - d, L_LONG, WA);             /* (d+4) <- h-d */
    Write (RUN_PASS, h, d - h, L_LONG, WA);                 /* (h) <- d-h, rls int */

    return (a == h)? CC_Z: 0;                               /* Z = 1 if a = h */
}

int32 op_insqti (RUN_DECL, int32 *opnd, int32 acc)
{
    if (use_native_interlocked)
        return op_insqti_native(RUN_PASS, opnd, acc);
    else
        return op_insqti_portable(RUN_PASS, opnd, acc);
}

static int32 op_insqti_native (RUN_DECL, int32 *opnd, int32 acc)
{
    int32 d = opnd[0];
    int32 h = opnd[1];
    int32 pa_c1, pa_d1, pa_d2, pa_h1, pa_h2;
    int32 a, c;
    sim_try_volatile t_bool release = FALSE;
    sim_try_volatile InterlockedOpLock iop(RUN_PASS, IOP_ILK);

    if (h == d || ((h | d) & 07))                           /* h, d quad align? */
        RSVD_OPND_FAULT;

    /* 
     * get physical addresses; note that "h" and "d" are quadword-aligned, 
     * therefore it cannot cross page boundary and hence
     * pa_h2 is in the same page as pa_h1
     */
    pa_h1 = TestMark (RUN_PASS, h, WA, NULL);
    pa_h2 = pa_h1 + 4;

    pa_d1 = TestMark (RUN_PASS, d, WA, NULL);
    pa_d2 = pa_d1 + 4;

    /* must be in memory (not IO space): do not need to test pa_h2 and pa_d2 */
    if (! (ADDR_IS_MEM(pa_h1) && ADDR_IS_MEM(pa_d1)))
        RSVD_OPND_FAULT;

    sim_try
    {
        /* elevate thread priority if required */
        iop.prio_lock();

        /* acquire secondary interlock */
        smp_pre_interlocked_mb();
        if (smp_native_bb(M, pa_h1, 0, TRUE))
        {
            iop.qxi_busy();
            smp_cpu_relax();
            return CC_C;
        }

        release = TRUE;

        smp_interlocked_uint32* vpa_h1 = (smp_interlocked_uint32*) ((t_byte*) M + pa_h1);
        a = ReadLP(RUN_PASS, pa_h1) & ~1;

        if (a & 06)                                         /* chk quad align */
            RSVD_OPND_FAULT;

        if (a == 0)
        {
            /* queue was empty, perform insqhi */
            WriteLP(RUN_PASS, pa_h2, d - h);
            WriteLP(RUN_PASS, pa_d1, h - d);
            WriteLP(RUN_PASS, pa_d2, h - d);

            /* store forward link and release secondary interlock */
            while (! smp_interlocked_cas_done(vpa_h1, weak_read(*vpa_h1), (uint32) (d - h)))  ;
            release = FALSE;
            smp_post_interlocked_mb();

            return CC_Z;
        }
        else
        {
            c = ReadLP(RUN_PASS, pa_h2) + h;                /* address of previous tail */
            if (c & 07)
                RSVD_OPND_FAULT;                                    
            pa_c1 = TestMark (RUN_PASS, c, WA, NULL);
            if (! ADDR_IS_MEM(pa_c1))
                RSVD_OPND_FAULT;

            WriteLP(RUN_PASS, pa_c1, d - c);
            WriteLP(RUN_PASS, pa_d1, h - d);
            WriteLP(RUN_PASS, pa_d2, c - d);
            WriteLP(RUN_PASS, pa_h2, d - h);

            /* release secondary interlock */
            while (! smp_interlocked_cas_done(vpa_h1, weak_read(*vpa_h1), a))  ;
            release = FALSE;
            smp_post_interlocked_mb();

            return 0;
        }
    }
    sim_catch_all
    {
        /* release secondary interlock and re-throw */
        if (release)
        {
            smp_native_bb(M, pa_h1, 0, FALSE);
            smp_post_interlocked_mb();
        }
        sim_rethrow;
    }
    sim_end_try

    sim_noreturn_int32;
}

static int32 op_insqti_portable (RUN_DECL, int32 *opnd, int32 acc)
{
    sim_try_volatile InterlockedOpLock iop(RUN_PASS, IOP_ILK);

    int32 h = opnd[1];
    int32 d = opnd[0];
    int32 a, c, t;

    if ((h == d) || ((h | d) & 07))                         /* h, d quad align? */
        RSVD_OPND_FAULT;
    Read (RUN_PASS, d, L_BYTE, WA);                         /* wchk ent */
    iop.virt_lock(h, acc);
    a = Read (RUN_PASS, h, L_LONG, WA);                     /* a <- (h), wchk */
    if (a == 0)                                             /* if empty, ins hd */
        return op_insqhi_portable_2 (RUN_PASS, opnd, acc, &iop);
    if (a & 06)                                             /* chk quad align */
        RSVD_OPND_FAULT;
    if (a & 01)                                             /* busy, cc = 0001 */
    {
        iop.qxi_busy();
        return CC_C;
    }
    iop.wmb = true;
    Write (RUN_PASS, h, a | 1, L_LONG, WA);                 /* acquire interlock */
    c = Read (RUN_PASS, h + 4, L_LONG, RA) + h;             /* c <- (h+4) + h */
    if (c & 07) {                                           /* c quad aligned? */
        Write (RUN_PASS, h, a, L_LONG, WA);                 /* release interlock */
        RSVD_OPND_FAULT;                                    /* fault */
        }
    if (Test (RUN_PASS, c, WA, &t) < 0)                     /* wtst c, rls if err */
        Write (RUN_PASS, h, a, L_LONG, WA);
    Write (RUN_PASS, c, d - c, L_LONG, WA);                 /* (c) <- d-c, flt ok */
    Write (RUN_PASS, d, h - d, L_LONG, WA);                 /* (d) <- h-d */
    Write (RUN_PASS, d + 4, c - d, L_LONG, WA);             /* (d+4) <- c-d */
    Write (RUN_PASS, h + 4, d - h, L_LONG, WA);             /* (h+4) <- d-h */
    Write (RUN_PASS, h, a, L_LONG, WA);                     /* release interlock */
    return 0;                                               /* q >= 2 entries */
}

/* Interlocked remove instructions

        opnd[0] =       header (hdr.aq)
        opnd[1:2] =     destination address (dst.al)

        Pictorially:

        BEFORE          AFTER REMQHI            AFTER REMQTI

        H:      A-H     H:      B-H     W       H:      A-H     W for interlock
        H+4:    C-H     H+4:    C-H             H+4:    B-H     W

        A:      B-A     A:      B-A     R       A:      B-A
        A+4:    H-A     A+4:    H-A             A+4:    H-A

        B:      C-B     B:      C-B             B:      H-B     W
        B+4:    A-B     B+4:    H-B     W       B+4:    A-B

        C:      H-C     C:      H-C             C:      H-C
        C+4:    B-C     C+4:    B-C             C+4:    B-C     R

        Note that the queue header and all the  entries that are
        "touched" in any way must be QUADWORD aligned.  In addition,
        the header and the destination must not be equal.
*/

int32 op_remqhi (RUN_DECL, int32 *opnd, int32 acc)
{
    if (use_native_interlocked)
        return op_remqhi_native(RUN_PASS, opnd, acc);
    else
        return op_remqhi_portable(RUN_PASS, opnd, acc);
}

static int32 op_remqhi_native (RUN_DECL, int32 *opnd, int32 acc)
{
    int32 h = opnd[0];
    int32 pa_h1;
    int32 ar, a, b = 0;                                     /* init b to suppress false GCC warning */
    sim_try_volatile t_bool release = FALSE;
    sim_try_volatile InterlockedOpLock iop(RUN_PASS, IOP_ILK);

    if (h & 07)                                             /* h quad aligned? */
        RSVD_OPND_FAULT;
    pa_h1 = TestMark (RUN_PASS, h, WA, NULL);

    /* must be in memory (not IO space) */
    if (! (ADDR_IS_MEM(pa_h1)))
        RSVD_OPND_FAULT;

    if (opnd[1] < 0)                                        /* mem destination? */
    {
        if (h == opnd[2])                                   /* hdr = dst? */
            RSVD_OPND_FAULT;
        Read (RUN_PASS, opnd[2], L_LONG, WA);               /* wchk dst */
    }

    sim_try
    {
        /* elevate thread priority if required */
        iop.prio_lock();

        /* acquire secondary interlock */
        smp_pre_interlocked_mb();
        if (smp_native_bb(M, pa_h1, 0, TRUE))
        {
            iop.qxi_busy();
            smp_cpu_relax();
            return CC_C | CC_V;
        }

        release = TRUE;

        ar = ReadLP(RUN_PASS, pa_h1) & ~1;                  /* ar <- (h) */
        if (ar & 06)                                        /* a quad aligned? */
            RSVD_OPND_FAULT;
        a = ar + h;                                         /* abs addr of a */

        if (ar)
        {
            b = Read (RUN_PASS, a, L_LONG, RA) + a;         /* b <- (a)+a, flt ok */
            if (b & 07)                                     /* b quad aligned? */
                RSVD_OPND_FAULT;                            /* fault */
            Write (RUN_PASS, b + 4, h - b, L_LONG, WA);     /* (b+4) <- h-b, flt ok */

            smp_interlocked_uint32* vpa_h1 = (smp_interlocked_uint32*) ((t_byte*) M + pa_h1);
            while (! smp_interlocked_cas_done(vpa_h1, weak_read(*vpa_h1), (uint32) (b - h)))  ;
            smp_post_interlocked_mb();
            release = FALSE;
        }
        else
        {
            smp_native_bb(M, pa_h1, 0, FALSE);
            smp_post_interlocked_mb();
            release = FALSE;
        }

        if (opnd[1] >= 0)                                   /* store result */
            R[opnd[1]] = a;
        else
            Write (RUN_PASS, opnd[2], a, L_LONG, WA);

        if (ar == 0)                                        /* queue was empty */
            return CC_Z | CC_V;

        return (b == h) ? CC_Z : 0;                         /* if b = h, queue empty after removal */
    }
    sim_catch_all
    {
        /* release secondary interlock and re-throw */
        if (release)
        {
            smp_native_bb(M, pa_h1, 0, FALSE);
            smp_post_interlocked_mb();
        }
        sim_rethrow;
    }
    sim_end_try

    sim_noreturn_int32;
}

int32 op_remqhi_portable (RUN_DECL, int32 *opnd, int32 acc)
{
    sim_try_volatile InterlockedOpLock iop(RUN_PASS, IOP_ILK);
    return op_remqhi_portable_2 (RUN_PASS, opnd, acc, &iop);
}

static int32 op_remqhi_portable_2 (RUN_DECL, int32 *opnd, int32 acc, sim_try_volatile InterlockedOpLock* iop)
{
    int32 h = opnd[0];
    int32 ar, a, b = 0, t;                                  /* init b to suppress false GCC warning */

    if (h & 07)                                             /* h quad aligned? */
        RSVD_OPND_FAULT;
    if (opnd[1] < 0)                                        /* mem destination? */
    {
        if (h == opnd[2])                                   /* hdr = dst? */
            RSVD_OPND_FAULT;
        Read (RUN_PASS, opnd[2], L_LONG, WA);               /* wchk dst */
    }
    iop->virt_lock(h, acc);
    ar = Read (RUN_PASS, h, L_LONG, WA);                    /* ar <- (h) */
    if (ar & 06)                                            /* a quad aligned? */
        RSVD_OPND_FAULT;
    if (ar & 01)                                            /* busy, cc = 0011 */
    {
        iop->qxi_busy();
        return CC_V | CC_C;
    }
    a = ar + h;                                             /* abs addr of a */
    if (ar)                                                 /* queue not empty? */
    {
        iop->wmb = true;
        Write (RUN_PASS, h, ar | 1, L_LONG, WA);            /* acquire interlock */
        if (Test (RUN_PASS, a, RA, &t) < 0)                 /* read tst a */
             Write (RUN_PASS, h, ar, L_LONG, WA);           /* release if error */
        b = Read (RUN_PASS, a, L_LONG, RA) + a;             /* b <- (a)+a, flt ok */
        if (b & 07) {                                       /* b quad aligned? */
            Write (RUN_PASS, h, ar, L_LONG, WA);            /* release interlock */
            RSVD_OPND_FAULT;                                /* fault */
            }
        /* since b is quad-aligned, b+4 is guaranteed to be in the same page as b */
        if (Test (RUN_PASS, b, WA, &t) < 0)                 /* write test b and b+4 */
            Write (RUN_PASS, h, ar, L_LONG, WA);            /* release if err */
        Write (RUN_PASS, b + 4, h - b, L_LONG, WA);         /* (b+4) <- h-b, flt ok */
        Write (RUN_PASS, h, b - h, L_LONG, WA);             /* (h) <- b-h, rls int */
    }
    if (opnd[1] >= 0)                                       /* store result */
        R[opnd[1]] = a;
    else
    { 
        iop->wmb = true;
        Write (RUN_PASS, opnd[2], a, L_LONG, WA);
    }
    if (ar == 0)                                            /* empty, cc = 0110 */
        return CC_Z | CC_V;
    return (b == h)? CC_Z: 0;                               /* if b = h, q empty */
}

int32 op_remqti (RUN_DECL, int32 *opnd, int32 acc)
{
    if (use_native_interlocked)
        return op_remqti_native(RUN_PASS, opnd, acc);
    else
        return op_remqti_portable(RUN_PASS, opnd, acc);
}

static int32 op_remqti_native (RUN_DECL, int32 *opnd, int32 acc)
{
    int32 h = opnd[0];
    int32 ar, b, c, rcc;
    int32 pa_h1, pa_h2;
    sim_try_volatile t_bool release = FALSE;
    sim_try_volatile InterlockedOpLock iop(RUN_PASS, IOP_ILK);

    if (h & 07)                                             /* h quad aligned? */
        RSVD_OPND_FAULT;

    /* 
     * get physical addresses; since "h" is quad-aligned, 
     * pa_h1 and pa_h2 are guaranteed to be in the same page
     */
    pa_h1 = TestMark (RUN_PASS, h, WA, NULL);
    pa_h2 = pa_h1 + 4;

    /* must be in memory (not IO space); 
       test for pa_h1 also covers pa_h2 too */
    if (! (ADDR_IS_MEM(pa_h1)))
        RSVD_OPND_FAULT;

    if (opnd[1] < 0)                                        /* mem destination? */
    {
        if (h == opnd[2])                                   /* hdr = dst? */
            RSVD_OPND_FAULT;
        Read (RUN_PASS, opnd[2], L_LONG, WA);               /* wchk dst */
    }

    sim_try
    {
        /* elevate thread priority if required */
        iop.prio_lock();

        /* acquire secondary interlock */
        smp_pre_interlocked_mb();
        if (smp_native_bb(M, pa_h1, 0, TRUE))
        {
            iop.qxi_busy();
            smp_cpu_relax();
            return CC_C | CC_V;
        }

        release = TRUE;

        ar = ReadLP(RUN_PASS, pa_h1) & ~1;                  /* ar <- (h) */
        if (ar & 06)                                        /* a quad aligned? */
            RSVD_OPND_FAULT;

        if (ar)                                             /* queue not empty */
        {
            c = ReadLP (RUN_PASS, pa_h2);                   /* c <- (h+4) */
            if (ar == c)                                    /* single entry ? */
            {
                c = ar + h;                                     /* result: abs addr of removed entry */
                WriteLP(RUN_PASS, pa_h2, 0);                    /* clear header, release interlock */
                smp_interlocked_uint32* vpa_h1 = (smp_interlocked_uint32*) ((t_byte*) M + pa_h1);
                while (! smp_interlocked_cas_done(vpa_h1, weak_read(*vpa_h1), 0))  ;
                smp_post_interlocked_mb();
                release = FALSE;
                rcc = CC_Z;                                     /* result code: queue is empty after removal */
            }
            else
            {
                if (c & 07)                                     /* c quad aligned? */
                    RSVD_OPND_FAULT;                            /* fault */
                c = c + h;                                      /* abs addr of c */
                b = Read (RUN_PASS, c + 4, L_LONG, RA) + c;     /* b <- (c+4)+c */
                if (b & 07)                                     /* b quad aligned? */
                    RSVD_OPND_FAULT;                            /* fault */
                Write (RUN_PASS, b, h - b, L_LONG, WA);         /* (b) <- h-b */
                WriteLP (RUN_PASS, pa_h2, b - h);               /* (h+4) <- b-h */
                smp_native_bb(M, pa_h1, 0, FALSE);              /* release interlock */
                smp_post_interlocked_mb();
                release = FALSE;
                rcc = 0;                                        /* result code: queue not empty after removal */
            }
        }
        else
        {
            smp_native_bb(M, pa_h1, 0, FALSE);              /* release interlock */
            smp_post_interlocked_mb();
            release = FALSE;
            c = h;                                          /* result address */
            rcc = CC_Z | CC_V;                              /* result code: queue was empty */
        }

        if (opnd[1] >= 0)                                   /* store result */
            R[opnd[1]] = c;
        else
            Write (RUN_PASS, opnd[2], c, L_LONG, WA);

        return rcc;
    }
    sim_catch_all
    {
        /* release secondary interlock and re-throw */
        if (release)
        {
            smp_native_bb(M, pa_h1, 0, FALSE);
            smp_post_interlocked_mb();
        }
        sim_rethrow;
    }
    sim_end_try

    sim_noreturn_int32;
}

int32 op_remqti_portable (RUN_DECL, int32 *opnd, int32 acc)
{
    sim_try_volatile InterlockedOpLock iop(RUN_PASS, IOP_ILK);

    int32 h = opnd[0];
    int32 ar, b, c, t;

    if (h & 07)                                             /* h quad aligned? */
        RSVD_OPND_FAULT;
    if (opnd[1] < 0) {                                      /* mem destination? */
        if (h == opnd[2])                                   /* hdr = dst? */
            RSVD_OPND_FAULT;
        Read (RUN_PASS, opnd[2], L_LONG, WA);               /* wchk dst */
        }
    iop.virt_lock(h, acc);
    ar = Read (RUN_PASS, h, L_LONG, WA);                    /* a <- (h) */
    if (ar & 06)                                            /* a quad aligned? */
        RSVD_OPND_FAULT;
    if (ar & 01)                                            /* busy, cc = 0011 */
    {
        iop.qxi_busy();
        return CC_V | CC_C;
    }
    if (ar)                                                 /* queue not empty */
    {
        iop.wmb = true;
        Write (RUN_PASS, h, ar | 1, L_LONG, WA);            /* acquire interlock */
        c = Read (RUN_PASS, h + 4, L_LONG, RA);             /* c <- (h+4) */
        if (ar == c) {                                      /* single entry? */
            Write (RUN_PASS, h, ar, L_LONG, WA);            /* release interlock */
            return op_remqhi_portable_2 (RUN_PASS, opnd, acc, &iop);       /* treat as remqhi */
            }
        if (c & 07) {                                       /* c quad aligned? */
            Write (RUN_PASS, h, ar, L_LONG, WA);            /* release interlock */
            RSVD_OPND_FAULT;                                /* fault */
            }
        c = c + h;                                          /* abs addr of c */
        if (Test (RUN_PASS, c + 4, RA, &t) < 0)             /* read test c+4 */
            Write (RUN_PASS, h, ar, L_LONG, WA);            /* release if error */
        b = Read (RUN_PASS, c + 4, L_LONG, RA) + c;         /* b <- (c+4)+c, flt ok */
        if (b & 07) {                                       /* b quad aligned? */
            Write (RUN_PASS, h, ar, L_LONG, WA);            /* release interlock */
            RSVD_OPND_FAULT;                                /* fault */
            }
        if (Test (RUN_PASS, b, WA, &t) < 0)                 /* write test b */
            Write (RUN_PASS, h, ar, L_LONG, WA);            /* release if error */
        Write (RUN_PASS, b, h - b, L_LONG, WA);             /* (b) <- h-b */
        Write (RUN_PASS, h + 4, b - h, L_LONG, WA);         /* (h+4) <- b-h */
        Write (RUN_PASS, h, ar, L_LONG, WA);                /* release interlock */
    }
    else
        c = h;                                              /* empty, result = h */
    if (opnd[1] >= 0)                                       /* store result */
        R[opnd[1]] = c;
    else
    {
        iop.wmb = true;
        Write (RUN_PASS, opnd[2], c, L_LONG, WA);
    }
    if (ar == 0)                                            /* empty, cc = 0110 */
        return CC_Z | CC_V;
    return 0;                                               /* q can't be empty */
}


/* String instructions */

#define MVC_FRWD        0                               /* movc state codes */
#define MVC_BACK        1
#define MVC_FILL        3                               /* must be 3 */
#define MVC_M_STATE     3
#define MVC_V_CC        2

/* MOVC3, MOVC5

   if PSL<fpd> = 0 and MOVC3,
        opnd[0] =       length
        opnd[1] =       source address
        opnd[2] =       dest address

   if PSL<fpd> = 0 and MOVC5,
        opnd[0] =       source length
        opnd[1] =       source address
        opnd[2] =       fill
        opnd[3] =       dest length
        opnd[4] =       dest address

   if PSL<fpd> = 1,
        R0      =       delta-PC/fill/initial move length
        R1      =       current source address
        R2      =       current move length
        R3      =       current dest address
        R4      =       dstlen - srclen (loop count if fill state)
        R5      =       cc/state
*/

int32 op_movc (RUN_DECL, int32 *opnd, int32 movc5, int32 acc)
{
int32 i, cc, fill, wd;
int32 j, lnt, mlnt[3];
static const int32 looplnt[3] = { L_BYTE, L_LONG, L_BYTE };

if (PSL & PSL_FPD) {                                    /* FPD set? */
    SETPC (fault_PC + STR_GETDPC (R[0]));               /* reset PC */
    fill = STR_GETCHR (R[0]);                           /* get fill */
    R[2] = R[2] & STR_LNMASK;                           /* mask lengths */
    if (R[4] > 0)
        R[4] = R[4] & STR_LNMASK;
    }
else {
    R[1] = opnd[1];                                     /* src addr */
    if (movc5) {                                        /* MOVC5? */
        R[2] = (opnd[0] < opnd[3])? opnd[0]: opnd[3];
        R[3] = opnd[4];                                 /* dst addr */
        R[4] = opnd[3] - opnd[0];                       /* dstlen - srclen */
        fill = opnd[2];                                 /* set fill */
        CC_CMP_W (opnd[0], opnd[3]);                    /* set cc's */
        }
    else {
        R[2] = opnd[0];                                 /* mvlen = srclen */
        R[3] = opnd[2];                                 /* dst addr */
        R[4] = fill = 0;                                /* no fill */
        cc = CC_Z;                                      /* set cc's */
        }
    R[0] = STR_PACK (fill, R[2]);                       /* initial mvlen */
    if (R[2]) {                                         /* any move? */
        if (((uint32) R[1]) < ((uint32) R[3])) {
            R[1] = R[1] + R[2];                         /* backward, adjust */
            R[3] = R[3] + R[2];                         /* addr to end */
            R[5] = MVC_BACK;                            /* set state */
            }
        else R[5] = MVC_FRWD;                           /* fwd, set state */
        }
    else R[5] = MVC_FILL;                               /* fill, set state */
    R[5] = R[5] | (cc << MVC_V_CC);                     /* pack with state */
    PSL = PSL | PSL_FPD;                                /* set FPD */
    }

/* At this point,

        R0      =       delta PC'fill'initial move length
        R1      =       current src addr
        R2      =       current move length
        R3      =       current dst addr
        R4      =       dst length - src length
        R5      =       cc'state
*/

switch (R[5] & MVC_M_STATE) {                           /* case on state */

    case MVC_FRWD:                                      /* move forward */
        mlnt[0] = (4 - R[3]) & 3;                       /* length to align */
        if (mlnt[0] > R[2])                             /* cant exceed total */
            mlnt[0] = R[2];
        mlnt[1] = (R[2] - mlnt[0]) & ~03;               /* aligned length */
        mlnt[2] = R[2] - mlnt[0] - mlnt[1];             /* tail */
        for (i = 0; i < 3; i++) {                       /* head, align, tail */
            lnt = looplnt[i];                           /* length for loop */
            for (j = 0; j < mlnt[i]; j = j + lnt, cpu_cycle()) {
                wd = Read (RUN_PASS, R[1], lnt, RA);    /* read src */
                Write (RUN_PASS, R[3], wd, lnt, WA);    /* write dst */
                R[1] = R[1] + lnt;                      /* inc src addr */
                R[3] = R[3] + lnt;                      /* inc dst addr */
                R[2] = R[2] - lnt;                      /* dec move lnt */
                }
            }
        goto FILL;                                      /* check for fill */

    case MVC_BACK:                                      /* move backward */
        mlnt[0] = R[3] & 03;                            /* length to align */
        if (mlnt[0] > R[2])                             /* cant exceed total */
            mlnt[0] = R[2];
        mlnt[1] = (R[2] - mlnt[0]) & ~03;               /* aligned length */
        mlnt[2] = R[2] - mlnt[0] - mlnt[1];             /* tail */
        for (i = 0; i < 3; i++) {                       /* head, align, tail */
            lnt = looplnt[i];                           /* length for loop */
            for (j = 0; j < mlnt[i]; j = j + lnt, cpu_cycle()) {
                wd = Read (RUN_PASS, R[1] - lnt, lnt, RA);        /* read src */
                Write (RUN_PASS, R[3] - lnt, wd, lnt, WA);        /* write dst */
                R[1] = R[1] - lnt;                      /* dec src addr */
                R[3] = R[3] - lnt;                      /* dec dst addr */
                R[2] = R[2] - lnt;                      /* dec move lnt */
                }
            }
        R[1] = R[1] + (R[0] & STR_LNMASK);              /* final src addr */
        R[3] = R[3] + (R[0] & STR_LNMASK);              /* final dst addr */

    case MVC_FILL:                                      /* fill */
    FILL:
        if (R[4] <= 0)                                  /* any fill? */
            break;
        R[5] = R[5] | MVC_FILL;                         /* set state */
        mlnt[0] = (4 - R[3]) & 3;                       /* length to align */
        if (mlnt[0] > R[4])                             /* cant exceed total */
            mlnt[0] = R[4];
        mlnt[1] = (R[4] - mlnt[0]) & ~03;               /* aligned length */
        mlnt[2] = R[4] - mlnt[0] - mlnt[1];             /* tail */
        for (i = 0; i < 3; i++) {                       /* head, align, tail */
            lnt = looplnt[i];                           /* length for loop */
            fill = fill & BMASK;                        /* fill for loop */
            if (lnt == L_LONG)
                fill = (((uint32) fill) << 24) | (fill << 16) | (fill << 8) | fill;
            for (j = 0; j < mlnt[i]; j = j + lnt, cpu_cycle()) {
                Write (RUN_PASS, R[3], fill, lnt, WA);            /* write fill */
                R[3] = R[3] + lnt;                      /* inc dst addr */
                R[4] = R[4] - lnt;                      /* dec fill lnt */
                }
            }
        break;

    default:                                            /* bad state */
        RSVD_OPND_FAULT;                                /* you lose */
        }

PSL = PSL & ~PSL_FPD;                                   /* clear FPD */
cc = (R[5] >> MVC_V_CC) & CC_MASK;                      /* get cc's */
R[0] = NEG (R[4]);                                      /* set R0 */
R[2] = R[4] = R[5] = 0;                                 /* clear reg */
return cc;
}

/* CMPC3, CMPC5

   if PSL<fpd> = 0 and CMPC3,
        opnd[0] =       length
        opnd[1] =       source1 address
        opnd[2] =       source2 address

   if PSL<fpd> = 0 and CMPC5,
        opnd[0] =       source1 length
        opnd[1] =       source1 address
        opnd[2] =       fill
        opnd[3] =       source2 length
        opnd[4] =       source2 address

   if PSL<fpd> = 1,
        R0      =       delta-PC/fill/source1 length
        R1      =       source1 address
        R2      =       source2 length
        R3      =       source2 address
*/

int32 op_cmpc (RUN_DECL, int32 *opnd, int32 cmpc5, int32 acc)
{
int32 cc, s1, s2, fill;

if (PSL & PSL_FPD) {                                    /* FPD set? */
    SETPC (fault_PC + STR_GETDPC (R[0]));               /* reset PC */
    fill = STR_GETCHR (R[0]);                           /* get fill */
    }
else {
    R[1] = opnd[1];                                     /* src1len */
    if (cmpc5) {                                        /* CMPC5? */
        R[2] = opnd[3];                                 /* get src2 opnds */
        R[3] = opnd[4];
        fill = opnd[2];
        }
    else {
        R[2] = opnd[0];                                 /* src2len = src1len */
        R[3] = opnd[2];
        fill = 0;
        }
    R[0] = STR_PACK (fill, opnd[0]);                    /* src1len + FPD data */
    PSL = PSL | PSL_FPD;
    }
R[2] = R[2] & STR_LNMASK;                               /* mask src2len */
for (s1 = s2 = 0; ((R[0] | R[2]) & STR_LNMASK) != 0;  cpu_cycle()) {
    if (R[0] & STR_LNMASK)                              /* src1? read */
        s1 = Read (RUN_PASS, R[1], L_BYTE, RA);
    else s1 = fill;                                     /* no, use fill */
    if (R[2])                                           /* src2? read */
        s2 = Read (RUN_PASS, R[3], L_BYTE, RA);
    else s2 = fill;                                     /* no, use fill */
    if (s1 != s2)                                       /* src1 = src2? */
        break;
    if (R[0] & STR_LNMASK) {                            /* if src1, decr */
        R[0] = (R[0] & ~STR_LNMASK) | ((R[0] - 1) & STR_LNMASK);
        R[1] = R[1] + 1;
        }
    if (R[2]) {                                         /* if src2, decr */
        R[2] = (R[2] - 1) & STR_LNMASK;
        R[3] = R[3] + 1;
        }
    }
PSL = PSL & ~PSL_FPD;                                   /* clear FPD */
CC_CMP_B (s1, s2);                                      /* set cc's */
R[0] = R[0] & STR_LNMASK;                               /* clear packup */
return cc;
}

/* LOCC, SKPC

   if PSL<fpd> = 0,
        opnd[0] =       match character
        opnd[1] =       source length
        opnd[2] =       source address

   if PSL<fpd> = 1,
        R0      =       delta-PC/match/source length
        R1      =       source address
*/

int32 op_locskp (RUN_DECL, int32 *opnd, int32 skpc, int32 acc)
{
int32 c, match;

if (PSL & PSL_FPD) {                                    /* FPD set? */
    SETPC (fault_PC + STR_GETDPC (R[0]));               /* reset PC */
    match = STR_GETCHR (R[0]);                          /* get match char */
    }
else {
    match = opnd[0];                                    /* get operands */
    R[0] = STR_PACK (match, opnd[1]);                   /* src len + FPD data */
    R[1] = opnd[2];                                     /* src addr */
    PSL = PSL | PSL_FPD;
    }
for ( ; (R[0] & STR_LNMASK) != 0;  cpu_cycle() ) {      /* loop thru string */
    c = Read (RUN_PASS, R[1], L_BYTE, RA);              /* get src byte */
    if ((c == match) ^ skpc)                            /* match & locc? */
        break;
    R[0] = (R[0] & ~STR_LNMASK) | ((R[0] - 1) & STR_LNMASK);
    R[1] = R[1] + 1;                                    /* incr src1adr */
    }
PSL = PSL & ~PSL_FPD;                                   /* clear FPD */
R[0] = R[0] & STR_LNMASK;                               /* clear packup */
return (R[0]? 0: CC_Z);                                 /* set cc's */
}

/* SCANC, SPANC

   if PSL<fpd> = 0,
        opnd[0] =       source length
        opnd[1] =       source address
        opnd[2] =       table address
        opnd[3] =       mask

   if PSL<fpd> = 1,
        R0      =       delta-PC/char/source length
        R1      =       source address
        R3      =       table address
*/

int32 op_scnspn (RUN_DECL, int32 *opnd, int32 spanc, int32 acc)
{
int32 c, t, mask;

if (PSL & PSL_FPD) {                                    /* FPD set? */
    SETPC (fault_PC + STR_GETDPC (R[0]));               /* reset PC */
    mask = STR_GETCHR (R[0]);                           /* get mask */
    }
else {
    R[1] = opnd[1];                                     /* src addr */
    R[3] = opnd[2];                                     /* tblad */
    mask = opnd[3];                                     /* mask */
    R[0] = STR_PACK (mask, opnd[0]);                    /* srclen + FPD data */
    PSL = PSL | PSL_FPD;
    }
for ( ; (R[0] & STR_LNMASK) != 0;  cpu_cycle() ) {      /* loop thru string */
    c = Read (RUN_PASS, R[1], L_BYTE, RA);              /* get byte */
    t = Read (RUN_PASS, R[3] + c, L_BYTE, RA);          /* get table ent */
    if (((t & mask) != 0) ^ spanc)                      /* test vs instr */
        break;
    R[0] = (R[0] & ~STR_LNMASK) | ((R[0] - 1) & STR_LNMASK);
    R[1] = R[1] + 1;
    }
PSL = PSL & ~PSL_FPD;
R[0] = R[0] & STR_LNMASK;                               /* clear packup */
R[2] = 0;
return (R[0]? 0: CC_Z);
}

/* Operating system interfaces */

/* Interrupt or exception

        vec     =       SCB vector
        cc      =       condition codes
        ipl     =       new IPL if interrupt
        ei      =       -1: severe exception
                        0:  normal exception
                        1:  interrupt
*/

t_stat cpu_set_hist (UNIT *uptr, int32 val, char *cptr, void *desc);
t_stat cpu_show_hist (SMP_FILE *st, UNIT *uptr, int32 val, void *desc);

int32 intexc (RUN_DECL, int32 vec, int32 cc, int32 ipl, int ei)
{
    int32 oldpsl = PSL | cc;
    int32 oldcur = PSL_GETCUR (oldpsl);
    int32 oldsp = SP;
    int32 newpsl;
    int32 newpc;
    int32 acc;

#if 0
    if (FALSE && oldcur == 0 && vec != SCB_MCHK && ei != IE_INT && mapen != 0)
    {
        SMP_FILE* fd = smp_fopen("kmode-exc.log", "a");
        fprintf(fd, "\n");
        fprintf(fd, "***\n");
        fprintf(fd, "*** Kernel mode exception at 0x%08X\n", PC);
        fprintf(fd, "***\n");
        fprintf(fd, "\n");
        cpu_show_hist(fd, cpu_unit, 0, NULL);
        cpu_set_hist(cpu_unit, 0, NULL, NULL);
        fclose(fd);
    }
#endif

    in_ie = 1;                                                /* flag int/exc */
    CLR_TRAPS;                                                /* clear traps */

    newpc = ReadLP (RUN_PASS, (SCBB + vec) & (PAMASK & ~3));  /* read new PC */
    if (newpc & 2)                                            /* bad flags? */
        ABORT (STOP_ILLVEC);
    if (ei == IE_SVE)                                         /* severe? on istk */
        newpc = newpc | 1;

    if (oldpsl & PSL_IS)                                      /* on int stk? */
    {
        newpsl = PSL_IS;
    }
    else
    {
        STK[oldcur] = SP;                                     /* no, save cur stk */
        if (newpc & 1)                                        /* to int stk? */
        {
            newpsl = PSL_IS;                                  /* flag */
            SP = IS;                                          /* new stack */
        }
        else
        {
            newpsl = 0;                                       /* to ker stk */
            SP = KSP;                                         /* new stack */
        }
    }

    if (ei == IE_INT)                                         /* if int, new IPL */
    {
        if (VAX_QBUS && vec >= VEC_Q)
            newpsl |= PSL_IPL17;
        else
            newpsl |= ipl << PSL_V_IPL;
    }
    else                                                      /* exception or severe exception */
    {
        if (newpc & 1)
        {
            newpsl |= PSL_IPL1F;
        }
        else
        {
            newpsl |= oldpsl & PSL_IPL;
        }
        newpsl |= oldcur << PSL_V_PRV;
    }

    PSL = newpsl;

    if (DEBUG_PRI (cpu_dev, LOG_CPU_I))
        fprintf (sim_deb, ">>IEX: PC=%08x, PSL=%08x, SP=%08x, VEC=%08x, nPSL=%08x, nSP=%08x\n",
                 PC, oldpsl, oldsp, vec, PSL, SP);

    /*
     * O/S virtualization assistance.
     * Raises thread priority level if required and performs synchronization window management.
     * For IE_INT thread priority re-evaluation had already been performed in a preeceding SET_IRQL
     * and will be performed again in SET_IRQL when intexc returns.
     */
    if (ei == IE_INT)
        cpu_on_changed_ipl(RUN_PASS, oldpsl, CHIPL_NO_THRDPRIO);
    else
        cpu_on_changed_ipl(RUN_PASS, oldpsl, 0);

    acc = ACC_MASK (KERN);                                    /* new mode is kernel */
    Write (RUN_PASS, SP - 4, oldpsl, L_LONG, WA);             /* push old PSL */
    Write (RUN_PASS, SP - 8, PC, L_LONG, WA);                 /* push old PC */
    SP = SP - 8;                                              /* update stk ptr */
    JUMP (newpc & ~3);                                        /* change PC */
    in_ie = 0;                                                /* out of flows */
    return 0;
}

/* CHMK, CHME, CHMS, CHMU

        opnd[0] =       operand
*/

int32 op_chm (RUN_DECL, int32 *opnd, int32 cc, int32 opc)
{
int32 mode = opc & PSL_M_MODE;
int32 cur = PSL_GETCUR (PSL);
int32 tsp, newpc, acc, sta;

if (PSL & PSL_IS)
    ABORT (STOP_CHMFI);
newpc = ReadLP (RUN_PASS, (SCBB + SCB_CHMK + (mode << 2)) & PAMASK);
if (cur < mode)                                         /* only inward */
    mode = cur;
STK[cur] = SP;                                          /* save stack */
tsp = STK[mode];                                        /* get new stk */
acc = ACC_MASK (mode);                                  /* set new mode */
if (Test (RUN_PASS, fault_p2 = tsp - 1, WA, &sta) < 0) {      /* probe stk */
    fault_p1 = MM_WRITE | (sta & MM_EMASK);
    ABORT ((sta & 4)? ABORT_TNV: ABORT_ACV);
    }
if (Test (RUN_PASS, fault_p2 = tsp - 12, WA, &sta) < 0) {
    fault_p1 = MM_WRITE | (sta & MM_EMASK);
    ABORT ((sta & 4)? ABORT_TNV: ABORT_ACV);
    }
Write (RUN_PASS, tsp - 12, SXTW (opnd[0]), L_LONG, WA); /* push argument */
Write (RUN_PASS, tsp - 8, PC, L_LONG, WA);              /* push PC */
Write (RUN_PASS, tsp - 4, PSL | cc, L_LONG, WA);        /* push PSL */
SP = tsp - 12;                                          /* set new stk */
PSL = (mode << PSL_V_CUR) | (PSL & PSL_IPL) |           /* set new PSL */
    (cur << PSL_V_PRV);                                 /* IPL unchanged */
// last_chm = fault_PC;
JUMP (newpc & ~03);                                     /* set new PC */
return 0;                                               /* cc = 0 */
}

/* REI - return from exception or interrupt

The lengthiest part of the REI instruction is the validity checking of the PSL
popped off the stack.  The new PSL is checked against the following eight rules:

let     tmp     =       new PSL popped off the stack
let     PSL     =       current PSL

Rule    SRM formulation                     Comment
----    ---------------                     -------
 1      tmp<25:24> GEQ PSL<25:24>           tmp<cur_mode> GEQ PSL<cur_mode>
 2      tmp<26> LEQ PSL<26>                 tmp<is> LEQ PSL<is>
 3      tmp<26> = 1 => tmp<25:24> = 0       tmp<is> = 1 => tmp<cur_mode> = ker
 4      tmp<26> = 1 => tmp<20:16> > 0       tmp<is> = 1 => tmp<ipl> > 0
 5      tmp<20:16> > 0 => tmp<25:24> = 0    tmp<ipl> > 0 => tmp<cur_mode> = ker
 6      tmp<25:24> LEQ tmp<23:22>           tmp<cur_mode> LEQ tmp<prv_mode>
 7      tmp<20:16> LEQ PSL<20:16>           tmp<ipl> LEQ PSL<ipl>
 8      tmp<31,29:28,21,15:8> = 0           tmp<mbz> = 0
 9      tmp<31> = 1 => tmp<cur_mode> = 3, tmp<prv_mode> = 3>, tmp<fpd,is,ipl> = 0 
*/

int32 op_rei (RUN_DECL, int32 acc)
{
    int32 oldpsl = PSL;
    int32 oldpc = PC;
    int32 newpc = Read (RUN_PASS, SP, L_LONG, RA);
    int32 newpsl = Read (RUN_PASS, SP + 4, L_LONG, RA);
    int32 newcur = PSL_GETCUR (newpsl);
    int32 oldcur = PSL_GETCUR (PSL);
    int32 newipl, i;

    if ((newpsl & PSL_MBZ) ||                               /* rule 8 */
        (newcur < oldcur))                                  /* rule 1 */
        RSVD_OPND_FAULT;

    newipl = PSL_GETIPL (newpsl);                           /* get new ipl */

    if (newcur)                                             /* to esu, skip 2,4,7 */
    {
        if ((newpsl & (PSL_IS | PSL_IPL)) ||                /* rules 3,5 */
            (newcur > PSL_GETPRV (newpsl)))                 /* rule 6 */
            RSVD_OPND_FAULT;                                /* end rei to esu */
    }
    else                                                    /* to k, skip 3,5,6 */
    {
        if ((newpsl & PSL_IS) &&                            /* setting IS? */
           (((PSL & PSL_IS) == 0) || (newipl == 0)))        /* test rules 2,4 */
            RSVD_OPND_FAULT;                                /* else skip 2,4 */
        if (newipl > PSL_GETIPL (PSL))                      /* test rule 7 */
            RSVD_OPND_FAULT;
    }                                                       /* end if kernel */

    if (newpsl & PSL_CM)                                    /* setting cmode? */
    {
        if (BadCmPSL (RUN_PASS, newpsl))                    /* validate PSL */
            RSVD_OPND_FAULT;
        for (i = 0; i < 7; i++)                             /* mask R0-R6, PC */
            R[i] = R[i] & WMASK;
        newpc = newpc & WMASK;
    }

    SP = SP + 8;                                            /* pop stack */
    if (PSL & PSL_IS)                                       /* save stack */
        IS = SP;
    else
        STK[oldcur] = SP;

    if (DEBUG_PRI (cpu_dev, LOG_CPU_R))
    {
        fprintf (sim_deb, ">>REI: PC=%08x, PSL=%08x, SP=%08x, nPC=%08x, nPSL=%08x, nSP=%08x\n",
                 PC, PSL, SP - 8, newpc, newpsl, ((newpsl & IS)? IS: STK[newcur]));
    }

    PSL = (PSL & PSL_TP) | (newpsl & ~CC_MASK);             /* set PSL */

    if (PSL & PSL_IS)                                       /* set new stack */
    {
        SP = IS;
    }
    else
    {
        SP = STK[newcur];                                   /* if ~IS, chk AST */
        if (newcur >= ASTLVL)
        {
            if (DEBUG_PRI (cpu_dev, LOG_CPU_R))
                fprintf (sim_deb, ">>REI: AST delivered\n");
            SISR = SISR | SISR_2;
        }
    }
    JUMP (newpc);                                           /* set new PC */

    /* 
     * When dropping IPL below CLK/IPI level, reset state flags indicating we are in CLK/IPI ISR.
     * If CLK or IPI interrupts are pending in cpu_intreg, SET_IRQL below will reinstate the flag(s) to TRUE.
     * SET_IRQL will also perform thread prority adjustment according to new state, accounting both for 
     * leaving CLK/IPI ISR and for any pending CLK/IPI IRQs.
     */
    if (newipl < IPL_ABS_CLK)
        cpu_unit->cpu_active_clk_interrupt = FALSE;
    if (newipl < IPL_ABS_IPINTR)
        cpu_unit->cpu_active_ipi_interrupt = FALSE;

    /*
     * Check if this is console ROM's CONTNUE REI
     */
    if (unlikely(oldpc == ROM_PC_CONTINUE_REI) && cpu_unit->cpu_con_rei_on && 
        mapen == 1 && CPU_CURRENT_CYCLES == cpu_unit->cpu_con_rei)
    {
        /* returning from console mode to OS: reenter synchronization window if required */
        syncw_enable_cpu(RUN_PASS);
        syncw_reeval_sys(RUN_PASS);

        /* thread priorty reevaluation will be performed by SET_IRQL below */
        cpu_on_changed_ipl(RUN_PASS, oldpsl, CHIPL_NO_THRDPRIO | CHIPL_NO_SYNCW);
    }
    else
    {
        /* thread priorty reevaluation will be performed by SET_IRQL below */
        cpu_on_changed_ipl(RUN_PASS, oldpsl, CHIPL_NO_THRDPRIO);
    }
    cpu_unit->cpu_con_rei_on = FALSE;

    SET_IRQL;                                               /* update intreq */

    return newpsl & CC_MASK;                                /* set new cc */
}

/* LDCPTX - load process context */

void op_ldpctx (RUN_DECL, int32 acc)
{
    int32 newpc, newpsl, pcbpa, t;

    if (PSL & PSL_CUR)                                      /* must be kernel */
        RSVD_INST_FAULT;

    syncw_leave_ilk(RUN_PASS);                              /* leave ILK synchronization window */

    pcbpa = PCBB & PAMASK;                                  /* phys address */

    KSP = ReadLP (RUN_PASS, pcbpa);                         /* restore stk ptrs */
    ESP = ReadLP (RUN_PASS, pcbpa + 4);
    SSP = ReadLP (RUN_PASS, pcbpa + 8);
    USP = ReadLP (RUN_PASS, pcbpa + 12);
    R[0] = ReadLP (RUN_PASS, pcbpa + 16);                   /* restore registers */
    R[1] = ReadLP (RUN_PASS, pcbpa + 20);
    R[2] = ReadLP (RUN_PASS, pcbpa + 24);
    R[3] = ReadLP (RUN_PASS, pcbpa + 28);
    R[4] = ReadLP (RUN_PASS, pcbpa + 32);
    R[5] = ReadLP (RUN_PASS, pcbpa + 36);
    R[6] = ReadLP (RUN_PASS, pcbpa + 40);
    R[7] = ReadLP (RUN_PASS, pcbpa + 44);
    R[8] = ReadLP (RUN_PASS, pcbpa + 48);
    R[9] = ReadLP (RUN_PASS, pcbpa + 52);
    R[10] = ReadLP (RUN_PASS, pcbpa + 56);
    R[11] = ReadLP (RUN_PASS, pcbpa + 60);
    R[12] = ReadLP (RUN_PASS, pcbpa + 64);
    R[13] = ReadLP (RUN_PASS, pcbpa + 68);
    newpc = ReadLP (RUN_PASS, pcbpa + 72);                            /* get PC, PSL */
    newpsl = ReadLP (RUN_PASS, pcbpa + 76);

    t = ReadLP (RUN_PASS, pcbpa + 80);
    ML_PXBR_TEST (t);                                       /* validate P0BR */
    P0BR = t & BR_MASK;                                     /* restore P0BR */
    t = ReadLP (RUN_PASS, pcbpa + 84);
    LP_MBZ84_TEST (t);                                      /* test mbz */
    ML_LR_TEST (t & LR_MASK);                               /* validate P0LR */
    P0LR = t & LR_MASK;                                     /* restore P0LR */
    t = (t >> 24) & AST_MASK;
    LP_AST_TEST (t);                                        /* validate AST */
    ASTLVL = t;                                             /* restore AST */
    t = ReadLP (RUN_PASS, pcbpa + 88);
    ML_PXBR_TEST (t + 0x800000);                            /* validate P1BR */
    P1BR = t & BR_MASK;                                     /* restore P1BR */
    t = ReadLP (RUN_PASS, pcbpa + 92);
    LP_MBZ92_TEST (t);                                      /* test MBZ */
    ML_LR_TEST (t & LR_MASK);                               /* validate P1LR */
    P1LR = t & LR_MASK;                                     /* restore P1LR */
    pme = (t >> 31) & 1;                                    /* restore PME */

    zap_tb (RUN_PASS, 0);                                   /* clear process TB */
    set_map_reg (RUN_PASS);
    if (DEBUG_PRI (cpu_dev, LOG_CPU_P))
        fprintf (sim_deb, ">>LDP: PC=%08x, PSL=%08x, SP=%08x, nPC=%08x, nPSL=%08x, nSP=%08x\n",
                 PC, PSL, SP, newpc, newpsl, KSP);
    if (PSL & PSL_IS)                                       /* if istk, */
        IS = SP;
    PSL = PSL & ~PSL_IS;                                    /* switch to kstk */
    SP = KSP - 8;
    Write (RUN_PASS, SP, newpc, L_LONG, WA);                /* push PC, PSL */
    Write (RUN_PASS, SP + 4, newpsl, L_LONG, WA);
}

/* SVPCTX - save processor context */

void op_svpctx (RUN_DECL, int32 acc)
{
    int32 savpc, savpsl, pcbpa;

    if (PSL & PSL_CUR)                                      /* must be kernel */
        RSVD_INST_FAULT;

    syncw_leave_ilk(RUN_PASS);                              /* leave ILK synchronization window */

    savpc = Read (RUN_PASS, SP, L_LONG, RA);                /* pop PC, PSL */
    savpsl = Read (RUN_PASS, SP + 4, L_LONG, RA);

    if (DEBUG_PRI (cpu_dev, LOG_CPU_P))
        fprintf (sim_deb, ">>SVP: PC=%08x, PSL=%08x, SP=%08x, oPC=%08x, oPSL=%08x\n",
                 PC, PSL, SP, savpc, savpsl);

    if (PSL & PSL_IS)                                       /* int stack? */
        SP = SP + 8;
    else
    {
        KSP = SP + 8;                                       /* pop kernel stack */
        SP = IS;                                            /* switch to int stk */
        if ((PSL & PSL_IPL) == 0)                           /* make IPL > 0 */
             PSL = PSL | PSL_IPL1;
        PSL = PSL | PSL_IS;                                 /* set PSL<is> */
    }

    pcbpa = PCBB & PAMASK;
    WriteLP (RUN_PASS, pcbpa, KSP);                         /* save stk ptrs */
    WriteLP (RUN_PASS, pcbpa + 4, ESP);
    WriteLP (RUN_PASS, pcbpa + 8, SSP);
    WriteLP (RUN_PASS, pcbpa + 12, USP);
    WriteLP (RUN_PASS, pcbpa + 16, R[0]);                   /* save registers */
    WriteLP (RUN_PASS, pcbpa + 20, R[1]);
    WriteLP (RUN_PASS, pcbpa + 24, R[2]);
    WriteLP (RUN_PASS, pcbpa + 28, R[3]);
    WriteLP (RUN_PASS, pcbpa + 32, R[4]);
    WriteLP (RUN_PASS, pcbpa + 36, R[5]);
    WriteLP (RUN_PASS, pcbpa + 40, R[6]);
    WriteLP (RUN_PASS, pcbpa + 44, R[7]);
    WriteLP (RUN_PASS, pcbpa + 48, R[8]);
    WriteLP (RUN_PASS, pcbpa + 52, R[9]);
    WriteLP (RUN_PASS, pcbpa + 56, R[10]);
    WriteLP (RUN_PASS, pcbpa + 60, R[11]);
    WriteLP (RUN_PASS, pcbpa + 64, R[12]);
    WriteLP (RUN_PASS, pcbpa + 68, R[13]);
    WriteLP (RUN_PASS, pcbpa + 72, savpc);                            /* save PC, PSL */
    WriteLP (RUN_PASS, pcbpa + 76, savpsl);
}

/* PROBER and PROBEW

        opnd[0] =       mode
        opnd[1] =       length
        opnd[2] =       base address
*/

int32 op_probe (RUN_DECL, int32 *opnd, int32 rw)
{
int32 mode = opnd[0] & PSL_M_MODE;                      /* mask mode */
int32 length = opnd[1];
int32 ba = opnd[2];
int32 prv = PSL_GETPRV (PSL);
int32 acc, sta, sta1;

if (prv > mode)                                         /* maximize mode */
    mode = prv;
acc = ACC_MASK (mode) << (rw? TLB_V_WACC: 0);           /* set acc mask */
Test (RUN_PASS, ba, acc, &sta);                         /* probe */
switch (sta) {                                          /* case on status */

    case PR_PTNV:                                       /* pte TNV */
        fault_p1 = MM_PARAM (rw, PR_PTNV);
        fault_p2 = ba;
        ABORT (ABORT_TNV);                              /* force TNV */

    case PR_TNV: case PR_OK:                            /* TNV or ok */
        break;                                          /* continue */

    default:                                            /* other */
        return CC_Z;                                    /* lose */
        }

Test (RUN_PASS, ba + length - 1, acc, &sta1);           /* probe end addr */
switch (sta1) {                                         /* case on status */

    case PR_PTNV:                                       /* pte TNV */
        fault_p1 = MM_PARAM (rw, PR_PTNV);
        fault_p2 = ba + length - 1;
        ABORT (ABORT_TNV);                              /* force TNV */

    case PR_TNV: case PR_OK:                            /* TNV or ok */
        break;                                          /* win */

    default:                                            /* other */
        return CC_Z;                                    /* lose */
        }

return 0;
}

/* MTPR - move to processor register

        opnd[0] =       data
        opnd[1] =       register number
*/

int32 op_mtpr (RUN_DECL, int32 *opnd)
{
    int32 val = opnd[0];
    int32 prn = opnd[1];
    int32 cc;
    t_bool set_irql = TRUE;
    t_bool keep_prefetch = FALSE;

    if (prn == MT_SIMH)
    {
        /*
         * SIMH API call by guest.
         * This register can be written in user mode for QUERY API only.
         * Other SIMH API calls will require kernel mode.
         */
        op_mtpr_simh(RUN_PASS, val);
        CC_IIZZ_L (val);                                /* set cc's */
        SET_IRQL;                                       /* update intreq */
        return cc;
    }

    if (PSL & PSL_CUR)                                  /* must be kernel */
        RSVD_INST_FAULT;

    if (prn > 63)                                       /* reg# > 63? fault */
        RSVD_OPND_FAULT;

    CC_IIZZ_L (val);                                    /* set cc's */

    switch (prn)                                        /* case on reg # */
    {
    case MT_KSP:                                        /* KSP */
        if (PSL & PSL_IS)                               /* on IS? store KSP */
            KSP = val;
        else
            SP = val;                                   /* else store SP */
        set_irql = FALSE;
        break;

    case MT_ESP: case MT_SSP: case MT_USP:              /* ESP, SSP, USP */
        STK[prn] = val;                                 /* store stack */
        set_irql = FALSE;
        break;

    case MT_IS:                                         /* IS */
        if (PSL & PSL_IS)                               /* on IS? store SP */
            SP = val;
        else
            IS = val;                                  /* else store IS */
        set_irql = FALSE;
        break;

    case MT_P0BR:                                       /* P0BR */
        ML_PXBR_TEST (val);                             /* validate */
        P0BR = val & BR_MASK;                           /* lw aligned */
        zap_tb (RUN_PASS, 0);                           /* clr proc TLB */
        set_map_reg (RUN_PASS);
        break;

    case MT_P0LR:                                       /* P0LR */
        ML_LR_TEST (val & LR_MASK);                     /* validate */
        P0LR = val & LR_MASK;
        zap_tb (RUN_PASS, 0);                           /* clr proc TLB */
        set_map_reg (RUN_PASS);
        break;

    case MT_P1BR:                                       /* P1BR */
        ML_PXBR_TEST (val + 0x800000);                  /* validate */
        P1BR = val & BR_MASK;                           /* lw aligned */
        zap_tb (RUN_PASS, 0);                           /* clr proc TLB */
        set_map_reg (RUN_PASS);
        break;

    case MT_P1LR:                                       /* P1LR */
        ML_LR_TEST (val & LR_MASK);                     /* validate */
        P1LR = val & LR_MASK;
        zap_tb (RUN_PASS, 0);                           /* clr proc TLB */
        set_map_reg (RUN_PASS);
        break;

    case MT_SBR:                                        /* SBR */
        ML_SBR_TEST (val);                              /* validate */
        SBR = val & BR_MASK;                            /* lw aligned */
        zap_tb (RUN_PASS, 1);                           /* clr entire TLB */
        set_map_reg (RUN_PASS);
        break;

    case MT_SLR:                                        /* SLR */
        ML_LR_TEST (val & LR_MASK);                     /* validate */
        SLR = val & LR_MASK;
        zap_tb (RUN_PASS, 1);                           /* clr entire TLB */
        set_map_reg (RUN_PASS);
        break;

    case MT_WHAMI:                                      /* WHAMI */
        WHAMI = val;
        set_irql = FALSE;
        break;

    case MT_SCBB:                                       /* SCBB */
        ML_PA_TEST (val);                               /* validate */
        SCBB = val & BR_MASK;                           /* lw aligned */
        /* set auxiliary variable for fast checks PA_MAY_BE_INSIDE_SCB() */
        cpu_unit->cpu_context.scb_range_pamask = (uint32) SCBB & SCB_RANGE_PAMASK;
        break;

    case MT_PCBB:                                       /* PCBB */
        ML_PA_TEST (val);                               /* validate */
        PCBB = val & BR_MASK;                           /* lw aligned */
        set_irql = FALSE;
        break;

    case MT_IPL:                                        /* IPL */
        {
            int32 newipl = val & PSL_M_IPL;

            if (newipl != PSL_GETIPL(PSL))
            {
                int32 oldpsl = PSL;
                PSL = (PSL & ~PSL_IPL) | (newipl << PSL_V_IPL);

                /* 
                 * When dropping IPL below CLK/IPI level, reset state flags indicating we are in CLK/IPI ISR.
                 * If CLK or IPI interrupts are pending in cpu_intreg, SET_IRQL below will reinstate the flag(s) to TRUE.
                 * SET_IRQL will also perform thread prority adjustment according to new state, accounting both for 
                 * leaving CLK/IPI ISR and for any pending CLK/IPI IRQs.
                 */
                if (newipl < IPL_ABS_CLK)
                    cpu_unit->cpu_active_clk_interrupt = FALSE;
                if (newipl < IPL_ABS_IPINTR)
                    cpu_unit->cpu_active_ipi_interrupt = FALSE;

                /* thread priorty reevaluation will be performed by SET_IRQL below */
                cpu_on_changed_ipl(RUN_PASS, oldpsl, CHIPL_NO_THRDPRIO);
            }
            else
            {
                /* IPL had not changed: no-op */
                set_irql = FALSE;
            }
        }
        break;

    case MT_ASTLVL:                                     /* ASTLVL */
        if (val > AST_MAX)                              /* > 4? fault */
            RSVD_OPND_FAULT;
        ASTLVL = val;
        break;

    case MT_SIRR:                                       /* SIRR */
        if (val > 0xF || val == 0)
            RSVD_OPND_FAULT;
        SISR = SISR | (1 << val);                       /* set bit in SISR */
        break;

    case MT_SISR:                                       /* SISR */
        SISR = val & SISR_MASK;
        break;

    case MT_MAPEN:                                      /* MAPEN */
        {
            int32 old_mapen = mapen;
            mapen = val & 1;
            if (mapen == 0)
            {
                cpu_on_clear_mapen(RUN_PASS);
            }
            else
            {
                if (old_mapen == 0 && PC == ROM_PC_CONTINUE_MAPEN)
                {
                    /* console ROM executing the CONTINUE command */
                    cpu_on_rom_continue(RUN_PASS);
                    cpu_unit->cpu_con_rei = CPU_CURRENT_CYCLES + 1;
                    cpu_unit->cpu_con_rei_on = TRUE;
                }

                cpu_reevaluate_thread_priority(RUN_PASS);

                /*
                 * Console ROM code contains sequence
                 *
                 *     MTPR #1, #MT_MAPEN
                 *     REI
                 *
                 * REI is fetched thanks to physical prefetch since no virtual mapping for ROM code exists.
                 *
                 * Similar sequences relying on physical prefetch going on after enabling MAPEN
                 * rather than using double mapping may exist as well in some operating systems
                 * bootstrap code and in some standalone software.
                 *
                 * Therefore it is crucial that prefetch is not reset at this point.
                 */
                keep_prefetch = TRUE;
            }
        }
        /* fall through */
    case MT_TBIA:                                       /* TBIA */
        zap_tb (RUN_PASS, 1, keep_prefetch);            /* clr entire TLB */
        set_irql = FALSE;
        break;

    case MT_TBIS:                                       /* TBIS */
        zap_tb_ent (RUN_PASS, val);
        set_irql = FALSE;
        break;

    case MT_TBCHK:                                      /* TBCHK */
        if (chk_tb_ent (RUN_PASS, val))
            cc = cc | CC_V;
        set_irql = FALSE;
        break;

    case MT_PME:                                        /* PME */
        pme = val & 1;
        break;

    default:
        WriteIPR (RUN_PASS, prn, val, set_irql);        /* others */
        break;
    }

    if (set_irql)
    {
        SET_IRQL;                                       /* update intreq */
    }

    return cc;
}

int32 op_mfpr (RUN_DECL, int32 *opnd)
{
int32 prn = opnd[0];
int32 val;

if (PSL & PSL_CUR)                                      /* must be kernel */
    RSVD_INST_FAULT;
if (prn > 63)                                           /* reg# > 63? fault */
    RSVD_OPND_FAULT;
switch (prn) {                                          /* case on reg# */

    case MT_KSP:                                        /* KSP */
        val = (PSL & PSL_IS)? KSP: SP;                  /* return KSP or SP */
        break;

    case MT_ESP: case MT_SSP: case MT_USP:              /* ESP, SSP, USP */
        val = STK[prn];                                 /* return stk ptr */
        break;

    case MT_IS:                                         /* IS */
        val = (PSL & PSL_IS)? SP: IS;                   /* return SP or IS */
        break;

    case MT_P0BR:                                       /* P0BR */
        val = P0BR;
        break;

    case MT_P0LR:                                       /* P0LR */
        val = P0LR;
        break;

    case MT_P1BR:                                       /* P1BR */
        val = P1BR;
        break;

    case MT_P1LR:                                       /* P1LR */
        val = P1LR;
        break;

    case MT_SBR:                                        /* SBR */
        val = SBR;
        break;

    case MT_SLR:                                        /* SLR */
        val = SLR;
        break;

    case MT_CPUID:                                      /* CPUID */
        val = ((int32) cpu_unit->cpu_id) & 0xFF;
        break;

    case MT_WHAMI:                                      /* WHAMI */
        val = WHAMI;
        break;

    case MT_SCBB:                                       /* SCBB */
        val = SCBB;
        break;

    case MT_PCBB:                                       /* PCBB */
        val = PCBB;
        break;

    case MT_IPL:                                        /* IPL */
        val = PSL_GETIPL (PSL);
        break;

    case MT_ASTLVL:                                     /* ASTLVL */
        val = ASTLVL;
        break;

    case MT_SISR:                                       /* SISR */
        val = SISR & SISR_MASK;
        break;

    case MT_MAPEN:                                      /* MAPEN */
        val = mapen & 1;
        break;

    case MT_PME:
        val = pme & 1;
        break;

    case MT_SIRR:
    case MT_TBIA:
    case MT_TBIS:
    case MT_TBCHK:
        RSVD_OPND_FAULT;                                /* write only */

    default:                                            /* others */
        val = ReadIPR (RUN_PASS, prn);                  /* read from SSC */
        break;
        }

return val;
}

void cpu_on_changed_ipl(RUN_DECL, int32 oldpsl, int32 flags)
{
    int32 old_ipl = PSL_GETIPL(oldpsl);
    int32 new_ipl = PSL_GETIPL(PSL);

    if (new_ipl == old_ipl)
        return;

    /*
     * reevaluate VCPU thread priority if IPL crosses sys_critical_section_ipl level
     */
    if (unlikely(0 == (flags & CHIPL_NO_THRDPRIO)) && sys_critical_section_ipl > 0)
    {
        if (old_ipl < sys_critical_section_ipl && new_ipl >= sys_critical_section_ipl ||
            old_ipl >= sys_critical_section_ipl && new_ipl < sys_critical_section_ipl)
        {
            cpu_reevaluate_thread_priority(RUN_PASS);
        }
    }

    /*
     * manage synchronization window transitions
     */
    if (0 == (flags & CHIPL_NO_SYNCW))
    {
        if (old_ipl < syncw.ipl_syslock && new_ipl >= syncw.ipl_syslock)
        {
            syncw_enter_sys(RUN_PASS);
        }
        else if (old_ipl >= syncw.ipl_syslock && new_ipl < syncw.ipl_syslock)
        {
            if (new_ipl < syncw.ipl_resched)
                syncw_leave_all(RUN_PASS, 0);
            else
                syncw_leave_sys(RUN_PASS);
        }
        else if (old_ipl >= syncw.ipl_resched && new_ipl < syncw.ipl_resched)
        {
            syncw_leave_ilk(RUN_PASS);
        }
    }
}

/*
 * Reevaluate and set proper VCPU thread priority based on VCPU state.
 *
 * Note that there is currently a race condition between cpu_reevaluate_thread_priority and interrupt_set_int,
 * in a sequence like this. Suppose external source is sending an interrupt to VCPUn. The source
 * can be a SYNCLK thread, interprocessor interrupt being sent by another procesor, or (for VCPU0) device handler
 * executed in the context of another processor:
 *
 *                  VCPUn                                      External Source
 *      (cpu_reevaluate_thread_priority)                     (interrupt_set_int)
 *
 *     check conditions, incl. interrupts
 *     found no interrupt
 *     calculate new thread priority = low
 *
 *                                                    send an interrupt
 *                                                    set VCPU0 thread priority to CRITICAL_VM
 *
 *     drop thread priority to calculated
 *     low level
 *
 * Thus VCPUn priority boosting (for prompt interrupt processing) by external source will be lost
 * and it is possible for VCPUn thread to stay at low priority and be preempted despite interrupts
 * pending.
 *
 * If would be easy to fix the issue by using a locking algorithm, but we exactly want to avoid this
 * and stick instead to lock-free and wait-free scheme for sending interrupts. It would be nice to
 * find lock-free/wait-free solution to this race condition later, if exists.
 *
 * For now, we just allow this race condition, because:
 *
 *     - It's probability is low (though surely not zero).
 *
 *     - If happens, VCPUn priority will be boosted again by the next interrupt sent to it.
 *       If SYNCLK thread is used (use_clock_thread is TRUE), this interrupt will be typically sent 
 *       within at most 1/100th of a second.
 *
 * Therefore we classify the issue as low priority. 
 * Things would change however if SYNCLK thread was not used.
 *
 */
void cpu_reevaluate_thread_priority(RUN_DECL, t_bool synced)
{
    RUN_SCOPE_RSCX_ONLY;
    sim_thread_priority_t newprio;
    t_bool is_active_clk_interrupt;
    t_bool is_active_ipi_interrupt;

    /*
     * Reevaluate only if called in the context of target VCPU's thread.
     *
     * Can also be called on the console thread (as a result of console commands)
     * or by another VCPU when that other VCPU is preparing to start this processor
     * in cpu_start_secondary. Do not perform reevaluation in these cases.
     * Instead, it will be performed when target VCPU's thread starts or resumes
     * execution of instruction stream.
     */
    if (unlikely(rscx->thread_type != SIM_THREAD_TYPE_CPU) || unlikely(rscx->thread_cpu_id != cpu_unit->cpu_id))
    {
        cpu_unit->cpu_thread_priority = SIMH_THREAD_PRIORITY_INVALID;
        return;
    }

    /* prune recursive calls */
    if (cpu_unit->cpu_inside_reevaluate_thread_priority)
    {
        cpu_unit->cpu_redo_reevaluate_thread_priority = TRUE;
        return;
    }

again:

    if (unlikely(mapen == 0) && cpu_unit->is_primary_cpu())
    {
        /* equivalent of is_os_running() */
        cpu_set_thread_priority(RUN_PASS, SIMH_THREAD_PRIORITY_CPU_RUN);
        return;
    }

    if (unlikely(tmr_is_active(RUN_PASS)))
    {
        /* check if SSC clocks are still active */
        cpu_set_thread_priority(RUN_PASS, SIMH_THREAD_PRIORITY_CPU_CALIBRATION);
        return;
    }

    if (unlikely(rscx->vm_critical_locks != 0))
    {
        /*
         * if code holds VM critical locks, run at elevated priority
         */
        cpu_set_thread_priority(RUN_PASS, SIMH_THREAD_PRIORITY_CPU_CRITICAL_VM);
        return;
    }

    if (unlikely(rscx->os_hi_critical_locks != 0))
    {
        /*
         * if code holds OS_HI critical locks, run at elevated priority
         */
        cpu_set_thread_priority(RUN_PASS, SIMH_THREAD_PRIORITY_CPU_CRITICAL_OS_HI);
        return;
    }

    if (unlikely(cpu_unit->cpu_active_clk_interrupt || cpu_unit->cpu_active_ipi_interrupt))
    {
        /*
         * if processing CLK or IPI interrupts, run at elevated priority
         */
        cpu_set_thread_priority(RUN_PASS, SIMH_THREAD_PRIORITY_CPU_CRITICAL_OS_HI);
        return;
    }

    /*
     * check if CLK or IPI interrupts are pending
     */
    if (! synced)
    {
        /* 
         * read_irqs_to_local can recursively call cpu_reevaluate_thread_priority.
         *
         * This recursion is pruned at the entrance to cpu_reevaluate_thread_priority 
         * and original invocation of cpu_reevaluate_thread_priority is re-processed.
         */
        cpu_unit->cpu_inside_reevaluate_thread_priority = TRUE;
        cpu_unit->cpu_redo_reevaluate_thread_priority = FALSE;

        smp_rmb();
        read_irqs_to_local(RUN_PASS);

        int32 hipl = 0;

        if (unlikely(hlt_pin))                                  /* hlt pin int */
        {
            hipl = IPL_HLTPIN;
        }
        else if (unlikely(mem_err))                             /* mem err int */
        {
            hipl = IPL_MEMERR;
        }
        else if (unlikely(crd_err))                             /* crd err int */
        {
            hipl = IPL_CRDERR;
        }
        else if (hipl = cpu_unit->cpu_intreg.highest_local_irql())
        {
            // use it
        }
        else if (SISR)
        {
            for (int32 i = IPL_SMAX;  i >= 1;  i--)
            {
                if (SISR & (1 << i))
                {
                    hipl = i;
                    break;
                }
            }
        }

        cpu_unit->cpu_context.highest_irql = hipl;

        cpu_unit->cpu_inside_reevaluate_thread_priority = FALSE;
        if (cpu_unit->cpu_redo_reevaluate_thread_priority)
        {
            cpu_unit->cpu_redo_reevaluate_thread_priority = FALSE;
            synced = TRUE;
            goto again;
        }
    }

    cpu_unit->cpu_intreg.query_local_clk_ipi(& is_active_clk_interrupt, & is_active_ipi_interrupt);
    if (unlikely(is_active_clk_interrupt))  cpu_unit->cpu_active_clk_interrupt = TRUE;
    if (unlikely(is_active_ipi_interrupt))  cpu_unit->cpu_active_ipi_interrupt = TRUE;

    if (unlikely(cpu_unit->cpu_active_clk_interrupt || cpu_unit->cpu_active_ipi_interrupt))
    {
        /*
         * if CLK or IPI interrupts are pending, run at elevated priority
         */
        cpu_set_thread_priority(RUN_PASS, SIMH_THREAD_PRIORITY_CPU_CRITICAL_OS_HI);
        return;
    }

    if (unlikely(cpu_unit->cpu_synclk_pending != SynclkNotPending))
    {
        /*
         * if inside a period protected from CLK raising, and CLK raising is pending,
         * run at elevated priority
         */
        cpu_set_thread_priority(RUN_PASS, SIMH_THREAD_PRIORITY_CPU_CRITICAL_OS_HI);
        return;
    }

    if (likely(sys_critical_section_ipl >= 0))
    {
        if (unlikely(
              sys_critical_section_ipl > 0 && PSL_GETIPL(PSL) >= sys_critical_section_ipl ||
              (cpu_unit->cpu_context.highest_irql = cpu_unit->cpu_intreg.highest_local_irql()) >= sys_critical_section_ipl && cpu_unit->cpu_context.highest_irql
            /* we do not need to check on the condition commented out below, a check for pending new interrupt,
               since we just performed an RMB synchronization above and acquired uptodate IRQ set */
            /* || IPL_HMAX >= sys_critical_section_ipl && cpu_unit->cpu_intreg.cas_changed(1, 1) */
            ))
        {
            /*
             * if inside operating system critical section (or interrupt request is pending with IPL high enough),
             * run at elevated priority
             */
            newprio = SIMH_THREAD_PRIORITY_CPU_CRITICAL_OS;
        }
        else
        {
            /* otherwise run at normal priority */
            newprio = SIMH_THREAD_PRIORITY_CPU_RUN;
        }
    }
    else
    {
        /* otherwise run at normal priority */
        newprio = SIMH_THREAD_PRIORITY_CPU_RUN;
    }

    cpu_set_thread_priority(RUN_PASS, newprio);
}

/*
 * Check whether operating system is running.
 *
 * The exact meaning of this check is somewhat diffuse, see its actual use.
 *
 * Current minimum requirement is that the operating system is past initial sysboot phase
 * with virtial memory mapping enabled on the primary processor.
 *
 * Practical check is that we are either running on the secondary processor,
 * or on the primary with MAPEN enabled.
 */
t_bool is_os_running(RUN_DECL)
{
    return cpu_unit->is_primary_cpu() ? (mapen != 0) : TRUE;
}

/*
 * Possible write to SCB had been detected.
 *
 * Cover an omission in VAX Architecture Standard and VMS code that may trigger problems
 * when running VMS on host architectures with weak memory model. VMS does not properly synchronize
 * writes to the SCB in inter-procesor system. Therefore if CPU2 changes shared SCB vector and CPU0
 * receives an interrupt, old (on undefined) vector can be used. Can happen for example if SYSGEN
 * is executed on secondary processor and connects the driver to the interrupt.
 *
 * Check if written physical address is indeed within SCB range.
 * If so, execute write memory barrier to make updated vector available to other processors ASAP.
 *
 * For interrupt vectors, read memory barrier will be executed by other processors when they
 * detect an interrupt, before it is dispatched through the SCB, so valid most recent value
 * of the vector will be fetched during interrupt sequence.
 *
 * However for exceptions and traps dispatch, read memory barrier is not performed. it would be
 * inefficient (and unlike for interrupts, is not required by VAX Architechire Standard)
 * to execute read memory barrier on every exception/trap. Therefore we do not do it, but we
 * do instead try to expedite other processors' cache view update by sending RMB request to them.
 * This can matter if kernel mode code such as XDELTA hooks into exception vectors.
 *
 * Note: a delay in update can stll occur since "lazy" checking for pending interrupts may delay
 * execution of remote RMB by up to a few VAX instructions, and also much longer if target processor
 * is currently executing interrupt processing. Should the latter ever become an issue, RMB request
 * can be moved to high or very high IPL, perhaps even above IPL_POWER, since RMB request interrupt
 * is not actually delivered to VAX code and is not visible at VAX code level, but causes only cache
 * synchronizaton at SIMH level.
 */
void cpu_scb_written(int32 pa)
{
    /*
     * Use RUN_SCOPE instead of RUN_DECL since this routine is invoked extremely infrequently,
     * but calls to it are dispersed throughout many places. Therefore try to maintain calling code
     * just a little bit more compact by avoiding extra argument.
     */
    RUN_SCOPE_RSCX;

    if ((uint32) pa >= (uint32) SCBB &&
        (uint32) pa < (uint32) SCBB + SCB_SIZE)
    {
        smp_wmb();

        for (uint32 cpu_ix = 0;  cpu_ix < sim_ncpus;  cpu_ix++)
        {
            if (cpu_units[cpu_ix] != cpu_unit || rscx->thread_type != SIM_THREAD_TYPE_CPU)
                interrupt_ipi_rmb(RUN_PASS, cpu_units[cpu_ix]);
        }
    }
}

static uint32 interrupt_percpu_devs[IPL_HLVL];
uint32 devs_per_irql[IPL_HLVL];
extern DEVICE sysd_dev;

void sim_init_interrupt_info()
{
    memset(interrupt_percpu_devs, 0, sizeof interrupt_percpu_devs);
    memset(devs_per_irql, 0, sizeof devs_per_irql);

    DEVICE* dptr;

    for (int i = 0; (dptr = sim_devices[i]) != NULL; i++)
    {
        DIB* dib = (DIB*) dptr->ctxt;
        if (dib == NULL || dib->vnum == 0)  continue;
        uint32 ix_ipl = IVCL_IPL(dib->vloc);
        uint32 dev = IVCL_DEV(dib->vloc);
        for (int vn = 0;  vn < dib->vnum;  vn++, dev++)
        {
            /* Record highest device index used */
            devs_per_irql[ix_ipl] = imax(devs_per_irql[ix_ipl], dev + 1);

            if (dptr->flags & DEV_PERCPU)
            {
                interrupt_percpu_devs[ix_ipl] |= (1 << dev);
            }
        }
    }

    devs_per_irql[IPL_IPINTR] = imax(devs_per_irql[IPL_IPINTR], (uint32) INT_V_IPINTR + 1);
    devs_per_irql[IPL_IPIRMB] = imax(devs_per_irql[IPL_IPIRMB], (uint32) INT_V_IPIRMB + 1);
    devs_per_irql[IPL_SECEXIT] = imax(devs_per_irql[IPL_SECEXIT], (uint32) INT_V_SECEXIT + 1);
    devs_per_irql[IPL_SYNCWSYS] = imax(devs_per_irql[IPL_SYNCWSYS], (uint32) INT_V_SYNCWSYS + 1);
    devs_per_irql[IPL_SYNCLK] = imax(devs_per_irql[IPL_SYNCLK], (uint32) INT_V_SYNCLK + 1);
    devs_per_irql[IPL_ASYNC_IO] = imax(devs_per_irql[IPL_ASYNC_IO], (uint32) INT_V_ASYNC_IO + 1);
    devs_per_irql[IPL_STOP] = imax(devs_per_irql[IPL_STOP], (uint32) INT_V_STOP + 1);

    interrupt_percpu_devs[IPL_IPINTR] |= (1 << INT_V_IPINTR);
    interrupt_percpu_devs[IPL_IPIRMB] |= (1 << INT_V_IPIRMB);
    interrupt_percpu_devs[IPL_SECEXIT] |= (1 << INT_V_SECEXIT);
    interrupt_percpu_devs[IPL_SYNCWSYS] |= (1 << INT_V_SYNCWSYS);
    interrupt_percpu_devs[IPL_SYNCLK] |= (1 << INT_V_SYNCLK);
    interrupt_percpu_devs[IPL_ASYNC_IO] |= (1 << INT_V_ASYNC_IO);
    interrupt_percpu_devs[IPL_STOP] |= (1 << INT_V_STOP);
}

/* raise device interrupt */
void interrupt_set_int(uint32 ix_ipl, uint32 dev, t_bool force_wakeup)
{
    interrupt_set_int(NULL, ix_ipl, dev, force_wakeup);
}

void interrupt_set_int(CPU_UNIT* xcpu, uint32 ix_ipl, uint32 dev, t_bool force_wakeup)
{
    RUN_SCOPE_RSCX;
    t_bool wakeup = FALSE;

    uint32 ipl = ix_ipl + IPL_HMIN;

    /* determine target VCPU */
    if (interrupt_percpu_devs[ix_ipl] & (1 << dev))
        xcpu = xcpu ? xcpu : cpu_unit;
    else
        xcpu = &cpu_unit_0;

    /* 
     * Check for (xcpu->cpu_id == rscx->thread_cpu_id) rather than (xcpu == cpu_unit) 
     * since while inside cpu_start_secondary() VCPU thread can be temporary bound to the
     * context of another VCPU and impersonating it.
     */
    t_bool toself = (xcpu->cpu_id == rscx->thread_cpu_id) && (rscx->thread_type == SIM_THREAD_TYPE_CPU);

    /* 
     * Raise interrupt flag.
     * Perform memory barrier if necessary.
     *
     * If this interrupt was not raised previously in the target CPU
     * and is targeted to other (non-current) virtual CPU, wake this CPU up.
     */
    if (FALSE == xcpu->cpu_intreg.set_int(ipl, dev, toself) && !toself || force_wakeup)
        wakeup = TRUE;

    /*
     * Raise target VCPU's thread level priority to reflect pending interrupt
     */
    if (sys_critical_section_ipl >= 0 && (int32) ipl >= sys_critical_section_ipl)
    {
        if (toself)
        {
            switch (cpu_unit->cpu_thread_priority)
            {
            case SIMH_THREAD_PRIORITY_CPU_CALIBRATION:
            case SIMH_THREAD_PRIORITY_CPU_CRITICAL_VM:
            case SIMH_THREAD_PRIORITY_CPU_CRITICAL_OS_HI:
                break;
            default:
                cpu_set_thread_priority(RUN_PASS, SIMH_THREAD_PRIORITY_CPU_CRITICAL_OS_HI);
                break;
            }
        }
        else if (rscx->thread_type == SIM_THREAD_TYPE_CPU || rscx->thread_type == SIM_THREAD_TYPE_CLOCK)
        {
            /* 
             * Really want to raise target CPU thread priority to CRITICAL_OS or CRITICAL_OS_HI here, but the target
             * can already be at CRITICAL_VM, so use CRITICAL_VM to avoid undermining the target, but leave the mark 
             * for the target to re-examine its thread priority ASAP.
             *
             * We assume that CRITICAL_VM is the highest thread priority target may usually have and it is not likely to be
             * at CALIBRATION priority since SSC clocks (associated with CALIBRATION) are generally not used by VMS
             * on a MicroVAX after SMP is activated, whereas device interrupts are unlikely during the INIT phase
             * when SSC calibration is performed. However SYNCLK are quite possible during CALIBRATION.
             * Nevertheless even if it happened, CRITICAL_VM is close to CALIBRATION, so we use CRITICAL_VM here
             * instead of CALIBRATION.
             *
             * We do however make limited attempt to keep the thread at CALIBRATON if it was already at CALIBRATION
             * and avoid resetting it to VM. The check below does not have to be perfect.
             */
            if (must_control_prio())
            {
                if (weak_read(xcpu->sysd_active_mask) == 0)
                    smp_set_thread_priority(xcpu->cpu_thread, SIMH_THREAD_PRIORITY_CPU_CRITICAL_VM);
                else
                    smp_set_thread_priority(xcpu->cpu_thread, SIMH_THREAD_PRIORITY_CPU_CALIBRATION);

                /* Force target VCPU to re-evaluate its thread priority ASAP. Can be xchg(changed, 1). */
                xcpu->cpu_intreg.cas_changed(0, 1);
            }
        }
        else if (rscx->thread_type == SIM_THREAD_TYPE_CONSOLE)
        {
            t_bool is_active_clk_interrupt;
            t_bool is_active_ipi_interrupt;

            /* console thread has access to xcpu->cpu_thread_priority */
            switch (xcpu->cpu_thread_priority)
            {
            case SIMH_THREAD_PRIORITY_CPU_CALIBRATION:
            case SIMH_THREAD_PRIORITY_CPU_CRITICAL_VM:
            case SIMH_THREAD_PRIORITY_CPU_CRITICAL_OS_HI:
                break;
            case SIMH_THREAD_PRIORITY_CPU_CRITICAL_OS:
                xcpu->cpu_intreg.query_local_clk_ipi(& is_active_clk_interrupt, & is_active_ipi_interrupt);
                if (is_active_clk_interrupt)  xcpu->cpu_active_clk_interrupt = TRUE;
                if (is_active_ipi_interrupt)  xcpu->cpu_active_ipi_interrupt = TRUE;
                if (xcpu->cpu_active_clk_interrupt || xcpu->cpu_active_ipi_interrupt)
                {
                    if (must_control_prio())
                        smp_set_thread_priority(xcpu->cpu_thread, SIMH_THREAD_PRIORITY_CPU_CRITICAL_OS_HI);
                    xcpu->cpu_thread_priority = SIMH_THREAD_PRIORITY_CPU_CRITICAL_OS_HI;
                }
                break;
            default:
                if (must_control_prio())
                    smp_set_thread_priority(xcpu->cpu_thread, SIMH_THREAD_PRIORITY_CPU_CRITICAL_OS);
                xcpu->cpu_thread_priority = SIMH_THREAD_PRIORITY_CPU_CRITICAL_OS;
                break;
            }
        }
    }

    if (wakeup)
        wakeup_cpu(xcpu);
}

/* 
 * Raise interprocessor interrupt on 'xcpu'.
 * It is ok to invoke this routine on 'xcpu' than is not running, it is no-op then.
 */
void interrupt_ipi(RUN_DECL, CPU_UNIT* xcpu, t_bool force_wakeup)
{
    interrupt_set_int(xcpu, IPL_IPINTR, INT_V_IPINTR, force_wakeup);
}

void interrupt_ipi_rmb(RUN_DECL, CPU_UNIT* xcpu)
{
    RUN_SCOPE_RSCX_ONLY;

    if (xcpu->cpu_id == rscx->thread_cpu_id && rscx->thread_type == SIM_THREAD_TYPE_CPU)
    {
        smp_rmb();
    }
    else
    {
        xcpu->cpu_intreg.set_int(IPL_ABS_IPIRMB, INT_V_IPIRMB, FALSE);
    }
}

/* clear device interrupt */
void interrupt_clear_int(uint32 ix_ipl, uint32 dev)
{
    RUN_SCOPE_RSCX;
    CPU_UNIT* xcpu;

    uint32 ipl = ix_ipl + IPL_HMIN;

    /* determine target VCPU */
    if (interrupt_percpu_devs[ix_ipl] & (1 << dev))
        xcpu = cpu_unit;
    else
        xcpu = &cpu_unit_0;

    /* 
     * Check for (xcpu->cpu_id == rscx->thread_cpu_id) rather than (xcpu == cpu_unit) 
     * since while inside cpu_start_secondary() VCPU thread can be temporary bound to the
     * context of another VCPU and impersonating it.
     */
    t_bool toself = (xcpu->cpu_id == rscx->thread_cpu_id) && (rscx->thread_type == SIM_THREAD_TYPE_CPU);

    xcpu->cpu_intreg.clear_int(ipl, dev, toself);
}

/*
 * Read/write routines for SIMH REG's of device INT registers,
 * encoded as IRDATA_DEV.
 */
t_value reg_irdata_dev_rd(REG* r, uint32 idx)
{
    RUN_SCOPE;
    uint32 ivcl = (uint32) (t_addr_val) r->loc;
    uint32 ix_ipl = IVCL_IPL(ivcl);
    uint32 dev = IVCL_DEV(ivcl);
    uint32 ipl = ix_ipl + IPL_HMIN;

    CPU_UNIT* xcpu;

    if (interrupt_percpu_devs[ix_ipl] & (1 << dev))
        xcpu = cpu_unit;
    else
        xcpu = &cpu_unit_0;

    return xcpu->cpu_intreg.examine_int(ipl, dev) ? 1 : 0;
}

void reg_irdata_dev_wr(REG* r, uint32 idx, t_value value)
{
    uint32 ivcl = (uint32) (t_addr_val) r->loc;
    uint32 ix_ipl = IVCL_IPL(ivcl);
    uint32 dev = IVCL_DEV(ivcl);

    if (value & 1)
        interrupt_set_int(ix_ipl, dev, TRUE);
    else
        interrupt_clear_int(ix_ipl, dev);
}

int reg_irdata_dev_vcpu(REG* r, uint32 idx)
{
    RUN_SCOPE;
    uint32 ivcl = (uint32) (t_addr_val) r->loc;
    uint32 ix_ipl = IVCL_IPL(ivcl);
    uint32 dev = IVCL_DEV(ivcl);

    if (interrupt_percpu_devs[ix_ipl] & (1 << dev))
        return cpu_unit->cpu_id;
    else
        return -1;
}

/*
 * Read/write routines for SIMH REG's of IPL level interrupts (14-17),
 * encoded as IRDATA_LVL.
 */
t_value reg_irdata_lvl_rd(REG* r, uint32 idx)
{
    RUN_SCOPE;
    uint32 what_code = (uint32) (t_addr_val) r->loc;
    uint32 ipl = what_code & 31;
    t_bool curr_cpu = 0 != (what_code & 32);

    if (ipl < IPL_HMIN || ipl > IPL_HMAX)
        return 0;

    CPU_UNIT* xcpu = curr_cpu ? cpu_unit : &cpu_unit_0;

    t_value res = 0;

    for (uint32 dev = 0; dev < devs_per_irql[ipl - IPL_HMIN];  dev++)
    {
        if (xcpu->cpu_intreg.examine_int(ipl, dev))
            res |= (t_value) (1u << dev);
    }

    return res;
}

void reg_irdata_lvl_wr(REG* r, uint32 idx, t_value value)
{
    RUN_SCOPE;
    uint32 what_code = (uint32) (t_addr_val) r->loc;
    uint32 ipl = what_code & 31;
    t_bool curr_cpu = 0 != (what_code & 32);
    uint32 ix_ipl = ipl - IPL_HMIN;

    if (ipl < IPL_HMIN || ipl > IPL_HMAX)
        return;

    CPU_UNIT* xcpu = curr_cpu ? cpu_unit : &cpu_unit_0;

    for (uint32 dev = 0; dev < devs_per_irql[ipl - IPL_HMIN];  dev++)
    {
        if (value & (t_value) (1u << dev))
        {
            interrupt_set_int(xcpu, ix_ipl, dev);
        }
        else
        {
            xcpu->cpu_intreg.clear_int(ipl, dev, FALSE);
        }
    }
}

int reg_irdata_lvl_vcpu(REG* r, uint32 idx)
{
    RUN_SCOPE;
    return cpu_unit->cpu_id;
}

t_value reg_sirr_rd(REG* r, uint32 idx)
{
    /* write-only register, return dummy value */
    return 0;
}

void reg_sirr_wr(REG* r, uint32 idx, t_value value)
{
    RUN_SCOPE;

    if (value >= 1 && value <= 0xF)
    {
        SISR = SISR | (1 << value);
        SET_IRQL;
    }
}


/*
 * Check if CPU may perform idle sleep.
 *
 * Called by sim_idle right before entering idle sleep.
 * If this function returns FALSE, idle sleep is not attempted.
 */
t_bool cpu_may_sleep(RUN_DECL)
{
    /* 
     * check if received new interrupt
     */
    if (cpu_unit->cpu_intreg.cas_changed(1, 1))
        return FALSE;

    /*
     * check if *deliverable* interrupt is pending
     */
    SET_IRQL;
    if (unlikely(GET_IRQL(trpirq) != 0))
        return FALSE;

    /*
     * Check if stop is pending
     */
    smp_post_interlocked_rmb();
    if (unlikely(weak_read(stop_cpus))) 
        return FALSE;
    
    /* 
     * check if the bit in idle CPU mask had been cleared
     */
    if (sys_idle_cpu_mask_va && mapen && is_os_running(RUN_PASS))
    {
        int32 acc = ACC_MASK(KERN);
        int32 mask = 0;         /* initialize to suppress false GCC warning */
        t_bool abort = FALSE;

        sim_try
        {
            mask = Read (RUN_PASS, sys_idle_cpu_mask_va, L_LONG, RA);
        }
        sim_catch (sim_exception_ABORT, exabort)
        {
            if (cpu_unit->cpu_exception_ABORT == NULL && exabort->isAutoDelete())
            {
                cpu_unit->cpu_exception_ABORT = exabort;
            }
            else
            {
                exabort->checkAutoDelete();
            }
            abort = TRUE;
        }
        sim_end_try

        if (abort)
        {
            smp_printf ("\nOperating System CPU idle mask is non-readable\n");
            if (sim_log)
                fprintf (sim_log, "\nOperating System CPU idle mask is non-readable\n");
            ABORT_INVALID_SYSOP;
        }

        if ((mask & (1 << cpu_unit->cpu_id)) == 0)
            return FALSE;
    }

    /*
     * should not have synclk -> clk conversion pending, that would be logical error,
     * especially important if (syncw.on & SYNCW_SYS)
     */
    if (cpu_unit->cpu_synclk_pending != SynclkNotPending)
        panic("cpu_may_sleep: cpu_synclk_pending != SynclkNotPending");

    return TRUE;
}

/* 
 * Wake-up virtual CPU that may be in an idle sleep.
 * It is ok to invoke this routine on 'xcpu' than is not sleeping or not running, it is no-op then.
 *
 * Because of race condition with sim_idle this routine can sometimes produce spurious wakeups
 * out of sim_idle sleep.
 */
void wakeup_cpu(CPU_UNIT *xcpu)
{
    if (smp_interlocked_cas_done_var(& xcpu->cpu_sleeping, 1, 1))
        xcpu->cpu_wakeup_event->set();
}

/*
 * Operating system scheduler had changed "idle CPUs" mask.
 * Wake up those CPUs whose "idle" bit had been reset so they can perform their scheduling cycle.
 * Called by the CPU that modifies the mask.
 */
void wakeup_cpus(RUN_DECL, uint32 old_idle_mask, uint32 new_idle_mask)
{
    /* calculate wakeup set */
    uint32 wmask = old_idle_mask & ~new_idle_mask;

    /* remove current CPU from the wakeup set, since it is already awake */
    wmask &= ~(1 << cpu_unit->cpu_id);
    if (wmask == 0)  return;

    /* 
     * flush out changed content of idle CPUs mask (operating system cell pointed by sys_idle_cpu_mask_va)
     * to other CPUs
     */
    smp_wmb();

    /* 
     * Wakeup CPUs indicated by the change in the idle mask.
     *
     * Note we do not need to send IPIRMB because idle_sleep and cpu_may_sleep will perform RMB and
     * idle CPU mask checking before trying to enter idle sleep and also perform RMB after exiting the sleep.
     */
    for (uint32 cpu_ix = 0;  cpu_ix < sim_ncpus;  cpu_ix++)
    {
        if (wmask & (1 << cpu_ix))
        {
            CPU_UNIT* xcpu = cpu_units[cpu_ix];
            wakeup_cpu(xcpu);
        }
    }
}

/*
 * Beginning and ending interlocked instructions.
 *
 * Beginning:
 *
 *     Elevate thread priority. This prevents VCPU thread from acquiring a lock on an interlocked
 *     data item and then being pre-empted for an extended time while holding the lock.
 *
 * Ending:
 *
 *     Thread priority will be demoted back to the original via cpu_end_interlocked()
 *     when interlocked instruction completes and interlock is released.
 *
 */
void cpu_begin_interlocked(RUN_DECL, volatile sim_thread_priority_t* sv_priority, volatile t_bool* sv_priority_stored)
{
    RUN_SCOPE_RSCX_ONLY;
    /*
     * See "VAX MP Techincal Overview" chapter "Implementation of VAX interlocked instructions"
     * for detailed discussion of what (and why) is done here.
     *
     * Code bracketed by cpu_begin_interlocked/cpu_end_interlocked may temporarily acquire and release locks with
     * non-null criticality level and may call cpu_reevaluate_thread_priority. To prevent thread priority from being
     * decreased by by these activities, we have to mark up thread's vm_critical_locks count.
     */
    ++rscx->vm_critical_locks;

    // *sv_priority = cpu_unit->cpu_thread_priority;
    // *sv_priority_stored = TRUE;

    if ((cpu_unit->cpu_thread_priority == SIMH_THREAD_PRIORITY_INVALID || 
         cpu_unit->cpu_thread_priority < SIMH_THREAD_PRIORITY_CPU_CRITICAL_VM) && 
        likely(is_os_running(RUN_PASS)))
    {
        cpu_set_thread_priority(RUN_PASS, SIMH_THREAD_PRIORITY_CPU_CRITICAL_VM);
    }
}

void cpu_end_interlocked(RUN_DECL, sim_thread_priority_t sv_priority, t_bool sv_priority_stored)
{
    /*
     * Restore thread priority. Note that we cannot simply restore priority to sv_priority,
     * since priority might have been purposefully altered if interrupt was sent to this processor 
     * by an external source.
     */
    vm_critical_unlock();
}

/* internal development/debugging tool */
t_stat xdev_cmd(int32 flag, char *cptr)
{
    RUN_SCOPE;
    char gbuf[CBUFSIZE];
    t_stat r;
    
    if (*cptr == 0)
        return SCPE_2FARG;

    cptr = get_glyph(cptr, gbuf, 0);

    /* XDEV IPI <cpu-id> */
    if (streqi(gbuf, "ipi") && cptr && *cptr)
    {
        uint32 ncpu = (uint32) get_uint(cptr, 10, sim_ncpus - 1, &r);
        if (r != SCPE_OK)
            return SCPE_ARG;
        if (ncpu >= sim_ncpus || cpu_units[ncpu] == NULL)  return SCPE_ARG;
        interrupt_ipi(RUN_PASS, cpu_units[ncpu]);
        return SCPE_OK;
    }
    else
    {
        return SCPE_ARG;
    }
}

/*
 * Pre-fault memory into working set
 */
void cpu_prefault_memory()
{
    int npages = (int) (cpu_unit_0.capac / VA_PAGSIZE);
    volatile t_byte* p = (volatile t_byte*) M;
    volatile static t_byte b;

    for (int k = 0;  k < npages;  k++)
    {
        b = *p;
        p += VA_PAGSIZE;
    }
}
