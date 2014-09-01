/* vax_cpu.c: VAX CPU

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

   cpu          VAX central processor

   20-Sep-11    MP      Fixed idle conditions for various versions of Ultrix, 
                        Quasijarus-4.3BSD, NetBSD and OpenBSD.
                        Note: Since NetBSD and OpenBSD are still actively 
                        developed operating systems, new versions of 
                        these OSes are moving targets with regard to 
                        providing idle detection.  At this time, recent versions 
                        of OpenBSD have veered from the traditional OS idle 
                        approach taken in the other BSD derived OSes.  
                        Determining a reasonable idle detection pattern does 
                        not seem possible for these versions.
   23-Mar-11    RMS     Revised for new idle design (from Mark Pizzolato)
   05-Jan-11    MP      Added Asynch I/O support
   24-Apr-10    RMS     Added OLDVMS idle timer option
                        Fixed bug in SET CPU IDLE
   21-May-08    RMS     Removed inline support
   28-May-08    RMS     Inlined instruction prefetch, physical memory routines
   13-Aug-07    RMS     Fixed bug in read access g-format indexed specifiers
   28-Apr-07    RMS     Removed clock initialization
   29-Oct-06    RMS     Added idle support
   22-May-06    RMS     Fixed format error in CPU history (found by Peter Schorn)
   10-May-06    RMS     Added -kesu switches for virtual addressing modes
                        Fixed bugs in examine virtual
                        Rewrote history function for greater usability
                        Fixed bug in reported VA on faulting cross-page write
   02-May-06    RMS     Fixed fault cleanup to clear PSL<tp>
                        Fixed ADAWI r-mode to preserve dst<31:16>
                        Fixed ACBD/G to test correct operand
                        Fixed access checking on modify-class specifiers
                        Fixed branch displacements in history buffer
                        (all reported by Tim Stark)
   17-Nov-05    RMS     Fixed CVTfi with integer overflow to trap if PSW<iv> set
   13-Nov-05    RMS     Fixed breakpoint test with 64b addresses
   25-Oct-05    RMS     Removed cpu_extmem
   22-Sep-05    RMS     Fixed declarations (from Sterling Garwood)
   16-Aug-05    RMS     Fixed C++ declaration and cast problems
   13-Jan-05    RMS     Fixed initial state of cpu_extmem
   06-Nov-04    RMS     Added =n to SHOW HISTORY
   30-Sep-04    RMS     Added octaword specifier decodes and instructions
                        Moved model-specific routines to system module
   02-Sep-04    RMS     Fixed bug in EMODD/G, second word of quad dst not probed
   28-Jun-04    RMS     Fixed bug in DIVBx, DIVWx (reported by Peter Trimmel)
   18-Apr-04    RMS     Added octaword macros
   25-Jan-04    RMS     Removed local debug logging support
                RMS,MP  Added extended physical memory support
   31-Dec-03    RMS     Fixed bug in set_cpu_hist
   21-Dec-03    RMS     Added autoconfiguration controls
   29-Oct-03    RMS     Fixed WriteB declaration (found by Mark Pizzolato)
   23-Sep-03    RMS     Revised instruction history for dynamic sizing
   17-May-03    RMS     Fixed operand order in EMODx
   23-Apr-03    RMS     Revised for 32b/64b t_addr
   05-Jan-02    RMS     Added memory size restore support
   25-Dec-02    RMS     Added instruction history (from Mark Pizzolato)
   29-Sep-02    RMS     Revised to build dib_tab dynamically
   14-Jul-02    RMS     Added halt to console, infinite loop detection
                        (from Mark Pizzolato)
   02-May-02    RMS     Fixed bug in indexed autoincrement register logging
   30-Apr-02    RMS     Added TODR powerup routine
   18-Apr-02    RMS     Cleanup ambiguous signed left shifts
   15-Apr-02    RMS     Fixed bug in CASEL condition codes

   The register state for the VAX is:

        R[0:15]         general registers
        PSL<31:0>       processor status longword
         TP<30>         trace pending
         FPD<27>        first part done
         IS<26>         interrupt stack
         CM<25:24>      current mode
         PM<23:22>      previous mode
         IPL<20:16>     interrupt priority level
         PSW<15:0>      non-privileged processor status word
          DV<7>         decimal overflow trap enable
          FU<6>         floating underflow fault enable
          IV<5>         integer overflow trap enable
          T<4>          trace trap enable
          CC<3:0>       condition codes
        SCBB            system control block base
        PCBB            process control block base
        SBR             system page table base
        SLR             system page table length
        P0BR            process region 0 page table base
        P0LR            process region 0 page table length
        P1BR            process region 1 page table base
        P1LR            process region 1 page table length
        SIRR/SISR       software interrupt request/summary register
        ASTLVL          AST level register

   The VAX has a variable length instruction format with up to six operands:

        opcode byte
        operand 1 specifier
         :
        operand n specifier

   Each operand specifier is a byte consisting of an addressing mode, a
   register, and possibly 1-8 bytes of extension:

        number  name        extension   mnemonic        operation

        0-3     short literal   -       #n      op <- specifier
        4       index           -       [Rn]    index by Rn
        5       register        -       Rn      op <- Rn
        6       register def    -       (Rn)    op <- M[Rn]
        7       autodecrement   -       -(Rn)   Rn <- Rn - length
                                                op <- M[Rn]
        8       autoincrement   -       (Rn)+   op <- M[Rn]
                                                Rn <- Rn + length
        9       auto deferred   -       @(Rn)+  op <- M[M[Rn]]
                                                Rn <- Rn + 4
        A       byte displ      byte d  d(Rn)   op <- M[Rn + sxt.d]
        B       byte displ def  byte d  @d(Rn)  op <- M[M[Rn + sxt.d]]
        C       word displ      word d  d(Rn)   op <- M[Rn + sxt.d]
        D       word displ def  word d  @d(Rn)  op <- M[M[Rn + sxt.d]]
        E       long displ      long d  d(Rn)   op <- M[Rn + d]
        F       long displ def  long d  @d(Rn)  op <- M[M[Rn + d]]

   When the general register is the PC, certain modes are forbidden, and
   others have special interpretations:

        4F      index           fault
        5F      register        fault
        6F      register def    fault
        7F      autodecrement   fault
        8F      immediate       1-8B    #imm    op <- imm
        9       absolute        4B      @#imm   op <- M[imm]
        A       byte relative   byte d  d(Rn)   op <- M[PC + sxt.d]
        B       byte rel def    byte d  @d(Rn)  op <- M[M[PC + sxt.d]]
        C       word relative   word d  d(Rn)   op <- M[PC + sxt.d]
        D       word rel def    word d  @d(Rn)  op <- M[M[PC + sxt.d]]
        E       long relative   long d  d(Rn)   op <- M[PC + d]
        F       long rel def    long d  @d(Rn)  op <- M[M[PC + d]]

   This routine is the instruction decode routine for the VAX.  It
   is called from the simulator control program to execute instructions
   in simulated memory, starting at the simulated PC.  It runs until an
   enabled exception is encountered.

   General notes:

   1. Traps and interrupts.  Variable trpirq microencodes the outstanding
        trap request (if any) and the level of the highest outstanding
        interrupt (if any).

   2. Interrupt requests are maintained in the array in cpu_intreg (of type InterruptRegister),
      one word per interrupt level, one bit per device.

   3. Adding I/O devices.  These modules must be modified:

        vax_defs.h      add device address and interrupt definitions
        vax_sys.c       add sim_devices table entry
*/

/* Definitions */

#include "sim_defs.h"
#include "vax_defs.h"
#include "sim_rev.h"

#define OP_MEM          -1
#define UNIT_V_CONH     (UNIT_V_UF + 0)                 /* halt to console */
#define UNIT_V_MSIZE    (UNIT_V_UF + 1)                 /* dummy */
#define UNIT_CONH       (1u << UNIT_V_CONH)
#define UNIT_MSIZE      (1u << UNIT_V_MSIZE)
#define GET_CUR         acc = ACC_MASK (PSL_GETCUR (PSL))
#define VAX_IDLE_VMS        0x01
#define VAX_IDLE_ULT        0x02
#define VAX_IDLE_ULTOLD     0x04
#define VAX_IDLE_QUAD       0x08

#define OPND_SIZE       16
#define INST_SIZE       52
#define op0             opnd[0]
#define op1             opnd[1]
#define op2             opnd[2]
#define op3             opnd[3]
#define op4             opnd[4]
#define op5             opnd[5]
#define op6             opnd[6]
#define op7             opnd[7]
#define op8             opnd[8]
#define CHECK_FOR_PC    if (rn == nPC) \
                            RSVD_ADDR_FAULT
#define CHECK_FOR_SP    if (rn >= nSP) \
                            RSVD_ADDR_FAULT
#define CHECK_FOR_AP    if (rn >= nAP) \
                            RSVD_ADDR_FAULT
#define WRITE_B(r)      if (spec > (GRN | nPC)) \
                            Write (RUN_PASS, va, r, L_BYTE, WA); \
                        else R[rn] = (R[rn] & ~BMASK) | ((r) & BMASK)
#define WRITE_W(r)      if (spec > (GRN | nPC)) \
                            Write (RUN_PASS, va, r, L_WORD, WA); \
                        else R[rn] = (R[rn] & ~WMASK) | ((r) & WMASK)
#define WRITE_L(r)      if (spec > (GRN | nPC)) \
                            Write (RUN_PASS, va, r, L_LONG, WA); \
                        else R[rn] = (r)
#define WRITE_Q(rl,rh)  if (spec > (GRN | nPC)) { \
                        if ((Test (RUN_PASS, va + 7, WA, &mstat) >= 0) || \
                            (Test (RUN_PASS, va, WA, &mstat) < 0)) \
                            Write (RUN_PASS, va, rl, L_LONG, WA); \
                            Write (RUN_PASS, va + 4, rh, L_LONG, WA); \
                            } \
                        else { \
                            if (rn >= nSP) \
                                RSVD_ADDR_FAULT; \
                            R[rn] = rl; \
                            R[rn + 1] = rh; \
                            }

#define HIST_MIN        64
#define HIST_MAX        65536

#include "vax_cpu.h"

class SIM_ALIGN_64 InstHistory
{
public:
    UINT64                  stamp;
    uint8                   isRecorded;
    SIM_ALIGN_32 int32      iPC;
    int32                   iPSL;
    int32                   opc;
    int32                   opnd[OPND_SIZE];
    uint8                   inst[INST_SIZE];
};

static uint32 hst_lnt = 0;             /* length of history buffer */
static t_bool hst_on = FALSE;          /* history recording enabled */
static t_bool hst_sync = TRUE;         /* true if using global stamp */
#include "vax_hist.h"
static void cpu_free_history ();

volatile uint32* M = NULL;             /* memory */
atomic_int32 hlt_pin = 0;              /* HLT pin intr */
int32 sys_idle_cpu_mask_va = 0;        /* virtual address of system idle CPUs mask (VMS: SCH$GL_IDLE_CPUS) or NULL */
int32 sys_critical_section_ipl = -1;   /* IPL for entering O/S critical section */
uint32 cpu_idle_mask = VAX_IDLE_VMS;   /* idle mask */
uint32 cpu_idle_type = 1;              /* default VMS */


REG *pcq_r = NULL;                     /* PC queue register descriptor (backing store is in per-CPU area) */
static uint32 bugcheck_ctrl = 0;       /* control handling of bugcheck pseudo-instructions */

// int32 cpu_astop = 0;

extern const uint32 byte_mask[33];
extern const uint32 align[4];

const uint32 byte_mask[33] = { 0x00000000,
 0x00000001, 0x00000003, 0x00000007, 0x0000000F,
 0x0000001F, 0x0000003F, 0x0000007F, 0x000000FF,
 0x000001FF, 0x000003FF, 0x000007FF, 0x00000FFF,
 0x00001FFF, 0x00003FFF, 0x00007FFF, 0x0000FFFF,
 0x0001FFFF, 0x0003FFFF, 0x0007FFFF, 0x000FFFFF,
 0x001FFFFF, 0x003FFFFF, 0x007FFFFF, 0x00FFFFFF,
 0x01FFFFFF, 0x03FFFFFF, 0x07FFFFFF, 0x0FFFFFFF,
 0x1FFFFFFF, 0x3FFFFFFF, 0x7FFFFFFF, 0xFFFFFFFF
 };
const uint32 byte_sign[33] = { 0x00000000,
 0x00000001, 0x00000002, 0x00000004, 0x00000008,
 0x00000010, 0x00000020, 0x00000040, 0x00000080,
 0x00000100, 0x00000200, 0x00000400, 0x00000800,
 0x00001000, 0x00002000, 0x00004000, 0x00008000,
 0x00010000, 0x00020000, 0x00040000, 0x00080000,
 0x00100000, 0x00200000, 0x00400000, 0x00800000,
 0x01000000, 0x02000000, 0x04000000, 0x08000000,
 0x10000000, 0x20000000, 0x40000000, 0x80000000
 };
const uint32 align[4] = {
 0xFFFFFFFF, 0x00FFFFFF, 0x0000FFFF, 0x000000FF
 };

/* External and forward references */

extern int32 sim_int_char;
extern int32 sim_switches;
extern uint32 sim_brk_types, sim_brk_dflt, sim_brk_summ; /* breakpoint info */
extern uint32 devs_per_irql[IPL_HLVL];

extern t_stat build_dib_tab (void);
extern UNIT rom_unit, nvr_unit;
extern int32 BadCmPSL (RUN_DECL, int32 newpsl);
extern int32 get_vector (RUN_DECL, int32 lvl);
extern void set_map_reg (RUN_DECL);
extern void rom_wr_B (RUN_DECL, int32 pa, int32 val);
extern int32 machine_check (RUN_DECL, int32 p1, int32 opc, int32 cc, int32 delta);
extern const uint16 drom[NUM_INST][MAX_SPEC + 1];
extern t_stat cpu_boot (int32 unitno, DEVICE *dptr);
extern int32 con_halt (int32 code, int32 cc);

t_stat cpu_reset (DEVICE *dptr);
t_stat cpu_ex (t_value *vptr, t_addr exta, UNIT *uptr, int32 sw);
t_stat cpu_ex_run (RUN_DECL, t_value *vptr, t_addr exta, UNIT *uptr, int32 sw);
t_stat cpu_dep (t_value val, t_addr exta, UNIT *uptr, int32 sw);
t_stat cpu_set_size (UNIT *uptr, int32 val, char *cptr, void *desc);
t_stat cpu_set_hist (UNIT *uptr, int32 val, char *cptr, void *desc);
t_stat cpu_show_hist (SMP_FILE *st, UNIT *uptr, int32 val, void *desc);
t_stat cpu_show_virt (SMP_FILE *st, UNIT *uptr, int32 val, void *desc);
t_stat cpu_set_idle (UNIT *uptr, int32 val, char *cptr, void *desc);
t_stat cpu_show_idle (SMP_FILE *st, UNIT *uptr, int32 val, void *desc);
int32 cpu_get_vsw (RUN_DECL, int32 sw);
int32 get_istr (RUN_DECL, int32 lnt, int32 acc);
int32 ReadOcta (RUN_DECL, int32 va, int32 *opnd, int32 j, int32 acc);
t_bool cpu_show_opnd (SMP_FILE *st, InstHistory *h, int32 line);
void cpu_idle (RUN_DECL);
t_stat cpu_idle_svc (RUN_SVC_DECL, UNIT *uptr);

static t_stat handle_abort(RUN_DECL, sim_exception_ABORT* exabort, 
                           volatile int32& cc, volatile int32& acc, volatile int32& opc);
static void op_reserved_ff(RUN_DECL, int32 acc);
static t_bool is_bug_instruction(RUN_DECL, int32 acc, int32 va, char* wl, uint32* bug_code);
static t_bool cpu_start_secondary(RUN_DECL, CPU_UNIT* xcpu, uint32* pcb, uint32 scbb, 
                                  uint32 x_mapen, uint32 sbr, uint32 slr, uint32 isp);
static void op_adawi(RUN_DECL, int32 *opnd, int32 acc, int32 spec, int32 rn, int32 va, volatile int32& cc);

/*
 * static part of initializer, cannot fail or throw exceptions
 */
CPU_UNIT::CPU_UNIT() : UNIT(&cpu_idle_svc, UNIT_FIX|UNIT_BINK|UNIT_ISCPU, INITMEMSIZE)
{
    device = &cpu_dev;
    cpu_state = CPU_STATE_STANDBY;
    atomic_var(cpu_adv_cycles) = 0;

    sim_time = 0;
    sim_rtime = 0;
    noqueue_time = 0;

    cpu_stop_code = SCPE_OK;
    cpu_dostop = FALSE;

    cpu_hst = NULL;
    UINT64_SET_ZERO(cpu_hst_stamp);
    cpu_hst_index = 0;

    memzero(sim_brk_pend);
    memzero(sim_brk_ploc);
    sim_brk_act = NULL;

    memzero(cpu_rtc_ticks);
    memzero(cpu_rtc_hz);
    memzero(cpu_rtc_rtime);
    memzero(cpu_rtc_vtime);
    memzero(cpu_rtc_nxintv);
    memzero(cpu_rtc_based);
    memzero(cpu_rtc_currd);
    memzero(cpu_rtc_initd);
    memzero(cpu_rtc_elapsed);

    cpu_idle_sleep_us = 0;
    cpu_idle_sleep_cycles = 0;

    cpu_ssc_delta_timer[0] = NULL;
    cpu_ssc_delta_timer[1] = NULL;

    clk_active = FALSE;
    cpu_last_synclk_cycles = 0;
    cpu_last_tslice_tick_cycles = 0;
    cpu_last_second_tick_cycles = 0;

    sysd_active_mask = 0;

    cpu_active_clk_interrupt = FALSE;
    cpu_active_ipi_interrupt = FALSE;

    cpu_synclk_protect = FALSE;
    cpu_synclk_protect_os = 0;
    cpu_synclk_protect_dev = 0;
    cpu_synclk_pending = SynclkNotPending;

    cpu_inside_reevaluate_thread_priority = FALSE;
    cpu_redo_reevaluate_thread_priority = FALSE;

    smp_var(cpu_sleeping) = 0;

    cpu_wakeup_event = NULL;
    cpu_run_gate = NULL;
    cpu_thread = SMP_THREAD_NULL;
    cpu_thread_created = FALSE;
    cpu_thread_priority = SIMH_THREAD_PRIORITY_INVALID;

    cpu_requeue_syswide_pending = FALSE;

    syncw_active = 0;
    syncw_countdown = 0;
    syncw_countdown_start = 0;
    syncw_countdown_sys = 0;
    syncw_countdown_ilk = 0;
    syncw_wait_event = NULL;
    syncw_wait_cpu_id = NO_CPU_ID;

    cpu_con_rei_on = FALSE;
}

/*
 * dynamic part of initializer, can fail and throw exceptions
 */
void CPU_UNIT::init(uint8 cpu_id, uint8 cpu_state)
{
    check_aligned(this, SMP_MAXCACHELINESIZE);
    smp_check_aligned(& cpu_adv_cycles);
    this->unitno = cpu_id;
    this->cpu_id = cpu_id;
    this->cpu_state = cpu_state;
    cpu_exception_ABORT = new sim_exception_ABORT(0, TRUE);
    cpu_intreg.init(IPL_HMIN, IPL_HMAX, devs_per_irql);
    init_clock_queue();
    sim_step = 0;
    sim_instrs = 0;
    cpu_ssc_delta_timer[0] = sim_delta_timer::create();
    cpu_ssc_delta_timer[1] = sim_delta_timer::create();
    cpu_wakeup_event = smp_event::create();
    this->cpu_run_gate = smp_semaphore::create(0);
    syncw_wait_event = smp_event::create();
    smp_create_thread(sim_cpu_work_thread_proc, this, & this->cpu_thread);
    cpu_thread_created = TRUE;
    cpu_context.reset(this);
}

static inline uint32 ROUNDUP(uint32 n, uint32 r)
{
    return ((n + r - 1) / r) * r;
}

void CPU_UNIT::init_clock_queue()
{
    this->clock_queue_freelist = NULL;
    this->clock_queue = NULL;

    /* two extra entries: one for throttle unit, another for step unit */
    int nentries = sim_units_percpu + sim_units_global + 2;
    int entrysize = ROUNDUP(sizeof(clock_queue_entry), __SIZEOF_POINTER__);
    t_byte* mp = (t_byte*) malloc_aligned(nentries * entrysize, __SIZEOF_POINTER__);
    if (mp == NULL)
        panic("Unable to allocate memory");
    clock_queue_entry* tail = NULL;
    for (int k = 0;  k < nentries;  k++)
    {
        clock_queue_entry* ep = (clock_queue_entry*) (mp + (k * entrysize));
        ep->next = NULL;
        if (tail)
        {
            tail->next = ep;
        }
        else
        {
            this->clock_queue_freelist = ep;
        }
        tail = ep;
    }
}

/* CPU data structures

   cpu_dev      CPU device descriptor
   cpu_unit     CPU unit
   cpu_reg      CPU register list
   cpu_mod      CPU modifier list
*/

CPU_UNIT cpu_unit_0;
CPU_UNIT* cpu_units[SIM_MAX_CPUS] = { & cpu_unit_0 };
UNIT* cpu_units_as_units[SIM_MAX_CPUS]  = { & cpu_unit_0 };

void init_cpu_unit_0()
{
    /*
     * Verify that a pointer to CPU_UNIT will hold the same memory address as the pointer to its base class UNIT.
     */
    if ((void*) (CPU_UNIT*) &cpu_unit_0 != (void*) (UNIT*) &cpu_unit_0)
        panic("Broken assumption: CPU_UNIT casts to UNIT with address change");

    for (int k = 1;  k < SIM_MAX_CPUS;  k++)
    {
        cpu_units[k] = NULL;
        cpu_units_as_units[k] = NULL;
    }

    cpu_unit_0.init(0, CPU_STATE_RUNNABLE);
}

t_bool cpu_create_cpus(uint32 ncpus)
{
    if (ncpus == sim_ncpus)  return TRUE;
    if (ncpus < sim_ncpus || ncpus > SIM_MAX_CPUS)  return FALSE;

    if (hst_lnt)
    {
        smp_printf ("Warning: Stopping history recording ...\n");
        if (sim_log)
            fprintf (sim_log, "Warning: Stopping history recording ...\n");
        cpu_free_history ();
    }

    for (uint32 k = sim_ncpus;  k < ncpus;  k++)
    {
        CPU_UNIT* cpu = new CPU_UNIT();
        cpu->capac = cpu_unit_0.capac;
        cpu->flags |= cpu_unit_0.flags & UNIT_CONH;

        cpu->init(k, CPU_STATE_STANDBY);

        cpu_units[k] = cpu;
        cpu_units_as_units[k] = cpu;
        smp_wmb();
        sim_ncpus++;
        smp_wmb();

        reset_cpu_and_its_devices (cpu);
    }

    return TRUE;
}

/*
 * For CPU0: reset the CPU and all devices, both per-CPU and global.
 *
 * For other CPUs: reset this CPU and its per-CPU devices, but not global devices.
 *
 * It is crucial that devices are reset in the order listed in sim_devices.
 * Otherwise system is not properly initialized and, at best, boot ROM tests fail.
 */
t_stat reset_cpu_and_its_devices (CPU_UNIT* cpu_unit)
{
    t_stat reason = SCPE_OK;
    DEVICE *dptr;

    run_scope_context* rscx = run_scope_context::get_current();
    CPU_UNIT* sv_cpu_unit = rscx->cpu_unit;
    rscx->cpu_unit = cpu_unit;

    for (int k = 0; (dptr = sim_devices[k]) != NULL; k++)
    {
        if (cpu_unit->is_primary_cpu() || (dptr->flags & DEV_PERCPU))
        {
            if ((reason = reset_dev_thiscpu (dptr)) != SCPE_OK)
            {
                break;
            }
        }
    }

    cpu_unit->cpu_state = CPU_STATE_STANDBY;
    cpu_running_set.clear(cpu_unit->cpu_id);
    sim_mp_active_update();

    rscx->cpu_unit = sv_cpu_unit;

    smp_wmb();

    return reason;
}

const char* cpu_describe_state(CPU_UNIT* cpu_unit)
{
    switch (cpu_unit->cpu_state)
    {
    case CPU_STATE_STANDBY:
        return "STANDBY";
    case CPU_STATE_RUNNABLE:
        return "RUNNABLE";
    case CPU_STATE_RUNNING:
        return "RUNNING";
    default:
        return "UNKNOWN STATE";
    }
}

REG cpu_reg[] = {
    { HRDATA_CPU ("PC", r_R[nPC], 32) },
    { HRDATA_CPU ("R0", r_R[0], 32) },
    { HRDATA_CPU ("R1", r_R[1], 32) },
    { HRDATA_CPU ("R2", r_R[2], 32) },
    { HRDATA_CPU ("R3", r_R[3], 32) },
    { HRDATA_CPU ("R4", r_R[4], 32) },
    { HRDATA_CPU ("R5", r_R[5], 32) },
    { HRDATA_CPU ("R6", r_R[6], 32) },
    { HRDATA_CPU ("R7", r_R[7], 32) },
    { HRDATA_CPU ("R8", r_R[8], 32) },
    { HRDATA_CPU ("R9", r_R[9], 32) },
    { HRDATA_CPU ("R10", r_R[10], 32) },
    { HRDATA_CPU ("R11", r_R[11], 32) },
    { HRDATA_CPU ("R12", r_R[12], 32) },
    { HRDATA_CPU ("R13", r_R[13], 32) },
    { HRDATA_CPU ("R14", r_R[14], 32) },
    { HRDATA_CPU ("AP", r_R[nAP], 32) },
    { HRDATA_CPU ("FP", r_R[nFP], 32) },
    { HRDATA_CPU ("SP", r_R[nSP], 32) },
    { HRDATA_CPU ("PSL", r_PSL, 32) },
    { HRDATA_CPU ("CC", r_PSL, 4) },
    { HRDATA_CPU ("KSP", r_STK[KERN], 32) },
    { HRDATA_CPU ("ESP", r_STK[EXEC], 32) },
    { HRDATA_CPU ("SSP", r_STK[SUPV], 32) },
    { HRDATA_CPU ("USP", r_STK[USER], 32) },
    { HRDATA_CPU ("IS", r_STK[4], 32) },
    { HRDATA_CPU ("SCBB", r_SCBB, 32) },
    { HRDATA_CPU ("PCBB", r_PCBB, 32) },
    { HRDATA_CPU ("P0BR", r_P0BR, 32) },
    { HRDATA_CPU ("P0LR", r_P0LR, 22) },
    { HRDATA_CPU ("P1BR", r_P1BR, 32) },
    { HRDATA_CPU ("P1LR", r_P1LR, 22) },
    { HRDATA_CPU ("SBR", r_SBR, 32) },
    { HRDATA_CPU ("SLR", r_SLR, 22) },
    { HRDATA_CPU ("SISR", r_SISR, 16) },
    { HRDATA_CPU ("ASTLVL", r_ASTLVL, 4) },
    { FLDATA_CPU ("MAPEN", r_mapen, 0) },
    { FLDATA_CPU ("PME", r_pme, 0) },
    { HRDATA_CPU ("TRPIRQ", r_trpirq, 8) },
    { IRDATA_LVL (IPL17, 17, 1), REG_RO },
    { IRDATA_LVL (IPL16, 16, 1), REG_RO },
    { IRDATA_LVL (IPL15, 15, 1), REG_RO },
    { IRDATA_LVL (IPL14, 14, 1), REG_RO },
    { FLDATA_CPU ("CRDERR", r_crd_err, 0) },
    { FLDATA_CPU ("MEMERR", r_mem_err, 0) },
    { FLDATA_GBL (HLTPIN, hlt_pin, 0) },
    { HRDATA_GBL (IDLE_MASK, cpu_idle_mask, 16), REG_HIDDEN },
    { DRDATA_GBL (IDLE_INDX, cpu_idle_type, 4), REG_HRO },
    { DRDATA_GBL (IDLE_ENAB, sim_idle_enab, 4), REG_HRO },
    { BRDATA_CPU ("PCQ", r_pcq, 16, 32, PCQ_SIZE), REG_RO+REG_CIRC, BRDATA_CPU_QPTR(r_pcq_qptr) },
    { HRDATA_CPU ("PCQP", r_pcq_p, 6), REG_HRO },
    { HRDATA_GBL (WRU, sim_int_char, 8) },
    { HRDATA_CPU ("BADABO", r_badabo, 32), REG_HRO },
    // { HRDATA_CPU ("CQBIC_SCR", r_cq_scr, 16) },
    // { HRDATA_CPU ("CQBIC_DSER", r_cq_dser, 8) },
    // { HRDATA_CPU ("CQBIC_MEAR", r_cq_mear, 13) },
    // { HRDATA_CPU ("CQBIC_SEAR", r_cq_sear, 20) },
    { HRDATA_DYN("SIRR", 4, reg_sirr_rd, reg_sirr_wr, NULL), REG_HIDDEN },
    { HRDATA_GBL (SYNCLK_SAFE_IPL, synclk_safe_ipl, 5), REG_HIDDEN },
    { HRDATA_GBL (SYNCLK_SAFE_CYCLES, synclk_safe_cycles, 32), REG_HIDDEN },
    { HRDATA_GBL (BUGCHECK_CTRL, bugcheck_ctrl, 32), REG_HIDDEN },
    { HRDATA_GBL (VCPU_PER_CORE, sim_vcpu_per_core, 1), REG_HIDDEN },
    { HRDATA_DYN("WS_MIN", 32, ws_min_rd, ws_min_wr, NULL), REG_HIDDEN },
    { HRDATA_DYN("WS_MAX", 32, ws_max_rd, ws_max_wr, NULL), REG_HIDDEN },
    { HRDATA_DYN("WS_LOCK", 1, ws_lock_rd, ws_lock_wr, NULL), REG_HIDDEN },
    { HRDATA_GBL (HOST_DEDICATED, sim_host_dedicated, 1), REG_HIDDEN },
    { HRDATA_GBL_RDX(HOST_TURBO, sim_host_turbo, 10, 10), REG_HIDDEN },
    { NULL }
};

MTAB cpu_mod[] = {
    { UNIT_CONH, 0, "HALT to SIMH", "SIMHALT", NULL },
    { UNIT_CONH, UNIT_CONH, "HALT to console", "CONHALT", NULL },
    { MTAB_XTD|MTAB_VDV, 0, "IDLE", "IDLE", &cpu_set_idle, &cpu_show_idle },
    { MTAB_XTD|MTAB_VDV, 0, NULL, "NOIDLE", &sim_clr_idle, NULL },
    /* bit flags 23...29 are also handled in cpu_sync_flags */
    { UNIT_MSIZE, (1u << 23), NULL, "8M", &cpu_set_size },
    { UNIT_MSIZE, (1u << 24), NULL, "16M", &cpu_set_size },
    { UNIT_MSIZE, (1u << 25), NULL, "32M", &cpu_set_size },
    { UNIT_MSIZE, (1u << 25) + (1u << 24), NULL, "48M", &cpu_set_size },
    { UNIT_MSIZE, (1u << 26), NULL, "64M", &cpu_set_size },
    { UNIT_MSIZE, (1u << 27), NULL, "128M", &cpu_set_size },
#if !defined (VAX_780)
    { UNIT_MSIZE, (1u << 28), NULL, "256M", &cpu_set_size },
    { UNIT_MSIZE, (1u << 29), NULL, "512M", &cpu_set_size },
#endif
    { MTAB_XTD|MTAB_VDV|MTAB_NMO|MTAB_SHP, 0, "HISTORY", "HISTORY",
      &cpu_set_hist, &cpu_show_hist },
    { MTAB_XTD|MTAB_VDV|MTAB_NMO|MTAB_SHP, 0, "VIRTUAL", NULL,
      NULL, &cpu_show_virt },
    { 0 }
};

/* after console sets flags for this CPU, replicate them to other CPUs */
void cpu_sync_flags(CPU_UNIT* uptr)
{
    uint32 mask = UNIT_CONH;
    for (uint32 k = 23;  k <= 29;  k++)
        mask |= (1 << k);

    for (uint32 k = 0;  k < sim_ncpus;  k++)
    {
        CPU_UNIT* xcpu = cpu_units[k];
        if (xcpu != uptr)
        {
            xcpu->flags = (xcpu->flags & ~mask) | (uptr->flags & mask);
        }
    }
}

DEBTAB cpu_deb[] = {
    { "INTEXC", LOG_CPU_I },
    { "REI", LOG_CPU_R },
    { "CONTEXT", LOG_CPU_P },
    { NULL, 0 }
};

DEVICE cpu_dev = {
    "CPU", cpu_units_as_units, cpu_reg, cpu_mod,
    1, 16, 32, 1, 16, 8,
    &cpu_ex, &cpu_dep, &cpu_reset,
    &cpu_boot, NULL, NULL,
    NULL, DEV_DYNM | DEV_DEBUG | DEV_PERCPU, 0,
    cpu_deb, &cpu_set_size, NULL
};


/*
 * Instruction stream prefetch routines.
 * Special inline-able cases of get_istr for 1/2/4 byte fetches from the instruction stream.
 *
 * VAX_DIRECT_PREFETCH is streamlined for direct fetching from memory with minimum overhead
 * when PC is in ADDR_IS_MEM range.
 *
 * When VAX_DIRECT_PREFETCH is enabled, prefetch can be in one of three modes:
 *
 * 1) Fetching from ADDR_IS_MEM range.
 *
 *        mppc_rem > 0
 *        mppc points to next byte to be fetched from M
 *
 *        ibcnt = 0
 *        ppc = -1
 *
 * 2) Fetching from any range other than MEM, such as ROM or IO range.
 *    Will also work for MEM range, but less efficient than mode 1.
 *
 *        mppc_rem = 0
 *        mppc = invalid
 *
 *        ibcnt >= 0
 *        ppc != -1
 *
 * 3) Fetching is in undefined mode and need to be flipped either to mode 1 or mode 2.
 *
 *        mppc_rem = 0
 *        mppc = invalid
 *
 *        ibcnt = 0
 *        ppc = -1
 *
 */
#if VAX_DIRECT_PREFETCH == 0
SIM_INLINE static int32 get_istr_b (RUN_DECL, int32 acc)
{
    int32 bo = PC & 3;
    const int32 lnt = L_BYTE;

    while (bo + lnt > ibcnt)
    {
        if (ppc < 0 || VA_GETOFF (ppc) == 0)
        {
            return get_istr (RUN_PASS, lnt, acc);
        }
        if (ibcnt == 0)
            ibufl = ReadLP (RUN_PASS, ppc);
        else
            ibufh = ReadLP (RUN_PASS, ppc);
        ppc = ppc + 4;
        ibcnt = ibcnt + 4;
    }

    PC += lnt;
    int32 val = (ibufl >> (bo << 3)) & BMASK;

    if (bo + lnt >= 4)
    {
        ibufl = ibufh;
        ibcnt = ibcnt - 4;
    }

    return val;
}

SIM_INLINE static int32 get_istr_w (RUN_DECL, int32 acc)
{
    int32 bo = PC & 3;
    int32 val;
    const int32 lnt = L_WORD;

    while (bo + lnt > ibcnt)
    {
        if (ppc < 0 || VA_GETOFF (ppc) == 0)
        {
            return get_istr (RUN_PASS, lnt, acc);
        }
        if (ibcnt == 0)
            ibufl = ReadLP (RUN_PASS, ppc);
        else
            ibufh = ReadLP (RUN_PASS, ppc);
        ppc = ppc + 4;
        ibcnt = ibcnt + 4;
    }

    PC += lnt;

    if (bo == 3)
        val = ((ibufl >> 24) & 0xFF) | ((ibufh & 0xFF) << 8);
    else
        val = (ibufl >> (bo << 3)) & WMASK;

    if (bo + lnt >= 4)
    {
        ibufl = ibufh;
        ibcnt = ibcnt - 4;
    }

    return val;
}

SIM_INLINE static int32 get_istr_l (RUN_DECL, int32 acc)
{
    int32 bo = PC & 3;
    int32 val;
    const int32 lnt = L_LONG;

    while (bo + lnt > ibcnt)
    {
        if (ppc < 0 || VA_GETOFF (ppc) == 0)
        {
            return get_istr (RUN_PASS, lnt, acc);
        }
        if (ibcnt == 0)
            ibufl = ReadLP (RUN_PASS, ppc);
        else
            ibufh = ReadLP (RUN_PASS, ppc);
        ppc = ppc + 4;
        ibcnt = ibcnt + 4;
    }

    PC += lnt;

    if (bo)
    {
        int32 sc = bo << 3;
        val = (((ibufl >> sc) & align[bo]) | (((uint32) ibufh) << (32 - sc)));
    }
    else
    {
        val = ibufl;
    }

    if (bo + lnt >= 4)
    {
        ibufl = ibufh;
        ibcnt = ibcnt - 4;
    }

    return val;
}
#endif // VAX_DIRECT_PREFETCH

SIM_INLINE static int32 get_istr_x (RUN_DECL, int32 lnt, int32 acc)
{
    int32 bo = PC & 3;
    int32 val;

    while (bo + lnt > ibcnt)
    {
        if (ppc < 0 || VA_GETOFF (ppc) == 0)
        {
            return get_istr (RUN_PASS, lnt, acc);
        }
        if (ibcnt == 0)
            ibufl = ReadLP (RUN_PASS, ppc);
        else
            ibufh = ReadLP (RUN_PASS, ppc);
        ppc = ppc + 4;
        ibcnt = ibcnt + 4;
    }

    PC += lnt;

    if (lnt == L_BYTE)
    {
        val = (ibufl >> (bo << 3)) & BMASK;
    }
    else if (lnt == L_WORD)
    {
        if (bo == 3)
            val = ((ibufl >> 24) & 0xFF) | ((ibufh & 0xFF) << 8);
        else
            val = (ibufl >> (bo << 3)) & WMASK;
    }
    else if (bo)                                           /* unaligned lw? */
    {
        int32 sc = bo << 3;
        val = (((ibufl >> sc) & align[bo]) | (((uint32) ibufh) << (32 - sc)));
    }
    else
    {
        val = ibufl;                                        /* aligned lw */
    }

    if (bo + lnt >= 4)
    {
        ibufl = ibufh;
        ibcnt = ibcnt - 4;
    }

    return val;
}

#if VAX_DIRECT_PREFETCH == 1
static int32 get_istr_x_ni (RUN_DECL, int32 lnt, int32 acc);
static int32 get_istr_x2 (RUN_DECL, int32 lnt, int32 acc);

SIM_INLINE static int32 get_istr_b_dir (RUN_DECL, int32 acc)
{
    const int32 lnt = L_BYTE;
    int32 val;
    if (likely(mppc_rem >= lnt))
    {
        val = * (t_byte*) mppc;
        PC += lnt;
        mppc += lnt;
        mppc_rem -= lnt;
    }
    else
    {
        val = get_istr_x_ni(RUN_PASS, lnt, acc);
    }
    return val;
}

SIM_INLINE static int32 get_istr_w_dir (RUN_DECL, int32 acc)
{
    const int32 lnt = L_WORD;
    int32 val;
    if (likely(mppc_rem >= lnt))
    {
        val = * (uint16*) mppc;
        PC += lnt;
        mppc += lnt;
        mppc_rem -= lnt;
    }
    else
    {
        val = get_istr_x_ni(RUN_PASS, lnt, acc);
    }
    return val;
}

SIM_INLINE static int32 get_istr_l_dir (RUN_DECL, int32 acc)
{
    const int32 lnt = L_LONG;
    int32 val;
    if (likely(mppc_rem >= lnt))
    {
        val = * (uint32*) mppc;
        PC += lnt;
        mppc += lnt;
        mppc_rem -= lnt;
    }
    else
    {
        val = get_istr_x_ni(RUN_PASS, lnt, acc);
    }
    return val;
}

SIM_INLINE static int32 get_istr_x_dir (RUN_DECL, int32 lnt, int32 acc)
{
    if (likely(mppc_rem >= lnt))
    {
        int32 val;
        switch (lnt)
        {
        case L_BYTE:
            val = * (t_byte*) mppc;
            PC += L_BYTE;
            mppc += L_BYTE;
            mppc_rem -= L_BYTE;
            return val;

        case L_WORD:
            val = * (uint16*) mppc;
            PC += L_WORD;
            mppc += L_WORD;
            mppc_rem -= L_WORD;
            return val;

        case L_LONG:
            val = * (uint32*) mppc;
            PC += L_LONG;
            mppc += L_LONG;
            mppc_rem -= L_LONG;
            return val;

        default:
            /* never happens, just to suppress false compiler warning */
            return 0;
        }
    }
    else
    {
        return get_istr_x2 (RUN_PASS, lnt, acc);
    }
}

static int32 get_istr_x2 (RUN_DECL, int32 lnt, int32 acc)
{
    int32 val = 0;
    t_byte* valp = (t_byte*) & val;

    if (likely(ppc < 0))
    {
        /* get_istr_x2 is called only when mppc_rem < 4 */
incomplete_again:
        switch (mppc_rem)
        {
        case 3:
            *valp++ = *mppc++;
            /* fall through */
        case 2:
            *valp++ = *mppc++;
            /* fall through */
        case 1:
            *valp++ = *mppc++;
            /* fall through */

            PC += mppc_rem;
            lnt -= mppc_rem;
            mppc = NULL;
            mppc_rem = 0;
            break;

        case 0:
            break;
        }

        int32 pa = Test (RUN_PASS, PC, RA, NULL);
        if (likely(ADDR_IS_MEM(pa)))
        {
            mppc = (t_byte*) M + pa;
            mppc_rem = VA_PAGSIZE - (pa & VA_M_OFF);
            if (mppc_rem < lnt)  goto incomplete_again;

            switch (lnt)
            {
            case 4:
                *valp++ = *mppc++;
                /* fall through */
            case 3:
                *valp++ = *mppc++;
                /* fall through */
            case 2:
                *valp++ = *mppc++;
                /* fall through */
            case 1:
                *valp++ = *mppc++;
                /* fall through */

                PC += lnt;
                mppc_rem -= lnt;
                break;
            }

            return val;
        }

        /* 
         * Detected crossover from RAM to non-RAM space.
         * Should normally never happen within an instruction, but just in case it did, back up.
         */
        int32 initial_mppc_rem = (int32) (valp - (t_byte*) & val);
        PC -= initial_mppc_rem;
        lnt += initial_mppc_rem;
    }

    mppc_rem = 0;
    mppc = NULL;

    return get_istr_x (RUN_PASS, lnt, acc);
}

static int32 get_istr_x_ni (RUN_DECL, int32 lnt, int32 acc)
{
    return get_istr_x_dir (RUN_PASS, lnt, acc);
}

void cpu_branch(RUN_DECL, int32 d)
{
    int32 newPC = PC + d;

    if (mppc_rem && (PC & ~VA_M_OFF) == (newPC & ~VA_M_OFF))
    {
        mppc += d;
        mppc_rem -= d;
    }
    else
    {
        FLUSH_ISTR;
    }

    PC = newPC;
}
#endif // VAX_DIRECT_PREFETCH


/*
 * Instruction loop
 */

#if defined(__GNUC__)
/* 
 * Suppress false GCC warning about the following variables being possibly uninitialized:
 *
 *     spec, rn, vfldrp1, brdisp, j, va
 *
 * We could have initialized them to 0 solely to suppress warning, but since they are inside
 * innermost loop, we do not want to do this, for performance sake (yes, measurements show
 * it does matter... to a surprisingly greater extent one would have expected).
 *
 * We could have also moved their declaration out of the loop into "try" block and initialized
 * there, but better keep them inside the inner loop to help the optimizer.
 *
 * Since GCC does not have selective per-variable suppression of messages, we disable messages
 * for the whole routine.
 */
#  if (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__ >= 40603)
#    pragma GCC diagnostic push
#    pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#  else
#    pragma message ""
#    pragma message "    Please ignore compiler's false warning that variables"
#    pragma message "        spec, rn, vfldrp1, brdisp, j, va"
#    pragma message "    may be used uninitialized in function sim_instr"
#    pragma message ""
#  endif
#endif

t_stat sim_instr (RUN_DECL)
{
/*
 * Unlike with C setjmp/longjmp or C try/catch/finally macro-based exception handling,
 * there is no requirement in C++ (ISO/ANSI 98) that automatic variables changed in the "try" block
 * and used in the "catch" block should be declared volatile.
 * We leave volatile qualifier nevertheless, to avoid the risk of running into possible bugs in a compiler.
 */
sim_try_volatile int32 opc;                             /* used by sim_catch_all, hence declared volatile */
sim_try_volatile int32 cc;                              /* ... */
int32 acc;                                              /* set by catch (...) */

if ((PSL & PSL_MBZ) ||                                  /* validate PSL<mbz> */
    ((PSL & PSL_CM) && BadCmPSL (RUN_PASS, PSL)) ||     /* validate PSL<cm> */
    ((PSL_GETCUR (PSL) != KERN) &&                      /* esu => is, ipl = 0 */
        (PSL & (PSL_IS|PSL_IPL))) ||
    ((PSL & PSL_IS) && ((PSL & PSL_IPL) == 0)))         /* is => ipl > 0 */
{
    smp_printf("Invalid PSL\n");
    if (sim_log)
        fprintf (sim_log, "Invalid PSL\n");
    return SCPE_STOP;
}
cc = PSL & CC_MASK;                                     /* split PSL */
PSL = PSL & ~CC_MASK;
in_ie = 0;                                              /* not in exc */
set_map_reg (RUN_PASS);                                 /* set map reg */
GET_CUR;                                                /* set access mask */
SET_IRQL;                                               /* eval interrupts */

/* Main instruction loop */
main_loop:

    FLUSH_ISTR;                                         /* clear prefetch */

sim_try
{
    for (;;)
    {
        int32 spec, disp, rn, index, numspec;
        int32 vfldrp1, brdisp, flg, mstat;
        int32 i, j, r, rh, temp;
        uint32 va, iad;
        int32 opnd[OPND_SIZE];                              /* operand queue */

        // if (cpu_astop) {
        //     cpu_astop = 0;
        //     ABORT_INVALID_SYSOP;
        // }

        fault_PC = PC;
        recqptr = 0;                                        /* clr recovery q */

        if (unlikely(cpu_unit->sim_step) &&                 /* check for step condition */
            cpu_unit->sim_step == cpu_unit->sim_instrs)
        {
            ABORT (SCPE_STEP);
        }

        if (unlikely(cpu_unit->cpu_synclk_protect))
        {
            if (likely(cpu_unit->cpu_synclk_protect_os))
                cpu_unit->cpu_synclk_protect_os--;

            /* 
             * we count down cpu_synclk_protect_dev by VAX instruction, not cycle, so sometimes
             * it can be excessive, but not by much
             */
            if (likely(cpu_unit->cpu_synclk_protect_dev))
                cpu_unit->cpu_synclk_protect_dev--;

            if (unlikely(cpu_unit->cpu_synclk_protect_os == 0 && cpu_unit->cpu_synclk_protect_dev == 0))
            {
                cpu_unit->cpu_synclk_protect = FALSE;
                check_synclk_pending(RUN_PASS);
            }
        }

        if (unlikely(weak_read(stop_cpus)))                 /* stop pending */
            ABORT (SCPE_STOP);

        if (unlikely(sim_interval <= 0))                    /* chk clock queue */
        {
            temp = sim_process_event (RUN_PASS);
            if (temp)
                ABORT (temp);
            SET_IRQL;                                       /* update interrupts */
        }
        else if (unlikely(cpu_unit->cpu_intreg.weak_changed()))   /* weak read check: possible change in interrupt register state */
        {
            SET_IRQL;                                       /* update interrupts */
        }

        /* Test for non-instruction dispatches, in SRM order

                - trap or interrupt (trpirq != 0)
                - PSL<tp> set

           If any of these conditions are met, re-dispatch; otherwise,
           set PSL<tp> from PSL<t>.
        */

        if (trpirq)                                         /* trap or interrupt? */
        {
            if (temp = GET_TRAP (trpirq))                   /* trap? */
            {
                cc = intexc (RUN_PASS, SCB_ARITH, cc, 0, IE_EXC);     /* take, clear trap */
                GET_CUR;                                    /* set cur mode */
                in_ie = 1;
                Write (RUN_PASS, SP - 4, temp, L_LONG, WA); /* write parameter */
                SP = SP - 4;
                in_ie = 0;
                SET_IRQL_NOSYNC;                            /* eval interrupts */
            }
            else if (temp = GET_IRQL (trpirq))              /* interrupt? */
            {
                int32 vec = 0;                              /* init vec to suppress false GCC warning */
                if (temp == IPL_HLTPIN)                     /* console halt? */
                {
                    // hlt_pin = 0;                         /* clear intr: in SMP version cleared in the console loop or in con_halt */
                    trpirq = 0;                             /* clear everything */
                    cc = con_halt (CON_HLTPIN, cc);         /* invoke firmware */
                    SET_IRQL;                               /* eval interrupts */
                    continue;                               /* continue */
                }
                else if (temp >= IPL_HMIN)                  /* hardware req? */
                    vec = get_vector (RUN_PASS, temp);      /* get vector */
                else if (temp > IPL_SMAX)
                    ABORT (STOP_UIPL);
                else
                {
                    vec = SCB_IPLSOFT + (temp << 2);
                    SISR = SISR & ~(1u << temp);
                }
                if (vec)                                    /* take intr */
                    cc = intexc (RUN_PASS, vec, cc, temp, IE_INT);
                GET_CUR;                                    /* set cur mode */
                SET_IRQL;                                   /* eval interrupts */
            }
            else
            {
                trpirq = 0;                                 /* clear everything */
                SET_IRQL;                                   /* eval interrupts */
            }
            continue;
        }

        if (PSL & (PSL_CM|PSL_TP|PSW_T))                    /* PSL event? */
        {
            if (PSL & PSL_TP)                              /* trace trap? */
            {
                PSL = PSL & ~PSL_TP;                        /* clear <tp> */
                cc = intexc (RUN_PASS, SCB_TP, cc, 0, IE_EXC);        /* take trap */
                GET_CUR;                                    /* set cur mode */
                continue;
            }
            if (PSL & PSW_T)                                /* if T, set TP */
                PSL = PSL | PSL_TP;
            if (PSL & PSL_CM)                               /* compat mode? */
            {
                cc = op_cmode (RUN_PASS, cc);               /* exec instr */
                continue;                                   /* skip fetch */
            }
        }                                                   /* end PSL event */

        if (sim_brk_summ && sim_brk_test (RUN_PASS, (uint32) PC, SWMASK ('E')))       /* breakpoint? */
        {
            ABORT (STOP_IBKPT);                             /* stop simulation */
        }

        if (unlikely(0 == --cpu_unit->syncw_countdown))     /* check synch window */
        {
            if ((temp = syncw_checkinterval(RUN_PASS, FALSE)) != SCPE_OK)
                ABORT (temp);
        }

        cpu_cycle();                                        /* count cycles */
        cpu_unit->sim_instrs++;                             /* ... and instructions */
        GET_ISTR_B (opc);                                   /* get opcode */
        if (opc == 0xFD)                                    /* 2 byte op? */
        {
            GET_ISTR_B (opc);                               /* get second byte */
            opc = opc | 0x100;                              /* flag */
        }
        numspec = drom[opc][0];                             /* get # specs */
        if (PSL & PSL_FPD) {
            if ((numspec & DR_F) == 0)
                RSVD_INST_FAULT;
        }
        else
        {
            numspec = numspec & DR_NSPMASK;                 /* get # specifiers */

        /* Specifier flows.  Operands are parsed and placed into queue opnd.

                r.bwl   opnd[j]         =       value of operand
                r.q     opnd[j:j+1]     =       value of operand
                r.o     opnd[j:j+3]     =       value of operand
                a.bwlqo opnd[j]         =       address of operand
                m.bwl   opnd[j]         =       value of operand
                m.q     opnd[j:j+1]     =       value of operand 
                m.o     opnd[j:j+3]     =       value of operand
                w.bwlqo opnd[j]         =       register/memory flag
                        opnd[j+1]       =       memory address

           For the last memory specifier, the specifier is in spec, the register
           number is in rn, and the effective address is in va.  Modify specifiers
           (always last) can test spec > reg+PC, as short literal are illegal for
           modifiers specifiers, and final index specifiers are always illegal.
        */

            for (i = 1, j = 0; i <= numspec; i++) {         /* loop thru specs */
                disp = drom[opc][i];                        /* get dispatch */
                if (disp >= BB) {
                    GET_ISTR_X (brdisp, DR_LNT (disp & 1));
                    break;
                    }
                GET_ISTR_B (spec);                          /* get spec byte */
                rn = spec & RGMASK;                         /* get reg # */
                disp = (spec & ~RGMASK) | disp;             /* merge w dispatch */
                switch (disp) {                             /* dispatch spec */

        /* Short literal - only read access permitted */

                case SH0|RB: case SH0|RW: case SH0|RL:
                case SH1|RB: case SH1|RW: case SH1|RL:
                case SH2|RB: case SH2|RW: case SH2|RL:
                case SH3|RB: case SH3|RW: case SH3|RL:
                    opnd[j++] = spec;
                    break;

                case SH0|RQ: case SH1|RQ: case SH2|RQ: case SH3|RQ:
                    opnd[j++] = spec;
                    opnd[j++] = 0;
                    break;

                case SH0|RO: case SH1|RO: case SH2|RO: case SH3|RO:
                    opnd[j++] = spec;
                    opnd[j++] = 0;
                    opnd[j++] = 0;
                    opnd[j++] = 0;
                    break;

                case SH0|RF: case SH1|RF: case SH2|RF: case SH3|RF:
                    opnd[j++] = (spec << 4) | 0x4000;
                    break;

                case SH0|RD: case SH1|RD: case SH2|RD: case SH3|RD:
                    opnd[j++] = (spec << 4) | 0x4000;
                    opnd[j++] = 0;
                    break;

                case SH0|RG: case SH1|RG: case SH2|RG: case SH3|RG:
                    opnd[j++] = (spec << 1) | 0x4000;
                    opnd[j++] = 0;
                    break;

                case SH0|RH: case SH1|RH: case SH2|RH: case SH3|RH:
                    opnd[j++] = ((spec & 0x7) << 29) | (0x4000 | ((spec >> 3) & 0x7));
                    opnd[j++] = 0;
                    opnd[j++] = 0;
                    opnd[j++] = 0;
                    break;

        /* Register */

                case GRN|RB: case GRN|MB:
                    CHECK_FOR_PC;
                    opnd[j++] = R[rn] & BMASK;
                    break;

                case GRN|RW: case GRN|MW:
                    CHECK_FOR_PC;
                    opnd[j++] = R[rn] & WMASK;
                    break;

                case GRN|VB:
                    vfldrp1 = R[(rn + 1) & RGMASK];
                case GRN|WB: case GRN|WW: case GRN|WL: case GRN|WQ: case GRN|WO:
                    opnd[j++] = rn;
                case GRN|RL: case GRN|RF: case GRN|ML:
                    CHECK_FOR_PC;
                    opnd[j++] = R[rn];
                    break;

                case GRN|RQ: case GRN|RD: case GRN|RG: case GRN|MQ:
                    CHECK_FOR_SP;
                    opnd[j++] = R[rn];
                    opnd[j++] = R[rn + 1];
                    break;

                case GRN|RO: case GRN|RH: case GRN|MO:
                    CHECK_FOR_AP;
                    opnd[j++] = R[rn];
                    opnd[j++] = R[rn + 1];
                    opnd[j++] = R[rn + 2];
                    opnd[j++] = R[rn + 3];
                    break;

        /*  Register deferred, autodecrement */

                case RGD|VB:
                case RGD|WB: case RGD|WW: case RGD|WL: case RGD|WQ: case RGD|WO:
                    opnd[j++] = OP_MEM;
                case RGD|AB: case RGD|AW: case RGD|AL: case RGD|AQ: case RGD|AO:
                    CHECK_FOR_PC;
                    va = opnd[j++] = R[rn];
                    break;

                case ADC|VB:
                case ADC|WB: case ADC|WW: case ADC|WL: case ADC|WQ: case ADC|WO:
                    opnd[j++] = OP_MEM;
                case ADC|AB: case ADC|AW: case ADC|AL: case ADC|AQ: case ADC|AO:
                    CHECK_FOR_PC;
                    va = opnd[j++] = R[rn] = R[rn] - DR_LNT (disp);
                    recq[recqptr++] = RQ_REC (disp, rn);
                    break;

                case ADC|RB: case ADC|RW: case ADC|RL: case ADC|RF:
                    R[rn] = R[rn] - (DR_LNT (disp));
                    recq[recqptr++] = RQ_REC (disp, rn);
                case RGD|RB: case RGD|RW: case RGD|RL: case RGD|RF:
                    CHECK_FOR_PC;
                    opnd[j++] = Read (RUN_PASS, va = R[rn], DR_LNT (disp), RA);
                    break;

                case ADC|RQ: case ADC|RD: case ADC|RG:
                    R[rn] = R[rn] - 8;
                    recq[recqptr++] = RQ_REC (disp, rn);
                case RGD|RQ: case RGD|RD: case RGD|RG:
                    CHECK_FOR_PC;
                    opnd[j++] = Read (RUN_PASS, va = R[rn], L_LONG, RA);
                    opnd[j++] = Read (RUN_PASS, R[rn] + 4, L_LONG, RA);
                    break;

                case ADC|RO: case ADC|RH:
                    R[rn] = R[rn] - 16;
                    recq[recqptr++] = RQ_REC (disp, rn);
                case RGD|RO: case RGD|RH:
                    CHECK_FOR_PC;
                    j = ReadOcta (RUN_PASS, va = R[rn], opnd, j, RA);
                    break;

                case ADC|MB: case ADC|MW: case ADC|ML:
                    R[rn] = R[rn] - (DR_LNT (disp));
                    recq[recqptr++] = RQ_REC (disp, rn);
                case RGD|MB: case RGD|MW: case RGD|ML:
                    CHECK_FOR_PC;
                    opnd[j++] = Read (RUN_PASS, va = R[rn], DR_LNT (disp), WA);
                    break;

                case ADC|MQ:
                    R[rn] = R[rn] - 8;
                    recq[recqptr++] = RQ_REC (disp, rn);
                case RGD|MQ:
                    CHECK_FOR_PC;
                    opnd[j++] = Read (RUN_PASS, va = R[rn], L_LONG, WA);
                    opnd[j++] = Read (RUN_PASS, R[rn] + 4, L_LONG, WA);
                    break;

                case ADC|MO:
                    R[rn] = R[rn] - 16;
                    recq[recqptr++] = RQ_REC (disp, rn);
                case RGD|MO:
                    CHECK_FOR_PC;
                    j = ReadOcta (RUN_PASS, va = R[rn], opnd, j, WA);
                    break;

        /* Autoincrement */

                case AIN|VB:
                case AIN|WB: case AIN|WW: case AIN|WL: case AIN|WQ: case AIN|WO:
        /*              CHECK_FOR_PC; */
                    opnd[j++] = OP_MEM;
                case AIN|AB: case AIN|AW: case AIN|AL: case AIN|AQ: case AIN|AO:
                    va = opnd[j++] = R[rn];
                    if (rn == nPC) {
                        if (DR_LNT (disp) >= L_QUAD) {
                            GET_ISTR_L (temp);
                            GET_ISTR_L (temp);
                            if (DR_LNT (disp) == L_OCTA) {
                                GET_ISTR_L (temp);
                                GET_ISTR_L (temp);
                                }
                            }
                        else GET_ISTR_X (temp, DR_LNT (disp));
                        }
                    else {
                        R[rn] = R[rn] + DR_LNT (disp);
                        recq[recqptr++] = RQ_REC (disp, rn);
                        }
                    break;

                case AIN|RB: case AIN|RW: case AIN|RL: case AIN|RF:
                    va = R[rn];
                    if (rn == nPC) {
                        GET_ISTR_X (opnd[j++], DR_LNT (disp));
                        }
                    else {
                        opnd[j++] = Read (RUN_PASS, R[rn], DR_LNT (disp), RA);
                        R[rn] = R[rn] + DR_LNT (disp);
                        recq[recqptr++] = RQ_REC (disp, rn);
                        }
                    break;

                case AIN|RQ: case AIN|RD: case AIN|RG:
                    va = R[rn];
                    if (rn == nPC) {
                        GET_ISTR_L (opnd[j++]);
                        GET_ISTR_L (opnd[j++]);
                        }
                    else {
                        opnd[j++] = Read (RUN_PASS, va, L_LONG, RA);
                        opnd[j++] = Read (RUN_PASS, va + 4, L_LONG, RA);  
                        R[rn] = R[rn] + 8;
                        recq[recqptr++] = RQ_REC (disp, rn);
                        }
                    break;

                case AIN|RO: case AIN|RH:
                    va = R[rn];
                    if (rn == nPC) {
                        GET_ISTR_L (opnd[j++]);
                        GET_ISTR_L (opnd[j++]);
                        GET_ISTR_L (opnd[j++]);
                        GET_ISTR_L (opnd[j++]);
                        }
                    else {
                        j = ReadOcta (RUN_PASS, va, opnd, j, RA);
                        R[rn] = R[rn] + 16;
                        recq[recqptr++] = RQ_REC (disp, rn);
                        }
                    break;

                case AIN|MB: case AIN|MW: case AIN|ML:
                    va = R[rn];
                    if (rn == nPC) {
                        GET_ISTR_X (opnd[j++], DR_LNT (disp));
                        }
                    else {
                        opnd[j++] = Read (RUN_PASS, R[rn], DR_LNT (disp), WA);
                        R[rn] = R[rn] + DR_LNT (disp);
                        recq[recqptr++] = RQ_REC (disp, rn);
                        }
                    break;

                case AIN|MQ:
                    va = R[rn];
                    if (rn == nPC) {
                        GET_ISTR_L (opnd[j++]);
                        GET_ISTR_L (opnd[j++]);
                        }
                    else {
                        opnd[j++] = Read (RUN_PASS, va, L_LONG, WA);
                        opnd[j++] = Read (RUN_PASS, va + 4, L_LONG, WA);  
                        R[rn] = R[rn] + 8;
                        recq[recqptr++] = RQ_REC (disp, rn);
                        }
                    break;

                case AIN|MO:
                    va = R[rn];
                    if (rn == nPC) {
                        GET_ISTR_L (opnd[j++]);
                        GET_ISTR_L (opnd[j++]);
                        GET_ISTR_L (opnd[j++]);
                        GET_ISTR_L (opnd[j++]);
                        }
                    else {
                        j = ReadOcta (RUN_PASS, va, opnd, j, WA);
                        R[rn] = R[rn] + 16;
                        recq[recqptr++] = RQ_REC (disp, rn);
                        }
                    break;

        /* Autoincrement deferred */

                case AID|VB:
                case AID|WB: case AID|WW: case AID|WL: case AID|WQ: case AID|WO:
                    opnd[j++] = OP_MEM;
                case AID|AB: case AID|AW: case AID|AL: case AID|AQ: case AID|AO:
                    if (rn == nPC) {
                        GET_ISTR_L (va = opnd[j++]);
                        }
                    else {
                        va = opnd[j++] = Read (RUN_PASS, R[rn], L_LONG, RA);
                        R[rn] = R[rn] + 4;
                        recq[recqptr++] = RQ_REC (AID|RL, rn);
                        }
                    break;

                case AID|RB: case AID|RW: case AID|RL: case AID|RF:
                    if (rn == nPC) {
                        GET_ISTR_L (va);
                        }
                    else {
                        va = Read (RUN_PASS, R[rn], L_LONG, RA);
                        R[rn] = R[rn] + 4;
                        recq[recqptr++] = RQ_REC (AID|RL, rn);
                        }
                    opnd[j++] = Read (RUN_PASS, va, DR_LNT (disp), RA);
                    break;

                case AID|RQ: case AID|RD: case AID|RG:
                    if (rn == nPC) {
                        GET_ISTR_L (va);
                        }
                    else {
                        va = Read (RUN_PASS, R[rn], L_LONG, RA);
                        R[rn] = R[rn] + 4;
                        recq[recqptr++] = RQ_REC (AID|RL, rn);
                        }
                    opnd[j++] = Read (RUN_PASS, va, L_LONG, RA);
                    opnd[j++] = Read (RUN_PASS, va + 4, L_LONG, RA);
                    break;

                case AID|RO: case AID|RH:
                    if (rn == nPC) {
                        GET_ISTR_L (va);
                        }
                    else {
                        va = Read (RUN_PASS, R[rn], L_LONG, RA);
                        R[rn] = R[rn] + 4;
                        recq[recqptr++] = RQ_REC (AID|RL, rn);
                        }
                    j = ReadOcta (RUN_PASS, va, opnd, j, RA);
                    break;

                case AID|MB: case AID|MW: case AID|ML:
                    if (rn == nPC) {
                        GET_ISTR_L (va);
                        }
                    else {
                        va = Read (RUN_PASS, R[rn], L_LONG, RA);
                        R[rn] = R[rn] + 4;
                        recq[recqptr++] = RQ_REC (AID|RL, rn);
                        }
                    opnd[j++] = Read (RUN_PASS, va, DR_LNT (disp), WA);
                    break;

                case AID|MQ:
                    if (rn == nPC) {
                        GET_ISTR_L (va);
                        }
                    else {
                        va = Read (RUN_PASS, R[rn], L_LONG, RA);
                        R[rn] = R[rn] + 4;
                        recq[recqptr++] = RQ_REC (AID|RL, rn);
                        }
                    opnd[j++] = Read (RUN_PASS, va, L_LONG, WA);
                    opnd[j++] = Read (RUN_PASS, va + 4, L_LONG, WA);
                    break;

                case AID|MO:
                    if (rn == nPC) {
                        GET_ISTR_L (va);
                        }
                    else {
                        va = Read (RUN_PASS, R[rn], L_LONG, RA);
                        R[rn] = R[rn] + 4;
                        recq[recqptr++] = RQ_REC (AID|RL, rn);
                        }
                    j = ReadOcta (RUN_PASS, va, opnd, j, WA);
                    break;

        /* Byte displacement */

                case BDP|VB:
                case BDP|WB: case BDP|WW: case BDP|WL: case BDP|WQ: case BDP|WO:
                    opnd[j++] = OP_MEM;
                case BDP|AB: case BDP|AW: case BDP|AL: case BDP|AQ: case BDP|AO:
                    GET_ISTR_B (temp);
                    va = opnd[j++] = R[rn] + SXTB (temp);
                    break;

                case BDP|RB: case BDP|RW: case BDP|RL: case BDP|RF:
                    GET_ISTR_B (temp);
                    va = R[rn] + SXTB (temp);
                    opnd[j++] = Read (RUN_PASS, va, DR_LNT (disp), RA);
                    break;

                case BDP|RQ: case BDP|RD: case BDP|RG:
                    GET_ISTR_B (temp);        
                    va = R[rn] + SXTB (temp);
                    opnd[j++] = Read (RUN_PASS, va, L_LONG, RA);
                    opnd[j++] = Read (RUN_PASS, va + 4, L_LONG, RA);
                    break;

                case BDP|RO: case BDP|RH:
                    GET_ISTR_B (temp);        
                    va = R[rn] + SXTB (temp);
                    j = ReadOcta (RUN_PASS, va, opnd, j, RA);
                    break;

                case BDP|MB: case BDP|MW: case BDP|ML:
                    GET_ISTR_B (temp);
                    va = R[rn] + SXTB (temp);
                    opnd[j++] = Read (RUN_PASS, va, DR_LNT (disp), WA);
                    break;

                case BDP|MQ:
                    GET_ISTR_B (temp);        
                    va = R[rn] + SXTB (temp);
                    opnd[j++] = Read (RUN_PASS, va, L_LONG, WA);
                    opnd[j++] = Read (RUN_PASS, va + 4, L_LONG, WA);
                    break;

                case BDP|MO:
                    GET_ISTR_B (temp);        
                    va = R[rn] + SXTB (temp);
                    j = ReadOcta (RUN_PASS, va, opnd, j, WA);
                    break;

        /* Byte displacement deferred */

                case BDD|VB:
                case BDD|WB: case BDD|WW: case BDD|WL: case BDD|WQ: case BDD|WO:
                    opnd[j++] = OP_MEM;
                case BDD|AB: case BDD|AW: case BDD|AL: case BDD|AQ: case BDD|AO:
                    GET_ISTR_B (temp);
                    iad = R[rn] + SXTB (temp);
                    va = opnd[j++] = Read (RUN_PASS, iad, L_LONG, RA);
                    break;

                case BDD|RB: case BDD|RW: case BDD|RL: case BDD|RF:
                    GET_ISTR_B (temp);        
                    iad = R[rn] + SXTB (temp);
                    va = Read (RUN_PASS, iad, L_LONG, RA);    
                    opnd[j++] = Read (RUN_PASS, va, DR_LNT (disp), RA);
                    break;

                case BDD|RQ: case BDD|RD: case BDD|RG:
                    GET_ISTR_B (temp);
                    iad = R[rn] + SXTB (temp);
                    va = Read (RUN_PASS, iad, L_LONG, RA);
                    opnd[j++] = Read (RUN_PASS, va, L_LONG, RA);
                    opnd[j++] = Read (RUN_PASS, va + 4, L_LONG, RA);
                    break;  

                case BDD|RO: case BDD|RH:
                    GET_ISTR_B (temp);
                    iad = R[rn] + SXTB (temp);
                    va = Read (RUN_PASS, iad, L_LONG, RA);
                    j = ReadOcta (RUN_PASS, va, opnd, j, RA);
                    break;  

                case BDD|MB: case BDD|MW: case BDD|ML:
                    GET_ISTR_B (temp);        
                    iad = R[rn] + SXTB (temp);
                    va = Read (RUN_PASS, iad, L_LONG, RA);    
                    opnd[j++] = Read (RUN_PASS, va, DR_LNT (disp), WA);
                    break;

                case BDD|MQ:
                    GET_ISTR_B (temp);
                    iad = R[rn] + SXTB (temp);
                    va = Read (RUN_PASS, iad, L_LONG, RA);
                    opnd[j++] = Read (RUN_PASS, va, L_LONG, WA);
                    opnd[j++] = Read (RUN_PASS, va + 4, L_LONG, WA);
                    break;  

                case BDD|MO:
                    GET_ISTR_B (temp);
                    iad = R[rn] + SXTB (temp);
                    va = Read (RUN_PASS, iad, L_LONG, RA);
                    j = ReadOcta (RUN_PASS, va, opnd, j, WA);
                    break;  

        /* Word displacement */

                case WDP|VB:
                case WDP|WB: case WDP|WW: case WDP|WL: case WDP|WQ: case WDP|WO:
                    opnd[j++] = OP_MEM;
                case WDP|AB: case WDP|AW: case WDP|AL: case WDP|AQ: case WDP|AO:
                    GET_ISTR_W (temp);
                    va = opnd[j++] = R[rn] + SXTW (temp);
                    break;

                case WDP|RB: case WDP|RW: case WDP|RL: case WDP|RF:
                    GET_ISTR_W (temp);
                    va = R[rn] + SXTW (temp);
                    opnd[j++] = Read (RUN_PASS, va, DR_LNT (disp), RA);
                    break;

                case WDP|RQ: case WDP|RD: case WDP|RG:
                    GET_ISTR_W (temp);
                    va = R[rn] + SXTW (temp);
                    opnd[j++] = Read (RUN_PASS, va, L_LONG, RA);
                    opnd[j++] = Read (RUN_PASS, va + 4, L_LONG, RA);
                    break;

                case WDP|RO: case WDP|RH:
                    GET_ISTR_W (temp);
                    va = R[rn] + SXTW (temp);
                    j = ReadOcta (RUN_PASS, va, opnd, j, RA);
                    break;

                case WDP|MB: case WDP|MW: case WDP|ML:
                    GET_ISTR_W (temp);
                    va = R[rn] + SXTW (temp);
                    opnd[j++] = Read (RUN_PASS, va, DR_LNT (disp), WA);
                    break;

                case WDP|MQ:
                    GET_ISTR_W (temp);
                    va = R[rn] + SXTW (temp);
                    opnd[j++] = Read (RUN_PASS, va, L_LONG, WA);
                    opnd[j++] = Read (RUN_PASS, va + 4, L_LONG, WA);
                    break;

                case WDP|MO:
                    GET_ISTR_W (temp);
                    va = R[rn] + SXTW (temp);
                    j = ReadOcta (RUN_PASS, va, opnd, j, WA);
                    break;

        /* Word displacement deferred */

                case WDD|VB:
                case WDD|WB: case WDD|WW: case WDD|WL: case WDD|WQ: case WDD|WO:
                    opnd[j++] = OP_MEM;
                case WDD|AB: case WDD|AW: case WDD|AL: case WDD|AQ: case WDD|AO:
                    GET_ISTR_W (temp);
                    iad = R[rn] + SXTW (temp);
                    va = opnd[j++] = Read (RUN_PASS, iad, L_LONG, RA);
                    break;

                case WDD|RB: case WDD|RW: case WDD|RL: case WDD|RF:
                    GET_ISTR_W (temp);
                    iad = R[rn] + SXTW (temp);
                    va = Read (RUN_PASS, iad, L_LONG, RA);
                    opnd[j++] = Read (RUN_PASS, va, DR_LNT (disp), RA);
                    break;

                case WDD|RQ: case WDD|RD: case WDD|RG:
                    GET_ISTR_W (temp);        
                    iad = R[rn] + SXTW (temp);
                    va = Read (RUN_PASS, iad, L_LONG, RA);
                    opnd[j++] = Read (RUN_PASS, va, L_LONG, RA);
                    opnd[j++] = Read (RUN_PASS, va + 4, L_LONG, RA);
                    break;

                case WDD|RO: case WDD|RH:
                    GET_ISTR_W (temp);        
                    iad = R[rn] + SXTW (temp);
                    va = Read (RUN_PASS, iad, L_LONG, RA);
                    j = ReadOcta (RUN_PASS, va, opnd, j, RA);
                    break;

                case WDD|MB: case WDD|MW: case WDD|ML:
                    GET_ISTR_W (temp);
                    iad = R[rn] + SXTW (temp);
                    va = Read (RUN_PASS, iad, L_LONG, RA);
                    opnd[j++] = Read (RUN_PASS, va, DR_LNT (disp), WA);
                    break;

                case WDD|MQ:
                    GET_ISTR_W (temp);        
                    iad = R[rn] + SXTW (temp);
                    va = Read (RUN_PASS, iad, L_LONG, RA);
                    opnd[j++] = Read (RUN_PASS, va, L_LONG, WA);
                    opnd[j++] = Read (RUN_PASS, va + 4, L_LONG, WA);
                    break;

                case WDD|MO:
                    GET_ISTR_W (temp);        
                    iad = R[rn] + SXTW (temp);
                    va = Read (RUN_PASS, iad, L_LONG, RA);
                    j = ReadOcta (RUN_PASS, va, opnd, j, WA);
                    break;

        /* Longword displacement */

                case LDP|VB:
                case LDP|WB: case LDP|WW: case LDP|WL: case LDP|WQ: case LDP|WO:
                    opnd[j++] = OP_MEM;
                case LDP|AB: case LDP|AW: case LDP|AL: case LDP|AQ: case LDP|AO:
                    GET_ISTR_L (temp);
                    va = opnd[j++] = R[rn] + temp;
                    break;

                case LDP|RB: case LDP|RW: case LDP|RL: case LDP|RF:
                    GET_ISTR_L (temp);
                    va = R[rn] + temp;
                    opnd[j++] = Read (RUN_PASS, va, DR_LNT (disp), RA);
                    break;

                case LDP|RQ: case LDP|RD: case LDP|RG:
                    GET_ISTR_L (temp);
                    va = R[rn] + temp;
                    opnd[j++] = Read (RUN_PASS, va, L_LONG, RA);
                    opnd[j++] = Read (RUN_PASS, va + 4, L_LONG, RA);
                    break;

                case LDP|RO: case LDP|RH:
                    GET_ISTR_L (temp);
                    va = R[rn] + temp;
                    j = ReadOcta (RUN_PASS, va, opnd, j, RA);
                    break;

                case LDP|MB: case LDP|MW: case LDP|ML:
                    GET_ISTR_L (temp);
                    va = R[rn] + temp;
                    opnd[j++] = Read (RUN_PASS, va, DR_LNT (disp), WA);
                    break;

                case LDP|MQ:
                    GET_ISTR_L (temp);
                    va = R[rn] + temp;
                    opnd[j++] = Read (RUN_PASS, va, L_LONG, WA);
                    opnd[j++] = Read (RUN_PASS, va + 4, L_LONG, WA);
                    break;

                case LDP|MO:
                    GET_ISTR_L (temp);
                    va = R[rn] + temp;
                    j = ReadOcta (RUN_PASS, va, opnd, j, WA);
                    break;

        /* Longword displacement deferred */

                case LDD|VB:
                case LDD|WB: case LDD|WW: case LDD|WL: case LDD|WQ: case LDD|WO:
                    opnd[j++] = OP_MEM;
                case LDD|AB: case LDD|AW: case LDD|AL: case LDD|AQ: case LDD|AO:
                    GET_ISTR_L (temp);
                    iad = R[rn] + temp;
                    va = opnd[j++] = Read (RUN_PASS, iad, L_LONG, RA);
                    break;

                case LDD|RB: case LDD|RW: case LDD|RL: case LDD|RF:
                    GET_ISTR_L (temp);
                    iad = R[rn] + temp;
                    va = Read (RUN_PASS, iad, L_LONG, RA);
                    opnd[j++] = Read (RUN_PASS, va, DR_LNT (disp), RA);
                    break;

                case LDD|RQ: case LDD|RD: case LDD|RG:
                    GET_ISTR_L (temp);
                    iad = R[rn] + temp;
                    va = Read (RUN_PASS, iad, L_LONG, RA);
                    opnd[j++] = Read (RUN_PASS, va, L_LONG, RA);
                    opnd[j++] = Read (RUN_PASS, va + 4, L_LONG, RA);
                    break;

                case LDD|RO: case LDD|RH:
                    GET_ISTR_L (temp);
                    iad = R[rn] + temp;
                    va = Read (RUN_PASS, iad, L_LONG, RA);
                    j = ReadOcta (RUN_PASS, va, opnd, j, RA);
                    break;

                case LDD|MB: case LDD|MW: case LDD|ML:
                    GET_ISTR_L (temp);
                    iad = R[rn] + temp;
                    va = Read (RUN_PASS, iad, L_LONG, RA);
                    opnd[j++] = Read (RUN_PASS, va, DR_LNT (disp), WA);
                    break;

                case LDD|MQ:
                    GET_ISTR_L (temp);
                    iad = R[rn] + temp;
                    va = Read (RUN_PASS, iad, L_LONG, RA);
                    opnd[j++] = Read (RUN_PASS, va, L_LONG, WA);
                    opnd[j++] = Read (RUN_PASS, va + 4, L_LONG, WA);
                    break;

                case LDD|MO:
                    GET_ISTR_L (temp);
                    iad = R[rn] + temp;
                    va = Read (RUN_PASS, iad, L_LONG, RA);
                    j = ReadOcta (RUN_PASS, va, opnd, j, WA);
                    break;

        /* Index */

                case IDX|VB:
                case IDX|WB: case IDX|WW: case IDX|WL: case IDX|WQ: case IDX|WO:
                case IDX|AB: case IDX|AW: case IDX|AL: case IDX|AQ: case IDX|AO:
                case IDX|MB: case IDX|MW: case IDX|ML: case IDX|MQ: case IDX|MO:
                case IDX|RB: case IDX|RW: case IDX|RL: case IDX|RQ: case IDX|RO:
                case IDX|RF: case IDX|RD: case IDX|RG: case IDX|RH:
                    CHECK_FOR_PC;
                    index = R[rn] << (disp & DR_LNMASK);
                    GET_ISTR_B (spec);
                    rn = spec & RGMASK;
                    switch (spec & ~RGMASK) {
                    case ADC:
                        R[rn] = R[rn] - DR_LNT (disp);
                        recq[recqptr++] = RQ_REC (ADC | (disp & DR_LNMASK), rn);
                    case RGD:
                        CHECK_FOR_PC;       
                        index = index + R[rn];
                        break;

                    case AIN:
                        CHECK_FOR_PC;
                        index = index + R[rn];
                        R[rn] = R[rn] + DR_LNT (disp);
                        recq[recqptr++] = RQ_REC (AIN | (disp & DR_LNMASK), rn);
                        break;

                    case AID:
                        if (rn == nPC) {
                            GET_ISTR_L (temp);
                            }
                        else {
                            temp = Read (RUN_PASS, R[rn], L_LONG, RA);
                            R[rn] = R[rn] + 4;
                            recq[recqptr++] = RQ_REC (AID|RL, rn);
                            }
                        index = temp + index;
                        break;

                    case BDP:
                        GET_ISTR_B (temp);
                        index = index + R[rn] + SXTB (temp);
                        break;

                    case BDD:
                        GET_ISTR_B (temp);
                        index = index + Read (RUN_PASS, R[rn] + SXTB (temp), L_LONG, RA);
                        break;

                    case WDP:
                        GET_ISTR_W (temp);
                        index = index + R[rn] + SXTW (temp);
                        break;

                    case WDD:
                        GET_ISTR_W (temp);
                        index = index + Read (RUN_PASS, R[rn] + SXTW (temp), L_LONG, RA);
                        break;

                    case LDP:
                        GET_ISTR_L (temp);
                        index = index + R[rn] + temp;
                        break;

                    case LDD:
                        GET_ISTR_L (temp);
                        index = index + Read (RUN_PASS, R[rn] + temp, L_LONG, RA);
                        break;

                    default:
                        RSVD_ADDR_FAULT;                    /* end case idxspec */
                        }

                    switch (disp & (DR_ACMASK|DR_SPFLAG|DR_LNMASK)) { /* case acc+lnt */
                    case VB:
                    case WB: case WW: case WL: case WQ: case WO:
                        opnd[j++] = OP_MEM;
                    case AB: case AW: case AL: case AQ: case AO:
                        va = opnd[j++] = index;
                        break;

                    case RB: case RW: case RL: case RF:
                        opnd[j++] = Read (RUN_PASS, va = index, DR_LNT (disp), RA);
                        break;

                    case RQ: case RD: case RG:
                        opnd[j++] = Read (RUN_PASS, va = index, L_LONG, RA);
                        opnd[j++] = Read (RUN_PASS, index + 4, L_LONG, RA);
                        break;

                    case RO: case RH:
                        j = ReadOcta (RUN_PASS, va = index, opnd, j, RA);
                        break;

                    case MB: case MW: case ML:
                        opnd[j++] = Read (RUN_PASS, va = index, DR_LNT (disp), WA);
                        break;

                    case MQ:
                        opnd[j++] = Read (RUN_PASS, va = index, L_LONG, WA);
                        opnd[j++] = Read (RUN_PASS, index + 4, L_LONG, WA);
                        break;

                    case MO:
                        j = ReadOcta (RUN_PASS, va = index, opnd, j, WA);
                        break;

                    default:                                /* all others */
                        RSVD_ADDR_FAULT;                    /* fault */
                        break;
                        }                                   /* end case access/lnt */
                    break;                                  /* end index */

                default:                                    /* all others */
                    RSVD_ADDR_FAULT;                        /* fault */
                    break;
                }                                           /* end case spec */
            }                                               /* end for */
       }                                                    /* end if not FPD */

        /* Optionally record instruction history */

        if (unlikely(hst_on))
        {
            /*
             * Select recording slot in the circular buffer
             */
            InstHistory* h = cpu_unit->cpu_hst + (cpu_unit->cpu_hst_index++ % hst_lnt);

            /*
             * Do the recoding into the slot
             */
            h->isRecorded = FALSE;
            if (hst_sync)
            {
#if HST_MAKE_STAMP_ISBOOL
                if (! hst_make_stamp(& h->stamp))
                {
                    smp_printf ("\nHistory recording failed: unable to generate sequence stamp\n");
                    if (sim_log)
                        fprintf (sim_log, "History recording failed: unable to generate sequence stamp\n");
                    hst_on = FALSE;
                    goto skipRecording;
                    // ABORT_INVALID_SYSOP;
                }
#else
                hst_make_stamp(& h->stamp);
#endif
            }
            else
            {
                UINT64_INC(cpu_unit->cpu_hst_stamp);
                h->stamp = cpu_unit->cpu_hst_stamp;
            }

            h->iPC = fault_PC;
            h->iPSL = PSL | cc;
            h->opc = opc;

            for (int i = 0; i < j; i++)
                h->opnd[i] = opnd[i];

            int32 lim = PC - fault_PC;
            if ((uint32) lim > INST_SIZE)
                lim = INST_SIZE;

            for (int i = 0; i < lim; i++)
            {
                t_value wd;

                if (cpu_ex_run (RUN_PASS, &wd, fault_PC + i, cpu_unit, SWMASK ('V')) == SCPE_OK)
                    h->inst[i] = (uint8) wd;
                else
                {
                    h->inst[0] = h->inst[1] = 0xFF;
                    break;
                }
            }

            h->isRecorded = TRUE;
        }

#if HST_MAKE_STAMP_ISBOOL
  skipRecording:
#endif

        /* Dispatch to instructions */

        switch (opc) {              

        /* Single operand instructions with dest, write only - CLRx dst.wx

                spec    =       reg/memory flag
                rn      =       register number
                va      =       virtual address
        */

        case CLRB:
            WRITE_B (0);                                    /* store result */
            CC_ZZ1P;                                        /* set cc's */
            break;

        case CLRW:
            WRITE_W (0);                                    /* store result */
            CC_ZZ1P;                                        /* set cc's */
            break;

        case CLRL:
            {
                t_bool b_sys_mask = FALSE;
                uint32 old_sys_mask = 0;                    /* initialize to suppress false GCC warning */

                /* Trap access to VMS CPU idle mask */
#ifndef _DEBUG
                /* When running under debugger this place can be hit with irrelevant "variable va uninitialized" warning
                   because for efficiency we order "va" check ahead of "spec" check */
                if (unlikely(va == (uint32) sys_idle_cpu_mask_va) &&
                    sys_idle_cpu_mask_va && mapen &&
                    spec > (GRN | nPC) &&
                    (PSL & PSL_CUR) == 0 && is_os_running(RUN_PASS))
#else
                if (spec > (GRN | nPC) &&
                    unlikely(va == (uint32) sys_idle_cpu_mask_va) &&
                    sys_idle_cpu_mask_va && mapen &&
                    (PSL & PSL_CUR) == 0 && is_os_running(RUN_PASS))
#endif
                {
                    b_sys_mask = TRUE;
                    old_sys_mask = Read (RUN_PASS, sys_idle_cpu_mask_va, L_LONG, RA);
                }

                WRITE_L (0);                                /* store result */
                CC_ZZ1P;                                    /* set cc's */

                /* If bits in VMS CPU idle mask were cleared, wakeup corresponding CPUs */
                if (unlikely(b_sys_mask))
                    wakeup_cpus(RUN_PASS, old_sys_mask, 0);
            }
            break;

        case CLRQ:
            WRITE_Q (0, 0);                                 /* store result */
            CC_ZZ1P;                                        /* set cc's */
            break;

    /* Single operand instructions with source, read only - TSTx src.rx

            opnd[0] =       source
    */

        case TSTB:
            CC_IIZZ_B (op0);                                /* set cc's */
            break;

        case TSTW:
            CC_IIZZ_W (op0);                                /* set cc's */
            break;

        case TSTL:
            CC_IIZZ_L (op0);                                /* set cc's */

            if (cc == CC_Z)
            {
                if ((cpu_idle_mask & VAX_IDLE_ULTOLD) && PSL_GETIPL (PSL) == 0x1 ||    /* running Old Ultrix or friends at IPL 1? */
                    (cpu_idle_mask & VAX_IDLE_QUAD) && PSL_GETIPL (PSL) == 0x0)        /* running Quasijarus or friends at IPL 0? */
                {
                    if ((fault_PC & 0x80000000) &&             /* in system space? */
                        (fault_PC & 0x7fffffff) < 0x4000 &&    /* in low system space? */
                        PC - fault_PC == 6)                    /* 6 byte instruction? */
                    {
                        cpu_idle (RUN_PASS);                   /* idle loop */
                    }
                }
            }

            break;

    /* Single operand instructions with source, read/write - op src.mx

            opnd[0] =       operand
            spec    =       reg/mem flag
            rn      =       register number
            va      =       operand address
    */

        case INCB:
            r = (op0 + 1) & BMASK;                          /* calc result */
            WRITE_B (r);                                    /* store result */
            CC_ADD_B (r, 1, op0);                           /* set cc's */
            break;

        case INCW:
            r = (op0 + 1) & WMASK;                          /* calc result */
            WRITE_W (r);                                    /* store result */
            CC_ADD_W (r, 1, op0);                           /* set cc's */
            break;

        case INCL:
            r = (op0 + 1) & LMASK;                          /* calc result */
            WRITE_L (r);                                    /* store result */
            CC_ADD_L (r, 1, op0);                           /* set cc's */
            break;

        case DECB:
            r = (op0 - 1) & BMASK;                          /* calc result */
            WRITE_B (r);                                    /* store result */
            CC_SUB_B (r, 1, op0);                           /* set cc's */
            break;

        case DECW:
            r = (op0 - 1) & WMASK;                          /* calc result */
            WRITE_W (r);                                    /* store result */
            CC_SUB_W (r, 1, op0);                           /* set cc's */
            break;

        case DECL:
            r = (op0 - 1) & LMASK;                          /* calc result */
            WRITE_L (r);                                    /* store result */
            CC_SUB_L (r, 1, op0);                           /* set cc's */
            break;

    /* Push instructions - PUSHL src.rl or PUSHAx src.ax
            
            opnd[0] =       source
    */

        case PUSHL: case PUSHAB: case PUSHAW: case PUSHAL: case PUSHAQ:
            Write (RUN_PASS, SP - 4, op0, L_LONG, WA);      /* push operand */
            SP = SP - 4;                                    /* decr stack ptr */
            CC_IIZP_L (op0);                                /* set cc's */
            break;

    /* Moves, converts, and ADAWI - op src.rx, dst.wx
            
            opnd[0] =       source
            spec    =       reg/mem flag
            rn      =       register number
            va      =       operand address
    */

        case MOVB:
            WRITE_B (op0);                                  /* result */
            CC_IIZP_B (op0);                                /* set cc's */
            break;

        case MOVW: case MOVZBW:
            WRITE_W (op0);                                  /* result */
            CC_IIZP_W (op0);                                /* set cc's */
            break;

        case MOVL: case MOVZBL: case MOVZWL:
        case MOVAB: case MOVAW: case MOVAL: case MOVAQ:
            WRITE_L (op0);                                  /* result */
            CC_IIZP_L (op0);                                /* set cc's */
            break;

        case MCOMB:
            r = op0 ^ BMASK;                                /* compl opnd */
            WRITE_B (r);                                    /* store result */
            CC_IIZP_B (r);                                  /* set cc's */
            break;

        case MCOMW:
            r = op0 ^ WMASK;                                /* compl opnd */
            WRITE_W (r);                                    /* store result */
            CC_IIZP_W (r);                                  /* set cc's */
            break;

        case MCOML:
            r = op0 ^ LMASK;                                /* compl opnd */
            WRITE_L (r);                                    /* store result */
            CC_IIZP_L (r);                                  /* set cc's */
            break;

        case MNEGB:
            r = (-op0) & BMASK;                             /* negate opnd */
            WRITE_B (r);                                    /* store result */
            CC_SUB_B (r, op0, 0);                           /* set cc's */
            break;

        case MNEGW:
            r = (-op0) & WMASK;                             /* negate opnd */
            WRITE_W (r);                                    /* store result */
            CC_SUB_W (r, op0, 0);                           /* set cc's */
            break;

        case MNEGL:
            r = (-op0) & LMASK;                             /* negate opnd */
            WRITE_L (r);                                    /* store result */
            CC_SUB_L (r, op0, 0);                           /* set cc's */
            break;

        case CVTBW:
            r = SXTBW (op0);                                /* ext sign */
            WRITE_W (r);                                    /* store result */
            CC_IIZZ_W (r);                                  /* set cc's */
            break;

        case CVTBL:
            r = SXTB (op0);                                 /* ext sign */
            WRITE_L (r);                                    /* store result */
            CC_IIZZ_L (r);                                  /* set cc's */
            break;

        case CVTWL:
            r = SXTW (op0);                                 /* ext sign */
            WRITE_L (r);                                    /* store result */
            CC_IIZZ_L (r);                                  /* set cc's */
            break;

        case CVTLB:
            r = op0 & BMASK;                                /* set result */
            WRITE_B (r);                                    /* store result */
            CC_IIZZ_B (r);                                  /* initial cc's */
            if ((op0 > 127) || (op0 < -128)) {
                V_INTOV;
                }
            break;

        case CVTLW:
            r = op0 & WMASK;                                /* set result */
            WRITE_W (r);                                    /* store result */
            CC_IIZZ_W (r);                                  /* initial cc's */
            if ((op0 > 32767) || (op0 < -32768)) {
                V_INTOV;
                }
            break;

        case CVTWB:
            r = op0 & BMASK;                                /* set result */
            WRITE_B (r);                                    /* store result */
            CC_IIZZ_B (r);                                  /* initial cc's */
            temp = SXTW (op0);                              /* cvt op to long */
            if ((temp > 127) || (temp < -128)) {
                V_INTOV;
                }
            break;

        case ADAWI:
            /* pass "va" as conditonal to suppress false GCC warning */
            op_adawi (RUN_PASS, opnd, acc, spec, rn, (spec > (GRN | nPC)) ? va : 0, cc /* cc passed by reference*/);
            break;

        /* Integer operates, 2 operand, read only - op src1.rx, src2.rx

                opnd[0] =       source1
                opnd[1] =       source2
        */

        case CMPB:
            CC_CMP_B (op0, op1);                            /* set cc's */
            break;

        case CMPW:
            CC_CMP_W (op0, op1);                            /* set cc's */
            break;

        case CMPL:
            CC_CMP_L (op0, op1);                            /* set cc's */
            break;

        case BITB:
            r = op1 & op0;                                  /* calc result */
            CC_IIZP_B (r);                                  /* set cc's */
            break;

        case BITW:
            r = op1 & op0;                                  /* calc result */
            CC_IIZP_W (r);                                  /* set cc's */
            break;

        case BITL:
            r = op1 & op0;                                  /* calc result */
            CC_IIZP_L (r);                                  /* set cc's */

            if (cc == CC_Z)
            {
                if ((cpu_idle_mask & VAX_IDLE_ULT) &&           /* running Ultrix or friends? */
                    (PSL & PSL_IS) != 0 &&                      /* on IS? */
                    PSL_GETIPL (PSL) == 0x18 &&                 /* at IPL 18? */
                    (fault_PC & 0x80000000) &&                  /* in system space? */
                    PC - fault_PC == 8 &&                       /* 8 byte instruction? */
                    (fault_PC & 0x7fffffff) < 0x6000)           /* in low system space? */
                {
                    cpu_idle (RUN_PASS);                        /* idle loop */
                }
            }

            break;

    /* Integer operates, 2 operand read/write, and 3 operand, also MOVQ
            op2 src.rx, dst.mx      op3 src.rx, src.rx, dst.wx

            opnd[0] =       source1
            opnd[1] =       source2
            spec    =       register/memory flag
            rn      =       register number
            va      =       memory address
    */

        case ADDB2: case ADDB3:
            r = (op1 + op0) & BMASK;                        /* calc result */
            WRITE_B (r);                                    /* store result */
            CC_ADD_B (r, op0, op1);                         /* set cc's */
            break;

        case ADDW2: case ADDW3:
            r = (op1 + op0) & WMASK;                        /* calc result */
            WRITE_W (r);                                    /* store result */
            CC_ADD_W (r, op0, op1);                         /* set cc's */
            break;

        case ADWC:
            r = (op1 + op0 + (cc & CC_C)) & LMASK;          /* calc result */
            WRITE_L (r);                                    /* store result */
            CC_ADD_L (r, op0, op1);                         /* set cc's */
            if ((r == op1) && op0)                          /* special case */
                cc = cc | CC_C;
            break;

        case ADDL2: case ADDL3:
            r = (op1 + op0) & LMASK;                        /* calc result */
            WRITE_L (r);                                    /* store result */
            CC_ADD_L (r, op0, op1);                         /* set cc's */
            break;

        case SUBB2: case SUBB3:
            r = (op1 - op0) & BMASK;                        /* calc result */
            WRITE_B (r);                                    /* store result */
            CC_SUB_B (r, op0, op1);                         /* set cc's */
            break;

        case SUBW2: case SUBW3:
            r = (op1 - op0) & WMASK;                        /* calc result */
            WRITE_W (r);                                    /* store result */
            CC_SUB_W (r, op0, op1);                         /* set cc's */
            break;

        case SBWC:
            r = (op1 - op0 - (cc & CC_C)) & LMASK;          /* calc result */
            WRITE_L (r);                                    /* store result */
            CC_SUB_L (r, op0, op1);                         /* set cc's */
            if ((op0 == op1) && r)                          /* special case */
                cc = cc | CC_C;
            break;

        case SUBL2: case SUBL3:
            r = (op1 - op0) & LMASK;                        /* calc result */
            WRITE_L (r);                                    /* store result */
            CC_SUB_L (r, op0, op1);                         /* set cc's */
            break;

        case MULB2: case MULB3:
            temp = SXTB (op0) * SXTB (op1);                 /* multiply */
            r = temp & BMASK;                               /* mask to result */
            WRITE_B (r);                                    /* store result */
            CC_IIZZ_B (r);                                  /* set cc's */
            if ((temp > 127) || (temp < -128)) {
                V_INTOV;
                }
            break;

        case MULW2: case MULW3:
            temp = SXTW (op0) * SXTW (op1);                 /* multiply */
            r = temp & WMASK;                               /* mask to result */
            WRITE_W (r);                                    /* store result */
            CC_IIZZ_W (r);                                  /* set cc's */
            if ((temp > 32767) || (temp < -32768)) {
                V_INTOV;
                }
            break;

        case MULL2: case MULL3:
            r = op_emul (RUN_PASS, op0, op1, &rh);          /* get 64b result */
            WRITE_L (r);                                    /* store result */
            CC_IIZZ_L (r);                                  /* set cc's */
            if (rh != ((r & LSIGN)? -1: 0)) {               /* chk overflow */
                V_INTOV;
                }
            break;

        case DIVB2: case DIVB3:
            if (op0 == 0) {                                 /* div by zero? */
                r = op1;
                temp = CC_V;
                SET_TRAP (TRAP_DIVZRO);
                }
            else if ((op0 == BMASK) && (op1 == BSIGN)) {    /* overflow? */
                r = op1;
                temp = CC_V;
                INTOV;
                }
            else {
                r = SXTB (op1) / SXTB (op0);                /* ok, divide */
                temp = 0;
                }
            r = r & BMASK;                                  /* mask to result */
            WRITE_B (r);                                    /* write result */
            CC_IIZZ_B (r);                                  /* set cc's */
            cc = cc | temp;                                 /* error? set V */
            break;

        case DIVW2: case DIVW3:
            if (op0 == 0) {                                 /* div by zero? */
                r = op1;
                temp = CC_V;
                SET_TRAP (TRAP_DIVZRO);
                }
            else if ((op0 == WMASK) && (op1 == WSIGN)) {    /* overflow? */
                r = op1;
                temp = CC_V;
                INTOV;
                }
            else {
                r = SXTW (op1) / SXTW (op0);                /* ok, divide */
                temp = 0;
                }
            r = r & WMASK;                                  /* mask to result */
            WRITE_W (r);                                    /* write result */
            CC_IIZZ_W (r);                                  /* set cc's */
            cc = cc | temp;                                 /* error? set V */
            break;

        case DIVL2: case DIVL3:
            if (op0 == 0) {                                 /* div by zero? */
                r = op1;
                temp = CC_V;
                SET_TRAP (TRAP_DIVZRO);
                }
            else if ((uint32) op0 == LMASK && (uint32) op1 == LSIGN) {    /* overflow? */
                r = op1;
                temp = CC_V;
                INTOV;
                }
            else {
                r = op1 / op0;                              /* ok, divide */
                temp = 0;
                }
            r = r & LMASK;                                  /* mask to result */
            WRITE_L (r);                                    /* write result */
            CC_IIZZ_L (r);                                  /* set cc's */
            cc = cc | temp;                                 /* error? set V */
            break;

        case BISB2: case BISB3:
            r = op1 | op0;                                  /* calc result */
            WRITE_B (r);                                    /* store result */
            CC_IIZP_B (r);                                  /* set cc's */
            break;

        case BISW2: case BISW3:
            r = op1 | op0;                                  /* calc result */
            WRITE_W (r);                                    /* store result */
            CC_IIZP_W (r);                                  /* set cc's */
            break;

        case BISL2: case BISL3:
            r = op1 | op0;                                  /* calc result */
            WRITE_L (r);                                    /* store result */
            CC_IIZP_L (r);                                  /* set cc's */
            break;

        case BICB2: case BICB3:
            r = op1 & ~op0;                                 /* calc result */
            WRITE_B (r);                                    /* store result */
            CC_IIZP_B (r);                                  /* set cc's */
            break;

        case BICW2: case BICW3:
            r = op1 & ~op0;                                 /* calc result */
            WRITE_W (r);                                    /* store result */
            CC_IIZP_W (r);                                  /* set cc's */
            break;

        case BICL2: case BICL3:
            {
                t_bool b_sys_mask = FALSE;
                uint32 old_sys_mask = 0;                    /* initialize to suppress false GCC warning */

                /* Trap access to VMS CPU idle mask */
#ifndef _DEBUG
                /* When running under debugger this place can be hit with irrelevant "variable va uninitialized" warning
                   because for efficiency we order "va" check ahead of "spec" check */
                if (unlikely(va == (uint32) sys_idle_cpu_mask_va) &&
                    sys_idle_cpu_mask_va && mapen &&
                    spec > (GRN | nPC) &&
                    (PSL & PSL_CUR) == 0 && is_os_running(RUN_PASS))
#else
                if (spec > (GRN | nPC) &&
                    unlikely(va == (uint32) sys_idle_cpu_mask_va) &&
                    sys_idle_cpu_mask_va && mapen &&
                    (PSL & PSL_CUR) == 0 && is_os_running(RUN_PASS))
#endif
                {
                    b_sys_mask = TRUE;
                    old_sys_mask = Read (RUN_PASS, sys_idle_cpu_mask_va, L_LONG, RA);
                }

                r = op1 & ~op0;                             /* calc result */
                WRITE_L (r);                                /* store result */
                CC_IIZP_L (r);                              /* set cc's */

                /* If bits in VMS CPU idle mask were cleared, wakeup corresponding CPUs */
                if (unlikely(b_sys_mask))
                    wakeup_cpus(RUN_PASS, old_sys_mask, r);
            }
            break;

        case XORB2: case XORB3:
            r = op1 ^ op0;                                  /* calc result */
            WRITE_B (r);                                    /* store result */
            CC_IIZP_B (r);                                  /* set cc's */
            break;

        case XORW2: case XORW3:
            r = op1 ^ op0;                                  /* calc result */
            WRITE_W (r);                                    /* store result */
            CC_IIZP_W (r);                                  /* set cc's */
            break;

        case XORL2: case XORL3:
            r = op1 ^ op0;                                  /* calc result */
            WRITE_L (r);                                    /* store result */
            CC_IIZP_L (r);                                  /* set cc's */
            break;

    /* MOVQ - movq src.rq, dst.wq

            opnd[0:1] =     source
            spec    =       register/memory flag
            rn      =       register number
            va      =       memory address
            
    */

        case MOVQ:
            WRITE_Q (op0, op1);                             /* store result */
            CC_IIZP_Q (op0, op1);
            break;

    /* Shifts - op shf.rb,src.rl,dst.wl

            opnd[0] =       shift count
            opnd[1] =       source
            spec    =       register/memory flag
            rn      =       register number
            va      =       memory address
    */

        case ROTL:
            j = op0 % 32;                                   /* reduce sc, mod 32 */
            if (j)
                r = ((((uint32) op1) << j) | (((uint32) op1) >> (32 - j))) & LMASK;
            else r = op1;
            WRITE_L (r);                                    /* store result */
            CC_IIZP_L (r);                                  /* set cc's */
            break;

        case ASHL:
            if (op0 & BSIGN) {                              /* right shift? */
                temp = 0x100 - op0;                         /* get |shift| */
                if (temp > 31)                              /* sc > 31? */
                    r = (op1 & LSIGN)? LMASK: 0;
                else r = op1 >> temp;                       /* shift */
                WRITE_L (r);                                /* store result */
                CC_IIZZ_L (r);                              /* set cc's */
                break;
                }
            else {
                if (op0 > 31)                               /* sc > 31? */
                    r = temp = 0;
                else {
                    r = (((uint32) op1) << op0) & LMASK;    /* shift */
                    temp = r >> op0;                        /* shift back */
                    }
                WRITE_L (r);                                /* store result */
                CC_IIZZ_L (r);                              /* set cc's */
                if (op1 != temp) {                          /* bits lost? */
                    V_INTOV;
                    }
                }
            break;

        case ASHQ:
            r = op_ashq (RUN_PASS, opnd, &rh, &flg);        /* do qw shift */
            WRITE_Q (r, rh);                                /* store results */
            CC_IIZZ_Q (r, rh);                              /* set cc's */
            if (flg) {                                      /* if ovflo, set */
                V_INTOV;
                }
            break;

    /* EMUL - emul mplr.rl,mpcn.rl,add.rl,dst.wq

            op0     =       multiplier
            op1     =       multiplicand
            op2     =       adder
            op3:op4 =       destination (.wq)
    */

        case EMUL:
            r = op_emul (RUN_PASS, op0, op1, &rh);          /* calc 64b result */
            r = r + op2;                                    /* add 32b value */
            rh = rh + (((uint32) r) < ((uint32) op2)) -     /* into 64b result */
                ((op2 & LSIGN)? 1: 0);
            WRITE_Q (r, rh);                                /* write result */
            CC_IIZZ_Q (r, rh);                              /* set cc's */
            break;

    /* EDIV - ediv dvr.rl,dvd.rq,quo.wl,rem.wl

            op0     =       divisor (.rl)
            op1:op2 =       dividend (.rq)
            op3:op4 =       quotient address (.wl)
            op5:op6 =       remainder address (.wl)
    */

        case EDIV:
            if (op5 < 0)                                    /* wtest remainder */
                Read (RUN_PASS, op6, L_LONG, WA);
            if (op0 == 0) {                                 /* divide by zero? */
                flg = CC_V;                                 /* set V */
                r = opnd[1];                                /* quo = low divd */
                rh = 0;                                     /* rem = 0 */
                SET_TRAP (TRAP_DIVZRO);                     /* set trap */
                }
            else {
                r = op_ediv (RUN_PASS, opnd, &rh, &flg);    /* extended divide */
                if (flg) {                                  /* if ovf+IV, set trap */
                    INTOV;
                    }
                }
            if (op3 >= 0)                                   /* store quotient */
                R[op3] = r;
            else Write (RUN_PASS, op4, r, L_LONG, WA);
            if (op5 >= 0)                                   /* store remainder */
                R[op5] = rh;
            else Write (RUN_PASS, op6, rh, L_LONG, WA);
            CC_IIZZ_L (r);                                  /* set cc's */
            cc = cc | flg;                                  /* set V if required */
            break;

    /* Control instructions */

    /* Simple branches and subroutine calls */

        case BRB:
            BRANCHB (brdisp);                               /* branch  */
            if (PC == fault_PC)
            {
                if (PSL_GETIPL (PSL) == 0x1F)
                    ABORT (STOP_LOOP);
                else
                    cpu_idle (RUN_PASS);
            }
            break;

        case BRW:
            BRANCHW (brdisp);                               /* branch */
            if (PC == fault_PC)
            {
                if (PSL_GETIPL (PSL) == 0x1F)
                    ABORT (STOP_LOOP);
                else
                    cpu_idle (RUN_PASS);
            }
            break;

        case BSBB:
            Write (RUN_PASS, SP - 4, PC, L_LONG, WA);       /* push PC on stk */
            SP = SP - 4;                                    /* decr stk ptr */
            BRANCHB (brdisp);                               /* branch  */
            break;

        case BSBW:
            Write (RUN_PASS, SP - 4, PC, L_LONG, WA);       /* push PC on stk */
            SP = SP - 4;                                    /* decr stk ptr */
            BRANCHW (brdisp);                               /* branch */
            break;

        case BGEQ:
            if (!(cc & CC_N))                               /* br if N = 0 */
                BRANCHB (brdisp);
            break;

        case BLSS:
            if (cc & CC_N)                                  /* br if N = 1 */
                BRANCHB (brdisp);
            break;

        case BNEQ:
            if (!(cc & CC_Z))                               /* br if Z = 0 */
                BRANCHB (brdisp);
            break;

        case BEQL:
            if (cc & CC_Z)                                  /* br if Z = 1 */
            {
                BRANCHB (brdisp);

                if (mapen == 0 &&                           /* MAPEN off? */
                    (PSL & PSL_IS) != 0 &&                  /* on IS? */
                    PSL_GETIPL (PSL) == 0x1F &&             /* at IPL 31 */
                    fault_PC == ROM_PC_CHAR_PROMPT)         /* Boot ROM Character Prompt */
                {
                    cpu_idle (RUN_PASS);
                }
            }

            break;

        case BVC:
            if (!(cc & CC_V))                               /* br if V = 0 */
                BRANCHB (brdisp);
            break;

        case BVS:
            if (cc & CC_V)                                  /* br if V = 1 */
                BRANCHB (brdisp);
            break;

        case BGEQU:
            if (!(cc & CC_C))                               /* br if C = 0 */
                BRANCHB (brdisp);
            break;

        case BLSSU:
            if (cc & CC_C)                                  /* br if C = 1 */
                BRANCHB (brdisp);
            break;

        case BGTR:
            if (!(cc & (CC_N | CC_Z)))                      /* br if N | Z = 0 */
                BRANCHB (brdisp);
            break;

        case BLEQ:
            if (cc & (CC_N | CC_Z))                         /* br if N | Z = 1 */
                BRANCHB (brdisp);
            break;

        case BGTRU:
            if (!(cc & (CC_C | CC_Z)))                      /* br if C | Z = 0 */
                BRANCHB (brdisp);
            break;

        case BLEQU:
            if (cc & (CC_C | CC_Z))                         /* br if C | Z = 1 */
                BRANCHB (brdisp);
            break;

    /* Simple jumps and subroutine calls - op addr.ab

            opnd[0] =       address
    */

        case JSB:
            Write (RUN_PASS, SP - 4, PC, L_LONG, WA);       /* push PC on stk */
            SP = SP - 4;                                    /* decr stk ptr */

        case JMP:
            JUMP (op0);                                     /* jump */
            break;

        case RSB:
            temp = Read (RUN_PASS, SP, L_LONG, RA);         /* get top of stk */
            SP = SP + 4;                                    /* incr stk ptr */
            JUMP (temp);
            break;

    /* SOB instructions - op idx.ml,disp.bb

            opnd[0] =       index
            spec    =       register/memory flag
            rn      =       register number
            va      =       memory address
    */

        case SOBGEQ:
            r = op0 - 1;                                    /* decr index */
            WRITE_L (r);                                    /* store result */
            CC_IIZP_L (r);                                  /* set cc's */
            V_SUB_L (r, 1, op0);                            /* test for ovflo */    
            if (r >= 0)                                     /* if >= 0, branch */
                BRANCHB (brdisp);
            break;

        case SOBGTR:
            r = op0 - 1;                                    /* decr index */
            WRITE_L (r);                                    /* store result */
            CC_IIZP_L (r);                                  /* set cc's */
            V_SUB_L (r, 1, op0);                            /* test for ovflo */    
            if (r > 0)                                      /* if >= 0, branch */
                BRANCHB (brdisp);
            break;

    /* AOB instructions - op limit.rl,idx.ml,disp.bb

            opnd[0] =       limit
            opnd[1] =       index
            spec    =       register/memory flag
            rn      =       register number
            va      =       memory address
    */

        case AOBLSS:
            r = op1 + 1;                                    /* incr index */
            WRITE_L (r);                                    /* store result */
            CC_IIZP_L (r);                                  /* set cc's */
            V_ADD_L (r, 1, op1);                            /* test for ovflo */
            if (r < op0)                                    /* if < lim, branch */
                BRANCHB (brdisp);
            break;

        case AOBLEQ:
            r = op1 + 1;                                    /* incr index */
            WRITE_L (r);                                    /* store result */
            CC_IIZP_L (r);                                  /* set cc's */
            V_ADD_L (r, 1, op1);                            /* test for ovflo */
            if (r <= op0)                                   /* if < lim, branch */
                BRANCHB (brdisp);
            break;

    /* ACB instructions - op limit.rx,add.rx,index.mx,disp.bw

            opnd[0] =       limit
            opnd[1] =       adder
            opnd[2] =       index
            spec    =       register/memory flag
            rn      =       register number
            va      =       memory address
    */

        case ACBB:
            r = (op2 + op1) & BMASK;                        /* calc result */
            WRITE_B (r);                                    /* store result */
            CC_IIZP_B (r);                                  /* set cc's */
            V_ADD_B (r, op1, op2);                          /* test for ovflo */
            if ((op1 & BSIGN)? (SXTB (r) >= SXTB (op0)): (SXTB (r) <= SXTB (op0)))
                BRANCHW (brdisp);
            break;

        case ACBW:
            r = (op2 + op1) & WMASK;                        /* calc result */
            WRITE_W (r);                                    /* store result */
            CC_IIZP_W (r);                                  /* set cc's */
            V_ADD_W (r, op1, op2);                          /* test for ovflo */
            if ((op1 & WSIGN)? (SXTW (r) >= SXTW (op0)): (SXTW (r) <= SXTW (op0)))
                BRANCHW (brdisp);
            break;

        case ACBL:
            r = (op2 + op1) & LMASK;                        /* calc result */
            WRITE_L (r);                                    /* store result */
            CC_IIZP_L (r);                                  /* set cc's */
            V_ADD_L (r, op1, op2);                          /* test for ovflo */
            if ((op1 & LSIGN)? (r >= op0): (r <= op0))
                BRANCHW (brdisp);
            break;

    /* CASE instructions - casex sel.rx,base.rx,lim.rx

            opnd[0] =       selector
            opnd[1] =       base
            opnd[2] =       limit
    */

        case CASEB:
            r = (op0 - op1) & BMASK;                        /* sel - base */
            CC_CMP_B (r, op2);                              /* r:limit, set cc's */
            if (r > op2)                                    /* r > limit (unsgnd)? */
                JUMP (PC + ((op2 + 1) * 2));
            else {
                temp = Read (RUN_PASS, PC + (r * 2), L_WORD, RA);
                BRANCHW (temp);
                }
            break;

        case CASEW:
            r = (op0 - op1) & WMASK;                        /* sel - base */
            CC_CMP_W (r, op2);                              /* r:limit, set cc's */
            if (r > op2)                                    /* r > limit (unsgnd)? */
                JUMP (PC + ((op2 + 1) * 2));
            else {
                temp = Read (RUN_PASS, PC + (r * 2), L_WORD, RA);
                BRANCHW (temp);
                }
            break;

        case CASEL:
            r = (op0 - op1) & LMASK;                        /* sel - base */
            CC_CMP_L (r, op2);                              /* r:limit, set cc's */
            if (((uint32) r) > ((uint32) op2))              /* r > limit (unsgnd)? */
                JUMP (PC + ((op2 + 1) * 2));
            else {
                temp = Read (RUN_PASS, PC + (r * 2), L_WORD, RA);
                BRANCHW (temp);
                }
            break;

    /* Branch on bit instructions - bbxy pos.rl,op.wb,disp.bb

            opnd[0] =       position
            opnd[1] =       register number/memory flag
            opnd[2] =       memory address, if memory
    */

        case BBS:
            if (op_bb_n(RUN_PASS, opnd, acc))              /* br if bit set */
            {
                BRANCHB(brdisp);

                if ((PSL & PSL_IS) &&                      /* on IS? */
                    PSL_GETIPL(PSL) == 0x3 &&              /* at IPL 3? */
                    (cpu_idle_mask & VAX_IDLE_VMS))        /* running VMS? */
                {
                    cpu_idle(RUN_PASS);                    /* idle loop */
                }
            }
            break;

        case BBC:
            if (!op_bb_n (RUN_PASS, opnd, acc))            /* br if bit clr */
                BRANCHB (brdisp);
            break;

        case BBSS: case BBSSI:
            if (op_bb_x (RUN_PASS, opnd, 1, acc, opc == BBSSI))    /* br if set, set */
                BRANCHB (brdisp);
            break;

        case BBCC: case BBCCI:
            if (!op_bb_x (RUN_PASS, opnd, 0, acc, opc == BBCCI))   /* br if clr, clr*/
                BRANCHB (brdisp);
            break;

        case BBSC:
            if (op_bb_x (RUN_PASS, opnd, 0, acc, FALSE))           /* br if clr, set */
                BRANCHB (brdisp);
            break;

        case BBCS:
            if (!op_bb_x (RUN_PASS, opnd, 1, acc, FALSE))          /* br if set, clr */
                BRANCHB (brdisp);
            break;

        case BLBS:
            if (op0 & 1)                                    /* br if bit set */
                BRANCHB (brdisp);
            break;

        case BLBC:
            if ((op0 & 1) == 0)                             /* br if bit clear */
                BRANCHB (brdisp);
            break;

    /* Extract field instructions - ext?v pos.rl,size.rb,base.wb,dst.wl

            opnd[0] =       position
            opnd[1] =       size
            opnd[2] =       register number/memory flag
            opnd[3] =       register content/memory address
            spec    =       register/memory flag
            rn      =       register number
            va      =       memory address
    */

        case EXTV:
            r = op_extv (RUN_PASS, opnd, vfldrp1, acc);     /* get field */
            if (r & byte_sign[op1])
                r = r | ~byte_mask[op1];
            WRITE_L (r);                                    /* store field */
            CC_IIZP_L (r);                                  /* set cc's */
            break;

        case EXTZV:
            r = op_extv (RUN_PASS, opnd, vfldrp1, acc);     /* get field */
            WRITE_L (r);                                    /* store field */
            CC_IIZP_L (r);                                  /* set cc's */
            break;

    /* Compare field instructions - cmp?v pos.rl,size.rb,base.wb,src2.rl

            opnd[0] =       position
            opnd[1] =       size
            opnd[2] =       register number/memory flag
            opnd[3] =       register content/memory address
            opnd[4] =       source2
    */

        case CMPV:
            r = op_extv (RUN_PASS, opnd, vfldrp1, acc);     /* get field */
            if (r & byte_sign[op1])
                r = r | ~byte_mask[op1];
            CC_CMP_L (r, op4);                              /* set cc's */
            break;

        case CMPZV:
            r = op_extv (RUN_PASS, opnd, vfldrp1, acc);     /* get field */
            CC_CMP_L (r, op4);                              /* set cc's */
            break;

    /* Find first field instructions - ff? pos.rl,size.rb,base.wb,dst.wl

            opnd[0] =       position
            opnd[1] =       size
            opnd[2] =       register number/memory flag
            opnd[3] =       register content/memory address
            spec    =       register/memory flag
            rn      =       register number
            va      =       memory address
    */

        case FFS:
            r = op_extv (RUN_PASS, opnd, vfldrp1, acc);     /* get field */
            temp = op_ffs (RUN_PASS, r, op1);               /* find first 1 */
            WRITE_L (op0 + temp);                           /* store result */
            cc = r? 0: CC_Z;                                /* set cc's */
            break;

        case FFC:
            r = op_extv (RUN_PASS, opnd, vfldrp1, acc);     /* get field */
            r = r ^ byte_mask[op1];                         /* invert bits */
            temp = op_ffs (RUN_PASS, r, op1);               /* find first 1 */
            WRITE_L (op0 + temp);                           /* store result */
            cc = r? 0: CC_Z;                                /* set cc's */
            break;

    /* Insert field instruction - insv src.rl,pos.rb,size.rl,base.wb

            opnd[0] =       source
            opnd[1] =       position
            opnd[2] =       size
            opnd[3] =       register number/memory flag
            opnd[4] =       register content/memory address
    */

        case INSV:
            op_insv (RUN_PASS, opnd, vfldrp1, acc);          /* insert field */
            break;

    /* Call and return - call? arg.rx,proc.ab

            opnd[0] =       argument
            opnd[1] =       procedure address
    */

        case CALLS:
            cc = op_call (RUN_PASS, opnd, TRUE, acc);
            break;

        case CALLG:
            cc = op_call (RUN_PASS, opnd, FALSE, acc);
            break;

        case RET:
            cc = op_ret (RUN_PASS, acc);
            break;

    /* Miscellaneous instructions */

        case HALT:
            if (PSL & PSL_CUR)                              /* not kern? rsvd inst */
                RSVD_INST_FAULT;
            else if (cpu_unit->flags & UNIT_CONH)           /* halt to console? */
                cc = con_halt (CON_HLTINS, cc);             /* enter firmware */
            else {
                ABORT (STOP_HALT);                          /* halt to simulator */
                }

        case NOP:
            break;

        case BPT:
            SETPC (fault_PC);
            cc = intexc (RUN_PASS, SCB_BPT, cc, 0, IE_EXC);
            GET_CUR;
            break;

        case XFC:
            SETPC (fault_PC);
            cc = intexc (RUN_PASS, SCB_XFC, cc, 0, IE_EXC);
            GET_CUR;
            break;

        case BISPSW:
            if (opnd[0] & PSW_MBZ)
                RSVD_OPND_FAULT;
            PSL = PSL | (opnd[0] & ~CC_MASK);
            cc = cc | (opnd[0] & CC_MASK);
            break;

        case BICPSW:
            if (opnd[0] & PSW_MBZ)
                RSVD_OPND_FAULT;
            PSL = PSL & ~opnd[0];
            cc = cc & ~opnd[0];
            break;

        case MOVPSL:
            r = PSL | cc;
            WRITE_L (r);
            break;

        case PUSHR:
            op_pushr (RUN_PASS, opnd, acc);
            break;

        case POPR:
            op_popr (RUN_PASS, opnd, acc);
            break;

        case INDEX:
            if ((op0 < op1) || (op0 > op2))
                SET_TRAP (TRAP_SUBSCR);
            r = (op0 + op4) * op3;
            WRITE_L (r);
            CC_IIZZ_L (r);
            break;

    /* Queue and interlocked queue */

        case INSQUE:
            cc = op_insque (RUN_PASS, opnd, acc);
            break;

        case REMQUE:
            cc = op_remque (RUN_PASS, opnd, acc);
            break;

        case INSQHI:
            cc = op_insqhi (RUN_PASS, opnd, acc);
            break;

        case INSQTI:
            cc = op_insqti (RUN_PASS, opnd, acc);
            break;

        case REMQHI:
            cc = op_remqhi (RUN_PASS, opnd, acc);
            break;

        case REMQTI:
            cc = op_remqti (RUN_PASS, opnd, acc);
            break;

    /* String instructions */

        case MOVC3: case MOVC5:
            cc = op_movc (RUN_PASS, opnd, opc & 4, acc);
            break;

        case CMPC3: case CMPC5:
            cc = op_cmpc (RUN_PASS, opnd, opc & 4, acc);
            break;

        case LOCC: case SKPC:
            cc = op_locskp (RUN_PASS, opnd, opc & 1, acc);
            break;

        case SCANC: case SPANC:
            cc = op_scnspn (RUN_PASS, opnd, opc & 1, acc);
            break;

    /* Floating point instructions */

        case TSTF: case TSTD:
            r = op_movfd (RUN_PASS, op0);
            CC_IIZZ_FP (r);
            break;

        case TSTG:
            r = op_movg (RUN_PASS, op0);
            CC_IIZZ_FP (r);
            break;

        case MOVF:
            r = op_movfd (RUN_PASS, op0);
            WRITE_L (r);
            CC_IIZP_FP (r);
            break;

        case MOVD:
            if ((r = op_movfd (RUN_PASS, op0)) == 0)
                op1 = 0;
            WRITE_Q (r, op1);
            CC_IIZP_FP (r);
            break;

        case MOVG:
            if ((r = op_movg (RUN_PASS, op0)) == 0)
                op1 = 0;
            WRITE_Q (r, op1);
            CC_IIZP_FP (r);
            break;

        case MNEGF:
            r = op_mnegfd (RUN_PASS, op0);
            WRITE_L (r);
            CC_IIZZ_FP (r);
            break;

        case MNEGD:
            if ((r = op_mnegfd (RUN_PASS, op0)) == 0)
                op1 = 0;
            WRITE_Q (r, op1);
            CC_IIZZ_FP (r);
            break;

        case MNEGG:
            if ((r = op_mnegg (RUN_PASS, op0)) == 0)
                op1 = 0;
            WRITE_Q (r, op1);
            CC_IIZZ_FP (r);
            break;

        case CMPF:
            cc = op_cmpfd (RUN_PASS, op0, 0, op1, 0);
            break;

        case CMPD:
            cc = op_cmpfd (RUN_PASS, op0, op1, op2, op3);
            break;

        case CMPG:
            cc = op_cmpg (RUN_PASS, op0, op1, op2, op3);
            break;

        case CVTBF:
            r = op_cvtifdg (RUN_PASS, SXTB (op0), NULL, opc);
            WRITE_L (r);
            CC_IIZZ_FP (r);
            break;

        case CVTWF:
            r = op_cvtifdg (RUN_PASS, SXTW (op0), NULL, opc);
            WRITE_L (r);
            CC_IIZZ_FP (r);
            break;

        case CVTLF:
            r = op_cvtifdg (RUN_PASS, op0, NULL, opc);
            WRITE_L (r);
            CC_IIZZ_FP (r);
            break;

        case CVTBD: case CVTBG:
            r = op_cvtifdg (RUN_PASS, SXTB (op0), &rh, opc);
            WRITE_Q (r, rh);
            CC_IIZZ_FP (r);
            break;

        case CVTWD: case CVTWG:
            r = op_cvtifdg (RUN_PASS, SXTW (op0), &rh, opc);
            WRITE_Q (r, rh);
            CC_IIZZ_FP (r);
            break;

        case CVTLD: case CVTLG:
            r = op_cvtifdg (RUN_PASS, op0, &rh, opc);
            WRITE_Q (r, rh);
            CC_IIZZ_FP (r);
            break;

        case CVTFB: case CVTDB: case CVTGB:
            r = op_cvtfdgi (RUN_PASS, opnd, &flg, opc) & BMASK;
            WRITE_B (r);
            CC_IIZZ_B (r);
            if (flg) {
                V_INTOV;
                }
            break;

        case CVTFW: case CVTDW: case CVTGW:
            r = op_cvtfdgi (RUN_PASS, opnd, &flg, opc) & WMASK;
            WRITE_W (r);
            CC_IIZZ_W (r);
            if (flg) {
                V_INTOV;
                }
            break;

        case CVTFL: case CVTDL: case CVTGL:
        case CVTRFL: case CVTRDL: case CVTRGL:
            r = op_cvtfdgi (RUN_PASS, opnd, &flg, opc) & LMASK;
            WRITE_L (r);
            CC_IIZZ_L (r);
            if (flg) {
                V_INTOV;
                }
            break;

        case CVTFD:
            r = op_movfd (RUN_PASS, op0);
            WRITE_Q (r, 0);
            CC_IIZZ_FP (r);
            break;

        case CVTDF:
            r = op_cvtdf (RUN_PASS, opnd);
            WRITE_L (r);
            CC_IIZZ_FP (r);
            break;

        case CVTFG:
            r = op_cvtfg (RUN_PASS, opnd, &rh);
            WRITE_Q (r, rh);
            CC_IIZZ_FP (r);
            break;

        case CVTGF:
            r = op_cvtgf (RUN_PASS, opnd);
            WRITE_L (r);
            CC_IIZZ_FP (r);
            break;

        case ADDF2: case ADDF3:
            r = op_addf (RUN_PASS, opnd, FALSE);
            WRITE_L (r);
            CC_IIZZ_FP (r);
            break;

        case ADDD2: case ADDD3:
            r = op_addd (RUN_PASS, opnd, &rh, FALSE);
            WRITE_Q (r, rh);
            CC_IIZZ_FP (r);
            break;

        case ADDG2: case ADDG3:
            r = op_addg (RUN_PASS, opnd, &rh, FALSE);
            WRITE_Q (r, rh);
            CC_IIZZ_FP (r);
            break;

        case SUBF2: case SUBF3:
            r = op_addf (RUN_PASS, opnd, TRUE);
            WRITE_L (r);
            CC_IIZZ_FP (r);
            break;

        case SUBD2: case SUBD3:
            r = op_addd (RUN_PASS, opnd, &rh, TRUE);
            WRITE_Q (r, rh);
            CC_IIZZ_FP (r);
            break;

        case SUBG2: case SUBG3:
            r = op_addg (RUN_PASS, opnd, &rh, TRUE);
            WRITE_Q (r, rh);
            CC_IIZZ_FP (r);
            break;

        case MULF2: case MULF3:
            r = op_mulf (RUN_PASS, opnd);
            WRITE_L (r);
            CC_IIZZ_FP (r);
            break;

        case MULD2: case MULD3:
            r = op_muld (RUN_PASS, opnd, &rh);
            WRITE_Q (r, rh);
            CC_IIZZ_FP (r);
            break;

        case MULG2: case MULG3:
            r = op_mulg (RUN_PASS, opnd, &rh);
            WRITE_Q (r, rh);
            CC_IIZZ_FP (r);
            break;

        case DIVF2: case DIVF3:
            r = op_divf (RUN_PASS, opnd);
            WRITE_L (r);
            CC_IIZZ_FP (r);
            break;

        case DIVD2: case DIVD3:
            r = op_divd (RUN_PASS, opnd, &rh);
            WRITE_Q (r, rh);
            CC_IIZZ_FP (r);
            break;

        case DIVG2: case DIVG3:
            r = op_divg (RUN_PASS, opnd, &rh);
            WRITE_Q (r, rh);
            CC_IIZZ_FP (r);
            break;

        case ACBF:
            r = op_addf (RUN_PASS, opnd + 1, FALSE);        /* add + index */
            temp = op_cmpfd (RUN_PASS, r, 0, op0, 0);       /* result : limit */
            WRITE_L (r);                                    /* write result */
            CC_IIZP_FP (r);                                 /* set cc's */
            if ((temp & CC_Z) || ((op1 & FPSIGN)?           /* test br cond */
               !(temp & CC_N): (temp & CC_N)))
               BRANCHW (brdisp);
            break;

        case ACBD:
            r = op_addd (RUN_PASS, opnd + 2, &rh, FALSE);
            temp = op_cmpfd (RUN_PASS, r, rh, op0, op1);
            WRITE_Q (r, rh);
            CC_IIZP_FP (r);
            if ((temp & CC_Z) || ((op2 & FPSIGN)?           /* test br cond */
               !(temp & CC_N): (temp & CC_N)))
               BRANCHW (brdisp);
            break;

        case ACBG:
            r = op_addg (RUN_PASS, opnd + 2, &rh, FALSE);
            temp = op_cmpg (RUN_PASS, r, rh, op0, op1);
            WRITE_Q (r, rh);
            CC_IIZP_FP (r);
            if ((temp & CC_Z) || ((op2 & FPSIGN)?           /* test br cond */
               !(temp & CC_N): (temp & CC_N)))
               BRANCHW (brdisp);
            break;

    /* EMODF

            op0     =       multiplier
            op1     =       extension
            op2     =       multiplicand
            op3:op4 =       integer destination (int.wl)
            op5:op6 =       floating destination (flt.wl)
    */

        case EMODF:
            r = op_emodf (RUN_PASS, opnd, &temp, &flg);
            if (op5 < 0)
                Read (RUN_PASS, op6, L_LONG, WA);
            if (op3 >= 0)
                R[op3] = temp;
            else Write (RUN_PASS, op4, temp, L_LONG, WA);
            WRITE_L (r);
            CC_IIZZ_FP (r);
            if (flg) {
                V_INTOV;
                }
            break;

    /* EMODD, EMODG

            op0:op1 =       multiplier
            op2     =       extension
            op3:op4 =       multiplicand
            op5:op6 =       integer destination (int.wl)
            op7:op8 =       floating destination (flt.wq)
    */

        case EMODD:
            r = op_emodd (RUN_PASS, opnd, &rh, &temp, &flg);
            if (op7 < 0) {
                Read (RUN_PASS, op8, L_BYTE, WA);
                Read (RUN_PASS, (op8 + 7) & LMASK, L_BYTE, WA);
                }
            if (op5 >= 0)
                R[op5] = temp;
            else Write (RUN_PASS, op6, temp, L_LONG, WA);
            WRITE_Q (r, rh);
            CC_IIZZ_FP (r);
            if (flg) {
                V_INTOV;
                }
            break;

        case EMODG:
            r = op_emodg (RUN_PASS, opnd, &rh, &temp, &flg);
            if (op7 < 0) {
                Read (RUN_PASS, op8, L_BYTE, WA);
                Read (RUN_PASS, (op8 + 7) & LMASK, L_BYTE, WA);
                }
            if (op5 >= 0)
                R[op5] = temp;
            else Write (RUN_PASS, op6, temp, L_LONG, WA);
            WRITE_Q (r, rh);
            CC_IIZZ_FP (r);
            if (flg) {
                V_INTOV;
                }
            break;

    /* POLY */

        case POLYF:
            op_polyf (RUN_PASS, opnd, acc);
            CC_IIZZ_FP (R[0]);
            break;

        case POLYD:
            op_polyd (RUN_PASS, opnd, acc);
            CC_IIZZ_FP (R[0]);
            break;

        case POLYG:
            op_polyg (RUN_PASS, opnd, acc);
            CC_IIZZ_FP (R[0]);
            break;

    /* Operating system instructions */

        case CHMK: case CHME: case CHMS: case CHMU:
            cc = op_chm (RUN_PASS, opnd, cc, opc);          /* CHMx */
            GET_CUR;                                        /* update cur mode */
            SET_IRQL;                                       /* update intreq */
            break;

        case REI:
            cc = op_rei (RUN_PASS, acc);                    /* REI */
            GET_CUR;                                        /* update cur mode */
            break;

        case LDPCTX:
            op_ldpctx (RUN_PASS, acc);
            break;

        case SVPCTX:
            op_svpctx (RUN_PASS, acc);
            break;

        case PROBER: case PROBEW:
            cc = (cc & CC_C) | op_probe (RUN_PASS, opnd, opc & 1);
            break;

        case MTPR:
            cc = (cc & CC_C) | op_mtpr (RUN_PASS, opnd);
            break;

        case MFPR:
            r = op_mfpr (RUN_PASS, opnd);
            WRITE_L (r);
            CC_IIZP_L (r);
            break;

    /* CIS or emulated instructions */

        case CVTPL:
        case MOVP: case CMPP3: case CMPP4: case CVTLP:
        case CVTPS: case CVTSP: case CVTTP: case CVTPT:
        case ADDP4: case ADDP6: case SUBP4: case SUBP6:
        case MULP: case DIVP: case ASHP: case CRC:
        case MOVTC: case MOVTUC: case MATCHC: case EDITPC:
            cc = op_cis (RUN_PASS, opnd, cc, opc, acc);
            break;

    /* Octaword or reserved instructions */

        case PUSHAO: case MOVAO: case CLRO: case MOVO:
        case TSTH: case MOVH: case MNEGH: case CMPH:
        case CVTBH: case CVTWH: case CVTLH:
        case CVTHB: case CVTHW: case CVTHL: case CVTRHL:
        case CVTFH: case CVTDH: case CVTGH:
        case CVTHF: case CVTHD: case CVTHG:
        case ADDH2: case ADDH3: case SUBH2: case SUBH3:
        case MULH2: case MULH3: case DIVH2: case DIVH3:
        case ACBH: case POLYH: case EMODH:
#if defined(_DEBUG) && !defined(FULL_VAX)
            /* When running under debugger this place can be hit with irrelevant "variable va uninitialized" 
               and "variable spec uninitialized" warnings */
            va = 0;
            spec = 0;
#endif
            cc = op_octa (RUN_PASS, opnd, cc, opc, acc, spec, va);
            if (cc & LSIGN) {                               /* ACBH branch? */
                BRANCHW (brdisp);
                cc = cc & CC_MASK;                          /* mask off flag */
                }
            break;

        case 0xFF:
            op_reserved_ff (RUN_PASS, acc);
            break;
        default:
            RSVD_INST_FAULT;
            break;
        }                                                   /* end case op */
    }                                                       /* end for */
} /* end try*/
sim_catch (sim_exception_ABORT, exabort)
{
    t_stat r = handle_abort(RUN_PASS, exabort, cc, acc, opc);
    if (r)  return r;

    /* 
     * after break from (abortval < 0) in handle_abort, i.e. exceptions and interrupts, 
     * rather than stops, go back to main loop
     */
    goto main_loop;

} /* end catch */
sim_end_try

ABORT (STOP_UNKNOWN);

#if defined(USE_C_TRY_CATCH)
    /* relax compiler false warning */
    return 0;
#endif
}                                                       /* end sim_instr */

#if defined(__GNUC__)
#  if (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__ >= 40603)
#    pragma GCC diagnostic pop
#  endif
#endif


/*
 * Note that ABORT can be thrown while handling previous abort, for example 
 * if stack is invalid while trying to push exception parameters, or if
 * SCB is not readable or SCB vector is corrupt (lower two bits are not 0 or 1).
 * We need to handle these nested ABORT's.
 */
static t_stat handle_abort(RUN_DECL, sim_exception_ABORT* exabort, volatile int32& cc, volatile int32& acc, volatile int32& opc)
{
    sim_try
    {
        int abortval = exabort->code;
        if (cpu_unit->cpu_exception_ABORT == NULL && exabort->isAutoDelete())
        {
            cpu_unit->cpu_exception_ABORT = exabort;
        }
        else
        {
            exabort->checkAutoDelete();
        }

        if (abortval > 0)                                       /* sim stop? */
        {
            PSL = PSL | cc;                                     /* put PSL together */
            pcq_r->setqptr(RUN_PASS, pcq_p);                    /* update pc q ptr */
            return abortval;                                    /* return to SCP */
        }
        else if (abortval < 0)                                  /* mm or rsrv or int */
        {
            int32 i, delta;
            if ((PSL & PSL_FPD) == 0)                           /* FPD? no recovery */
            {
                for (i = 0; i < recqptr; i++)                   /* unwind inst */
                {
                    int32 rrn, rlnt;
                    rrn = RQ_GETRN (recq[i]);                   /* recover reg # */
                    rlnt = DR_LNT (RQ_GETLNT (recq[i]));        /* recovery lnt */
                    if (recq[i] & RQ_DIR)
                        R[rrn] = R[rrn] - rlnt;
                    else
                        R[rrn] = R[rrn] + rlnt;
                }
            }
            PSL = PSL & ~PSL_TP;                                /* clear <tp> */
            recqptr = 0;                                        /* clear queue */
            delta = PC - fault_PC;                              /* save delta PC */
            SETPC (fault_PC);                                   /* restore PC */
            switch (-abortval)                                  /* case on abort code */
            {
            case SCB_RESIN:                                     /* rsrv inst fault */
            case SCB_RESAD:                                     /* rsrv addr fault */
            case SCB_RESOP:                                     /* rsrv opnd fault */
                if (in_ie)                                      /* in exc? panic */
                    ABORT (STOP_INIE);
                cc = intexc (RUN_PASS, -abortval, cc, 0, IE_EXC);         /* take exception */
                GET_CUR;                                        /* PSL<cur> changed */
                break;

            case SCB_CMODE:                                     /* comp mode fault */
            case SCB_ARITH:                                     /* arithmetic fault */
                if (in_ie)                                      /* in exc? panic */
                    ABORT (STOP_INIE);
                cc = intexc (RUN_PASS, -abortval, cc, 0, IE_EXC);         /* take exception */
                GET_CUR;
                in_ie = 1;
                Write (RUN_PASS, SP - 4, fault_p1, L_LONG, WA);          /* write arith param */
                SP = SP - 4;
                in_ie = 0;
                break;

            case SCB_ACV:                                       /* ACV fault */
            case SCB_TNV:                                       /* TNV fault */
                if (in_ie)                                      /* in exception? */
                {
                    if (PSL & PSL_IS)                           /* on is? panic */
                        ABORT (STOP_INIE);
                    cc = intexc (RUN_PASS, SCB_KSNV, cc, 0, IE_SVE);      /* ksnv */
                    GET_CUR;
                }
                else
                {
                    cc = intexc (RUN_PASS, -abortval, cc, 0, IE_EXC);     /* take exception */
                    GET_CUR;
                    in_ie = 1;
                    Write (RUN_PASS, SP - 8, fault_p1, L_LONG, WA);   /* write mm params */
                    Write (RUN_PASS, SP - 4, fault_p2, L_LONG, WA);
                    SP = SP - 8;
                    in_ie = 0;
                }
                break;

            case SCB_MCHK:                                      /* machine check */
                if (in_ie)                                      /* in exc? panic */
                    ABORT (STOP_INIE);
                cc = machine_check (RUN_PASS, fault_p1, opc, cc, delta);        /* system specific */
                in_ie = 0;
                GET_CUR;                                        /* PSL<cur> changed */
                break;

            case 1:                                             /* interrupt */
                break;                                          /* just proceed */

            case (-ABORT_INVSYSOP):
                return STOP_INVSYSOP;

            default:                                            /* other */
                badabo = abortval;                              /* save code */
                ABORT (STOP_UNKABO);                            /* panic */
            }
        }

        return 0;
    }
    sim_catch (sim_exception_ABORT, exabort2)
    {
        return handle_abort(RUN_PASS, exabort2, cc, acc, opc);
    }
    sim_end_try

#if defined(USE_C_TRY_CATCH)
    /* relax compiler false warning */
    return 0;
#endif
}

void cpu_backstep_pc(RUN_DECL)
{
    SETPC(fault_PC);
}


/* Prefetch buffer routine

   Prefetch buffer state

        ibufl, ibufh    =       the prefetch buffer
        ibcnt           =       number of bytes available (0, 4, 8)
        ppc             =       physical PC

   The get_istr routines fetches the indicated number of bytes from
   the prefetch buffer.  Although it is complicated, it is faster
   than calling Read on every byte of the instruction stream.

   If the prefetch buffer has enough bytes, the required bytes are
   extracted from the prefetch buffer and returned. If it does not
   have enough bytes, enough prefetch words are fetched until there
   are.  A longword is only prefetched if data is needed from it,
   so any translation errors are real.
*/

int32 get_istr (RUN_DECL, int32 lnt, int32 acc)
{
    int32 bo = PC & 3;
    int32 sc, val, t;

    while (bo + lnt > ibcnt)                                /* until enuf bytes */
    {
        if (ppc < 0 || VA_GETOFF (ppc) == 0)                /* PPC inv, xpg? */
        {
            ppc = Test (RUN_PASS, (PC + ibcnt) & ~03, RD, &t);        /* xlate PC */
            if (ppc < 0)
                Read (RUN_PASS, (PC + ibcnt) & ~03, L_LONG, RA);
        }
        if (ibcnt == 0)                                     /* fill low */
            ibufl = ReadLP (RUN_PASS, ppc);
        else
            ibufh = ReadLP (RUN_PASS, ppc);                 /* or high */
        ppc = ppc + 4;                                      /* incr phys PC */
        ibcnt = ibcnt + 4;                                  /* incr ibuf cnt */
    }

    PC = PC + lnt;                                          /* incr PC */
    if (lnt == L_BYTE)                                      /* byte? */
        val = (ibufl >> (bo << 3)) & BMASK;
    else if (lnt == L_WORD)                                 /* word? */
    {
        if (bo == 3)
            val = ((ibufl >> 24) & 0xFF) | ((ibufh & 0xFF) << 8);
        else
            val = (ibufl >> (bo << 3)) & WMASK;
    }
    else if (bo)                                           /* unaligned lw? */
    {
        sc = bo << 3;
        val = (((ibufl >> sc) & align[bo]) | (((uint32) ibufh) << (32 - sc)));
    }
    else
        val = ibufl;                                        /* aligned lw */
    if (bo + lnt >= 4)                                      /* retire ibufl? */
    {
        ibufl = ibufh;
        ibcnt = ibcnt - 4;
    }
    return val;
}

/*
 * Process opcode 0xFF.
 *
 * This routine detects VMS bugchecks (BUGW and BUGL pseudo-instructions) and performs
 * the following essential actions:
 *
 *     1) If directed by the setting in bugcheck_ctrl, shut off instruction history recording,
 *        so the history leading to the bugcheck (or as close to it as possible) can be easily
 *        examined, rather than being flushed out of the buffer or lost in overwhelming amount
 *        of subsequent recording.
 *
 *     2) If the guest is VMS and VSMP had been loaded, SYNCW-relevant bugchecks will cause
 *        the dump of CPU and SYNCW state being produced by executing command "CPU INFO".
 *        Note that various items of other CPUs accessed by dump procedure may be in transient
 *        state and their latest updates not propagated yet to this VCPU yet via cache coherence
 *        mechanism. Therefore certain displayed items may have limited validity.
 *
 * Note that BUGW/BUGL instruction may be not recognized and produce none of the above actions
 * if the instuction is located in paged code, crosses page boundary and the second page is not
 * currently mapped. This cannot happen for (2), but can happen for (1) in case of bugchecks in
 * paged kernel-mode or executive-mode code.
 */
static void op_reserved_ff (RUN_DECL, int32 acc)
{
    t_bool r1 = PSL_GETCUR(PSL) == KERN;
    t_bool r2 = PSL_GETCUR(PSL) <= EXEC && (bugcheck_ctrl & 1) != 0 && hst_on;

    t_bool locked = FALSE;

    uint32 bugcode = 0;          /* initialize to suppress false GCC warning */
    char wl = 0;                 /* ... */

    const uint32 VMS_BUG_BADQHDR = 0x478;
    const uint32 VMS_BUG_CPUSANITY = 0x788;
    const uint32 VMS_BUG_CPUSPINWAIT = 0x7A0;
    const uint32 VMS_SEVERITY_MASK = 0x7;

    const char* bugexpl = "";

    if (r1 || r2)
    {
        cpu_database_lock->lock();
        locked = TRUE;
    }

    if (r1 || r2)
    {
        /* 
         * is_bug_instruction may not work properly for paged code
         */
        if (is_bug_instruction(RUN_PASS, acc, fault_PC, & wl, & bugcode))
        {
            if (r1 && sim_vsmp_active && sim_vsmp_os == VAXMP_API_OS_VMS)
            {
                if (wl == 'W')
                    bugcode &= 0xFFFF;
                if ((bugcode & ~VMS_SEVERITY_MASK) == VMS_BUG_BADQHDR)
                    bugexpl = " (BADQHDR)";
                if ((bugcode & ~VMS_SEVERITY_MASK) == VMS_BUG_CPUSPINWAIT)
                    bugexpl = " (CPUSPINWAIT)";
                /* VAX MP syncw is not meant to protect against CPUSANITY yet */
                if ((bugcode & ~VMS_SEVERITY_MASK) == VMS_BUG_CPUSANITY && FALSE)
                    bugexpl = " (CPUSANITY)";
                r1 = bugexpl[0] != '\0';
            }
            else
            {
                r1 = FALSE;
            }
        }
        else
        {
            r1 = r2 = FALSE;
        }
    }

    if (r1 || r2)
    {
        if (r2)
        {
            /* shut off history recording */
            hst_on = FALSE;
        }

        const char* divider = "***************************************************************************";

        smp_printf("\n\r\n%s\n\n\r", divider);
        if (sim_log)
            fprintf(sim_log, "\n%s\n\n", divider);

        if (wl == 'W')
        {
            smp_printf("%s detected OpenVMS bugcheck BUGW %04X%s on CPU%d\n\r", sim_name, bugcode, bugexpl, cpu_unit->cpu_id);
            if (sim_log)
                fprintf(sim_log, "%s detected OpenVMS bugcheck BUGW %04X%s on CPU%d\n", sim_name, bugcode, bugexpl, cpu_unit->cpu_id);
        }
        else
        {
            smp_printf("%s detected OpenVMS bugcheck BUGL %08X%s on CPU%d\n\r", sim_name, bugcode, bugexpl, cpu_unit->cpu_id);
            if (sim_log)
                fprintf(sim_log, "%s detected OpenVMS bugcheck BUGL %08X%s on CPU%d\n", sim_name, bugcode, bugexpl, cpu_unit->cpu_id);
        }

        if (r2)
        {
            smp_printf("\n\rStopping instruction history recording.\n\r");
            if (sim_log)
                fprintf(sim_log, "\nStopping instruction history recording.\n");
        }

        if (r1)
        {
            smp_printf("\n\rCPU and SYNCW state dump follows. Remember it was taken on the fly,\n\r");
            smp_printf("therefore certain items displayed for other CPUs may have limited validity.\n\n\r");
            if (sim_log)
            {
                fprintf(sim_log, "\nCPU and SYNCW state dump follow. Remember it was taken on the fly,\n");
                fprintf(sim_log, "therefore certain items displayed for other CPUs may have limited validity.\n\n");
            }

            /* perform "CPU INFO" command */
            cpu_cmd_info(smp_stdout, &cpu_dev, cpu_unit, 0, "");
            if (sim_log)
                cpu_cmd_info(sim_log, &cpu_dev, cpu_unit, 0, "");

            smp_printf("\n\rOpenVMS bugcheck sequence will now be executed.\n\r");
            if (sim_log)
                fprintf(sim_log, "\nOpenVMS bugcheck sequence will now be executed.\n");
        }

        smp_printf("\n\r%s\n\n\r", divider);
        if (sim_log)
            fprintf(sim_log, "\n%s\n\n", divider);
    }

    if (locked)
        cpu_database_lock->unlock();

    RSVD_INST_FAULT;
}

/* 
 * Detect and decode BUGW/BUGL pseudo-instruction only if it is wholly resident in memory.
 * If only partially resident, do not generate page faults and return FALSE;
 */
static t_bool is_bug_instruction(RUN_DECL, int32 acc, int32 va, char* wl, uint32* bug_code)
{
    int32 mstat;

    if (Test (RUN_PASS, va, RA, &mstat) < 0 ||
        Test (RUN_PASS, va + 3, RA, &mstat) < 0)
    {
        return FALSE;
    }

    if (Read (RUN_PASS, va, L_BYTE, RA) != 0xFF)
        return FALSE;

    switch (Read (RUN_PASS, va + 1, L_BYTE, RA))
    {
    case 0xFD:
        if (Test (RUN_PASS, va + 5, RA, &mstat) < 0)
            return FALSE;
        *wl = 'L';
        *bug_code = Read (RUN_PASS, va + 2, L_LONG, RA);
        return TRUE;

    case 0xFE:
        *wl = 'W';
        *bug_code = Read (RUN_PASS, va + 2, L_WORD, RA);
        return TRUE;

    default:
        return FALSE;
    }
}

/* Read octaword specifier */

int32 ReadOcta (RUN_DECL, int32 va, int32 *opnd, int32 j, int32 acc)
{
    opnd[j++] = Read (RUN_PASS, va, L_LONG, acc);
    opnd[j++] = Read (RUN_PASS, va + 4, L_LONG, acc);
    opnd[j++] = Read (RUN_PASS, va + 8, L_LONG, acc);  
    opnd[j++] = Read (RUN_PASS, va + 12, L_LONG, acc);  
    return j;
}


static void op_adawi(RUN_DECL, int32 *opnd, int32 acc, int32 spec, int32 rn, int32 va, volatile int32& cc)
{
    int32 r, temp;
    int32 pa_op2 = 0;      /* initialize to prevent false GCC warning */

    if (use_native_interlocked)
    {
        /*
         * Handle ADAWI via native host routine if available,
         * provided that the destination operand is in VAX main memory space
         * (not general-purpose register, and not IO or processor register space) 
         */
        if (op1 < 0)
        {
            if (op2 & 1)                            /* mem? chk align */
                RSVD_OPND_FAULT;
            pa_op2 = TestMark (RUN_PASS, op2, WA, NULL);
            if (ADDR_IS_MEM(pa_op2))
            {
                if (PSL_GETIPL(PSL) >= syncw.ipl_resched)
                    syncw_enter_ilk(RUN_PASS);
                cc = smp_native_adawi(M, pa_op2, (uint16) op0);
                if (cc & CC_V) { INTOV; }
                return;
            }
        }
        /*
         * Fall through if "sum" operand is not in VAX main memory space
         */
    }

    sim_try_volatile InterlockedOpLock iop(RUN_PASS);
    if (op1 >= 0)
    {
        temp = R[op1] & WMASK;                      /* reg? ADDW2 */
    }
    else
    {
        if (op2 & 1)                                /* mem? chk align */
            RSVD_OPND_FAULT;
         /* 
         * According to "VAX Architecture Reference Manual" and "VAX Architecture Standard",
         * ADAWI is the only interlocked instruction allowed on IO space, with interlock
         * supported in Unibus space and implementation-dependent status in non-Unibus IO space.
         */
        if (use_native_interlocked)
            iop.phys_lock(pa_op2);                  /* base longword will be interlocked by iop.phys_lock(), not target word */
        else
            iop.virt_lock(op2, acc);                /* base longword will be interlocked by iop.virt_lock(), not target word */
        temp = Read (RUN_PASS, op2, L_WORD, WA);    /* ok, ADDW2 */
    }
    r = (op0 + temp) & WMASK;
    iop.wmb = TRUE;
    if (PSL_GETIPL(PSL) >= syncw.ipl_resched)
        syncw_enter_ilk(RUN_PASS);
    WRITE_W (r);
    CC_ADD_W (r, op0, temp);                        /* set cc's */
}


/* Schedule idle before the next instruction */
void cpu_idle (RUN_DECL)
{
    /* When SMP is active, sleeping is controlled by paravirtualization module via VAXMP API */
    if (! sim_vsmp_active)
        sim_activate_abs (cpu_unit, 0);
}

t_stat cpu_idle_svc (RUN_SVC_DECL, UNIT *uptr)
{
    if (cpu_unit->cpu_dostop)
        return STOP_LOOP;

    // RUN_SVC_CHECK_CANCELLED(uptr);  // not required for per-CPU devices
    if (sim_idle_enab && !sim_vsmp_active)
        return sim_idle(RUN_PASS, TMR_CLK, FALSE, UINT32_MAX);
    else
        return SCPE_OK;
}

/*
 * Reset current CPU, usually will be called by reset_all for each CPU, 0 to (sim_ncpus - 1).
 *
 * Can also be called on individual CPU basis by reset_cmd ("reset CPU2"), or when CPU is being
 * created (init_cpu_unit_0, cpu_create_cpus).
 *
 * Can also be called for secondary CPU when it is about to be started and another CPU performs
 * pre-staring initialization on it with cpu_start_secondary.
 */

t_stat cpu_reset (DEVICE *dptr)
{
    RUN_SCOPE;

    cpu_unit->cpu_dostop = FALSE;

    cpu_unit->cpu_synclk_protect = FALSE;
    cpu_unit->cpu_synclk_protect_os = 0;
    cpu_unit->cpu_synclk_protect_dev = 0;
    cpu_unit->cpu_synclk_pending = SynclkNotPending;

    cpu_unit->cpu_context.reset(cpu_unit);
    cpu_unit->cpu_intreg.reset();

    cpu_unit->cpu_con_rei_on = FALSE;

    mem_err = 0;
    crd_err = 0;
    PSL = PSL_IS | PSL_IPL1F;
    SISR = 0;
    ASTLVL = 4;
    mapen = 0;
    FLUSH_ISTR;
    cpu_on_clear_mapen(RUN_PASS);
    pcq_r = find_reg ("PCQ", NULL, dptr);
    if (pcq_r)
        pcq_r->setqptr(RUN_PASS, 0);
    else
        return SCPE_IERR;

    if (M == NULL)
        M = (uint32*) calloc_aligned (((uint32) MEMSIZE) >> 2, sizeof (uint32), /*SMP_MAXCACHELINESIZE*/ 512);
    if (M == NULL)
        return SCPE_MEM;

    /* reset clock queue and release entries back to freelist */
    clock_queue_entry* cqe;
    while ((cqe = cpu_unit->clock_queue) != NULL)
    {
        cpu_unit->clock_queue = cqe->next;

        cqe->next = cpu_unit->clock_queue_freelist;
        cpu_unit->clock_queue_freelist = cqe;
    }
    cpu_unit->cpu_requeue_syswide_pending = FALSE;

    /* mark all SSC timers as inactive */
    cpu_unit->sysd_active_mask = 0;

    cpu_database_lock->lock();
    if (cpu_unit->is_running())
        cpu_unit->cpu_state = CPU_STATE_RUNNABLE;
    cpu_running_set.clear(cpu_unit->cpu_id);
    sim_mp_active_update();
    cpu_database_lock->unlock();

    cpu_reevaluate_thread_priority(RUN_PASS);

    if (cpu_unit->is_primary_cpu())
    {
        // do this just once for the reset cycle across all CPUs, and only from the primary CPU
        hlt_pin = 0;
        sim_brk_types = sim_brk_dflt = SWMASK ('E');
        use_native_interlocked = FALSE;
        syncw_reset();
        return build_dib_tab ();
    }
    else
    {
        syncw_leave_all(RUN_PASS, SYNCW_OVERRIDE_ALL | SYNCW_ENABLE_CPU);
        return SCPE_OK;
    }
}

/* Memory examine */

t_stat cpu_ex (t_value *vptr, t_addr exta, UNIT *uptr, int32 sw)
{
    RUN_SCOPE;
    return cpu_ex_run (RUN_PASS, vptr, exta, uptr, sw);
}

t_stat cpu_ex_run (RUN_DECL, t_value *vptr, t_addr exta, UNIT *uptr, int32 sw)
{
int32 st;
uint32 addr = (uint32) exta;

if (vptr == NULL) 
    return SCPE_ARG;
if (sw & SWMASK ('V')) {
    int32 acc = cpu_get_vsw (RUN_PASS, sw);
    addr = Test (RUN_PASS, addr, acc, &st);
    }
else addr = addr & PAMASK;
if (ADDR_IS_MEM (addr) || ADDR_IS_CDG (addr) ||
    ADDR_IS_ROM (addr) || ADDR_IS_NVR (addr)) {
    *vptr = (uint32) ReadB (RUN_PASS, addr);
    return SCPE_OK;
    }
return SCPE_NXM;
}

/* Memory deposit */

t_stat cpu_dep (t_value val, t_addr exta, UNIT *uptr, int32 sw)
{
RUN_SCOPE;
int32 st;
uint32 addr = (uint32) exta;

if (sw & SWMASK ('V')) {
    int32 acc = cpu_get_vsw (RUN_PASS, sw);
    addr = Test (RUN_PASS, addr, acc, &st);
    }
else addr = addr & PAMASK;
if (ADDR_IS_MEM (addr) || ADDR_IS_CDG (addr) ||
    ADDR_IS_NVR (addr)) {
    WriteB (RUN_PASS, addr, (int32) val);
    return SCPE_OK;
    }
if (ADDR_IS_ROM (addr)) {
    rom_wr_B (RUN_PASS, addr, (int32) val);
    return SCPE_OK;
    }
return SCPE_NXM;
}

/* Memory allocation */

t_stat cpu_set_size (UNIT *uptr, int32 val, char *cptr, void *desc)
{
    RUN_SCOPE;
    int32 mc = 0;
    uint32 i, clim;
    uint32 *nM = NULL;

    if (val <= 0 || val > MAXMEMSIZE_X)
        return SCPE_ARG;
    for (i = val; i < MEMSIZE; i = i + 4)
        mc = mc | M[i >> 2];
    if (mc != 0 && !get_yn ("Really truncate memory [N]?", FALSE))
        return SCPE_OK;
    nM = (uint32 *) calloc_aligned (val >> 2, sizeof (uint32), /*SMP_MAXCACHELINESIZE*/ 512);
    if (nM == NULL)
        return SCPE_MEM;
    clim = (uint32) (((uint32) val) < MEMSIZE ? val : MEMSIZE);
    for (i = 0; i < clim; i = i + 4)
        nM[i >> 2] = M[i >> 2];
    free_aligned ((void*) M);
    M = nM;
    CPU_UNIT* sv_cpu_unit = cpu_unit;
    /*
     * replicate new size across all other CPUs and flush prefetch
     */
    for (uint32 k = 0;  k < sim_ncpus;  k++)
    {
        CPU_UNIT* cpu_unit = cpu_units[k];
        cpu_unit->capac = val;
        FLUSH_ISTR;
    }
    cpu_unit = sv_cpu_unit;
    sim_ws_prefaulted = FALSE;
    sim_ws_settings_changed = TRUE;
    return SCPE_OK;
}

/* Virtual address translation */

t_stat cpu_show_virt (SMP_FILE *of, UNIT *uptr, int32 val, void *desc)
{
RUN_SCOPE;
t_stat r;
char *cptr = (char *) desc;
uint32 va, pa;
int32 st;
static const char *mm_str[] = {
    "Access control violation",
    "Length violation",
    "Process PTE access control violation",
    "Process PTE length violation",
    "Translation not valid",
    "Internal error",
    "Process PTE translation not valid"
    };

if (cptr) {
    va = (uint32) get_uint (cptr, 16, 0xFFFFFFFF, &r);
    if (r == SCPE_OK) {
        int32 acc = cpu_get_vsw (RUN_PASS, sim_switches);
        pa = Test (RUN_PASS, va, acc, &st);
        if (st == PR_OK)
            fprintf (of, "Virtual %-X = physical %-X\n", va, pa);
        else fprintf (of, "Virtual %-X: %s\n", va, mm_str[st]);
        return SCPE_OK;
        }
    }
fprintf (of, "Invalid argument\n");
return SCPE_OK;
}

/* Get access mode for examine, deposit, show virtual */

int32 cpu_get_vsw (RUN_DECL, int32 sw)
{
int32 md;

set_map_reg (RUN_PASS);                                         /* update dyn reg */
if (sw & SWMASK ('K'))
    md = KERN;
else if (sw & SWMASK ('E')) 
    md = EXEC;
else if (sw & SWMASK ('S'))
    md = SUPV;
else if (sw & SWMASK ('U'))
    md = USER;
else md = PSL_GETCUR (PSL);
return ACC_MASK (md);
}

/* Set history */

static const char* find_abbrev(const char* cmd, const char** cmd_options);

t_stat cpu_set_hist (UNIT *uptr, int32 val, char *cptr, void *desc)
{
    uint32 k, i, lnt = 0;                                       /* init lnt to suppress false GCC warning */
    t_stat r;
    char* ptok;
    const char* xp;
    t_bool sync = TRUE;
    t_bool lnt_set = FALSE;
    t_bool sync_set = FALSE;

#if defined (__x86_64__)
    smp_check_aligned(& hst_stamp);
#endif

#if defined (__x86_32__)
    smp_check_aligned(& hst_stamp_counter);
    smp_check_aligned(& hst_stamp_epoch);
#endif

    if (cptr == NULL)
    {
        // reset history buffers
        if (hst_lnt == 0)
            return SCPE_OK;

        for (k = 0;  k < sim_ncpus;  k++)
        {
            CPU_UNIT* xcpu = cpu_units[k];
            if (xcpu->cpu_hst == NULL)
            {
                xcpu->cpu_hst = (InstHistory*) calloc (hst_lnt, sizeof (InstHistory));
                if (xcpu->cpu_hst == NULL)
                    return SCPE_MEM;
            }
            for (i = 0; i < hst_lnt; i++)
                xcpu->cpu_hst[i].isRecorded = FALSE;
            UINT64_SET_ZERO(xcpu->cpu_hst_stamp);
            xcpu->cpu_hst_index = 0;
            hst_reinit_stamp();
            hst_on = TRUE;
        }

        return SCPE_OK;
    }

    ptok = strtok (cptr,"/");
    while (ptok)
    {
        const char* cmd_options[] = { "sync", "unsync", NULL };
        if (xp = find_abbrev(ptok, cmd_options))
        {
            if (sync_set)
                return SCPE_ARG;
            if (0 == strcmp(xp, "sync"))
                sync = TRUE;
            else if (0 == strcmp(xp, "unsync"))
                sync = FALSE;
            sync_set = TRUE;
        }
        else
        {
            if (lnt_set)
                return SCPE_ARG;
            lnt = (int32) get_uint (ptok, 10, HIST_MAX, &r);
            if (r != SCPE_OK || lnt && lnt < HIST_MIN)
                return SCPE_ARG;
            lnt_set = TRUE;
        }
        ptok = strtok (NULL, "/");
    }

    if (! lnt_set)
        return SCPE_ARG;

    cpu_free_history ();

    if (lnt)
    {
        for (k = 0;  k < sim_ncpus;  k++)
        {
            CPU_UNIT* xcpu = cpu_units[k];
            xcpu->cpu_hst = (InstHistory*) calloc (lnt, sizeof (InstHistory));
            if (xcpu->cpu_hst == NULL)
            {
                cpu_free_history ();
                return SCPE_MEM;
            }
        }

        hst_lnt = lnt;
        hst_sync = sync;
        hst_reinit_stamp();
        hst_on = TRUE;
    }

    return SCPE_OK;
}

static const char* find_abbrev(const char* cmd, const char** cmd_options)
{
    if (! (cmd && *cmd))
        return NULL;

    int cmd_len = (int) strlen(cmd);
    int candidate = -1;
    for (int k = 0; cmd_options[k] != NULL;  k++)
    {
        int opt_len = (int) strlen(cmd_options[k]);
        if (cmd_len <= opt_len)
        {
            int len = imin(cmd_len, opt_len);
#if defined(_WIN32)
            if (0 == strnicmp(cmd, cmd_options[k], len))
#else
            if (0 == strncasecmp(cmd, cmd_options[k], len))
#endif
            {
                if (candidate >= 0)
                    return NULL;
                candidate = k;
            }
        }
    }

    if (candidate < 0)
        return NULL;

    return cmd_options[candidate];
}

static void cpu_free_history ()
{
    for (uint32 k = 0;  k < sim_ncpus;  k++)
    {
        CPU_UNIT* xcpu = cpu_units[k];
        if (xcpu->cpu_hst != NULL)
        {
            free (xcpu->cpu_hst);
            xcpu->cpu_hst = NULL;
        }
        UINT64_SET_ZERO(xcpu->cpu_hst_stamp);
        xcpu->cpu_hst_index = 0;
    }

    hst_lnt = 0;
    hst_on = FALSE;
}

t_bool cpu_stop_history ()
{
    if (hst_lnt)
    {
        cpu_free_history ();
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

/* Show history */

extern const char *opcode[];
extern t_value* sim_eval;
extern t_stat fprint_sym (SMP_FILE *ofile, t_addr addr, t_value *val, UNIT *uptr, int32 sw);
typedef struct
{
    InstHistory* pInstHistory;
    uint32       cpu_id;
}
HistListEntry;

static int
#ifdef _WIN32
__cdecl
#endif
qsort_hle_compare(const void* vp1, const void* vp2)
{
    HistListEntry* h1 = (HistListEntry*) vp1;
    HistListEntry* h2 = (HistListEntry*) vp2;
    if (UINT64_LT(h1->pInstHistory->stamp, h2->pInstHistory->stamp))
        return -1;
    if (UINT64_EQ(h1->pInstHistory->stamp, h2->pInstHistory->stamp))
        return 0;
    else
        return 1;
} 

t_stat cpu_show_hist (SMP_FILE *st, UNIT *uptr, int32 val, void *desc)
{
    RUN_SCOPE;
    int32 max_lnt = 0;                                      /* init max_lnt to suppress false GCC warning */
    t_bool max_lnt_set = FALSE;
    t_bool all_cpus = FALSE;
    t_bool all_cpus_set = FALSE;

    if (hst_lnt == 0)                                       /* enabled? */
        return SCPE_NOFNC;

    char* cptr = (char*) desc;

    if (cptr)
    {
        const char* xp;
        const char* ptok = strtok (cptr,"/");
        while (ptok)
        {
            const char* cmd_options[] = { "sync", "unsync", NULL };
            if (xp = find_abbrev(ptok, cmd_options))
            {
                if (all_cpus_set)
                    return SCPE_ARG;
                if (0 == strcmp(xp, "sync"))
                    all_cpus = TRUE;
                else if (0 == strcmp(xp, "unsync"))
                    all_cpus = FALSE;
                all_cpus_set = TRUE;
            }
            else
            {
                if (max_lnt_set)
                    return SCPE_ARG;
                t_stat r;
                max_lnt = (int32) get_uint ((char*) ptok, 10, 0xFFFFFFFF, &r);
                if (r != SCPE_OK)
                    return SCPE_ARG;
                max_lnt_set = TRUE;
            }
            ptok = strtok (NULL, "/");
        }
    }

    if (all_cpus && !hst_sync)
    {
        smp_printf ("Unable to display synchronized instruction history for all CPUs because SYNC option was not requested for recording\n");
        if (sim_log)
            fprintf (sim_log, "Unable to display synchronized instruction history for all CPUs because SYNC option was not requested for recording\n");
        return SCPE_NOFNC;
    }

    if (max_lnt_set)
    {
        max_lnt = imin(max_lnt, (int32) (all_cpus ? hst_lnt * sim_ncpus : hst_lnt));
    }
    else
    {
        max_lnt = all_cpus ? hst_lnt * sim_ncpus : hst_lnt;
    }

    HistListEntry* hlist = new HistListEntry[all_cpus ? hst_lnt * sim_ncpus : hst_lnt];
    if (hlist == NULL)
        return SCPE_MEM;
    int avl_lnt = 0;

    for (uint32 k = 0;  k < sim_ncpus;  k++)
    {
        CPU_UNIT* xcpu = cpu_units[k];
        if (xcpu->cpu_hst && (all_cpus || xcpu == cpu_unit))
        {
            for (uint32 i = 0;  i < hst_lnt;  i++)
            {
                if (xcpu->cpu_hst[i].isRecorded)
                {
                    hlist[avl_lnt].pInstHistory = & xcpu->cpu_hst[i];
                    hlist[avl_lnt++].cpu_id = xcpu->cpu_id;
                }
            }
        }
    }

    fprintf (st, "CPU PC       PSL       IR\n\n");
    fprintf (st, "--- -------- --------- -------------------------\n\n");

    if (avl_lnt == 0)
    {
        delete [] hlist;
        return SCPE_OK;
    }

    qsort(hlist, avl_lnt, sizeof(HistListEntry), qsort_hle_compare);

    int32 istart = 0;
    if (avl_lnt > max_lnt)
        istart = avl_lnt - max_lnt;

    for (int32 i = istart; i < avl_lnt ; i++)
    {
        InstHistory* h = hlist[i].pInstHistory;

        fprintf(st, " %02d %08X %08X| ", hlist[i].cpu_id, h->iPC, h->iPSL);  /* CPU ID, PC, PSL */

        int32 numspec = drom[h->opc][0] & DR_NSPMASK;       /* #specifiers */

        if (opcode[h->opc] == NULL)                         /* undefined? */
        {
            fprintf (st, "%03X (undefined)", h->opc);
        }
        else if (h->iPSL & PSL_FPD)                         /* FPD set? */
        {
            fprintf (st, "%s FPD set", opcode[h->opc]);
        }
        else                                                /* normal */
        {
            for (int32 i = 0; i < INST_SIZE; i++)
                sim_eval[i] = h->inst[i];

            if ((fprint_sym (st, h->iPC, sim_eval, cpu_unit, SWMASK ('M'))) > 0)
                fprintf (st, "%03X (undefined)", h->opc);

            if ((numspec > 1) ||
                ((numspec == 1) && (drom[h->opc][1] < BB)))
            {
                if (cpu_show_opnd (st, h, 0))               /* operands; more? */
                {
                    if (cpu_show_opnd (st, h, 1))           /* 2nd line; more? */
                    {
                        cpu_show_opnd (st, h, 2);           /* octa, 3rd/4th */
                        cpu_show_opnd (st, h, 3);
                    }
                }
            }
        }

        /* end line */
        fputc ('\n', st);
    }

    delete [] hlist;

    return SCPE_OK;
}

t_bool cpu_show_opnd (SMP_FILE *st, InstHistory *h, int32 line)
{

int32 numspec, i, j, disp;
t_bool more;

numspec = drom[h->opc][0] & DR_NSPMASK;                 /* #specifiers */
fputs ("\n                      ", st);                 /* space */
for (i = 1, j = 0, more = FALSE; i <= numspec; i++) {   /* loop thru specs */
    disp = drom[h->opc][i];                             /* specifier type */
    if (disp == RG)                                     /* fix specials */
        disp = RQ;
    else if (disp >= BB)
        break;                         /* ignore branches */
    else switch (disp & (DR_LNMASK|DR_ACMASK)) {

    case RB: case RW: case RL:                          /* read */
    case AB: case AW: case AL: case AQ: case AO:        /* address */
    case MB: case MW: case ML:                          /* modify */
        if (line == 0)
            fprintf (st, " %08X", h->opnd[j]);
        else fputs ("         ", st);
        j = j + 1;
        break;
    case RQ: case MQ:                                   /* read, modify quad */
        if (line <= 1)
            fprintf (st, " %08X", h->opnd[j + line]);
        else fputs ("         ", st);
        if (line == 0)
            more = TRUE;
        j = j + 2;
        break;
    case RO: case MO:                                   /* read, modify octa */
        fprintf (st, " %08X", h->opnd[j + line]);
        more = TRUE;
        j = j + 4;
        break;
    case WB: case WW: case WL: case WQ: case WO:        /* write */
        if (line == 0)
            fprintf (st, " %08X", h->opnd[j + 1]);
        else fputs ("         ", st);
        j = j + 2;
        break;
        }                                       /* end case */
    }                                           /* end for */
return more;
}

struct os_idle {
    char        *name;
    uint32      mask;
    };

static struct os_idle os_tab[] = {
    { "VMS", VAX_IDLE_VMS },
    { "NETBSD", VAX_IDLE_ULTOLD },
    { "ULTRIX", VAX_IDLE_ULT },
    { "ULTRIXOLD", VAX_IDLE_ULTOLD },
    { "OPENBSD", VAX_IDLE_QUAD },
    { "QUASIJARUS", VAX_IDLE_QUAD },
    { "32V", VAX_IDLE_QUAD },
    { "ALL", VAX_IDLE_VMS|VAX_IDLE_ULTOLD|VAX_IDLE_ULT|VAX_IDLE_QUAD },
    { NULL, 0 }
    };

/* Set and show idle */

t_stat cpu_set_idle (UNIT *uptr, int32 val, char *cptr, void *desc)
{
    char* xarg = NULL;

    if (! (cptr && *cptr))
        return sim_set_idle (uptr, val, NULL, desc);

    const char* ptok = strtok (cptr,"/");
    while (ptok)
    {
        t_bool tok_sysname = FALSE;
        for (uint32 i = 0;  os_tab[i].name != NULL;  i++)
        {
            if (strcmp (os_tab[i].name, ptok) == 0)
            {
                cpu_idle_type = i + 1;
                cpu_idle_mask = os_tab[i].mask;
                tok_sysname = TRUE;
                break;
            }
        }
        if (! tok_sysname)
        {
            if (xarg)
            {
                free(xarg);
                return SCPE_ARG;
            }
            else
            {   
                if ((xarg = dupstr(ptok)) == NULL)
                    return SCPE_MEM;
            }
        }
        ptok = strtok (NULL, "/");
    }

    t_stat res = sim_set_idle (uptr, val, xarg, desc);
    if (xarg)  free (xarg);
    return res;
}

t_stat cpu_show_idle (SMP_FILE *st, UNIT *uptr, int32 val, void *desc)
{
    if (sim_idle_enab && cpu_idle_type != 0)
        fprintf (st, "idle enabled=%s, stability wait = %ds", os_tab[cpu_idle_type - 1].name, sim_idle_stable);
    else if (sim_idle_enab)
        fprintf (st, "idle enabled, stability wait = %ds", sim_idle_stable);
    else
        fprintf (st, "idle disabled");
    return SCPE_OK;
}

/*
 * cpu_cmd_info is normally called on the console thread,
 * however it can also be called on VCPU thread from op_reserved_ff
 */
t_stat cpu_cmd_info (SMP_FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, char *cptr)
{
    run_scope_context* rscx = run_scope_context::get_current();
    CPU_UNIT* sv_cpu_unit = rscx->cpu_unit;
    CPU_UNIT* cpu_unit;
    const char* modes = "KESU";
    const char* cprio;
    int tprio;
    t_bool none;
    char c;

    /*
     * Display basic VCPUs state
     */
    fprintf(st, "CP                   MO IP LI  THRD  ACLK\n");
    fprintf(st, "U#    PC      PSL    DE L# RQ  PRIO  AIPI\n");
    fprintf(st, "-- -------- -------- -- -- -- ------ ----\n");

    for (uint32 cpu_ix = 0;  cpu_ix < sim_ncpus;  cpu_ix++)
    {
        rscx->cpu_unit = cpu_unit = cpu_units[cpu_ix];
        fprintf(st, "%02d ", cpu_unit->cpu_id);
        if (cpu_running_set.is_set(cpu_ix) != (cpu_unit->cpu_state == CPU_STATE_RUNNING))
        {
            fprintf(st, "**INCONSISTENT**\n");
            continue;
        }

        if (cpu_unit->cpu_state != CPU_STATE_RUNNING)
        {
            fprintf(st, "(%s)\n", cpu_describe_state(cpu_unit));
            continue;
        }

        fprintf(st, "%08X %08X ", PC, PSL);
        c = modes[PSL_GETCUR(PSL)];
        if (PSL & PSL_IS)  c = 'I';
        fprintf(st, "%c%c %2d %2d ", c, modes[PSL_GETPRV(PSL)], PSL_GETIPL(PSL), cpu_unit->cpu_context.highest_irql);

        switch (cpu_unit->cpu_thread_priority)
        {
        case SIMH_THREAD_PRIORITY_INVALID:
            cprio = "INV";
            break;
        case SIMH_THREAD_PRIORITY_CPU_RUN:
            cprio = "RUN";
            break;
        case SIMH_THREAD_PRIORITY_CPU_CRITICAL_OS:
            cprio = "OS ";
            break;
        case SIMH_THREAD_PRIORITY_CPU_CRITICAL_OS_HI:
            cprio = "OSH";
            break;
        case SIMH_THREAD_PRIORITY_CPU_CRITICAL_VM:
            cprio = "VM ";
            break;
        case SIMH_THREAD_PRIORITY_CPU_CALIBRATION:
            cprio = "CAL";
            break;
        default:
            cprio = "???";
            break;
        }

        fprintf(st, "%s", cprio);
        tprio = smp_get_thread_os_priority(cpu_unit->cpu_thread);
        if (tprio < 0)
            fprintf(st, " ??");
        else
            fprintf(st, "%3d", tprio);
        fprintf(st, "   %c%c", cpu_unit->cpu_active_clk_interrupt ? 'C' : ' ',
                               cpu_unit->cpu_active_ipi_interrupt ? 'I' : ' ');
        fprintf(st, "\n");
    }

    fprintf(st, "\n");

    /*
     * Display pending interrupts
     */
    fprintf(st, "CP  External pending interrupts          \n");
    fprintf(st, "U#  Local and buffered interrupts        \n");
    fprintf(st, "-- --------------------------------------\n");

    for (uint32 cpu_ix = 0;  cpu_ix < sim_ncpus;  cpu_ix++)
    {
        rscx->cpu_unit = cpu_unit = cpu_units[cpu_ix];
        if (cpu_unit->cpu_state != CPU_STATE_RUNNING)
            continue;

        /* Dispay external interrupts */
        fprintf(st, "%02d XT: ", cpu_unit->cpu_id);
        none = TRUE;
        cpu_unit->cpu_intreg.show_external(st, none);
        if (none)  fprintf(st, "(none)");
        fprintf(st, "\n");

        /* Dispay local and buffered interrupts */
        fprintf(st, "   LC: ");
        none = TRUE;
        if (hlt_pin)
        {
            fprintf(st, "%s%s", none ? "" : " ", "HLTPIN");
            none = FALSE;
        }
        if (mem_err)
        {
            fprintf(st, "%s%s", none ? "" : " ", "MEMERR");
            none = FALSE;
        }
        if (crd_err)
        {
            fprintf(st, "%s%s", none ? "" : " ", "CRDERR");
            none = FALSE;
        }

        cpu_unit->cpu_intreg.show_local(st, none);

        if (SISR)
        {
            for (int32 i = IPL_SMAX;  i >= 1;  i--)
            {
                if (SISR & (1 << i))
                {
                    fprintf(st, "%s%d", none ? "" : " ", i);
                    none = FALSE;
                }
            }
        }

        if (none)  fprintf(st, "(none)");
        fprintf(st, "\n");
    }

    rscx->cpu_unit = sv_cpu_unit;

    fprintf(st, "\n");
    syncw_display(st);

    return SCPE_OK;
}

void InterruptRegister::show_external(SMP_FILE* st, t_bool& none)
{
    if (smp_var(changed))
        show_aux_2(st, "NEW", none);

    show_aux(st, (uint32*) irqs, none);
}

void InterruptRegister::show_local(SMP_FILE* st, t_bool& none)
{
    show_aux(st, local_irqs, none);
}

void InterruptRegister::show_aux(SMP_FILE* st, uint32* rqs, t_bool& none)
{
    if (rqs[IPL_IPIRMB] & (1 << INT_V_IPIRMB))
        show_aux_2(st, "IPIRMB", none);
    if (rqs[IPL_SECEXIT] & (1 << INT_V_SECEXIT))
        show_aux_2(st, "SECEXIT", none);
    if (rqs[IPL_SYNCWSYS] & (1 << INT_V_SYNCWSYS))
        show_aux_2(st, "SYNCWSYS", none);
    if (rqs[IPL_SYNCLK] & (1 << INT_V_SYNCLK))
        show_aux_2(st, "SYNCLK", none);
    if (rqs[IPL_ASYNC_IO] & (1 << INT_V_ASYNC_IO))
        show_aux_2(st, "ASYNC_IO", none);
    if (rqs[IPL_STOP] & (1 << INT_V_STOP))
        show_aux_2(st, "STOP", none);
    if (rqs[IPL_CLK] & (1 << INT_V_CLK))
        show_aux_2(st, "CLK", none);
    if (rqs[IPL_IPINTR] & (1 << INT_V_IPINTR))
        show_aux_2(st, "IPINTR", none);
    if (rqs[IPL_RQ] & (1 << INT_V_RQ))
        show_aux_2(st, "RQ", none);
    if (rqs[IPL_RL] & (1 << INT_V_RL))
        show_aux_2(st, "RL", none);
    if (rqs[IPL_DZRX] & (1 << INT_V_DZRX))
        show_aux_2(st, "DZRX", none);
    if (rqs[IPL_DZTX] & (1 << INT_V_DZTX))
        show_aux_2(st, "DZTX", none);
    if (rqs[IPL_TS] & (1 << INT_V_TS))
        show_aux_2(st, "TS", none);
    if (rqs[IPL_TQ] & (1 << INT_V_TQ))
        show_aux_2(st, "TQ", none);
    if (rqs[IPL_XQ] & (1 << INT_V_XQ))
        show_aux_2(st, "XQ", none);
    if (rqs[IPL_RY] & (1 << INT_V_RY))
        show_aux_2(st, "RY", none);
    if (rqs[IPL_TTI] & (1 << INT_V_TTI))
        show_aux_2(st, "TTI", none);
    if (rqs[IPL_TTO] & (1 << INT_V_TTO))
        show_aux_2(st, "TTO", none);
    if (rqs[IPL_PTR] & (1 << INT_V_PTR))
        show_aux_2(st, "PTR", none);
    if (rqs[IPL_PTP] & (1 << INT_V_PTP))
        show_aux_2(st, "PTP", none);
    if (rqs[IPL_LPT] & (1 << INT_V_LPT))
        show_aux_2(st, "LPT", none);
    if (rqs[IPL_CSI] & (1 << INT_V_CSI))
        show_aux_2(st, "CSI", none);
    if (rqs[IPL_CSO] & (1 << INT_V_CSO))
        show_aux_2(st, "CSO", none);
    if (rqs[IPL_TMR0] & (1 << INT_V_TMR0))
        show_aux_2(st, "TMR0", none);
    if (rqs[IPL_TMR1] & (1 << INT_V_TMR1))
        show_aux_2(st, "TMR1", none);
    if (rqs[IPL_VHRX] & (1 << INT_V_VHRX))
        show_aux_2(st, "VHRX", none);
    if (rqs[IPL_VHTX] & (1 << INT_V_VHTX))
        show_aux_2(st, "VHTX", none);
    if (rqs[IPL_QDSS] & (1 << INT_V_QDSS))
        show_aux_2(st, "QDSS", none);
    if (rqs[IPL_CR] & (1 << INT_V_CR))
        show_aux_2(st, "CR", none);
}

void InterruptRegister::show_aux_2(SMP_FILE* st, const char* intr, t_bool& none)
{
    fprintf(st, "%s%s", none ? "" : " ", intr);
    none = FALSE;
}

/*
 * API for communication between guest and VAX MP VM
 */

void op_mtpr_simh (RUN_DECL, int32 va)
{
    int32 acc;
    GET_CUR; /* init acc */

    uint32 args[40];
    int32 op, guest_api_version, k;
    int32 argc = 0;                /* initialize to suppress false GCC warning */
    uint32 ix;

    /*
     * if in kernel mode, require header to be non-paged, this way we avoid paging exception
     * when checking the signature in kernel mode and can always pretend MT_SIMH does not exist
     * if the signature does not check out, i.e. on random reference can respond with reserved operand fault
     * rather than page fault
     */
    if (0 == (PSL & PSL_CUR))
    {
        int32 mstat;
        if (Test (RUN_PASS, va, RA, &mstat) < 0 ||
            Test (RUN_PASS, va + 11, RA, &mstat) < 0)
        {
            RSVD_OPND_FAULT;
        }
    }

    /*
     * check request signature
     */
    if (Read (RUN_PASS, va, L_LONG, RA) != VAXMP_API_SIGNATURE)
    {
        if (0 == (PSL & PSL_CUR))
            RSVD_OPND_FAULT;
        else
            RSVD_INST_FAULT;
    }

    op = Read (RUN_PASS, va + 4, L_LONG, RA);
    guest_api_version = Read (RUN_PASS, va + 8, L_LONG, RA);

    /* in non-kernel mode accept only QUERY */
    if (0 != (PSL & PSL_CUR) && op != VAXMP_API_OP_QUERY)
        RSVD_INST_FAULT;

    switch (op)
    {
    case VAXMP_API_OP_QUERY:       argc = 15;  break;
    case VAXMP_API_OP_MEMBAR:      argc = 3;   break;
    case VAXMP_API_OP_INIT_SMP:    argc = 17;  break;
    case VAXMP_API_OP_START_CPU:   argc = 34;  break;
    case VAXMP_API_OP_IPINTR:      argc = 5;   break;
    case VAXMP_API_OP_IDLE:        argc = 5;   break;
    case VAXMP_API_OP_IDLE_PULSE:  argc = 4;   break;
    case VAXMP_API_OP_SET_IDLE:    argc = 5;   break;
    case VAXMP_API_OP_GETTIME_VMS: argc = 6;   break;
    default:                       RSVD_OPND_FAULT;
    }

    /* pre-read argument block, make sure it is in memory and is writable */
    for (k = 0;  k < argc;  k++)
        args[k] = Read (RUN_PASS, va + 4 * k, L_LONG, WA);

    /*****************************************************************************
    *                                                                            *
    *  VAXMP_API_OP_QUERY -- Query simulator configuration                       *
    *                                                                            *
    *  Argument block:                                                           *
    *                                                                            *
    *      0  R    signature (VAXMP_API_SIGNATURE)                               *
    *      1  R    request code (VAXMP_API_OP_QUERY)                             *
    *      2  R    client API version                                            *
    *                                                                            *
    *      3  W    response status (1 = OK)                                      *
    *      4  W    SIMH API version                                              *
    *      5  W    number of configured VCPUs                                    *
    *      6  W    mask of configured VCPU IDs                                   *
    *      7  W    VM brand ('VXMP' = SIMH VAX MP)                               *
    *      8  W    VM version (such as 3.8.1.0)                                  *
    *      9  W    SMP code revision level                                       *
    *     10  W    SMP options                                                   *
    *     11  W    SMT (Hyper-Threading) slow-down factor multipler              *
    *     12  W    SMT (Hyper-Threading) slow-down factor divider                *
    *     13  W    Host turbo factor                                             *
    *     14  W    Active synchronization window mask                            *
    *                                                                            *
    *****************************************************************************/

    if (op == VAXMP_API_OP_QUERY)
    {
        args[3] = 1;             /* response status back to guest */
        args[4] = 1;             /* SIMH API version */
        args[5] = sim_ncpus;     /* number of configured CPUs */
        args[6] = 0;             /* mask of configured CPU IDs */
        for (ix = 0;  ix < sim_ncpus;  ix++)
            args[6] |= (1 << ix);
        args[7] = VAXMP_VM_ID;
        args[8] = (SIM_MAJOR << 24) | (SIM_MINOR << 16) | (SIM_PATCH << 8) | SIM_DELTA;
        args[9] = VSMP_REVISION;

        args[10] = 0;
        args[10] |= VAXMP_SMP_OPTION_PORTABLE_INTERLOCK;
#if SMP_NATIVE_INTERLOCKED
        args[10] |= VAXMP_SMP_OPTION_NATIVE_INTERLOCK;
#endif

        if (smp_smt_factor_set)
        {
            args[11] = (uint32) (smp_smt_factor * 100);
            args[12] = 100;
        }
        else if (smp_nsmt_per_core == 2)
        {
            args[11] = 18;
            args[12] = 10;
        }
        else
        {
            args[11] = 1;
            args[12] = 1;
        }

        args[13] = sim_host_turbo;
    }

    /*****************************************************************************
    *                                                                            *
    *  VAXMP_API_OP_MEMBAR -- Execute memory barrier                             *
    *                                                                            *
    *  Argument block:                                                           *
    *                                                                            *
    *      0  R    signature (VAXMP_API_SIGNATURE)                               *
    *      1  R    request code (VAXMP_API_OP_MEMBAR)                            *
    *      2  R    memory barrier to execute:                                    *
    *                                                                            *
    *                 0 or 3: full memory barrier                                *
    *                 1: WMB                                                     *
    *                 2: RMB                                                     *
    *                                                                            *
    *****************************************************************************/

    else if (op == VAXMP_API_OP_MEMBAR)
    {
        /* 
         * We rely here on design for device handlers (XQ, RQ and TQ) that access shared areas,
         * such as UQSSP COMM area and XQ BDL, that performs such access only within the context
         * of VCPU threads and not IOP threads. Otherwise MB would have been always required
         * here, regardless of whether sim_mp_active is TRUE or not.
         */
        if (weak_read(sim_mp_active))
        {
            switch (args[2] & 3)
            {
            case 0:
            case 3:
            default:
                smp_mb();  break;
            case 1:
                smp_wmb();  break;
            case 2:
                smp_rmb();  break;
            }
        }
    }

    /*****************************************************************************
    *                                                                            *
    *  VAXMP_API_OP_INIT_SMP -- Prepare to activate multiprocessing              *
    *                                                                            *
    *  Argument block:                                                           *
    *                                                                            *
    *      0  R    signature (VAXMP_API_SIGNATURE)                               *
    *      1  R    request code (VAXMP_API_OP_INIT_SMP)                          *
    *      2  R    client API version                                            *
    *                                                                            *
    *      3  W    response status (1 = OK)                                      *
    *      4  R    threshold IPL for system critical section                     *
    *      5  R    virtual address of CPUs idle mask (VMS SCH$GL_IDLE_CPUS)      *
    *      6  R    guest OS id (VMS = 1)                                         *
    *      7  R    guest OS version (or 0)                                       *
    *      8  R    "must" options                                                *
    *      9  R    "want" options                                                *
    *     10  W    granted options                                               *
    *     11  R    syncw on flags (SYNCW_SYS, SYNCW_ILK)                         *
    *     12  R    syncw winsize sys                                             *
    *     13  R    syncw winsize ilk                                             *
    *     14  R    syncw ipl syslock                                             *
    *     15  R    syncw ipl resched                                             *
    *     16  R    idle sleep is enabled (1 or 0)                                *
    *                                                                            *
    *****************************************************************************/

    else if (op == VAXMP_API_OP_INIT_SMP)
    {
        uint32 must_options = args[8];
        uint32 want_options = args[9];
        uint32 syncw_on = args[11];

        args[10] = 0;

        uint32 avopts = VAXMP_SMP_OPTION_PORTABLE_INTERLOCK;
#if SMP_NATIVE_INTERLOCKED
        avopts |= VAXMP_SMP_OPTION_NATIVE_INTERLOCK;
#endif

        if (sim_vsmp_active || 
            0 != (must_options & ~avopts) || 
            0 != (syncw_on & ~(SYNCW_SYS | SYNCW_ILK)))
        {
            args[3] = 0;             /* response status back to guest */
        }
        else
        {
            sim_vsmp_active = TRUE;

            sys_critical_section_ipl = args[4];
            sys_idle_cpu_mask_va = args[5];
            sim_vsmp_idle_sleep = (args[16] != 0);
            sim_vsmp_os = args[6];

            /* process interlock mode options */
#if SMP_NATIVE_INTERLOCKED
            if ((must_options | want_options) & VAXMP_SMP_OPTION_NATIVE_INTERLOCK)
            {
                use_native_interlocked = TRUE;
                args[10] |= VAXMP_SMP_OPTION_NATIVE_INTERLOCK;
            }
#endif

            if (! use_native_interlocked)
                args[10] |= VAXMP_SMP_OPTION_PORTABLE_INTERLOCK;

            /* process syncw */
            syncw.on = syncw_on;
            syncw.vsmp_winsize_sys = args[12];
            syncw.vsmp_winsize_ilk = args[13];
            syncw.ipl_syslock = (int32) args[14];
            syncw.ipl_resched = (int32) args[15];
            syncw_start(RUN_PASS);
            syncw_enable_cpu(RUN_PASS, SYNCW_NOLOCK);
            syncw_reeval_sys(RUN_PASS);

            /* disable built-in idle sleep detection mechanism, idle sleep is now under the control of guest OS */
            sim_cancel (cpu_unit);

            if (sim_clock_thread_created)
                smp_set_thread_priority(sim_clock_thread, SIMH_THREAD_PRIORITY_CLOCK);

            args[3] = 1;             /* response status back to guest */
        }
    }

    /*******************************************************************************
    *                                                                              *
    *  VAXMP_API_OP_START_CPU -- Start another CPU                                 *
    *                                                                              *
    *  Argument block:                                                             *
    *                                                                              *
    *       0  R    signature (VAXMP_API_SIGNATURE)                                *
    *       1  R    request code (VAXMP_API_OP_START_CPU)                          *
    *       2  R    client API version                                             *
    *                                                                              *
    *       3  W    response status (1 = OK, 2 = nx CPU ID, 3 = CPU not STANDBY,   *
    *                   4 = invalid HWPCB/SCBB/SBR/SLR/MAPEN values)               *
    *       4  R    CPU ID of the processor to start                               *
    *                                                                              *
    *  5 - 28  R    24 words with register values in VAX hardware PCB format       *
    *      29  R    SCBB                                                           *
    *      30  R    MAPEN                                                          *
    *      31  R    SBR                                                            *
    *      32  R    SLR                                                            *
    *      33  R    ISP                                                            *
    *                                                                              *
    *******************************************************************************/

    else if (op == VAXMP_API_OP_START_CPU)
    {
        cpu_database_lock->lock();

        /* 
         * If CPUs stop is pending, do not attempt to start the CPU (since console thread may have
         * already calculated number of running CPUs expected to join stop barrier at stop and re-ized
         * the barrier object accordingly). Instead throw SCPE_STOP now.  When processors are resumed,
         * current VCPU will re-execute MFPR instruction, and START_CPU request will be retried.
         */
        if (stop_cpus)
        {
            cpu_database_lock->unlock();
            ABORT (SCPE_STOP);
        }

        uint32 cpu_id = args[4];
        uint32* pcb = args + 5;
        uint32 scbb = args[29];
        uint32 x_mapen = args[30];
        uint32 sbr = args[31];
        uint32 slr = args[32];
        uint32 isp = args[33];

        if (cpu_id >= sim_ncpus)
        {
            /* invalid requested CPU ID */
            args[3] = 2;
        }
        else if (cpu_running_set.is_set(cpu_id) || cpu_units[cpu_id]->cpu_state != CPU_STATE_STANDBY)
        {
            /* requested processor is not in STANDBY state */
            args[3] = 3;
        }
        else if (cpu_start_secondary(RUN_PASS, cpu_units[cpu_id], pcb, scbb, x_mapen, sbr, slr, isp))
        {
            /* successfully started processor */
            args[3] = 1;
        }
        else
        {
            /* invalid registers values passed as request arguments */
            args[3] = 4;
        }

        cpu_database_lock->unlock();
    }

    /*****************************************************************************
    *                                                                            *
    *  VAXMP_API_OP_IDLE -- Enter idle sleep                                     *
    *                                                                            *
    *  Argument block:                                                           *
    *                                                                            *
    *      0  R    signature (VAXMP_API_SIGNATURE)                               *
    *      1  R    request code (VAXMP_API_OP_IDLE)                              *
    *      2  R    client API version                                            *
    *                                                                            *
    *      3  W    response status (1 = OK)                                      *
    *      4  R    max tick count to sleep (0 = till next clock tick,            *
    *              1 = till tick after next, 2 = till second tick after next     *
    *              and so on)                                                    *
    *                                                                            *
    *****************************************************************************/

    else if (op == VAXMP_API_OP_IDLE)
    {
        uint32 maxticks = args[4];
        args[3] = 1;             /* response status back to guest */

        syncw_leave_ilk(RUN_PASS);

        /* syncw_leave_sys is not needed for VMS since idle loop is executing at IPL RESCHED < IPL QUEUEAST */
        // syncw_leave_sys(RUN_PASS);

        /* 
         * do not process SCPE_STOP here, let it rather be caught at next instruction,
         * so that if CPUs are paused to console, exiting console action does not cause
         * the CPU to re-enter the sleep immediatelly
         */
        (void) sim_idle(RUN_PASS, TMR_CLK, FALSE, maxticks);
    }

    /*****************************************************************************
    *                                                                            *
    *  VAXMP_API_OP_IDLE_PULSE -- Signal idle loop, but do not enter sleep       *
    *                                                                            *
    *  Argument block:                                                           *
    *                                                                            *
    *      0  R    signature (VAXMP_API_SIGNATURE)                               *
    *      1  R    request code (VAXMP_API_OP_IDLE_PULSE)                        *
    *      2  R    client API version                                            *
    *                                                                            *
    *      3  W    response status (1 = OK)                                      *
    *                                                                            *
    *****************************************************************************/

    else if (op == VAXMP_API_OP_IDLE_PULSE)
    {
        uint32 maxticks = args[4];
        args[3] = 1;             /* response status back to guest */

        syncw_leave_ilk(RUN_PASS);

        /* syncw_leave_sys is not needed for VMS since idle loop is executing at IPL RESCHED < IPL QUEUEAST */
        // syncw_leave_sys(RUN_PASS);
    }

    /*****************************************************************************
    *                                                                            *
    *  VAXMP_API_OP_SET_IDLE -- Signal whether guest OS idle loop will be using  *
    *                           idle sleep (by issuing VAXMP_API_OP_IDLE calls)  *
    *                                                                            *
    *  Argument block:                                                           *
    *                                                                            *
    *      0  R    signature (VAXMP_API_SIGNATURE)                               *
    *      1  R    request code (VAXMP_API_OP_SET_IDLE)                          *
    *      2  R    client API version                                            *
    *                                                                            *
    *      3  R    idle sleep is enabled (1 or 0)                                *
    *      4  W    response status (1 = OK)                                      *
    *                                                                            *
    *****************************************************************************/

    else if (op == VAXMP_API_OP_SET_IDLE)
    {
        sim_vsmp_idle_sleep = (args[3] != 0);
        args[4] = 1;             /* response status back to guest */
    }

    /*****************************************************************************
    *                                                                            *
    *  VAXMP_API_OP_IPINTR -- Emit interprocessor interrupt                      *
    *                                                                            *
    *  Argument block:                                                           *
    *                                                                            *
    *      0  R    signature (VAXMP_API_SIGNATURE)                               *
    *      1  R    request code (VAXMP_API_OP_IPINTR)                            *
    *      2  R    client API version                                            *
    *                                                                            *
    *      3  W    response status (1 = OK)                                      *
    *      4  R    mask of VCPU IDs to be interrupted                            *
    *                                                                            *
    *****************************************************************************/

    else if (op == VAXMP_API_OP_IPINTR)
    {
        uint32 intmask = args[4];  /* ID-mask of CPUs to be interrupted */
        smp_wmb();                 /* push out memory changes to targets */

        /* interrupt all requested targets that are running */
        for (uint32 cpu_ix = 0;  cpu_ix < sim_ncpus;  cpu_ix++)
        {
            if (intmask & (1 << cpu_ix))
            {
                interrupt_ipi(RUN_PASS, cpu_units[cpu_ix]);
            }
        }

        args[3] = 1;             /* response status back to guest */
    }

    /*****************************************************************************
    *                                                                            *
    *  VAXMP_API_OP_GETTIME_VMS -- Get host system time in VMS format            *
    *                                                                            *
    *  VMS time format is the number of 100-nanosecond intervals                 *
    *  since 00:00 o’clock, November 17, 1858                                    *
    *                                                                            *
    *  Argument block:                                                           *
    *                                                                            *
    *      0  R    signature (VAXMP_API_SIGNATURE)                               *
    *      1  R    request code (VAXMP_API_OP_GETTIME_VMS)                       *
    *      2  R    client API version                                            *
    *                                                                            *
    *      3  W    response status (1 = OK)                                      *
    *      4  W    low longword of time                                          *
    *      5  W    high longword of time                                         *
    *                                                                            *
    *****************************************************************************/

    else if (op == VAXMP_API_OP_GETTIME_VMS)
    {
        sim_os_gettime_vms(args + 4);
        args[3] = 1;             /* response status back to guest */
    }

    /*****************************************************************************
    *  Write back argument block, except for the header of first 3 longwords     *
    *  (skip: signature, op-code and guest api version)                          *
    *****************************************************************************/

    for (k = 3;  k < argc;  k++)
        Write (RUN_PASS, va + 4 * k, args[k], L_LONG, WA);
}

/*
 * Start "xcpu" initializing its context with data from passed "pcb", scbb, x_mapen and sbr/slr arguments.
 * Called while holding cpu_database_lock.
 */
static t_bool cpu_start_secondary(RUN_DECL, CPU_UNIT* xcpu, uint32* pcb, uint32 scbb, uint32 x_mapen, uint32 sbr, uint32 slr, uint32 isp)
{
    t_bool valid = TRUE;
    int32 opnd[2];
    uint32 t;

    /*
     * Temporarily switch to the target VCPU's context.
     */
    CPU_UNIT* local_cpu = cpu_unit;
    RUN_SCOPE_RSCX_ONLY;
    cpu_unit = rscx->cpu_unit = xcpu;

    /*
     * Reset target VCPU state.
     *
     * Note that since we are holding cpu_database_lock, "vm_critical_locks" count in RSCX is non-zero.
     * This prevents the code in the invoked "reset" routines from altering current thread's priority.
     * Same goes for setting "mapen" below.
     *
     * Note: we do not reinitialize clock event queue. Therefore if there were any leftover events
     * for system-wide devices queued during the previous execution of this VCPU and that had not been
     * transferred to the primary VCPU yet (i.e. xcpu->cpu_requeue_syswide_pending == TRUE), these evtns
     * would not be lost. Instead of being transferred to the primary VCPU's queue, they will resume
     * in the current VCPU's queue again. However we have to reset per-CPU devices.
     * 
     * While resetting them, disable use of synchronization window for target CPU while it is being
     * initialized. This prevents clk_dev.reset and other calls from entering target VCPU into SYNCW_SYS
     * before it is fully set up and marked runnable.
     */
    syncw_leave_all(RUN_PASS, SYNCW_OVERRIDE_ALL | SYNCW_DISABLE_CPU);
    cpu_dev.reset(& cpu_dev);
    syncw_leave_all(RUN_PASS, SYNCW_OVERRIDE_ALL | SYNCW_DISABLE_CPU);
    tlb_dev.reset(& tlb_dev);
    sysd_dev.reset(& sysd_dev);

    /* clone initial RTC clock calibration from local VCPU data, must do it before CLK reset */
    sim_rtcn_init_secondary_cpu(cpu_unit, local_cpu, TMR_CLK, 0, TRUE);
    clk_dev.reset(& clk_dev);

    tti_dev.reset(& tti_dev);
    tto_dev.reset(& tto_dev);

    /*
     * Set up target VCPU's state from passed HWPCB and other passed register values.
     */
    sim_try
    {
        /* validate new PSL */
        uint32 newpsl = pcb[19];

        if ((newpsl & PSL_MBZ) || 
            (newpsl & PSL_CM) || 
            PSL_GETCUR(newpsl) != KERN && (newpsl & (PSL_IS|PSL_IPL)) ||
            (newpsl & PSL_IS) && (newpsl & PSL_IPL) == 0)
        {
            RSVD_OPND_FAULT;
        }

        /* load basic privileged processor registers, in this order */
#define mtpr(val, prn)  opnd[0] = (val);  opnd[1] = (prn);  op_mtpr(RUN_PASS, opnd);
        mtpr(scbb, MT_SCBB);
        mtpr(sbr, MT_SBR);
        mtpr(slr, MT_SLR);
        mtpr(x_mapen, MT_MAPEN);
        mtpr(1, MT_TBIA);
#undef mtpr

        IS = isp;

        KSP = pcb[0];
        ESP = pcb[1];
        SSP = pcb[2];
        USP = pcb[3];
        R[0] = pcb[4];
        R[1] = pcb[5];
        R[2] = pcb[6];
        R[3] = pcb[7];
        R[4] = pcb[8];
        R[5] = pcb[9];
        R[6] = pcb[10];
        R[7] = pcb[11];
        R[8] = pcb[12];
        R[9] = pcb[13];
        R[10] = pcb[14];
        R[11] = pcb[15];
        R[12] = pcb[16];
        R[13] = pcb[17];
        PC = pcb[18];
        PSL = newpsl;

        /* validate and set P0BR */
        t = pcb[20];
        ML_PXBR_TEST(t);
        P0BR = t & BR_MASK;

        /* P0LR */
        t = pcb[21];
        LP_MBZ84_TEST(t);
        ML_LR_TEST(t & LR_MASK);
        P0LR = t & LR_MASK;

        /* ASTLVL */
        t = (t >> 24) & AST_MASK;
        LP_AST_TEST(t);
        ASTLVL = t;

        /* P1BR */
        t = pcb[22];
        ML_PXBR_TEST(t + 0x800000);
        P1BR = t & BR_MASK;

        /* P1LR */
        t = pcb[23];
        LP_MBZ92_TEST(t);
        ML_LR_TEST(t & LR_MASK);
        P1LR = t & LR_MASK;

        /* PME */
        pme = (t >> 31) & 1;

        /* clear TLB */
        zap_tb(RUN_PASS, 1);

        /* reload microcode registers */
        set_map_reg(RUN_PASS);

        /* clone TODR from the primary VCPU */
        todr_reg = primary_todr_reg;
        todr_blow = primary_todr_blow;

        /* set up secondary CPU's model-specific registers */
        cpu_setup_secondary_model_specific(RUN_PASS, local_cpu);
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

        valid = FALSE;
    }
    sim_end_try

    /* Switch the context back to the local VCPU */
    cpu_unit = rscx->cpu_unit = local_cpu;

    if (valid)
    {
        /* clear target VCPU's "requeue pending" flag (old entries for syswide devices) */
        xcpu->cpu_requeue_syswide_pending = FALSE;

        /* mark target VCPU as in running state */
        t_bool initial_sim_mp_active = weak_read(sim_mp_active);
        cpu_running_set.set(xcpu->cpu_id);
        xcpu->cpu_state = CPU_STATE_RUNNING;
        sim_mp_active_update();

        /* 
         * if starting first secondary, thread priority management is transitioned from disabled
         * to possibly enabled (unless host_dedicated is TRUE) and we must propagate primary VCPU
         * logical thread priority to host thread priority; see detailed explanation in a comment in cpu_once_a_second()
         */
        if (! initial_sim_mp_active)
        {
            cpu_unit->cpu_thread_priority = SIMH_THREAD_PRIORITY_INVALID;
            cpu_reevaluate_thread_priority(RUN_PASS);
        }

        /* mark current VCPU as enabled for synchronizaton window */
        syncw_enable_cpu(RUN_PASS, SYNCW_NOLOCK);
        if (initial_sim_mp_active == FALSE)
        {
            /* 
             * sincw syncw is not maintained while SMP is not active (initial_sim_mp_active == FALSE),
             * and thus there is no syncw pre-history at this point, as a precaution activate full syncw
             * constraint set on current VCPU
             */
            syncw_reeval_sys(RUN_PASS);
            syncw_enter_ilk(RUN_PASS);
        }

        /* mark target VCPU as enabled for synchronizaton window */
        cpu_unit = rscx->cpu_unit = xcpu;
        syncw_reinit_cpu(RUN_PASS, SYNCW_NOLOCK);
        syncw_reeval_sys(RUN_PASS);
        cpu_unit = rscx->cpu_unit = local_cpu;

        /* set up SYNCLK and clock tracking */
        xcpu->cpu_last_synclk_cycles = XCPU_CURRENT_CYCLES - (CPU_CURRENT_CYCLES - cpu_unit->cpu_last_synclk_cycles);
        xcpu->cpu_last_tslice_tick_cycles = XCPU_CURRENT_CYCLES - (CPU_CURRENT_CYCLES - cpu_unit->cpu_last_tslice_tick_cycles);
        xcpu->cpu_last_second_tick_cycles = XCPU_CURRENT_CYCLES - (CPU_CURRENT_CYCLES - cpu_unit->cpu_last_second_tick_cycles);

        /* signal "go" to the target VCPU's work thread */
        smp_wmb();
        xcpu->cpu_run_gate->release(1);

        return TRUE;
    }
    else
    {
        /* passed arguments were invalid */
        return FALSE;
    }
}

/*
 * Called on ROM access. Detect ROM execution.
 * 
 * In case primary processor jumped into ROM while secondary processors were active,
 * perform emergency shutdown of secondaries.
 *
 * This condition should normally never happen, this is just an emergency safeguard
 * in case guest OS fails to shut down properly.
 */
void cpu_on_rom_rd(RUN_DECL)
{
    if (weak_read(sim_mp_active) && cpu_unit->is_primary_cpu() && mapen == 0 && ADDR_IS_ROM(PC))
    {
        syncw_leave_all(RUN_PASS, SYNCW_OVERRIDE_ALL | SYNCW_DISABLE_CPU);
        cpu_shutdown_secondaries(RUN_PASS);
    }
}

/*
 * Called when primary is about to shutdown or enter uniprocessor mode.
 *
 * Perform emerghency shutdown of secondary processors in unlikely case
 * guest OS failed to shut them down properly.
 *
 * Note: it will mark all running CPUs as SYNCW_NOSYNC (i.e. uneligible for waiting or
 * being waited on in the inter-processor synchronization window).
 */
void cpu_shutdown_secondaries(RUN_DECL)
{
    /* uniprocessor? */
    if (weak_read(sim_mp_active) == FALSE)
        return;

    /* let some time for sim_mp_active to update in case secondaries are being stopped
       (not critical for correctness, just performance) */
    sim_os_ms_sleep(3);

    if (weak_read(sim_mp_active) == FALSE)
        return;

    smp_printf ("\nPrimary processor in SMP system is about to halt.\n");
    smp_printf ("Forcing emergency shutdown of seconary CPUs...\n");
    if (sim_log)
    {
        fprintf (sim_log, "Primary processor in SMP system is about to halt.\n");
        fprintf (sim_log, "Forcing emergency shutdown of seconary CPUs...\n");
    }

    cpu_database_lock->lock();
    syncw.on = 0;
    /* send non-maskable IPI to stop secondaries */
    for (uint32 cpu_ix = 0;  cpu_ix < sim_ncpus;  cpu_ix++)
    {
        if (cpu_running_set.is_set(cpu_ix))
        {
            interrupt_set_int(cpu_units[cpu_ix], IPL_STOP, INT_V_STOP);
        }
    }
    syncw_wakeup_all_cpus(SYNCW_DISABLE_CPU);
    cpu_database_lock->unlock();

    /* wait until all secondaries are stopped or stop_cpus is set */
    for (;;)
    {
        sim_os_ms_sleep(3);
        cpu_database_lock->lock();
        t_bool doexit = stop_cpus || weak_read(sim_mp_active) == FALSE;
        cpu_database_lock->unlock();
        if (doexit)  break;
    }

    smp_printf ("\nCompleted emergency shutdown of seconary CPUs.\n");
    if (sim_log)
        fprintf (sim_log, "Completed emergency shutdown of seconary CPUs.\n");
}

/*
 * This routine is invoked on each running VCPU once a second,
 * more specifically once a second in virtual time scale.
 *
 * Simulator tries to keep virtual time close to wall time, and normally it is
 * more or less close, but can also diverge.
 */
void cpu_once_a_second(RUN_DECL)
{
    /*
     * When running in uniprocessor mode with only only one processor active and secondaries not started
     * (or all stopped), or when running on a dedicated host, VCPU thread priority is not controlled.
     *
     * This is an optimization in order to avoid the overhead of "set thread priority" calls to host OS
     * that are not essential in this case.
     *
     * Thread priority control is governed by macro
     *
     *      #define must_control_prio()  (sim_mp_active && !sim_host_dedicated)
     *
     * When must_control_prio() evaluates to TRUE, "set thread priority" calls are executed, otherwise
     * they are not. cpu_unit->cpu_thread_priority is still maintained, but actuall underlying base
     * priority is not changed and is kept at CPU_RUN level.
     *
     * The only exception to this is calibration: thread is always allowed to actually raise
     * to CALIBRATION level and drop down from it (see cpu_should_actually_change_thread_priority).
     *
     * Thus, when multiple processor are active, actual VCPU thread priority is subject to change.
     * When only the primary is active, it is kept at CPU_RUN level.
     *
     *     One exceptions to this is calibration: processor is allowed to raise to CALIBRATION level
     *     and drop down from it (in the latter case VCPU is always forced down to CPU_RUN).
     *
     *     Second exception is a dedicated host: if host is marked by user as dedicated to SIMH,
     *     thread priority is never changed, except that transition to CALIBRATION is still allowed,
     *     as well as transition from CALIBRATION down.
     *
     * There is however a number of borderline cases with described governance by must_control_prio()
     * that must be taken care about:
     *
     * 1) When last active secondary is exiting, primary can be at elevated priority and must
     *    drop it down. This is handled inside SECEXIT interrupt processing.
     *
     * 2) When starting first secondary, propagate cpu_unit->cpu_thread_priority to actual
     *    thread priority of the primary (secondary being started will get it automatically).
     *    This will be performed by any invocation of cpu_reevaluate_thread_priority() on the
     *    primary that causes any change in priority, i.e. in very short order after
     *    sim_mp_active changes to TRUE. Just to be super-correct, we perform such invocation
     *    right in cpu_start_secondary().
     *
     * 3) When first secondary is started and (assuming host_dedicated is FALSE) thread priority
     *    control is enabled, it may take some time before IOP and CLOCK threads get wind of change.
     *
     *    It may be that for a short while when sending interrupts to VCPUs, CLOCK thread will not
     *    elevate their prority to CRITICAL_VM. This can at most be for the duration of one clock
     *    cycle (1/100 sec on VAX) and already the next cycle will bump VCPU threads priority up.
     *    Thus, even apart from low likelyhood of this problem in the first place, its impact is
     *    very small and not worth bothering about.
     *
     *    It may also be that for a short while when sending interrupts to the primary VCPU thread,
     *    IOP threasds will not elevate its priority to CRITICAL_VM. However, apart from low likelyhood
     *    of this problem in the first place, its impact is quite limited. Furthermore, if
     *    use_clock_thread is TRUE, then even in the worst case the primary will have its priority 
     *    bumped up in at most 1/100 secs even if IOP did not bump it. Thus this problem does not 
     *    look worth bothering about.
     *
     * 4) When last secondary is being stopped, the primary will get a notification of this via
     *    SECEXIT interrupt and will drop its priority (except in very unlikely case that calibration
     *    is active), but it may take some time before IOP and CLOCK threads get wind of change,
     *    and thus IOP and CLOCK threads may continue for some (typically short) time bumping up
     *    primary VCPU thread priority. They may do it even after SECEXIT dropped priority and thus
     *    negate the drop performed in SECEXIT handler. Furthermore, it is possible (though very
     *    unlikely) for IOP or CLOCK thread to be preempted for some time after it fetched "sim_mp_active"
     *    but before it acted on it. Thus, CLOCK and IOP threads can theoretically use stale value
     *    of "sim_mp_active" some extended time after it went FALSE, and still act on it as if it
     *    was TRUE. This is very unlikely, but possible.
     *
     *    We must safeguard against such a possibility by counteracting it. Even though the event
     *    is very unlikely, its impact is severe: if left uncounteracted, it may leave primary VCPU
     *    thread running all the time at CRITICAL_VM level while in uniprocessor mode again.
     *
     *    The easiest solution is simply to reset primary VCPU thread priority periodically to
     *    CPU_RUN (for some time after going from MP to UniP mode) -- in case thread priority control
     *    is makerd as "should be disabled" and unless (unlikely) calibration is active.
     *
     *    Such resets can be performed in SYNCLK or CLK interrupt handlers, for instance. But the most 
     *    convenient solution is to place the reset in cpu_once_a_second() routine.
     *
     */

    if (!must_control_prio() && !tmr_is_active(RUN_PASS))
        smp_set_thread_priority(SIMH_THREAD_PRIORITY_CPU_RUN);
}

#define VAX_MP_CPUCTX_METHODS
#include "vax_cpuctx.h"