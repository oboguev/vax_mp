/* vax_io.c: VAX 3900 Qbus IO simulator

   Copyright (c) 1998-2008, Robert M Supnik

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

   qba          Qbus adapter

   28-May-08    RMS     Inlined physical memory routines
   25-Jan-08    RMS     Fixed declarations (from Mark Pizzolato)
   03-Dec-05    RMS     Added SHOW QBA VIRT and ex/dep via map
   05-Oct-05    RMS     Fixed bug in autoconfiguration (missing XU)
   25-Jul-05    RMS     Revised autoconfiguration algorithm and interface
   30-Sep-04    RMS     Revised Qbus interface
                        Moved mem_err, crd_err interrupts here from vax_cpu.c
   09-Sep-04    RMS     Integrated powerup into RESET (with -p)
   05-Sep-04    RMS     Added CRD interrupt handling
   28-May-04    RMS     Revised I/O dispatching (from John Dundas)
   21-Mar-04    RMS     Added RXV21 support
   21-Dec-03    RMS     Fixed bug in autoconfigure vector assignment; added controls
   21-Nov-03    RMS     Added check for interrupt slot conflict (found by Dave Hittner)
   29-Oct-03    RMS     Fixed WriteX declaration (found by Mark Pizzolato)
   19-Apr-03    RMS     Added optimized byte and word DMA routines
   12-Mar-03    RMS     Added logical name support
   22-Dec-02    RMS     Added console halt support
   12-Oct-02    RMS     Added autoconfigure support
                        Added SHOW IO space routine
   29-Sep-02    RMS     Added dynamic table support
   07-Sep-02    RMS     Added TMSCP and variable vector support
*/

#include "sim_defs.h"
#include "vax_defs.h"

/*
 * For more information on CQBIC see "KA655 CPU System Maintenance" manual, pp 1.3, 1.5, 1.8, 1.11.
 */

/* CQBIC system configuration register */

#define CQSCR_POK       0x00008000                      /* power ok RO1 */
#define CQSCR_BHL       0x00004000                      /* BHALT enb */
#define CQSCR_AUX       0x00000400                      /* aux mode RONI */
#define CQSCR_DBO       0x0000000C                      /* offset NI */
#define CQSCR_RW        (CQSCR_BHL | CQSCR_DBO)
#define CQSCR_MASK      (CQSCR_RW | CQSCR_POK | CQSCR_AUX)

/* CQBIC DMA system error register - W1C */

#define CQDSER_BHL      0x00008000                      /* BHALT NI */
#define CQDSER_DCN      0x00004000                      /* DC ~OK NI */
#define CQDSER_MNX      0x00000080                      /* master NXM */
#define CQDSER_MPE      0x00000020                      /* master par NI */
#define CQDSER_SME      0x00000010                      /* slv mem err NI */
#define CQDSER_LST      0x00000008                      /* lost err */
#define CQDSER_TMO      0x00000004                      /* no grant NI */
#define CQDSER_SNX      0x00000001                      /* slave NXM */
#define CQDSER_ERR      (CQDSER_MNX | CQDSER_MPE | CQDSER_TMO | CQDSER_SNX)
#define CQDSER_MASK     0x0000C0BD

/* CQBIC master error address register */

#define CQMEAR_MASK     0x00001FFF                      /* Qbus page */

/* CQBIC slave error address register */

#define CQSEAR_MASK     0x000FFFFF                      /* mem page */

/* CQBIC map base register */

#define CQMBR_MASK      0x1FFF8000                      /* 32KB aligned */

/* CQBIC IPC register */

#define CQIPC_QME       0x00008000                      /* Qbus read NXM W1C */
#define CQIPC_INV       0x00004000                      /* CAM inval NIWO */
#define CQIPC_AHLT      0x00000100                      /* aux halt NI */
#define CQIPC_DBIE      0x00000040                      /* dbell int enb NI */
#define CQIPC_LME       0x00000020                      /* local mem enb */
#define CQIPC_DB        0x00000001                      /* doorbell req NI */
#define CQIPC_W1C       CQIPC_QME
#define CQIPC_RW        (CQIPC_AHLT | CQIPC_DBIE | CQIPC_LME | CQIPC_DB)
#define CQIPC_MASK      (CQIPC_RW | CQIPC_QME )

/* CQBIC map entry */

#define CQMAP_VLD       0x80000000                      /* valid */
#define CQMAP_PAG       0x000FFFFF                      /* mem page */

/* VAX/VMS definitions */

#define IPL_VMS_QUEUEAST 6                              /* QUEUEAST, lowest spinlock-holding IPL level */
#define IPL_VMS_IOPOST 6                                /* IOPOST, lowest kernel short-term resource holding level */

/* 
 * SCR, DSER, MEAR, SEAR and IPC were moved to per-CPU context.
 * MBR is shared by all CPUs, but is writable by CPU0 only.
 */

atomic_int32 cq_mbr = 0;                                /* MBR */
int32 autcon_enb = 1;                                   /* autoconfig enable */
uint32 synclk_safe_cycles = 2000;                       /* minimum VCPU cycles between consequtive CLK events raised by SYNCLK */
int32 synclk_safe_ipl = IPL_VMS_QUEUEAST;               /* minimum IPL level to be protected against starvation by synclk_safe_cycles */

extern int32 sim_switches;
extern SMP_FILE *sim_log;

t_stat dbl_rd (int32 *data, int32 addr, int32 access);
t_stat dbl_wr (int32 data, int32 addr, int32 access);
void cq_merr (int32 pa);
void cq_serr (int32 pa);
t_stat qba_reset (DEVICE *dptr);
t_stat qba_ex (t_value *vptr, t_addr exta, UNIT *uptr, int32 sw);
t_stat qba_dep (t_value val, t_addr exta, UNIT *uptr, int32 sw);
t_bool qba_map_addr (RUN_DECL, uint32 qa, uint32 *ma);
t_bool qba_map_addr_c (RUN_DECL, uint32 qa, uint32 *ma);
t_stat set_autocon (UNIT *uptr, int32 val, char *cptr, void *desc);
t_stat show_autocon (SMP_FILE *st, UNIT *uptr, int32 val, void *desc);
t_stat show_iospace (SMP_FILE *st, UNIT *uptr, int32 val, void *desc);
t_stat qba_show_virt (SMP_FILE *of, UNIT *uptr, int32 val, void *desc);

/* Qbus adapter data structures

   qba_dev      QBA device descriptor
   qba_unit     QBA units
   qba_reg      QBA register list
*/

DIB qba_dib = { IOBA_DBL, IOLN_DBL, &dbl_rd, &dbl_wr, 0 };

UNIT qba_unit UDATA_SINGLE (NULL, 0, 0);
UNIT_TABLE_SINGLE(qba_unit);

REG qba_reg[] = {
    { HRDATA_CPU ("SCR", r_cq_scr, 16) },
    { HRDATA_CPU ("DSER", r_cq_dser, 8) },
    { HRDATA_CPU ("MEAR", r_cq_mear, 13) },
    { HRDATA_CPU ("SEAR", r_cq_sear, 20) },
    { HRDATA_GBL (MBR, cq_mbr, 29) },
    { HRDATA_CPU ("IPC", r_cq_ipc, 16) },
    { IRDATA_LVL (IPL17, 17, 0), REG_RO },
    { IRDATA_LVL (IPL16, 16, 0), REG_RO },
    { IRDATA_LVL (IPL15, 15, 0), REG_RO },
    { IRDATA_LVL (IPL14, 14, 0), REG_RO },
    { FLDATA_GBL (AUTOCON, autcon_enb, 0), REG_HRO },
    { NULL }
    };

MTAB qba_mod[] = {
    { MTAB_XTD|MTAB_VDV|MTAB_NMO, 0, "IOSPACE", NULL,
      NULL, &show_iospace },
    { MTAB_XTD|MTAB_VDV, 1, "AUTOCONFIG", "AUTOCONFIG",
      &set_autocon, &show_autocon },
    { MTAB_XTD|MTAB_VDV, 0, NULL, "NOAUTOCONFIG",
      &set_autocon, NULL },
    { MTAB_XTD|MTAB_VDV|MTAB_NMO|MTAB_SHP, 0, "VIRTUAL", NULL,
      NULL, &qba_show_virt },
    { 0 }
    };

DEVICE qba_dev = {
    "QBA", qba_unit_table, qba_reg, qba_mod,
    1, 16, CQMAWIDTH, 2, 16, 16,
    &qba_ex, &qba_dep, &qba_reset,
    NULL, NULL, NULL,
    &qba_dib, DEV_QBUS
    };

/* IO page dispatches */

t_stat (*iodispR[IOPAGESIZE >> 1])(int32 *dat, int32 ad, int32 md);
t_stat (*iodispW[IOPAGESIZE >> 1])(int32 dat, int32 ad, int32 md);

/* Interrupt request to interrupt action map */

SIM_ALIGN_PTR int32 (* volatile int_ack[IPL_HLVL][32])();                       /* int ack routines */

/* Interrupt request to vector map */

atomic_int32 int_vec[IPL_HLVL][32];                            /* int req to vector */

/* The KA65x handles errors in I/O space as follows

        - read: set DSER<7>, latch addr in MEAR, machine check
        - write: set DSER<7>, latch addr in MEAR, MEMERR interrupt
*/

int32 ReadQb (RUN_DECL, uint32 pa)
{
    int32 idx, val;

    idx = (pa & IOPAGEMASK) >> 1;
    if (iodispR[idx])
    {
        iodispR[idx] (&val, pa, READ);
        return val;
    }
    cq_merr (pa);
    MACH_CHECK (MCHK_READ);
    return 0;
}

void WriteQb (RUN_DECL, uint32 pa, int32 val, int32 mode)
{
    int32 idx;

    idx = (pa & IOPAGEMASK) >> 1;
    if (iodispW[idx])
    {
        iodispW[idx] (val, pa, mode);
        return;
    }
    cq_merr (pa);
    mem_err = 1;
}

/* ReadIO - read I/O space

   Inputs:
        pa      =       physical address
        lnt     =       length (BWLQ)
   Output:
        longword of data
*/

int32 ReadIO (RUN_DECL, uint32 pa, int32 lnt)
{
    int32 iod;

    iod = ReadQb (RUN_PASS, pa);                            /* wd from Qbus */
    if (lnt < L_LONG)                                       /* bw? position */
        iod = iod << ((pa & 2)? 16: 0);
    else
        iod = (ReadQb (RUN_PASS, pa + 2) << 16) | iod;      /* lw, get 2nd wd */

    /*
     * Checking here for the change in pending interrupts made sense on a uniprocessor version of the emulator.
     * In the SMP version, device interrupt may be delivered to another processor.
     * Therefore moved the check to instruction loop, except for Qbus memory access errors.
     */
    if (mem_err)
    {
        SET_IRQL;
    }

    return iod;
}

/* WriteIO - write I/O space

   Inputs:
        pa      =       physical address
        val     =       data to write, right justified in 32b longword
        lnt     =       length (BWLQ)
   Outputs:
        none
*/

void WriteIO (RUN_DECL, uint32 pa, int32 val, int32 lnt)
{
    if (lnt == L_BYTE)
        WriteQb (RUN_PASS, pa, val, WRITEB);
    else if (lnt == L_WORD)
        WriteQb (RUN_PASS, pa, val, WRITE);
    else
    {
        WriteQb (RUN_PASS, pa, val & 0xFFFF, WRITE);
        WriteQb (RUN_PASS, pa + 2, (val >> 16) & 0xFFFF, WRITE);
    }

    /*
     * Checking here for the change in pending interrupts made sense on a uniprocessor version of the emulator.
     * In the SMP version, device interrupt may be delivered to another processor.
     * Therefore moved the check to instruction loop, except for Qbus memory access errors.
     */
    if (mem_err)
    {
        SET_IRQL;
    }
}

/*
 * Find highest priority outstanding deliverable interrupt, called by SET_IRQL.
 *
 * As a side effect, record highest pending interrupt (whether deliverable or not)
 * and adjust thread priority.
 */

int32 eval_int (RUN_DECL, t_bool sync)
{
    RUN_SCOPE_RSCX_ONLY;

    int32 cipl = PSL_GETIPL (PSL);
    t_bool was_changed = FALSE;
    int32 hipl = 0;
    t_bool synced = FALSE;
    t_bool nmi = FALSE;

    /*
     * Cancel protection period against raising CLK by pending SYNCLK
     * if IPL drops below a threshold or goes out of kernel mode.
     */
    if (unlikely(cpu_unit->cpu_synclk_protect_os))
    {
        t_bool cancel_protection = FALSE;

        if (PSL_GETCUR(PSL))
        {
            cancel_protection = TRUE;
        }
        else if (sys_critical_section_ipl >= 0)
        {
            cancel_protection = (cipl < sys_critical_section_ipl);
        }
        else
        {
            cancel_protection = (cipl < synclk_safe_ipl);
        }

        if (cancel_protection)
        {
            if (cpu_unit->cpu_synclk_pending != SynclkPendingIE1)
            {
                cpu_unit->cpu_synclk_protect_os = 0;

                if (cpu_unit->cpu_synclk_protect_dev == 0)
                {
                    cpu_unit->cpu_synclk_protect = FALSE;
                    check_synclk_pending(RUN_PASS);
                }
            }
        }
    }

    if (unlikely(cpu_unit->cpu_intreg.weak_changed()))
    {
        if (was_changed = cpu_unit->cpu_intreg.cas_changed(1, 0))
        {
            smp_post_interlocked_rmb();
            read_irqs_to_local(RUN_PASS);
            synced = TRUE;
        }
    }

    if (likely(sync) && !synced)
    {
        smp_rmb();
        read_irqs_to_local(RUN_PASS);
        synced = TRUE;
    }

    if (unlikely(hlt_pin))                                  /* hlt pin int */
    {
        hipl = IPL_HLTPIN;
        nmi = TRUE;
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

    if (rscx->thread_type == SIM_THREAD_TYPE_CPU)
    {
        /* 
         * Interrupt sender may kick target priority to CRITICAL_VM without reflecting it in cpu_unit->cpu_thread_priority.
         * Therefore when we receive an interrupt, clear cached record of thread priority in order to force actual 
         * thread priority change even when new calculated priority evaluates to the priority the thread was recorded 
         * to hold before.
         */
        if (was_changed && must_control_prio())
            cpu_unit->cpu_thread_priority = SIMH_THREAD_PRIORITY_INVALID;
        cpu_reevaluate_thread_priority(RUN_PASS, synced);
    }
    else if (rscx->thread_type == SIM_THREAD_TYPE_CONSOLE)
    {
        /*
         * Called on the console thread, e.g. when writing to SIRR.
         *
         * Pretty much could do just nothing at all, since reevaluation of thread priority will be performed
         * anyway by SET_IRQL at the start of resumed instruction loop. However for a marginal case of starved
         * VCPU thread (e.g. trying to break into XDelta while other VCPU threads are hung spinning at elevated IPL)
         * boost target thread's priority here.
         */
        t_bool is_active_clk_interrupt;
        t_bool is_active_ipi_interrupt;

        if (sys_critical_section_ipl >= 0 && (int32) hipl >= sys_critical_section_ipl)
        {
            /* console thread has access to xcpu->cpu_thread_priority */
            switch (cpu_unit->cpu_thread_priority)
            {
            case SIMH_THREAD_PRIORITY_CPU_CALIBRATION:
            case SIMH_THREAD_PRIORITY_CPU_CRITICAL_VM:
            case SIMH_THREAD_PRIORITY_CPU_CRITICAL_OS_HI:
                break;
            case SIMH_THREAD_PRIORITY_CPU_CRITICAL_OS:
                cpu_unit->cpu_intreg.query_local_clk_ipi(& is_active_clk_interrupt, & is_active_ipi_interrupt);
                if (is_active_clk_interrupt)  cpu_unit->cpu_active_clk_interrupt = TRUE;
                if (is_active_ipi_interrupt)  cpu_unit->cpu_active_ipi_interrupt = TRUE;
                if (cpu_unit->cpu_active_clk_interrupt || cpu_unit->cpu_active_ipi_interrupt)
                {
                    if (must_control_prio())
                        smp_set_thread_priority(cpu_unit->cpu_thread, SIMH_THREAD_PRIORITY_CPU_CRITICAL_OS_HI);
                    cpu_unit->cpu_thread_priority = SIMH_THREAD_PRIORITY_CPU_CRITICAL_OS_HI;
                }
                break;
            default:
                if (must_control_prio())
                    smp_set_thread_priority(cpu_unit->cpu_thread, SIMH_THREAD_PRIORITY_CPU_CRITICAL_OS);
                cpu_unit->cpu_thread_priority = SIMH_THREAD_PRIORITY_CPU_CRITICAL_OS;
                break;
            }
        }
    }

    return (hipl > cipl || nmi) ? hipl : 0;
}

/* Return vector for highest priority hardware interrupt at IPL lvl,
   called after previous eval_int */

int32 get_vector (RUN_DECL, int32 lvl)
{
    if (lvl == IPL_MEMERR)                                  /* mem error? */
    {
        mem_err = 0;
        return SCB_MEMERR;
    }
    else if (lvl == IPL_CRDERR)                             /* CRD error? */
    {
        crd_err = 0;
        return SCB_CRDERR;
    }
    else if (lvl > IPL_HMAX)                                /* error req lvl? */
    {
        ABORT (STOP_UIPL);                                  /* unknown intr */
    }

    uint32 dev;

    if (cpu_unit->cpu_intreg.check_int_atipl_clr(RUN_PASS, (uint32) lvl, & dev))
    {
        int32 l = lvl - IPL_HMIN;

        /* 
         * Execute RMB -- even if requesting device is per-CPU, since there may be
         * other interrupts pending, for system-wide devices, and there potentially
         * may be a tricky dependence.
         */
        smp_post_interlocked_rmb();

        if (l == IPL_CLK && dev == INT_V_CLK)
        {
            /* entering CLK ISR */
            cpu_unit->cpu_active_clk_interrupt = TRUE;
        }
        else if (l == IPL_IPINTR && dev == INT_V_IPINTR)
        {
            /* entering IPI ISR */
            cpu_unit->cpu_active_ipi_interrupt = TRUE;
        }

        int32 (*ack)() = weak_read(int_ack[l][dev]);
        if (ack)
            return (*ack)();
        else
            return weak_read(int_vec[l][dev]);
    }

    return 0;
}

/*
 * Copy IRQs from interlocked to local storage. 
 * Caller must execute RMB before calling read_irqs_to_local.
 */
void read_irqs_to_local(RUN_DECL)
{
    for (;;)
    {
        cpu_unit->cpu_intreg.copy_irqs_to_local();

        /*
         * process internal non-maskable interrupts
         */

        /* handle IPIRMB and reload */
        if (unlikely(cpu_unit->cpu_intreg.is_local_int(IPL_ABS_IPIRMB, INT_V_IPIRMB)))
        {
            cpu_unit->cpu_intreg.dismiss_int(RUN_PASS, IPL_ABS_IPIRMB, INT_V_IPIRMB);
            smp_post_interlocked_rmb();
            continue;
        }

        /* notification: secondary VCPU had stopped */
        if (unlikely(cpu_unit->cpu_intreg.is_local_int(IPL_ABS_SECEXIT, INT_V_SECEXIT)))
        {
            cpu_unit->cpu_intreg.dismiss_int(RUN_PASS, IPL_ABS_SECEXIT, INT_V_SECEXIT);

            /* 
             * Transfer pending events for system-wide device from exited secndary (or exited secondaries)
             * to the primary.
             */
            sim_requeue_syswide_events(RUN_PASS);

            /*
             * If last secondary had exited and only the primary remains active, stop managing thread
             * priority and drop primary VCPU actual thread priority down to CPU_RUN level. The only 
             * (extremely unlikely) exception is if calibration is active, in which case priority will
             * be dropped later when processor exits CALIBRATION level.
             *
             * For more detailed explanation of what is done here and why, refer to the comment in cpu_once_a_second().
             */
            if (!must_control_prio() && !tmr_is_active(RUN_PASS))
                smp_set_thread_priority(SIMH_THREAD_PRIORITY_CPU_RUN);

            continue;
        }

        /* notification: this VCPU had been entered into synchronization subwindow SYNCW_SYS by another VCPU */
        if (unlikely(cpu_unit->cpu_intreg.is_local_int(IPL_ABS_SYNCWSYS, INT_V_SYNCWSYS)))
        {
            syncw_process_syncwsys_interrupt(RUN_PASS);
            cpu_unit->cpu_intreg.dismiss_int(RUN_PASS, IPL_ABS_SYNCWSYS, INT_V_SYNCWSYS);
            continue;
        }

        /* handle SYNCLK (sent by clock strobe thread) */
        if (cpu_unit->cpu_intreg.is_local_int(IPL_ABS_SYNCLK, INT_V_SYNCLK))
        {
            cpu_unit->cpu_intreg.dismiss_int(RUN_PASS, IPL_ABS_SYNCLK, INT_V_SYNCLK);

            /* record current CPU cycles count at SYNCLK for future delta timing */
            cpu_unit->cpu_last_synclk_cycles = CPU_CURRENT_CYCLES;

            if (cpu_unit->cpu_synclk_protect == FALSE)
            {
                /* process SYNCLK and set protection period */
                SynclkPending sv_pending = cpu_unit->cpu_synclk_pending;

                if (clk_csr & CSR_IE)
                    cpu_unit->cpu_synclk_protect_os = synclk_safe_cycles;
                cpu_unit->cpu_synclk_protect_dev = (uint32) sim_calculate_device_activity_protection_interval(RUN_PASS);
                cpu_unit->cpu_synclk_protect = cpu_unit->cpu_synclk_protect_os && cpu_unit->cpu_synclk_protect_dev;
                process_synclk(RUN_PASS, TRUE);

                /*
                 * Leave cpu_unit->cpu_synclk_pending at previous value.
                 * Normally it would be NotPending when cpu_synclk_protect is FALSE and will be left NotPending.
                 * In the "should not happen" case it was set, leave it set, but record current state of CLK CSR_IE.
                 */
                if (sv_pending != SynclkNotPending)  /* should not happen, but handle anyway */
                    cpu_unit->cpu_synclk_pending = (clk_csr & CSR_IE) ? SynclkPendingIE1 : SynclkPendingIE0;
            }
            else if (cpu_unit->cpu_synclk_pending != SynclkNotPending)
            {
                /* 
                 * Clock interrupt generation from previous SYNCLK signal is already pending (as soon as
                 * cpu_synclk_protect is counted down), just dismiss newly received signal
                 * and "lose" new clock interrupt.
                 */
                 cpu_unit->cpu_synclk_pending = (clk_csr & CSR_IE) ? SynclkPendingIE1 : SynclkPendingIE0;
            }
            else
            {
                /*
                 * Still inside the interval protected from raising CLK by SYNCLK.
                 * Mark SYNCLK processing as pending.
                 */
                 cpu_unit->cpu_synclk_pending = (clk_csr & CSR_IE) ? SynclkPendingIE1 : SynclkPendingIE0;
                 cpu_reevaluate_thread_priority(RUN_PASS);
            }
        }

        /* handle ASYNC_IO */
        if (cpu_unit->cpu_intreg.is_local_int(IPL_ABS_ASYNC_IO, INT_V_ASYNC_IO))
        {
            cpu_unit->cpu_intreg.dismiss_int(RUN_PASS, IPL_ABS_ASYNC_IO, INT_V_ASYNC_IO);
            sim_async_process_io_events(RUN_PASS);
            continue;
        }

        /* handle emergency STOP request sent by the primary */
        if (cpu_unit->cpu_intreg.is_local_int(IPL_ABS_STOP, INT_V_STOP))
        {
            cpu_unit->cpu_intreg.dismiss_int(RUN_PASS, IPL_ABS_STOP, INT_V_STOP);
            cpu_unit->cpu_dostop = TRUE;
            sim_activate_abs(cpu_unit, 0);
            syncw_leave_all(RUN_PASS, SYNCW_OVERRIDE_ALL | SYNCW_DISABLE_CPU);
        }

        break;
    }
}

/*
 * Check if SYNCLK event is pending, and if so, process it.
 */
t_bool check_synclk_pending(RUN_DECL)
{
    if (cpu_unit->cpu_synclk_pending != SynclkNotPending)
    {
        t_bool clk_ie = cpu_unit->cpu_synclk_pending == SynclkPendingIE1;
        cpu_unit->cpu_synclk_pending = SynclkNotPending;
        process_synclk(RUN_PASS, clk_ie);
        syncw_reeval_sys(RUN_PASS);
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

/*
 * Process SYNCLK event.
 */
void process_synclk(RUN_DECL, t_bool clk_ie)
{
    /* schedule events co-scheduled with clock for immediate execution */
    sim_reschedule_cosched(RUN_PASS, RescheduleCosched_OnSynClk);

    /* invoke clock handler */
    clk_svc_ex(RUN_PASS, clk_ie);

    cpu_reevaluate_thread_priority(RUN_PASS);
}

/* CQBIC registers

   SCR          system configuration register
   DSER         DMA system error register (W1C)
   MEAR         master error address register (RO)
   SEAR         slave error address register (RO)
   MBR          map base register
   IPC          inter-processor communication register
*/

int32 cqbic_rd (RUN_DECL, int32 pa)
{
    int32 rg = (pa - CQBICBASE) >> 2;

    switch (rg)
    {
    case 0:                                             /* SCR */
        return (cq_scr | CQSCR_POK) & CQSCR_MASK;

    case 1:                                             /* DSER */
        return cq_dser & CQDSER_MASK;

    case 2:                                             /* MEAR */
        return cq_mear & CQMEAR_MASK;

    case 3:                                             /* SEAR */
        return cq_sear & CQSEAR_MASK;

    case 4:                                             /* MBR */
        return cq_mbr & CQMBR_MASK;
    }
    return 0;
}

void cqbic_wr (RUN_DECL, int32 pa, int32 val, int32 lnt)
{
    int32 nval, rg = (pa - CQBICBASE) >> 2;

    if (lnt < L_LONG)
    {
        int32 sc = (pa & 3) << 3;
        int32 mask = (lnt == L_WORD)? 0xFFFF: 0xFF;
        int32 t = cqbic_rd (RUN_PASS, pa);
        nval = ((val & mask) << sc) | (t & ~(mask << sc));
        val = val << sc;
    }
    else
    {
        nval = val;
    }

    switch (rg)
    {
    case 0:                                             /* SCR */
        cq_scr = ((cq_scr & ~CQSCR_RW) | (nval & CQSCR_RW)) & CQSCR_MASK;
        break;

    case 1:                                             /* DSER */
        cq_dser = (cq_dser & ~val) & CQDSER_MASK;
        if (val & CQDSER_SME)
            cq_ipc = cq_ipc & ~CQIPC_QME;
        break;

    case 2: case 3:
        cq_merr (pa);                                   /* MEAR, SEAR */
        MACH_CHECK (MCHK_WRITE);
        break;

    case 4:                                             /* MBR */
        if (! cpu_unit->is_primary_cpu())
        {
            smp_printf ("\nNon-primary CPU (CPU%d) attempted to write QBus map base register (CQBIC MBR)\n", cpu_unit->cpu_id);
            if (sim_log)
                fprintf (sim_log, "Non-primary CPU (CPU%d) attempted to write QBus map base register (CQBIC MBR)\n", cpu_unit->cpu_id);
            ABORT_INVALID_SYSOP;
        }
        /*
         * We do not execute memory barrier when writing or reading MBR, that would
         * degrade performance during QBus references.
         *
         * It is presumed MBR will be set by the primary (boot) processor at operating system
         * initialization time only, before other processors are started.
         *
         * In most unlikely case that OS should ever want to change MBR after booting,
         * it will do it within proper locking protocol on adapter structure that will ultimately
         * use VAX interlocked instuctions and thus cause memory barriers.
         */
        cq_mbr = nval & CQMBR_MASK;
        break;
    }
}

/* IPC can be read as local register or as Qbus I/O
   Because of the W1C */

int32 cqipc_rd (RUN_DECL, int32 pa)
{
    return cq_ipc & CQIPC_MASK;                             /* IPC */
}

void cqipc_wr (RUN_DECL, int32 pa, int32 val, int32 lnt)
{
    int32 nval = val;

    if (lnt < L_LONG)
    {
        int32 sc = (pa & 3) << 3;
        nval = val << sc;
    }

    cq_ipc = cq_ipc & ~(nval & CQIPC_W1C);                  /* W1C */
    if ((pa & 3) == 0)                                      /* low byte only */
        cq_ipc = ((cq_ipc & ~CQIPC_RW) | (val & CQIPC_RW)) & CQIPC_MASK;
}

/* I/O page routines */

t_stat dbl_rd (int32 *data, int32 addr, int32 access)
{
    RUN_SCOPE;
    *data = cq_ipc & CQIPC_MASK;
    return SCPE_OK;
}

t_stat dbl_wr (int32 data, int32 addr, int32 access)
{
    RUN_SCOPE;
    cqipc_wr (RUN_PASS, addr, data, (access == WRITEB)? L_BYTE: L_WORD);
    return SCPE_OK;
}

/*
 * CQBIC map read and write (reflects to main memory)
 *
 *  Read error: set DSER<0>, latch slave address, machine check
 *  Write error: set DSER<0>, latch slave address, memory error interrupt
 */

int32 cqmap_rd (RUN_DECL, int32 pa)
{
    int32 ma = (pa & CQMAPAMASK) + cq_mbr;                  /* mem addr */

    if (ADDR_IS_MEM (ma))
        return M[ma >> 2];
    cq_serr (ma);                                           /* set err */
    MACH_CHECK (MCHK_READ);                                 /* mcheck */
    return 0;
}

void cqmap_wr (RUN_DECL, int32 pa, int32 val, int32 lnt)
{
    int32 ma = (pa & CQMAPAMASK) + cq_mbr;                  /* mem addr */

    if (ADDR_IS_MEM (ma))
    {
        if (lnt < L_LONG)
        {
            int32 sc = (pa & 3) << 3;
            int32 mask = (lnt == L_WORD)? 0xFFFF: 0xFF;
            int32 t = M[ma >> 2];
            val = ((val & mask) << sc) | (t & ~(mask << sc));
        }
        M[ma >> 2] = val;
    }
    else
    {
        cq_serr (ma);                                       /* error */
        mem_err = 1;
    }
}

/*
 * CQBIC Qbus memory read and write (reflects to main memory).
 * May give master or slave error, depending on where the failure occurs
 */

int32 cqmem_rd (RUN_DECL, int32 pa)
{
    int32 qa = pa & CQMAMASK;                               /* Qbus addr */
    uint32 ma;

    if (qba_map_addr (RUN_PASS, qa, &ma))                   /* map addr */
        return M[ma >> 2];
    MACH_CHECK (MCHK_READ);                                 /* err? mcheck */
    return 0;
}

void cqmem_wr (RUN_DECL, int32 pa, int32 val, int32 lnt)
{
    int32 qa = pa & CQMAMASK;                               /* Qbus addr */
    uint32 ma;

    if (qba_map_addr (RUN_PASS, qa, &ma))                    /* map addr */
    {
        if (lnt < L_LONG)
        {
            int32 sc = (pa & 3) << 3;
            int32 mask = (lnt == L_WORD)? 0xFFFF: 0xFF;
            int32 t = M[ma >> 2];
            val = ((val & mask) << sc) | (t & ~(mask << sc));
        }
        M[ma >> 2] = val;
    }
    else
    {
        mem_err = 1;
    }
}

/* Map an address via the translation map */

t_bool qba_map_addr (RUN_DECL, uint32 qa, uint32 *ma)
{
    int32 qblk = (qa >> VA_V_VPN);                          /* Qbus blk */
    int32 qmma = ((qblk << 2) & CQMAPAMASK) + cq_mbr;       /* map entry */

    if (ADDR_IS_MEM (qmma))                                 /* legit? */
    {
        int32 qmap = M[qmma >> 2];                          /* get map */
        if (qmap & CQMAP_VLD)                               /* valid? */
        {
            *ma = ((qmap & CQMAP_PAG) << VA_V_VPN) + VA_GETOFF (qa);
            if (ADDR_IS_MEM (*ma))                          /* legit addr */
                return TRUE;
            cq_serr (*ma);                                  /* slave nxm */
            return FALSE;
        }
        cq_merr (qa);                                       /* master nxm */
        return FALSE;
    }
    cq_serr (0);                                            /* inv mem */
    return FALSE;
}

/* Map an address via the translation map - console version (no status changes) */

t_bool qba_map_addr_c (RUN_DECL, uint32 qa, uint32 *ma)
{
    int32 qblk = (qa >> VA_V_VPN);                          /* Qbus blk */
    int32 qmma = ((qblk << 2) & CQMAPAMASK) + cq_mbr;       /* map entry */

    if (ADDR_IS_MEM (qmma))                                 /* legit? */
    {
        int32 qmap = M[qmma >> 2];                          /* get map */
        if (qmap & CQMAP_VLD)                               /* valid? */
        {
            *ma = ((qmap & CQMAP_PAG) << VA_V_VPN) + VA_GETOFF (qa);
            return TRUE;                                    /* legit addr */
        }
    }
    return FALSE;
}

/* Set master error */

void cq_merr (int32 pa)
{
    RUN_SCOPE;
    if (cq_dser & CQDSER_ERR)
        cq_dser = cq_dser | CQDSER_LST;
    cq_dser = cq_dser | CQDSER_MNX;                         /* master nxm */
    cq_mear = (pa >> VA_V_VPN) & CQMEAR_MASK;               /* page addr */
}

/* Set slave error */

void cq_serr (int32 pa)
{
    RUN_SCOPE;
    if (cq_dser & CQDSER_ERR)
        cq_dser = cq_dser | CQDSER_LST;
    cq_dser = cq_dser | CQDSER_SNX;                         /* slave nxm */
    cq_sear = (pa >> VA_V_VPN) & CQSEAR_MASK;
}

/* Reset I/O bus */

void ioreset_wr (RUN_DECL, int32 data)
{
    /* disrallow writing to IORESET on any processor but the primary */
    if (cpu_unit->is_primary_cpu())
    {
        reset_all (sim_device_index (&qba_dev));   /* reset from qba on... */
    }
    else
    {
        smp_printf ("\nNon-primary CPU (CPU%d) attempted to reset QBus and IO devices by writing to IORESET register\n", cpu_unit->cpu_id);
        if (sim_log)
            fprintf (sim_log, "Non-primary CPU (CPU%d) attempted to reset QBus  and IO devices by writing to IORESET register\n", cpu_unit->cpu_id);
        ABORT_INVALID_SYSOP;
    }
}

/* Reset CQBIC */

t_stat qba_reset (DEVICE *dptr)
{
    RUN_SCOPE;

    if (! cpu_unit->is_primary_cpu())
    {
        // QBA reset should be performed only by the primary CPU
        smp_printf ("\nNon-primary CPU (CPU%d) attempted to reset QBus\n", cpu_unit->cpu_id);
        if (sim_log)
            fprintf (sim_log, "Non-primary CPU (CPU%d) attempted to reset QBus\n", cpu_unit->cpu_id);
        ABORT_INVALID_SYSOP;
    }

    if (sim_switches & SWMASK ('P'))
    {
        /* Powerup CQBIC */
        if (! smp_check_aligned(& cq_mbr, FALSE) ||
            ! smp_check_aligned((atomic_int32*) int_vec, FALSE) ||
            ! smp_check_aligned(& ((DIB*)0)->vec, FALSE))
        {
            return SCPE_IERR;
        }
        cq_mbr = 0;
        cqbic_reset_percpu(RUN_PASS, TRUE);
    }
    else
    {
        cqbic_reset_percpu(RUN_PASS, FALSE);
    }

    /*
     * Reset interrupts for all QBus devices.
     * 
     * Note we cannot do generic cpu_unit->cpu_intreg.reset() since that resets
     * not only QBus interrupts, but all other interrupts for the primary CPU,
     * such as IPIRMB, SECEXIT, SYNCWSYS, ASYNC_IO, CLK, SYNCLK, IPINTR etc., that should be retained.
     */
    CLR_INT(RQ);
    CLR_INT(RL);
    CLR_INT(DZRX);
    CLR_INT(DZTX);
    CLR_INT(TS);
    CLR_INT(TQ);
    CLR_INT(XQ);
    CLR_INT(RY);
    // CLR_INT(TTI);    // TTI is not architecturally a QBus device
    // CLR_INT(TTO);    // TTO is not architecturally a QBus device
    // CLR_INT(PTR);    // PTR is not a part of VAX MP
    // CLR_INT(PTP);    // PTP is not a part of VAX MP
    CLR_INT(LPT);
    CLR_INT(CSI);
    CLR_INT(CSO);
    CLR_INT(TMR0);
    CLR_INT(TMR1);
    CLR_INT(VHRX);
    CLR_INT(VHTX);
    CLR_INT(QDSS);
    CLR_INT(CR);

    return SCPE_OK;
}

void cqbic_reset_percpu(RUN_DECL, t_bool powerup)
{
    if (powerup)
        cq_scr = CQSCR_POK;
    else
        cq_scr = (cq_scr & CQSCR_BHL) | CQSCR_POK;

    cq_dser = cq_mear = cq_sear = cq_ipc = 0;
}

/* Qbus I/O buffer routines, aligned access

   Map_ReadB    -       fetch byte buffer from memory
   Map_ReadW    -       fetch word buffer from memory
   Map_WriteB   -       store byte buffer into memory
   Map_WriteW   -       store word buffer into memory
*/

int32 Map_ReadB (RUN_DECL, uint32 ba, int32 bc, uint8 *buf)
{
    int32 i;
    uint32 ma, dat;

    if ((ba | bc) & 03)                                     /* check alignment */
    {
        for (i = ma = 0; i < bc; i++, buf++)                /* by bytes */
        {
            if ((ma & VA_M_OFF) == 0)                       /* need map? */
            {
                if (!qba_map_addr (RUN_PASS, ba + i, &ma))  /* inv or NXM? */
                    return (bc - i);
            }
            *buf = ReadB (RUN_PASS, ma);
            ma = ma + 1;
        }
    }
    else
    {
        for (i = ma = 0; i < bc; i = i + 4, buf++)          /* by longwords */
        {
            if ((ma & VA_M_OFF) == 0)                       /* need map? */
            {
                if (!qba_map_addr (RUN_PASS, ba + i, &ma))  /* inv or NXM? */
                    return (bc - i);
            }
            dat = ReadL_NA (RUN_PASS, ma);                  /* get lw */
            *buf++ = dat & BMASK;                           /* low 8b */
            *buf++ = (dat >> 8) & BMASK;                    /* next 8b */
            *buf++ = (dat >> 16) & BMASK;                   /* next 8b */
            *buf = (dat >> 24) & BMASK;
            ma = ma + 4;
        }
    }
    return 0;
}

int32 Map_ReadW (RUN_DECL, uint32 ba, int32 bc, uint16 *buf)
{
    int32 i;
    uint32 ma,dat;

    /*
     * Multiprocessor note:
     *
     * Devices that interact with CPU via shared memory structures (XQ, TQ, RQ) need
     * to fetch and store certain elements as an atomic operation. This includes words
     * in XQ BDL descriptor and words in UQSSP (RQ, TQ) command ring and com-area.
     * Map_ReadW must ensure that they are read from memory atomically, as a word, 
     * by using host "read word" instruction, and without splitting them into multiple byte reads.
     *
     * Implementation of ReadW_NA/ReadL_NA for x86/x64 ensures it, for other processors
     * code for Map_ReadW may need to be revised to ensure atomicity of QBus word transactions,
     * whenever required.
     *
     * Also, should Map_ReadW be optimizied in the future to use memcpy, this optimization
     * should not apply for small reads that may be atomic (identified as either with bc == 2
     * or perhaps special flag passed from XQ, TQ and RQ).
     */

    ba = ba & ~01;
    bc = bc & ~01;
    if ((ba | bc) & 03)                                     /* check alignment */
    {
        for (i = ma = 0; i < bc; i = i + 2, buf++)          /* by words */
        {
            if ((ma & VA_M_OFF) == 0)                       /* need map? */
            {
                if (!qba_map_addr (RUN_PASS, ba + i, &ma))  /* inv or NXM? */
                    return (bc - i);
            }
            *buf = ReadW_NA (RUN_PASS, ma);
            ma = ma + 2;
        }
    }
    else
    {
        for (i = ma = 0; i < bc; i = i + 4, buf++)          /* by longwords */
        {
            if ((ma & VA_M_OFF) == 0)                       /* need map? */
            {
                if (!qba_map_addr (RUN_PASS, ba + i, &ma))  /* inv or NXM? */
                    return (bc - i);
            }
            dat = ReadL_NA (RUN_PASS, ma);                  /* get lw */
            *buf++ = dat & WMASK;                           /* low 16b */
            *buf = (dat >> 16) & WMASK;                     /* high 16b */
            ma = ma + 4;
        }
    }
    return 0;
}

int32 Map_WriteB (RUN_DECL, uint32 ba, int32 bc, uint8 *buf)
{
    int32 i;
    uint32 ma, dat;

    if ((ba | bc) & 03)                                     /* check alignment */
    {
        for (i = ma = 0; i < bc; i++, buf++)                /* by bytes */
        {
            if ((ma & VA_M_OFF) == 0)                       /* need map? */
            {
                if (!qba_map_addr (RUN_PASS, ba + i, &ma))  /* inv or NXM? */
                    return (bc - i);
            }
            WriteB (RUN_PASS, ma, *buf);
            ma = ma + 1;
        }
    }
    else
    {
        for (i = ma = 0; i < bc; i = i + 4, buf++)          /* by longwords */
        {
            if ((ma & VA_M_OFF) == 0)                       /* need map? */
            {
                if (!qba_map_addr (RUN_PASS, ba + i, &ma))  /* inv or NXM? */
                    return (bc - i);
            }
            dat = (uint32) *buf++;                          /* get low 8b */
            dat = dat | (((uint32) *buf++) << 8);           /* merge next 8b */
            dat = dat | (((uint32) *buf++) << 16);          /* merge next 8b */
            dat = dat | (((uint32) *buf) << 24);            /* merge hi 8b */
            WriteL_NA (RUN_PASS, ma, dat);                  /* store lw */
            ma = ma + 4;
        }
    }
    return 0;
}

int32 Map_WriteW (RUN_DECL, uint32 ba, int32 bc, uint16 *buf)
{
    int32 i;
    uint32 ma, dat;

    /*
     * Multiprocessor note:
     *
     * Devices that interact with CPU via shared memory structures (XQ, TQ, RQ) need
     * to fetch and store certain elements as an atomic operation. This includes words
     * in XQ BDL descriptor and words in UQSSP (RQ, TQ) command ring and com-area.
     * Map_WriteW must ensure that they are written to memory atomically, as a word, 
     * by using host "write word" instruction, and without splitting them into multiple byte writes.
     *
     * Implementation of WriteW_NA/WriteL_NA for x86/x64 ensures it, for other processors
     * code for Map_WriteW may need to be revised to ensure atomicity of QBus word transactions,
     * whenever required.
     *
     * Also, should Map_WriteW be optimizied in the future to use memcpy, this optimization
     * should not apply for small writes that may be atomic (identified as either with bc == 2
     * or perhaps special flag passed from XQ, TQ and RQ).
     */

    ba = ba & ~01;
    bc = bc & ~01;
    if ((ba | bc) & 03)                                     /* check alignment */
    {
        for (i = ma = 0; i < bc; i = i + 2, buf++)          /* by words */
        {
            if ((ma & VA_M_OFF) == 0)                       /* need map? */
            {
                if (!qba_map_addr (RUN_PASS, ba + i, &ma))  /* inv or NXM? */
                    return (bc - i);
            }
            WriteW_NA (RUN_PASS, ma, *buf);
            ma = ma + 2;
        }
    }
    else
    {
        for (i = ma = 0; i < bc; i = i + 4, buf++)          /* by longwords */
        {
            if ((ma & VA_M_OFF) == 0)                       /* need map? */
            {
                if (!qba_map_addr (RUN_PASS, ba + i, &ma))  /* inv or NXM? */
                    return (bc - i);
            }
            dat = (uint32) *buf++;                          /* get low 16b */
            dat = dat | (((uint32) *buf) << 16);            /* merge hi 16b */
            WriteL_NA (RUN_PASS, ma, dat);                  /* store lw */
            ma = ma + 4;
        }
    }
    return 0;
}

/* Memory examine via map (word only) */

t_stat qba_ex (t_value *vptr, t_addr exta, UNIT *uptr, int32 sw)
{
    RUN_SCOPE;
    uint32 qa = (uint32) exta, pa;

    if ((vptr == NULL) || (qa >= CQMSIZE))
        return SCPE_ARG;
    if (qba_map_addr_c (RUN_PASS, qa, &pa) && ADDR_IS_MEM (pa))
    {
        *vptr = (uint32) ReadW_NA (RUN_PASS, pa);
        return SCPE_OK;
    }
    return SCPE_NXM;
}

/* Memory deposit via map (word only) */

t_stat qba_dep (t_value val, t_addr exta, UNIT *uptr, int32 sw)
{
    RUN_SCOPE;
    uint32 qa = (uint32) exta, pa;

    if (qa >= CQMSIZE)
        return SCPE_ARG;
    if (qba_map_addr_c (RUN_PASS, qa, &pa) && ADDR_IS_MEM (pa))
    {
        WriteW_NA (RUN_PASS, pa, (int32) val);
        return SCPE_OK;
    }
    return SCPE_NXM;
}

/* Build dib_tab from device list */

t_stat build_dib_tab (void)
{
    int32 i;
    DEVICE *dptr;
    DIB *dibp;
    t_stat r;

    init_ubus_tab ();                                       /* init bus tables */
    for (i = 0; (dptr = sim_devices[i]) != NULL; i++)       /* loop thru dev */
    {
        dibp = (DIB *) dptr->ctxt;                          /* get DIB */
        if (dibp && !(dptr->flags & DEV_DIS))               /* defined, enabled? */
        {
            if (r = build_ubus_tab (dptr, dibp))            /* add to bus tab */
                return r;
        }
    }
    return SCPE_OK;
}

/* Show QBA virtual address */

t_stat qba_show_virt (SMP_FILE *of, UNIT *uptr, int32 val, void *desc)
{
    RUN_SCOPE;
    t_stat r;
    char *cptr = (char *) desc;
    uint32 qa, pa;

    if (cptr) {
        qa = (uint32) get_uint (cptr, 16, CQMSIZE - 1, &r);
        if (r == SCPE_OK) {
            if (qba_map_addr_c (RUN_PASS, qa, &pa))
                fprintf (of, "Qbus %-X = physical %-X\n", qa, pa);
            else fprintf (of, "Qbus %-X: invalid mapping\n", qa);
            return SCPE_OK;
            }
        }
    fprintf (of, "Invalid argument\n");
    return SCPE_OK;
}
