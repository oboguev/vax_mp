/* vax_stddev.c: VAX 3900 standard I/O devices

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

   tti          terminal input
   tto          terminal output
   clk          100Hz and TODR clock

   17-Aug-08    RMS     Resync TODR on any clock reset
   18-Jun-07    RMS     Added UNIT_IDLE flag to console input, clock
   17-Oct-06    RMS     Synced keyboard poll to real-time clock for idling
   22-Nov-05    RMS     Revised for new terminal processing routines
   09-Sep-04    RMS     Integrated powerup into RESET (with -p)
   28-May-04    RMS     Removed SET TTI CTRL-C
   29-Dec-03    RMS     Added console backpressure support
   25-Apr-03    RMS     Revised for extended file support
   02-Mar-02    RMS     Added SET TTI CTRL-C
   22-Dec-02    RMS     Added console halt capability
   01-Nov-02    RMS     Added 7B/8B capability to terminal
   12-Sep-02    RMS     Removed paper tape, added variable vector support
   30-May-02    RMS     Widened POS to 32b
   30-Apr-02    RMS     Automatically set TODR to VMS-correct value during boot
*/

#include "sim_defs.h"
#include "vax_defs.h"
#include <time.h>

#define TTICSR_IMP      (CSR_DONE + CSR_IE)             /* terminal input */
#define TTICSR_RW       (CSR_IE)
#define TTIBUF_ERR      0x8000                          /* error */
#define TTIBUF_OVR      0x4000                          /* overrun */
#define TTIBUF_FRM      0x2000                          /* framing error */
#define TTIBUF_RBR      0x0400                          /* receive break */
#define TTOCSR_IMP      (CSR_DONE + CSR_IE)             /* terminal output */
#define TTOCSR_RW       (CSR_IE)
#define CLKCSR_IMP      (CSR_IE)                        /* real-time clock */
#define CLKCSR_RW       (CSR_IE)
#define CLK_DELAY       5000                            /* 100 Hz */

extern int32 sim_switches;

int32 clk_tps = CLK_TPS;                                                  /* ticks/second */
atomic_int32_var tmxr_poll = atomic_var_init(CLK_DELAY * TMXR_MULT);      /* term mux poll */
atomic_int32_var tmr_poll = atomic_var_init(CLK_DELAY);                   /* pgm timer poll */

t_stat tti_svc (RUN_SVC_DECL, UNIT *uptr);
t_stat tto_svc (RUN_SVC_DECL, UNIT *uptr);
t_stat clk_svc (RUN_SVC_DECL, UNIT *uptr);
t_stat tti_reset (DEVICE *dptr);
t_stat tto_reset (DEVICE *dptr);
t_stat clk_reset (DEVICE *dptr);
t_stat todr_resync (void);

extern int32 sysd_hlt_enb (void);

/* TTI data structures

   tti_dev      TTI device descriptor
   tti_unit     TTI unit descriptor
   tti_reg      TTI register list
*/

#define TTI_TYPEAHEAD_SIZE 256
static sim_ring_buffer<int32, TTI_TYPEAHEAD_SIZE> tti_typeahead;
AUTO_INIT_LOCK(tti_typeahead_lock, SIM_LOCK_CRITICALITY_OS_HI, DEVLOCK_SPINWAIT_CYCLES);                        

DIB tti_dib = { 0, 0, NULL, NULL, 1, IVCL (TTI), SCB_TTI, { NULL } };

UNIT tti_unit UDATA_SINGLE_WAIT (&tti_svc, UNIT_IDLE|TT_MODE_8B, 0, 0);
UNIT_TABLE_SINGLE(tti_unit);

REG tti_reg[] = {
    { HRDATA_GBL (BUF, tti_unit.buf, 16) },
    { HRDATA_CPU ("CSR", r_tti_csr, 16) },
    { IRDATA_DEV (INT, IVCL (TTI)) },
    { FLDATA_CPU ("DONE", r_tti_csr, CSR_V_DONE) },
    { FLDATA_CPU ("IE", r_tti_csr, CSR_V_IE) },
    { DRDATA_GBL (POS, tti_unit.pos, T_ADDR_W), PV_LEFT },
    { DRDATA_GBL (TIME, tti_unit.wait, 24), PV_LEFT },
    { NULL }
    };

MTAB tti_mod[] = {
    { TT_MODE, TT_MODE_7B, "7b", "7B", NULL },
    { TT_MODE, TT_MODE_8B, "8b", "8B", NULL },
    { MTAB_XTD|MTAB_VDV, 0, "VECTOR", NULL,
      NULL, &show_vec, NULL },
    { 0 }
    };

DEVICE tti_dev = {
    "TTI", tti_unit_table, tti_reg, tti_mod,
    1, 10, 31, 1, 16, 8,
    NULL, NULL, &tti_reset,
    NULL, NULL, NULL,
    &tti_dib, DEV_PERCPU
    };

/* TTO data structures

   tto_dev      TTO device descriptor
   tto_unit     TTO unit descriptor
   tto_reg      TTO register list
*/

DIB tto_dib = { 0, 0, NULL, NULL, 1, IVCL (TTO), SCB_TTO, { NULL } };

UNIT tto_unit UDATA_SINGLE_WAIT (&tto_svc, TT_MODE_8B, 0, SERIAL_OUT_WAIT);
UNIT_TABLE_SINGLE(tto_unit);

REG tto_reg[] = {
    { HRDATA_CPU ("BUF", r_tto_buf, 8) },
    { HRDATA_CPU ("CSR", r_tto_csr, 16) },
    { IRDATA_DEV (INT, IVCL (TTO)) },
    { FLDATA_CPU ("DONE", r_tto_csr, CSR_V_DONE) },
    { FLDATA_CPU ("IE", r_tto_csr, CSR_V_IE) },
    { DRDATA_GBL (POS, tto_unit.pos, T_ADDR_W), PV_LEFT },
    { DRDATA_GBL (TIME, tto_unit.wait, 24), PV_LEFT },
    { NULL }
    };

MTAB tto_mod[] = {
    { TT_MODE, TT_MODE_7B, "7b", "7B", NULL },
    { TT_MODE, TT_MODE_8B, "8b", "8B", NULL },
    { TT_MODE, TT_MODE_7P, "7p", "7P", NULL },
    { MTAB_XTD|MTAB_VDV, 0, "VECTOR", NULL,     NULL, &show_vec },
    { 0 }
    };

DEVICE tto_dev = {
    "TTO", tto_unit_table, tto_reg, tto_mod,
    1, 10, 31, 1, 16, 8,
    NULL, NULL, &tto_reset,
    NULL, NULL, NULL,
    &tto_dib, DEV_PERCPU
    };

/* CLK data structures

   clk_dev      CLK device descriptor
   clk_unit     CLK unit descriptor
   clk_reg      CLK register list
*/

DIB clk_dib = { 0, 0, NULL, NULL, 1, IVCL (CLK), SCB_INTTIM, { NULL } };

UNIT clk_unit UDATA_SINGLE_WAIT (&clk_svc, UNIT_IDLE, 0, CLK_DELAY);
UNIT_TABLE_SINGLE(clk_unit);

REG clk_reg[] = {
    { HRDATA_CPU ("CSR", r_clk_csr, 16) },
    { IRDATA_DEV (INT, IVCL (CLK)) },
    { FLDATA_CPU ("IE", r_clk_csr, CSR_V_IE) },
    { DRDATA_CPU ("TODR", r_todr_reg, 32), PV_LEFT },
    { FLDATA_CPU ("BLOW", r_todr_blow, 0) },
    { DRDATA_GBL (TIME, clk_unit.wait, 24), REG_NZ + PV_LEFT },
    { DRDATA_GBL (POLL, atomic_var(tmr_poll), 24), REG_NZ + PV_LEFT + REG_HRO },
    { DRDATA_GBL (TPS, clk_tps, 8), REG_NZ + PV_LEFT },
    { NULL }
    };

MTAB clk_mod[] = {
    { MTAB_XTD|MTAB_VDV, 0, "VECTOR", NULL,     NULL, &show_vec },
    { 0 }
    };

DEVICE clk_dev = {
    "CLK", clk_unit_table, clk_reg, clk_mod,
    1, 0, 0, 0, 0, 0,
    NULL, NULL, &clk_reset,
    NULL, NULL, NULL,
    &clk_dib, DEV_PERCPU
    };

/* Clock and terminal MxPR routines

   iccs_rd/wr   interval timer
   todr_rd/wr   time of year clock
   rxcs_rd/wr   input control/status
   rxdb_rd      input buffer
   txcs_rd/wr   output control/status
   txdb_wr      output buffer
*/

int32 iccs_rd (RUN_DECL)
{
    return (clk_csr & CLKCSR_IMP);
}

int32 todr_rd (RUN_DECL)
{
    if (cpu_unit->is_primary_cpu())
        return todr_reg;
    else
        return weak_read(primary_todr_reg);
}

int32 rxcs_rd (RUN_DECL)
{
    return (tti_csr & TTICSR_IMP);
}

int32 rxdb_rd (RUN_DECL)
{
    /* only the primary processor receives console input */
    if (cpu_unit->is_primary_cpu())
    {
        int32 t = tti_unit.buf;                                 /* char + error */

        tti_csr = tti_csr & ~CSR_DONE;                          /* clr done */
        tti_unit.buf = tti_unit.buf & 0377;                     /* clr errors */
        CLR_INT (TTI);
        return t;
    }
    else
    {
        /* non-primary CPU tries to read RXDB: not expected */
        tti_csr = tti_csr & ~CSR_DONE;                          /* clr done */
        CLR_INT (TTI);
        return CSR_ERR;
    }
}

int32 txcs_rd (RUN_DECL)
{
    return (tto_csr & TTOCSR_IMP);
}

void iccs_wr (RUN_DECL, int32 data)
{
    if ((data & CSR_IE) == 0)
    {
        CLR_INT (CLK);
        syncw_reeval_sys(RUN_PASS);
    }
    clk_csr = (clk_csr & ~CLKCSR_RW) | (data & CLKCSR_RW);
}

void todr_wr (RUN_DECL, int32 data)
{
    if (cpu_unit->is_primary_cpu())
    {
        todr_reg = data;
        if (data)
            todr_blow = 0;
    }
}

void rxcs_wr (RUN_DECL, int32 data)
{
    if ((data & CSR_IE) == 0)
        CLR_INT (TTI);
    else if ((tti_csr & (CSR_DONE + CSR_IE)) == CSR_DONE)
        SET_INT (TTI);
    tti_csr = (tti_csr & ~TTICSR_RW) | (data & TTICSR_RW);
}

void txcs_wr (RUN_DECL, int32 data)
{
    if ((data & CSR_IE) == 0)
        CLR_INT (TTO);
    else if ((tto_csr & (CSR_DONE + CSR_IE)) == CSR_DONE)
        SET_INT (TTO);
    tto_csr = (tto_csr & ~TTOCSR_RW) | (data & TTOCSR_RW);
}

void txdb_wr (RUN_DECL, int32 data)
{
    tto_buf = data & 0377;
    tto_csr = tto_csr & ~CSR_DONE;
    CLR_INT (TTO);
    sim_activate (&tto_unit, tto_unit.wait);
}

/* Terminal input routines

   tti_svc      process event (character ready)
   tti_reset    process reset
*/

t_stat tti_svc (RUN_SVC_DECL, UNIT *uptr)
{
    // RUN_SVC_CHECK_CANCELLED(uptr);  // not required for per-CPU devices
    int32 c;

    /* Since console input is available only for the primary CPU,
       this routine is never executed by secondaries */

    /* continue poll */
    if (use_clock_thread)
    {
        sim_activate_clk_cosched (uptr, 1);
    }
    else
    {
        int32 x_tmr_poll = weak_read_var(tmr_poll);
        sim_activate (uptr, KBD_WAIT (uptr->wait, x_tmr_poll));
    }

    /* skip if tti_unit.buf is not empty and still has not been consumed */
    if (tti_csr & CSR_DONE)
        return SCPE_OK;

    AUTO_LOCK_NM(ta_autolock, tti_typeahead_lock);
    if (! tti_typeahead.get(& c))
    {
        ta_autolock.unlock();
        c = sim_poll_kbd (FALSE);                             /* poll Telnet connection if any */
    }
    else
    {
        ta_autolock.unlock();
    }

    if (c  < SCPE_KFLAG)                                      /* no char or error? */
        return c;
    if (c & SCPE_BREAK)                                       /* break? */
    {
        if (sysd_hlt_enb() && smp_hlt_enb(& hlt_pin))         /* if enabled, halt */
            hlt_pin = 1;
        tti_unit.buf = TTIBUF_ERR | TTIBUF_FRM | TTIBUF_RBR;
    }
    else
    {
        tti_unit.buf = sim_tt_inpcvt (c, TT_GET_MODE (uptr->flags));
    }
    uptr->pos = uptr->pos + 1;
    tti_csr = tti_csr | CSR_DONE;
    if (tti_csr & CSR_IE)
        SET_INT (TTI);
    return SCPE_OK;
}

t_stat tti_reset (DEVICE *dptr)
{
    RUN_SCOPE;
    tti_csr = 0;
    CLR_INT (TTI);
    /* console input is available only for the primary CPU */
    if (cpu_unit->is_primary_cpu())
    {
        tti_unit.buf = 0;

        if (use_clock_thread)
        {
            sim_activate_clk_cosched_abs (&tti_unit, 1);
        }
        else
        {
            int32 x_tmr_poll = weak_read_var(tmr_poll);
            sim_activate_abs (&tti_unit, KBD_WAIT (tti_unit.wait, x_tmr_poll));
        }
    }
    return SCPE_OK;
}

t_bool tti_rcv_char(int32 c)
{
    AUTO_LOCK(tti_typeahead_lock);

    if ((c & SCPE_BREAK) && sysd_hlt_enb())
    {
        if (! smp_hlt_enb())
            return FALSE;
        tti_typeahead.clear();
    }

    if (tti_typeahead.put(c))
    {
        wakeup_cpu(& cpu_unit_0);
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

void tti_clear_pending_typeahead()
{
    AUTO_LOCK(tti_typeahead_lock);
    tti_typeahead.clear();
}

/* Terminal output routines

   tto_svc      process event (character typed)
   tto_reset    process reset
*/

t_stat tto_svc (RUN_SVC_DECL, UNIT *uptr)
{
    // RUN_SVC_CHECK_CANCELLED(uptr);  // not required for per-CPU devices
    int32 c;
    t_stat r;

    c = sim_tt_outcvt (tto_buf, TT_GET_MODE (uptr->flags));
    if (c >= 0)
    {
        if ((r = sim_putchar_s (c)) != SCPE_OK)             /* output; error? */
        {
            sim_activate (uptr, uptr->wait);                /* retry */
            return (r == SCPE_STALL) ? SCPE_OK : r;         /* !stall? report */
        }
    }
    tto_csr = tto_csr | CSR_DONE;
    if (tto_csr & CSR_IE)
        SET_INT (TTO);
    // uptr->pos = uptr->pos + 1;                           /* non-essential counter, not worth locking */
    return SCPE_OK;
}

t_stat tto_reset (DEVICE *dptr)
{
    RUN_SCOPE;
    tto_buf = 0;
    tto_csr = CSR_DONE;
    CLR_INT (TTO);
    sim_cancel (&tto_unit);                                 /* deactivate unit */
    return SCPE_OK;
}

/* Clock routines

   clk_svc      process event (clock tick)
   clk_reset    process reset
   todr_powerup powerup for TODR (get date from system)
*/

t_stat clk_svc (RUN_SVC_DECL, UNIT *uptr)
{
    // RUN_SVC_CHECK_CANCELLED(uptr);  // not required for per-CPU devices
    return clk_svc_ex(RUN_PASS, TRUE);
}

/*
 * clk_svc_ex is invoked:
 *
 *   * if SYNCLK clock is enabled (use_clock_thread is TRUE),
 *     then by read_irqs_to_local when SYNCLK interrupt is received
 *
 *   * if SYNCLK clock is not enabled (use_clock_thread is FALSE),
 *     then by clk_svc on expiration of clock tick interval
 *
 */
t_stat clk_svc_ex (RUN_DECL, t_bool clk_ie)
{
    int32 t;

    if ((clk_csr & CSR_IE) && clk_ie)
    {
        SET_INT (CLK);
    }
    else if (!use_clock_thread && cpu_unit->cpu_thread_priority == SIMH_THREAD_PRIORITY_CPU_CRITICAL_VM)
    {
        /* 
         * When clock strobe thread is not used, sim_idle leaves thread priority elevated in case clock event
         * is about to happen, so the event may get processed promptly. If clock interrupt processing is
         * enabled via CSR_IE, priority will be dropped during interrupt delivery when it performs
         * cpu_reevaluate_thread_priority. If CLK interrupt signalling is disabled however by the CSR,
         * drop priority here.
         */
        cpu_reevaluate_thread_priority(RUN_PASS);
    }

    t = sim_rtcn_calb (RUN_PASS, clk_tps, TMR_CLK);         /* calibrate clock */
    sim_activate (&clk_unit, t);                            /* reactivate unit */

    if (cpu_unit->is_primary_cpu())
    {
        if (!todr_blow)                                     /* incr TODR */
            todr_reg = todr_reg + 1;
    }

    return SCPE_OK;
}

/* TODR resync routine */

t_stat todr_resync (RUN_DECL)
{
    uint32 base;
    time_t curr;
    struct tm *ctm;

    curr = time (NULL);                                     /* get curr time */
    if (curr == (time_t) -1)                                /* error? */
        return SCPE_NOFNC;
    ctm = localtime (&curr);                                /* decompose */
    if (ctm == NULL)                                        /* error? */
        return SCPE_NOFNC;
    base = (((((ctm->tm_yday * 24) +                        /* sec since 1-Jan */
            ctm->tm_hour) * 60) +
            ctm->tm_min) * 60) +
            ctm->tm_sec;
    todr_reg = (base * 100) + 0x10000000;                   /* cvt to VAX form */
    todr_blow = 0;
    return SCPE_OK;
}

/* Reset routine */

t_stat clk_reset (DEVICE *dptr)
{
    RUN_SCOPE;
    int32 t;

    if (cpu_unit->is_primary_cpu())
        todr_resync (RUN_PASS);                             /* resync clock */

    clk_csr = 0;
    CLR_INT (CLK);
    syncw_reeval_sys(RUN_PASS);

    t = sim_rtcn_init (RUN_PASS, clk_unit.wait, TMR_CLK);   /* init timer */
    sim_activate_abs (&clk_unit, t);                        /* activate unit */

    return SCPE_OK;
}

/*
 * Estimate of cycles till next expected SYNCLK event
 */
int32 synclk_expected_next(RUN_DECL)
{
    uint32 period = (uint32) weak_read(atomic_var(tmr_poll));
    uint32 delta = CPU_CURRENT_CYCLES - cpu_unit->cpu_last_synclk_cycles;
    if (delta >= period)
    {
        period = 1;
    }
    else
    {
        period -= delta;
        const uint32 max_period = INT32_MAX / 4;  /* avoid overflows */
        if (period >= max_period)
            period = max_period;
    }

    return (int32) period;
}