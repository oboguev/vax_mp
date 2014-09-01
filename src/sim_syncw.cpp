#include "sim_defs.h"
#include "vax_defs.h"

/*
 * This file implements optional interprocessor synchronization window. For detailed description of what and why
 * is performed here refer to "VAX MP Technical Overview", chapters "Interprocessor synchronization window" and
 * "Implementation of VAX interlocked instructions".
 *
 * ToDo: Implement performance improvements suggested in "VAX MP Technical Overview", chapter "Interprocessor
 *       synchronization window".
 */

/*
 * Minimum number of quants (slices) to divide synchronization window into. During execution of VM (VAX) instruction 
 * stream, VCPU calls syncw_checkinterval() every (window size / SYNCW_QUANTS) VM instructon cycles or more often.
 */
#define SYNCW_QUANTS 10

/*
 * Maximum size of syncw scale quant, in VM instruction cycles
 */
#define MAX_QUANT_SIZE (1000 * 1000)

/*
 * Maximum delay for inter-VCPU memory updates propagation, in VM instruction cycles.
 * See the explanation below.
 *
 * As a rough estimate, at VM performance of 20-40 virtual MIPS it translates to 5-10 usec,
 * which should be ample for updates propagation through cache coherency protocol.
 *
 * Alternatively, instead of guestimating this value, we could have raised and cleared
 * SYNCLK, CLK and IPINTR under the protection of a lock, but it is desirable to avoid
 * the overhead and serialization this would have introduced.
 */
#define INTERCPU_DELAY 200

/*
 * Default base value for syncw.cpu[].pos, when VCPU initially enters itself into SYNCW.
 *
 * As explained below, VCPUs may be entered in SYNCW by other VCPUs at position
 *      (entering_VCPU.pos - INTERCPU_DELAY),
 * so we need to reserve space at the start of the scale for enough INTERCPU_DELAY decrements.
 */
#define SYNCW_BASE_POS (20 * SIM_MAX_CPUS * INTERCPU_DELAY)

syncw_data syncw;
smp_interlocked_uint32_var syncw_sys_active[SIM_MAX_CPUS];
uint32 interrupt_reeval_syncw_sys[IPL_HLVL];

#define SYNCW_SYS_ILK (SYNCW_SYS | SYNCW_ILK)

#define syncw_wakeup_waitset(ix) do { if (unlikely(syncw.cpu[ix].waitset.is_any_set())) syncw_wakeup(& syncw.cpu[ix].waitset, SYNCW_NOLOCK); } while (0)
#define syncw_wakeup_wakeset()   do { if (unlikely(wakeset.is_any_set())) syncw_wakeup(& wakeset, SYNCW_NOLOCK); } while (0)

#define syncw_process_external_syncw_sys(cx)                                                                                           \
    do {                                                                                                                               \
        if (unlikely((syncw.cpu[cx].active ^ cpu_unit->syncw_active) & SYNCW_SYS) && (syncw.cpu[cx].active & SYNCW_SYS))               \
            syncw_accept_external_syncw_sys(RUN_PASS);                                                                                 \
    } while (0)

static void syncw_reinit(t_bool reset);
static void syncw_wakeup(cpu_set* waitset, uint32 flags);
static void syncw_accept_external_syncw_sys(RUN_DECL);
static void syncw_adjust_pos_all(uint32 delta);
static int strwidth(char* fmt, ...);
static uint32 higher_or_equal_multiple(uint32 q, uint32 d);

/*
 * Called once at simulator startup
 */
void syncw_init()
{
    syncw_reinit(FALSE);

    /* 
     * When fetching these interrupts, VCPU must enter SYNCW_SYS.
     * If making changes to this code (i.e. value of interrupt_reeval_syncw_sys mask),
     * be sure to also review InterruptRegister::query_syncw_sys().
     */
    memzero(interrupt_reeval_syncw_sys);
    interrupt_reeval_syncw_sys[IPL_SYNCLK] |= (1 << INT_V_SYNCLK);
    interrupt_reeval_syncw_sys[IPL_CLK] |= (1 << INT_V_CLK);
    interrupt_reeval_syncw_sys[IPL_IPINTR] |= (1 << INT_V_IPINTR);
}


/*
 * Called when primary CPU is reset, such as at "BOOT CPU" command
 */
void syncw_reset()
{
    syncw_reinit(TRUE);
}


/*
 * Common code for syncw_init and syncw_reset
 */
static void syncw_reinit(t_bool reset)
{
    uint32 ix;

    if (cpu_database_lock)
        cpu_database_lock->lock();

    syncw.on = 0;
    syncw.vsmp_winsize_sys = syncw.winsize_sys = 0;
    syncw.vsmp_winsize_ilk = syncw.winsize_ilk = 0;
    syncw.ipl_syslock = 6;
    syncw.ipl_resched = 3;
    syncw.checkinterval_sys = 0;
    syncw.checkinterval_ilk = 0;
    syncw.checkinterval_none = 2000000;

    for (ix = 0;  ix < SIM_MAX_CPUS;  ix++)
    {
        syncw.cpu[ix].active = 0;
        syncw.cpu[ix].pos = SYNCW_BASE_POS;
        syncw.cpu[ix].waitset.clear_all();
        smp_check_aligned(& syncw_sys_active[ix]);
        if (reset)
        {
            /* primary VCPU reset, such as during BOOT CPU command */
            syncw_sys_active_do_clear(ix);
        }
        else
        {
            /* simulator startup */
            smp_var(syncw_sys_active[ix]) = 0;
        }
    }

    for (ix = 0;  ix < sim_ncpus;  ix++)
    {
        CPU_UNIT* xcpu = cpu_units[ix];
        xcpu->syncw_active = 0;
        xcpu->syncw_countdown_start = xcpu->syncw_countdown = syncw.checkinterval_none;
        xcpu->syncw_countdown_sys = 0;
        xcpu->syncw_countdown_ilk = 0;
        if (reset)  xcpu->syncw_wait_event->set();
        xcpu->syncw_wait_cpu_id = NO_CPU_ID;
    }

    if (cpu_database_lock)
        cpu_database_lock->unlock();
}


/*
 * Called when VSMP gets loaded and initializes for actual use of synchronization window
 */
void syncw_start(RUN_DECL)
{
    /*
     * We need to slash winsize_sys and winsize_ilk in half compared to the values reported by VSMP and
     * based on maximum number of wait-cycles allowed. Here is why. Consider syncw scale that limits maximum
     * relative "drift" between the processors to SW. Suppose there are two processors A and B active
     * on the scale. B initially is near the top of the scale and A is near the base of the scale.
     *
     *     top      |      B             ^
     *              |      |             |
     *              |      |             |
     *              |      |        (SW cycles)
     *              |      |             |
     *              |      |             |
     *     base     A      |             v
     *
     * A performs interaction event (such as sending IPI to B or trying to acquire a spinlock held by B)
     * and begins spin-waiting for B. B's thread is preempted and makes no progress. Syncw scale will allow A
     * to make forward progress until it gets "SW" cycles ahead of "B":
     *
     *     top      A      |             ^
     *              |      |             |
     *              |      |             |
     *              |      |        (SW cycles)
     *              |      |             |
     *              |      |             |
     *     base     |      B             v
     *
     * Thus until A is suspended by syncw scale, it will be able to make forward progress by (2 * SW)
     * cycles, which must not exceed winsize value reported by VSMP: 2 * SW <= vsmp_winsize_xxx.
     *
     * Yielding: SW <= vsmp_winsize_xxx / 2.
     */

    if (syncw.on & SYNCW_SYS)
    {
        /* note: VSMP is also responsible to limit requested winsize_sys to below 0x80000000 */
        syncw.winsize_sys = imin(syncw.vsmp_winsize_sys / 2, (uint32) 0x7F000000);
        syncw.checkinterval_sys = syncw.winsize_sys / SYNCW_QUANTS;
        if (syncw.checkinterval_sys == 0)
            syncw.checkinterval_sys = 1;
        if (syncw.checkinterval_sys > MAX_QUANT_SIZE)
            syncw.checkinterval_sys = MAX_QUANT_SIZE;

        /* VSMP is responsible for never allowing such a low value of winsize_sys */
        if (syncw.checkinterval_sys <= INTERCPU_DELAY)
        {
            syncw.checkinterval_sys = INTERCPU_DELAY + 1;
            syncw.winsize_sys = syncw.checkinterval_sys * SYNCW_QUANTS;
            smp_printf("\n*** %s: SYNCW SYS size is too low, safe operation is impossible\n\n", sim_name);
            if (sim_log)
                fprintf(sim_log, "\n*** %s: SYNCW SYS size is too low, safe operation is impossible\n\n", sim_name);
        }
    }
    else
    {
        syncw.checkinterval_sys = 0;
    }

    if (syncw.on & SYNCW_ILK)
    {
        /* note: VSMP is also responsible to limit requested winsize_ilk to below 0x80000000 */
        syncw.winsize_ilk = imin(syncw.vsmp_winsize_ilk / 2, (uint32) 0x7F000000);
        syncw.checkinterval_ilk = syncw.winsize_ilk / SYNCW_QUANTS;
        if (syncw.checkinterval_ilk == 0)
            syncw.checkinterval_ilk = 1;
        if (syncw.checkinterval_ilk > MAX_QUANT_SIZE)
            syncw.checkinterval_ilk = MAX_QUANT_SIZE;
    }
    else
    {
        syncw.checkinterval_ilk = 0;
    }

    if ((syncw.on & SYNCW_SYS_ILK) == SYNCW_SYS_ILK)
    {
        if (syncw.winsize_ilk < syncw.winsize_sys)
        {
            syncw.winsize_sys = syncw.winsize_ilk;
            syncw.checkinterval_sys = syncw.checkinterval_ilk;
        }
        else if (syncw.winsize_ilk > syncw.winsize_sys)
        {
            syncw.winsize_ilk = syncw.winsize_sys;
            syncw.checkinterval_ilk = syncw.checkinterval_sys;
        }
    }

    uint32 winsize;

    switch (syncw.on & SYNCW_SYS_ILK)
    {
    case SYNCW_SYS_ILK:
        winsize = imin(syncw.winsize_sys, syncw.winsize_ilk);
        syncw.quant = imin(syncw.checkinterval_sys, syncw.checkinterval_ilk);  
        break;
    case SYNCW_SYS:
        winsize = syncw.winsize_sys;
        syncw.quant = syncw.checkinterval_sys;  
        break;
    case SYNCW_ILK:
        winsize = syncw.winsize_ilk;
        syncw.quant = syncw.checkinterval_ilk;  
        break;
    case 0:
    default:
        winsize = 0;
        syncw.quant = 0;  
        break;
    }

    /*
     * "maxdrift" is the maximum number of cycles VCPUs are allowed to drift apart,
     * as checked in syncw_checkinterval(). "maxdrift" is calculated by shaving off
     * the followng value off winsize:
     *
     *     quant + higher_or_equal_multiple(quant, (uint32) INTERCPU_DELAY) + 4
     *
     * First part ("quant") is because VCPU won't have a chance to re-check driftng conditions
     * for another "quant" cycles.
     *
     * Second part (imax...) is the maximum number of cycles that may pass between another VCPU
     * is sent CLK/SYNCLK/IPINTR interrupt and time the condition is noticed by this VCPU and that
     * other VCPU is entered by this VCPU in synchronization window. All this time this VCPU may
     * keep running, and thus pre-accumulate drift even before the condition is noticed. 
     * Second member in the sum reflects the maximum amount of this possible pre-accumulated drift
     * before another VCPU is forced by this (or other VCPU) into SYNCW_SYS.
     * It is the first multiple of "quant" higher or equal than INTERCPU_DELAY.
     * In practice it normally be just one "quant".
     *
     * Final part (4) is a safeguard to avoid rounding problems and effects of ">=" vs ">" and
     * also because in some busy-wait loop structures VCPU may signal entrance into SYNCW an
     * instruction or two later after actual busy-wait loop starts.
     *
     */
    if (syncw.on & SYNCW_SYS_ILK)
    {
        uint32 reserve = syncw.quant + higher_or_equal_multiple(syncw.quant, (uint32) INTERCPU_DELAY) + 4;
        syncw.maxdrift = winsize - reserve;
    }
    else
    {
        syncw.maxdrift = 0;
    }

    syncw.seq = 0;
}

/* calculate first multiple of "q" >= "d", but not lesser than "q" */
static uint32 higher_or_equal_multiple(uint32 q, uint32 d)
{
    if (d == 0)
        return q;

    for (uint32 v = q * (d / q);  ; v += q)
    {
        if (v >= d)
            return v;
    }
}

/*
 * Called when secondary is about to start.
 * Note that current thread may be another VCPU's thread.
 * "Flags" argument may contain SYNCW_NOLOCK.
 */
void syncw_reinit_cpu(RUN_DECL, uint32 flags)
{
    uint32 cx = cpu_unit->cpu_id;

    if (! (flags & SYNCW_NOLOCK))
        cpu_database_lock->lock();

    cpu_unit->syncw_active = 0;
    cpu_unit->syncw_countdown_start = cpu_unit->syncw_countdown = syncw.checkinterval_none;
    cpu_unit->syncw_wait_cpu_id = NO_CPU_ID;
    syncw.cpu[cx].active = 0;
    syncw_sys_active_do_clear(cx);
    syncw_wakeup_waitset(cx);

    if (! (flags & SYNCW_NOLOCK))
        cpu_database_lock->unlock();
}


/* 
 * Calculate position when entering syncw.
 * Calculate new position to be at the bottom of the range i.e. min(all other VCPUs in syncw),
 * excluding "cx" from consideration. Note: "cx" can also be NO_CPU_ID.
 * If no other VCPUs in syncw, default to SYNCW_BASE_POS.
 */
SIM_INLINE static uint32 syncw_entry_pos(uint32 cx)
{
    t_bool found = FALSE;
    uint32 pos = SYNCW_BASE_POS;

    for (uint32 ix = 0;  ix < sim_ncpus;  ix++)
    {
        if (syncw.cpu[ix].active & SYNCW_SYS_ILK)
        {
            if (ix == cx)  continue;
            if (cpu_running_set.is_clear(ix))  continue;
            if (syncw.cpu[ix].active & SYNCW_NOSYNC)  continue;

            if (found)
            {
                if (syncw.cpu[ix].pos < pos)
                    pos = syncw.cpu[ix].pos;
            }
            else
            {
                pos = syncw.cpu[ix].pos;
                found = TRUE;
            }
        }
    }

    return pos;
}


/*
 * Enter SYS synchronization window.
 * Actual routne for syncw_enter_sys.
 * Called by syncw_enter_sys macro after checking for preliminary conditions.
 */
void syncw_doenter_sys(RUN_DECL)
{
    uint32 cx = cpu_unit->cpu_id;
    uint32 old_active;

    cpu_database_lock->lock();

    /*
     * We could check other VCPUs here for SYNCLK, CLK or IPINTR pendng and enter them into SYS window
     * if they were not there yet. However this would create an extra overhead, and the assumption is 
     * that most stays in synchronization window are shorter than the duration of check interval.
     * Therefore we check other VCPUs only in syncw_checkinterval, but not in syncw_doenter_xxx.
     * Effectively we trade off a quant of syncw scale space in favor of better performance.
     */ 

    /* process possible setting of our SYNCW_SYS by other VCPU */
    syncw_process_external_syncw_sys(cx);

    if (unlikely(syncw.cpu[cx].active & SYNCW_NOSYNC))
        goto exit;

    if (unlikely(syncw.cpu[cx].active & SYNCW_SYS))
        goto exit;

    if (syncw.cpu[cx].active & SYNCW_ILK)
    {
        // keep current syncw.cpu[cx].pos
    }
    else
    {
        syncw.cpu[cx].pos = syncw_entry_pos(cx);
    }

    old_active = syncw.cpu[cx].active & SYNCW_SYS_ILK;
    syncw.cpu[cx].active |= SYNCW_SYS;
    syncw_sys_active_do_set(cx);
    cpu_database_lock->unlock();

    cpu_unit->syncw_active = old_active | SYNCW_SYS;
    cpu_unit->syncw_countdown_sys = syncw.checkinterval_sys;
    if (old_active & SYNCW_ILK)
    {
        cpu_unit->syncw_countdown_ilk -= cpu_unit->syncw_countdown_start - cpu_unit->syncw_countdown;
        cpu_unit->syncw_countdown = imin(cpu_unit->syncw_countdown_sys, cpu_unit->syncw_countdown_ilk);
    }
    else
    {
        cpu_unit->syncw_countdown = cpu_unit->syncw_countdown_sys;
    }
    cpu_unit->syncw_countdown_start = cpu_unit->syncw_countdown;

    return;

exit:

    cpu_unit->syncw_active = syncw.cpu[cx].active & SYNCW_SYS_ILK;

    cpu_database_lock->unlock();
}


/*
 * Leave SYS synchronization window.
 * Actual routne for syncw_leave_sys.
 * Called by syncw_leave_sys macro after checking for preliminary conditions.
 */
void syncw_doleave_sys(RUN_DECL)
{
    uint32 cx = cpu_unit->cpu_id;
    cpu_set wakeset;

    cpu_database_lock->lock();

    /* process possible setting of our SYNCW_SYS by other VCPU */
    syncw_process_external_syncw_sys(cx);

    t_bool leave_sys = FALSE;

    if (syncw.cpu[cx].active & SYNCW_SYS)
    {
        leave_sys = TRUE;
        if (0 == (syncw.on & SYNCW_SYS) || weak_read(sim_mp_active) == FALSE || (syncw.cpu[cx].active & SYNCW_NOSYNC))
        {
            /* leave regardless of any pending interrupts */
        }
        else if (cpu_unit->cpu_synclk_pending == SynclkPendingIE1 || cpu_unit->cpu_intreg.query_syncw_sys())
        {
            /* stay in SYNCW_SYS */
            leave_sys = FALSE;
        }

        if (leave_sys)
        {
            syncw.cpu[cx].active &= ~SYNCW_SYS;
            syncw_sys_active_do_clear(cx);

            wakeset = syncw.cpu[cx].waitset;
            syncw.cpu[cx].waitset.clear_all();
        }
    }

    cpu_unit->syncw_active = syncw.cpu[cx].active & SYNCW_SYS_ILK;

    cpu_database_lock->unlock();

    if (leave_sys)
    {
        syncw_wakeup_wakeset();

        if (cpu_unit->syncw_active & SYNCW_ILK)
        {
            cpu_unit->syncw_countdown_ilk -= cpu_unit->syncw_countdown_start - cpu_unit->syncw_countdown;
            cpu_unit->syncw_countdown_start = cpu_unit->syncw_countdown = cpu_unit->syncw_countdown_ilk;
        }
        else
        {
            cpu_unit->syncw_countdown_start = cpu_unit->syncw_countdown = syncw.checkinterval_none;
        }
    }
}


/*
 * Enter ILK synchronization window.
 * Actual routne for syncw_enter_ilk.
 * Called by syncw_enter_ilk macro after checking for preliminary conditions.
 */
t_bool syncw_doenter_ilk(RUN_DECL)
{
    uint32 cx = cpu_unit->cpu_id;
    uint32 old_active;

    cpu_database_lock->lock();

    /* process possible setting of our SYNCW_SYS by other VCPU */
    syncw_process_external_syncw_sys(cx);

    if (unlikely(syncw.cpu[cx].active & SYNCW_NOSYNC))
        goto exit;

    if (syncw.cpu[cx].active & SYNCW_SYS)
    {
        // keep current syncw.cpu[cx].pos
    }
    else
    {
        syncw.cpu[cx].pos = syncw_entry_pos(cx);
    }
    old_active = syncw.cpu[cx].active & SYNCW_SYS_ILK;
    syncw.cpu[cx].active |= SYNCW_ILK;
    cpu_database_lock->unlock();

    cpu_unit->syncw_active = old_active | SYNCW_ILK;
    cpu_unit->syncw_countdown_ilk = syncw.checkinterval_ilk;
    if (old_active & SYNCW_SYS)
    {
        cpu_unit->syncw_countdown_sys -= cpu_unit->syncw_countdown_start - cpu_unit->syncw_countdown;
        cpu_unit->syncw_countdown = imin(cpu_unit->syncw_countdown_sys, cpu_unit->syncw_countdown_ilk);
    }
    else
    {
        cpu_unit->syncw_countdown = cpu_unit->syncw_countdown_ilk;
    }
    cpu_unit->syncw_countdown_start = cpu_unit->syncw_countdown;

    return TRUE;

exit:

    cpu_unit->syncw_active = syncw.cpu[cx].active & SYNCW_SYS_ILK;

    cpu_database_lock->unlock();

    return FALSE;
}


/*
 * Leave ILK synchronization window.
 * Actual routne for syncw_leave_ilk.
 * Called by syncw_leave_ilk macro after checking for preliminary conditions.
 */
void syncw_doleave_ilk(RUN_DECL)
{
    uint32 cx = cpu_unit->cpu_id;
    cpu_set wakeset;

    cpu_database_lock->lock();

    /* process possible setting of our SYNCW_SYS by other VCPU */
    syncw_process_external_syncw_sys(cx);

    syncw.cpu[cx].active &= ~SYNCW_ILK;
    cpu_unit->syncw_active = syncw.cpu[cx].active & SYNCW_SYS_ILK;
    wakeset = syncw.cpu[cx].waitset;
    syncw.cpu[cx].waitset.clear_all();
    cpu_database_lock->unlock();

    syncw_wakeup_wakeset();

    if (cpu_unit->syncw_active & SYNCW_SYS)
    {
        cpu_unit->syncw_countdown_sys -= cpu_unit->syncw_countdown_start - cpu_unit->syncw_countdown;
        cpu_unit->syncw_countdown_start = cpu_unit->syncw_countdown = cpu_unit->syncw_countdown_sys;
    }
    else
    {
        cpu_unit->syncw_countdown_start = cpu_unit->syncw_countdown = syncw.checkinterval_none;
    }
}


/*
 * Leave both SYS and ILK synchronization windows.
 * "Flags" argument can contain SYNCW_DISABLE_CPU and SYNCW_ENABLE_CPU.
 *
 * SYNCW_DISABLE_CPU disables use of synchronization window for this CPU, i.e. this CPU will no
 * longer wait for other VCPUs, and other VCPUs will no longer wait for this VCPU.
 *
 * SYNCW_ENABLE_CPU reenables the use of synchronization window for this CPU.
 */
void syncw_leave_all(RUN_DECL, uint32 flags)
{
    uint32 cx = cpu_unit->cpu_id;
    cpu_set wakeset;

    cpu_database_lock->lock();

    /* process possible setting of our SYNCW_SYS by other VCPU */
    syncw_process_external_syncw_sys(cx);

    if (flags & SYNCW_DISABLE_CPU)
        syncw.cpu[cx].active |= SYNCW_NOSYNC;

    if (flags & SYNCW_ENABLE_CPU)
        syncw.cpu[cx].active &= ~SYNCW_NOSYNC;

    uint32 old_active = syncw.cpu[cx].active & SYNCW_SYS_ILK;
    t_bool leave_sys = FALSE;

    if (syncw.cpu[cx].active & SYNCW_SYS)
    {
        leave_sys = TRUE;
        if ((flags & SYNCW_OVERRIDE_ALL) || 0 == (syncw.on & SYNCW_SYS) || weak_read(sim_mp_active) == FALSE || (syncw.cpu[cx].active & SYNCW_NOSYNC))
        {
            /* leave regardless of any pending interrupts */
        }
        else if (cpu_unit->cpu_synclk_pending == SynclkPendingIE1 || cpu_unit->cpu_intreg.query_syncw_sys())
        {
            /* stay in SYNCW_SYS */
            leave_sys = FALSE;
        }

        if (leave_sys)
        {
            syncw.cpu[cx].active &= ~SYNCW_SYS;
            syncw_sys_active_do_clear(cx);
        }
    }

    syncw.cpu[cx].active &= ~SYNCW_ILK;

    cpu_unit->syncw_active = syncw.cpu[cx].active & SYNCW_SYS_ILK;

    if (cpu_unit->syncw_active != old_active || (flags & SYNCW_DISABLE_CPU))
    {
        wakeset = syncw.cpu[cx].waitset;
        syncw.cpu[cx].waitset.clear_all();
    }

    cpu_database_lock->unlock();

    syncw_wakeup_wakeset();

    if (cpu_unit->syncw_active & SYNCW_SYS)
    {
        if (old_active & SYNCW_ILK)
        {
            cpu_unit->syncw_countdown_sys -= cpu_unit->syncw_countdown_start - cpu_unit->syncw_countdown;
            cpu_unit->syncw_countdown_start = cpu_unit->syncw_countdown = cpu_unit->syncw_countdown_sys;
        }
    }
    else
    {
        cpu_unit->syncw_countdown_start = cpu_unit->syncw_countdown = syncw.checkinterval_none;
    }
}


/*
 * Called when this VCPU receives SYNCWSYS interrupt from another VCPU.
 * This interrupt is sent when one VCPU enters another VCPU in SYS window,
 * so target VCPU sets up its local syncw data correspondingly.
 */
void syncw_process_syncwsys_interrupt(RUN_DECL)
{
    uint32 cx = cpu_unit->cpu_id;
    cpu_database_lock->lock();
    syncw_process_external_syncw_sys(cx);
    cpu_database_lock->unlock();
}


/*
 * Other VCPU has entered this VCPU into SYNCW_SYS by marking it as (syncw.cpu[].active & SYNCW_SYS),
 * setting syncw.cpu[].pos for it and sending SYNCWSYS interrupt. Once this VCPU notices this event,
 * it should make appropriate changes in its local syncw-related structures as well, including
 * cpu_unit->countdown_xxx fields and cpu_unit->syncw_active.
 *
 * This routine is called under the protection of cpu_database_lock and only when SYNCW_SYS is set
 * in syncw.cpu[].active but not set in cpu_unit->syncw_active.
 *
 * It is unlikely, but marginally possible that signalling VCPU entered this VCPU into SYNCW_SYS
 * improperly, because it saw one of SYNCLK, CLK or IPINTR interrupts pending in its stale memory
 * view, after they already had been already cleared. Therefore recheck the condition for the
 * interrupts and if it does not hold true anymore then leave SYNCW_SYS.
 */
static void syncw_accept_external_syncw_sys(RUN_DECL)
{
    uint32 cx = cpu_unit->cpu_id;
    t_bool noenter = FALSE;

    cpu_unit->cpu_intreg.dismiss_int(RUN_PASS, IPL_ABS_SYNCWSYS, INT_V_SYNCWSYS);

    if (unlikely(syncw.cpu[cx].active & SYNCW_NOSYNC))
        noenter = TRUE;

    if (unlikely((syncw.on & SYNCW_SYS) == 0))
        noenter = TRUE;

    if (PSL_GETIPL(PSL) >= syncw.ipl_syslock || 
        cpu_unit->cpu_synclk_pending == SynclkPendingIE1 || 
        cpu_unit->cpu_intreg.query_syncw_sys())
    {
        /* must proceeed with entering into SYNCW_SYS */
    }
    else
    {
        noenter = TRUE;
    }

    if (unlikely(noenter))
    {
        syncw.cpu[cx].active &= ~SYNCW_SYS; 
        syncw_sys_active_do_clear(cx);
        syncw_wakeup_waitset(cx);
        return;
    }

    /* 
     * INTERCPU_DELAY is a presumed maximum time for memory updates propagation in MP system,
     * i.e. maximum time before one processor notices IPI, CLK or SYNCLK sent to another VCPU
     * by any other VCPU or clock thread
     */
    cpu_unit->syncw_countdown_sys = syncw.checkinterval_sys - INTERCPU_DELAY;
    cpu_unit->syncw_active |= SYNCW_SYS;
    if (cpu_unit->syncw_active & SYNCW_ILK)
    {
        cpu_unit->syncw_countdown_ilk -= cpu_unit->syncw_countdown_start - cpu_unit->syncw_countdown;
        cpu_unit->syncw_countdown = imin(cpu_unit->syncw_countdown_sys, cpu_unit->syncw_countdown_ilk);
    }
    else
    {
        cpu_unit->syncw_countdown = cpu_unit->syncw_countdown_sys;
    }
    cpu_unit->syncw_countdown_start = cpu_unit->syncw_countdown;
}


/*
 * Reenable the use of synchronization window for this CPU
 */
void syncw_enable_cpu(RUN_DECL, uint32 flags)
{
    if (flags & SYNCW_NOLOCK)
    {
        syncw.cpu[cpu_unit->cpu_id].active &= ~SYNCW_NOSYNC;
    }
    else
    {
        cpu_database_lock->lock();
        syncw.cpu[cpu_unit->cpu_id].active &= ~SYNCW_NOSYNC;
        cpu_database_lock->unlock();
    }
}


/*
 * Reevaluate appropriate SYNCW_SYS state ("in" or "out") for current VCPU
 * and either enter SYNCW_SYS or leave SYNCW_SYS window.
 */
void syncw_reeval_sys(RUN_DECL)
{
    if (PSL_GETIPL(PSL) >= syncw.ipl_syslock || 
        cpu_unit->cpu_synclk_pending == SynclkPendingIE1 || 
        cpu_unit->cpu_intreg.query_syncw_sys())
    {
        syncw_enter_sys(RUN_PASS);
    }
    else
    {
        syncw_leave_sys(RUN_PASS);
    }
}


/*
 * Wake up all VCPUs out of synchronization window waiting.
 * Called when primary performs emergency shutdown of secondaries.
 *
 * "Flags" may include SYNCW_DISABLE_CPU to disable further use of syncw for all VCPUs,
 * so all VCPUs may resume unconstrained by syncw and make their best attempt to shut down.
 */
void syncw_wakeup_all_cpus(uint32 flags)
{
    cpu_database_lock->lock();
    for (uint32 ix = 0;  ix < sim_ncpus;  ix++)
    {
        if (cpu_running_set.is_set(ix))
        {
            if (flags & SYNCW_DISABLE_CPU)
                syncw.cpu[ix].active |= SYNCW_NOSYNC;
            syncw.cpu[ix].waitset.clear_all();
            cpu_units[ix]->syncw_wait_event->set();
        }
    }
    cpu_database_lock->unlock();
}



/*
 * Wake up all VCPUs in the waitset.
 * Clear waitset.
 * "Flags" can include SYNCW_NOLOCK.
 */
static void syncw_wakeup(cpu_set* waitset, uint32 flags)
{
    if (! (flags & SYNCW_NOLOCK))
        cpu_database_lock->lock();

    for (uint32 ix = 0;  ix < sim_ncpus;  ix++)
    {
        if (waitset->is_set(ix))
        {
            waitset->clear(ix);
            cpu_units[ix]->syncw_wait_event->set();
        }
    }

    if (! (flags & SYNCW_NOLOCK))
        cpu_database_lock->unlock();
}


/*
 * VCPUs are about to start or resume execution as a result of console commands
 * BOOT, CONTINUE, STEP, RUN or GO. Initialize wait data or reset wait data left
 * by prior execution.
 */
void syncw_resuming()
{
    for (uint32 ix = 0;  ix < sim_ncpus;  ix++)
    {
        CPU_UNIT* xcpu = cpu_units[ix];
        syncw.cpu[ix].waitset.clear_all();
        xcpu->syncw_wait_cpu_id = NO_CPU_ID;
    }
}


/*
 * If "resuming" is FALSE: called by main instruction loop when syncw_countdown reaches zero.
 *
 * If "resuming" is TRUE: called when VCPU thread is resumed after suspension to console,
 * before entering main instruction loop. In this case countdowns should not be recalculated
 * and syncw.cpu[].pos should not be advanced.
 */
t_stat syncw_checkinterval(RUN_DECL, t_bool resuming)
{
    uint32 cx = cpu_unit->cpu_id;
    uint32 ix;
    uint32 delta;

    if (0 == (SYNCW_SYS_ILK & weak_read(syncw.cpu[cx].active)))
    {
        cpu_unit->syncw_countdown_start = cpu_unit->syncw_countdown = syncw.checkinterval_none;
        return SCPE_OK;
    }

    cpu_set wakeset;

    cpu_database_lock->lock();
    syncw.seq++;

    /* 
     * set TRUE to check if other VCPUs may need to be entered in SYS window;
     * will be set TRUE if SYS quant had expired or if resuming VCPU from console suspension
     */
    t_bool syscheck_other_vcpus = FALSE;

    if (resuming)
    {
        delta = 0;
        if (syncw.on & SYNCW_SYS)
            syscheck_other_vcpus = TRUE;
    }
    else
    {
        /* record the number of cycles this VCPU advanced forward through the synchronization window */
        delta = (cpu_unit->syncw_active & SYNCW_SYS_ILK) ? cpu_unit->syncw_countdown_start - cpu_unit->syncw_countdown
                                                         : 0;

        /* refill countdown */
        switch (cpu_unit->syncw_active & SYNCW_SYS_ILK)
        {
        case SYNCW_SYS_ILK:
            if ((cpu_unit->syncw_countdown_sys -= delta) == 0)
            {
                cpu_unit->syncw_countdown_sys = syncw.checkinterval_sys;
                syscheck_other_vcpus = TRUE;
            }
            if ((cpu_unit->syncw_countdown_ilk -= delta) == 0)
                cpu_unit->syncw_countdown_ilk = syncw.checkinterval_ilk;
            cpu_unit->syncw_countdown_start = cpu_unit->syncw_countdown = imin(cpu_unit->syncw_countdown_sys, cpu_unit->syncw_countdown_ilk);
            break;
        case SYNCW_SYS:
            cpu_unit->syncw_countdown_start = cpu_unit->syncw_countdown = cpu_unit->syncw_countdown_sys = syncw.checkinterval_sys;
            syscheck_other_vcpus = TRUE;
            break;
        case SYNCW_ILK:
            cpu_unit->syncw_countdown_start = cpu_unit->syncw_countdown = cpu_unit->syncw_countdown_ilk = syncw.checkinterval_ilk;
            break;
        case 0:
        default:
            cpu_unit->syncw_countdown_start = cpu_unit->syncw_countdown = syncw.checkinterval_none;
            break;
        }
    }

    /* process possible setting of our SYNCW_SYS by other VCPU */
    syncw_process_external_syncw_sys(cx);

    /* if any of syncw-relevant interrupts are pending, enter sys window */
    if (cpu_unit->cpu_synclk_pending == SynclkPendingIE1 || cpu_unit->cpu_intreg.query_syncw_sys())
    {
        if (! (syncw.cpu[cx].active & SYNCW_NOSYNC))
            syncw_enter_sys(RUN_PASS);
    }

    /* check if gone out of all windows or if syncw's had been disabled by emergency shutdown */
    if (unlikely(0 == (cpu_unit->syncw_active & syncw.on & SYNCW_SYS_ILK)))
    {
        syncw_leave_all(RUN_PASS, 0);
        if (0 == (cpu_unit->syncw_active & SYNCW_SYS_ILK))
        {
            cpu_database_lock->unlock();
            return SCPE_OK;
        }
    }

    /*
     * syncw_checkinterval performs the check on all other processors to see if any of them have SYNCLK, CLK or IPINTR 
     * interrupts pending, but not in SYNCW_SYS yet. If so, syncw_checkinterval will enter such processors into SYNCW_SYS.
     *
     * Those processors will be entered in SYNCW_SYS at position equal to minimum of positions of any other processor
     * already active either in SYNCW_SYS or SYNCW_ILK. For the purposes of this calculalation, position for current
     * processor (in case it is active in synchronization window) will assumed to be that at previous call to
     * syncw_checkinterval, i.e. the value *before* advancement by this invocation of syncw_checkinterval. The latter
     * is required since SYNCLK/CLK/IPINTR might have been signalled to the processor being entered at any time
     * during current VCPU's check interval, including at it's very beginning.
     *
     * Furthermore, since it takes time for other processor's SYNCLK/CLK/IPINTR to propagate before these signals
     * become visible by this processor, some value to cover propagation delay will be substracted from computed 
     * min(all VCPU pos). Macro INTERCPU_DELAY defines the number of VM (VAX) instruction cycles that is presumed to
     * be safely larger than any inter-cpu memory update propagation delay in any host system. In practice in
     * contemporary low-NUMA systems propagation delay in most cases is under 1000 host CPU cycles and thus will not
     * exceeed few VM cycles at most. We use for INTERCPU_DELAY the value of 200 VM cycles that should amply cover
     * any propagation delay.
     *
     * Thus, when a VCPU1 is entered into SYNCW_SYS by VCPU2 because the latter observed VCPU1 having SYNCLK, CLK
     * or IPINTR set, but VCPU1 not in SYNCW_SYS yet, VCPU2 will enter VCPU1 in SYNCW_SYS at position
     *
     *     VCPU1.pos = min(VCPUx.pos) - INTERCPU_DELAY
     *
     * Where "x" is all VCPUs active either in SYS or ILK windows. If "x" includes current VCPU, its pos
     * used in calculations will be one before advancement to next quant.
     *
     * If VCPU1 was already active in SYNCW_ILK, it will remain at its existing position, since interrupts
     * could not have been signalled much before it entered SYNCW_ILK.
     *
     * This check is performed only on SYS intervals, not ILK intervals.
     * It is also performed on VCPU thread resumption from console assumung SYNCW_SYS is enabled in syncw.on.
     */

    if (syscheck_other_vcpus)
    {
        uint32 pos = 0;
        t_bool pos_valid = FALSE;

        for (ix = 0;  ix < sim_ncpus;  ix++)
        {
            /* skip CPUs that should not be checked */
            if (cpu_running_set.is_clear(ix))  continue;
            if (ix == cx)  continue;
            if (syncw.cpu[ix].active & SYNCW_SYS)  continue;
            if (syncw.cpu[ix].active & SYNCW_NOSYNC)  continue;

            /* check if xcpu should be entered in sys window */
            CPU_UNIT* xcpu = cpu_units[ix];
            if (xcpu->cpu_intreg.query_syncw_sys() && (syncw.on & SYNCW_SYS))
            {
                /* if not in ILK yet, should calculate position for entering */
                if (! (syncw.cpu[ix].active & SYNCW_ILK))
                {
                    /* if multiple CPUs are being entered into sys window, calculate position just once for all of them */
                    if (! pos_valid)
                    {
                        pos = syncw_entry_pos(NO_CPU_ID);
                        /* if target position is too low, move all VCPUs up */
                        if (pos < INTERCPU_DELAY)
                        {
                            syncw_adjust_pos_all(SYNCW_BASE_POS + INTERCPU_DELAY);
                            pos += SYNCW_BASE_POS + INTERCPU_DELAY;
                        }
                        pos -= INTERCPU_DELAY;

                        pos_valid = TRUE;
                    }
                    /* set xcpu's calculated position */
                    syncw.cpu[ix].pos = pos;
                }

                /* mark as entered in sys window */
                syncw.cpu[ix].active |= SYNCW_SYS;
                syncw_sys_active_do_set(ix);

                /* send SYNCWSYS interrupt to xcpu to cause xcpu update its syncw_countdown_xxx */
                interrupt_set_int(xcpu, IPL_SYNCWSYS, INT_V_SYNCWSYS);
            }
        }
    }

    /*
     * Advance position.
     * Note : pos should be advanced by "countdown", not VCPU cycle counters since the latter is also advanced by idle sleep
     */
    syncw.cpu[cx].pos += delta;

    /* wakeup waiters if any */
    if (likely(delta))
    {
        wakeset = syncw.cpu[cx].waitset;
        syncw.cpu[cx].waitset.clear_all();
    }

    /* prevent overflow */
    if (unlikely(syncw.cpu[cx].pos > 0xE0000000))
    {
        /* bump all VCPU's pos down towards SYNCW_BASE_POS */
        uint32 minpos = syncw_entry_pos(NO_CPU_ID);
        syncw_adjust_pos_all(SYNCW_BASE_POS - minpos);
    }

    /*
     * see if we are being blocked from making further forward progress by any VCPU lagging behind,
     * if so, enter wait for it
     */
    for (;;)
    {
        /* do not wait if we have been disabled from synchronization window */
        if (syncw.cpu[cx].active & SYNCW_NOSYNC)
            break;

        t_bool proceed = TRUE;

        for (ix = 0;  ix < sim_ncpus;  ix++)
        {
            /* skip irrelevant VCPUs */
            if (cpu_running_set.is_clear(ix))  continue;
            if (ix == cx)  continue;
            if (! (syncw.cpu[ix].active & SYNCW_SYS_ILK))  continue;
            if (syncw.cpu[ix].active & SYNCW_NOSYNC)  continue;

            if (syncw.cpu[cx].pos > syncw.cpu[ix].pos && syncw.cpu[cx].pos - syncw.cpu[ix].pos > syncw.maxdrift)
            {
                /* enter wait on VCPU ix */
                cpu_unit->syncw_wait_cpu_id = ix;
                syncw.cpu[ix].waitset.set(cx);
                syncw.cpu[cx].seq = syncw.seq;

                /* is console stopping VCPUs? */
                if (unlikely(weak_read(stop_cpus)))
                {
                    syncw.cpu[cx].waitset.op_or(& wakeset);
                    cpu_database_lock->unlock();
                    return SCPE_STOP;
                }

                /* if must enter sleep and STEP mode is on, return condition code (that will print to console) */
                if (unlikely(cpu_unit->sim_step))
                {
                    syncw.cpu[cx].waitset.op_or(& wakeset);
                    cpu_database_lock->unlock();
                    return SCPE_SWSTP;
                }

                /* sleep */
                cpu_unit->syncw_wait_event->clear();
                cpu_database_lock->unlock();
                syncw_wakeup_wakeset();
                cpu_unit->syncw_wait_event->wait();
                cpu_database_lock->lock();
                syncw.seq++;

                /* when here, had been resumed either by other VCPU or by the console */

                /* might have been entered into SYNCW_SYS while sleeping: process it */
                syncw_process_external_syncw_sys(cx);

                /* if any of syncw-relevant interrupts are now pending, enter into sys window */
                if ((syncw.cpu[cx].active & (SYNCW_NOSYNC | SYNCW_SYS)) == 0)
                {
                    if (cpu_unit->cpu_intreg.query_syncw_sys())
                        syncw_enter_sys(RUN_PASS);
                }

                if (unlikely(cpu_unit->syncw_active & ~syncw.on & SYNCW_SYS_ILK))
                {
                    /* if SYS had been disabled system-wide, shut it down for this VCPU too */
                    if ((cpu_unit->syncw_active & SYNCW_SYS) && !(syncw.on & SYNCW_SYS))
                        syncw_leave_sys(RUN_PASS);

                    /* if ILK had been disabled system-wide, shut it down for this VCPU too */
                    if ((cpu_unit->syncw_active & SYNCW_ILK) && !(syncw.on & SYNCW_ILK))
                        syncw_leave_ilk(RUN_PASS);
                }

                /* is console stopping VCPUs? */
                if (unlikely(stop_cpus))
                {
                    /* 
                     * SIMH console requested VCPUs to pause and simulator to enter console mode.
                     * Note that since console raises stop_cpus signal, then locks cpu_database_lock, wakes up CPUs and
                     * unlocks cpu_database_lock, stop_cpus change is guaranteed to be visible to VCPUs after they aquire
                     * cpu_database_lock, since lock performs memory barrier and ensures memory update visibility 
                     * propagation of stop_cpus.
                     *
                     * Do not reset syncw_wait_cpu_id and waitset, since their data will be used by "CPU INFO" command
                     * to display syncw state. They will be reset by the console when the console calls syncw_resuming() 
                     * before resuming the execution of VCPUs.
                     */
                    cpu_database_lock->unlock();
                    return SCPE_STOP;
                }

                /* mark as not waiting */
                cpu_unit->syncw_wait_cpu_id = NO_CPU_ID;
                syncw.cpu[ix].waitset.clear(cx);

                /* if out of any window now, resume VCPU execution */
                if ((unlikely(cpu_unit->syncw_active & SYNCW_SYS_ILK) == 0))
                    break;

                /* go recalc syncw constraint again */
                proceed = FALSE;
                break;
            }
        }

        if (proceed)  break;
    }

    cpu_database_lock->unlock();

    syncw_wakeup_wakeset();

    return SCPE_OK;
}

/*
 * Adjust positions of all CPUs in syncw by "delta".
 * Although "delta" is uint32, it can be negavite for downward adjustments.
 */
static void syncw_adjust_pos_all(uint32 delta)
{
    for (uint32 ix = 0;  ix < sim_ncpus;  ix++)
    {
        if (syncw.cpu[ix].active & SYNCW_SYS_ILK)
            syncw.cpu[ix].pos += delta;
    }
}

/*
 * Display syncw state.
 * 
 * This routine is normally called on the console thread, however it can also be called 
 * on VCPU thread from op_reserved_ff when processing BUGW/BUGL bugcheck. In the latter
 * case cpu_database_lock is held by the caller.
 */
void syncw_display(SMP_FILE* st)
{
    RUN_SCOPE_RSCX_ONLY;

    t_bool nactive = 0;
    uint32 minpos = 0;              /* initialize to suppress false GCC warning */
    const char* vs;
    int w1 = 0, w2 = 0;

    /* Display current primary interlock mode */
    fprintf(st, "Interlock mode: %s\n", use_native_interlocked ? "native" : "portable");

    /* Display syncw settings */
    switch (syncw.on & SYNCW_SYS_ILK)
    {
    case SYNCW_SYS_ILK:
        vs = "SYS,ILK";
        w1 = imax(strwidth("%u", syncw.checkinterval_sys), strwidth("%u", syncw.checkinterval_ilk));
        w2 = imax(strwidth("%u", syncw.winsize_sys), strwidth("%u", syncw.winsize_ilk));
        break;
    case SYNCW_SYS:
        vs = "SYS";
        w1 = strwidth("%u", syncw.checkinterval_sys);
        w2 = strwidth("%u", syncw.winsize_sys);
        break;
    case SYNCW_ILK:
        w1 = strwidth("%u", syncw.checkinterval_ilk);
        w2 = strwidth("%u", syncw.winsize_ilk);
        vs = "ILK";
        break;
    case 0:
    default:
        vs = "none";
        break;
    }
    fprintf(st, "Active synchronization windows: %s\n", vs);
    if ((syncw.on & SYNCW_SYS_ILK) == SYNCW_SYS_ILK && 
        syncw.checkinterval_sys == syncw.checkinterval_ilk &&
        syncw.winsize_sys == syncw.winsize_ilk)
    {
        fprintf(st, "SYS and ILK check interval / window size: %*u / %*u\n", w1, syncw.checkinterval_sys, w2, syncw.winsize_sys);
    }
    else 
    {
        if (syncw.on & SYNCW_SYS)
            fprintf(st, "SYS check interval / window size: %*u / %*u\n", w1, syncw.checkinterval_sys, w2, syncw.winsize_sys);
        if (syncw.on & SYNCW_ILK)
            fprintf(st, "ILK check interval / window size: %*u / %*u\n", w1, syncw.checkinterval_ilk, w2, syncw.winsize_ilk);
    }

    if (syncw.on & SYNCW_SYS_ILK)
        fprintf(st, "Synchronization window quant / maxdrift:  %d / %d\n", syncw.quant, syncw.maxdrift);

    /* Find minimum position in syncw */
    for (uint32 ix = 0;  ix < sim_ncpus;  ix++)
    {
        if (cpu_running_set.is_clear(ix))  continue;
        if (syncw.cpu[ix].active & SYNCW_NOSYNC)  continue;
        if (syncw.cpu[ix].active & SYNCW_SYS_ILK)
        {
            if (nactive == 0)
            {
                minpos = syncw.cpu[ix].pos;
            }
            else
            {
                if (syncw.cpu[ix].pos < minpos)
                    minpos = syncw.cpu[ix].pos;
            }
            nactive++;
        }
    }

    if (nactive == 0)
    {
        fprintf(st, "\n");
        fprintf(st, "No CPUs currently active in synchronization window\n");
    }
    else
    {
        fprintf(st, "\n");
        fprintf(st, "CPUs active in synchronization window:\n");
        fprintf(st, "\n");
        fprintf(st, "CP  active    rel       wait   wt\n");
        fprintf(st, "U#    in      pos       seq#   on         waited by\n");
        fprintf(st, "-- ------- ---------- -------- -- ---------------------------\n");
    }

    /* Display per-CPU state of VCPUs active in syncw */
    for (uint32 ix = 0;  ix < sim_ncpus;  ix++)
    {
        CPU_UNIT* xcpu = cpu_units[ix];
        if (cpu_running_set.is_clear(ix))  continue;
        if (syncw.cpu[ix].active & SYNCW_NOSYNC)  continue;

        switch (syncw.cpu[ix].active & SYNCW_SYS_ILK)
        {
        case SYNCW_SYS_ILK:
            vs = "SYS,ILK";  break;
        case SYNCW_SYS:
            vs = "SYS";  break;
        case SYNCW_ILK:
            vs = "ILK";  break;
        default:
            continue;
        }

        fprintf(st, "%02d %7s %10d ", ix, vs, syncw.cpu[ix].pos - minpos);
        volatile uint32 xcid = xcpu->syncw_wait_cpu_id;
        if (xcid == NO_CPU_ID)
            fprintf(st, "            ");
        else
            fprintf(st, "%02d %08X ", xcid, syncw.cpu[ix].seq);

        t_bool first_waiter = TRUE;
        for (uint32 k = 0;  k < sim_ncpus;  k++)
        {
            if (syncw.cpu[ix].waitset.is_set(k))
            {
                fprintf(st, "%s%02d", first_waiter ? "" : ", ");
                first_waiter = FALSE;
            }
        }
        fprintf(st, "\n");
    }

    /* Perform consistency checks */
    for (uint32 ix = 0;  ix < sim_ncpus;  ix++)
    {
        CPU_UNIT* xcpu = cpu_units[ix];
        if (cpu_running_set.is_clear(ix))  continue;
        if (syncw.cpu[ix].active & SYNCW_NOSYNC)  continue;

        if (syncw.cpu[ix].active & SYNCW_SYS_ILK)
        {
            if (syncw.cpu[ix].pos < SYNCW_BASE_POS)
                fprintf(st, "*** Warning: syncw pos is below SYNCW_BASE_POS (CPU%d)\n", ix);
        }

        if (syncw_sys_active_is_set(ix) != (0 != (syncw.cpu[ix].active & SYNCW_SYS)))
            fprintf(st, "*** Warning: syncw_sys_active_is_set mismatches (active & SYNCW_SYS) for CPU%d [%d/%d]\n", ix, syncw_sys_active_is_set(ix), 0 != (syncw.cpu[ix].active & SYNCW_SYS));

        if ((syncw.cpu[ix].active & SYNCW_ILK) != (xcpu->syncw_active & SYNCW_ILK) && rscx->thread_type == SIM_THREAD_TYPE_CONSOLE)
            fprintf(st, "*** Warning: (syncw_active & SYNCW_ILK) mismatches (active & SYNCW_ILK) for CPU%d [%d/%d]\n", ix, 0 != (syncw.cpu[ix].active & SYNCW_ILK), 0 != (xcpu->syncw_active & SYNCW_ILK));
    }
}

/*
 * get width of printf-formatted string
 */
static int strwidth(char* fmt, ...)
{
    char buf[1025];

    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf) - 1, fmt, args);
    va_end(args);
    buf[1024] = '\0';
    return (int) strlen(buf);
}
