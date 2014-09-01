#ifndef __SYNCW_H_INCLUDED__
#define __SYNCW_H_INCLUDED__

typedef struct
{
    uint32  active;                     /* can keep SYNCW_SYS, SYNCW_ILK, SYNCW_NOSYNC */
    uint32  pos;                        /* current position in syncw */
    cpu_set waitset;                    /* set of VCPUs waiting on this VCPU */
    uint32  seq;                        /* wait sequence number, for debugging only */
}
syncw_cpu_data;

typedef struct SIM_ALIGN_CACHELINE
{
    uint32 on;
    uint32 vsmp_winsize_sys;            /* vsmp_winsize_xxx are kept for record only and are not actively used */
    uint32 vsmp_winsize_ilk;            /* ... */
    uint32 winsize_sys;                 /* constraining SYS window size, i.e. maximum drift between VCPUs in SYS window */
    uint32 winsize_ilk;                 /* constraining ILK window size, i.e. maximum drift between VCPUs in ILK window */
    uint32 maxdrift;                    /* max inter-CPU drift that should cause forward VCPU to pause */
    int32  ipl_syslock;                 /* IPLs used to detect SYS and ILK entry/leave conditions */
    int32  ipl_resched;                 /* ... */
    uint32 checkinterval_sys;           /* compare drifts in SYS window every checkinterval_sys cycles */
    uint32 checkinterval_ilk;           /* compare drifts in SYS window every checkinterval_ilk cycles */
    uint32 checkinterval_none;          /* refill value for cpu_unit->syncw_countdown when it is not active in any sync window */
    uint32 quant;                       /* scale quant size */
    uint32 seq;                         /* event sequence number used for diagnostics only */
    t_byte pad1[SMP_MAXCACHELINESIZE - sizeof(uint32) * 8];
    syncw_cpu_data cpu[SIM_MAX_CPUS];   /* access to cpu[] data is protected by cpu_database_lock */
    t_byte pad2[SMP_MAXCACHELINESIZE];
}
syncw_data;

extern syncw_data syncw;

extern uint32 interrupt_reeval_syncw_sys[IPL_HLVL];

/*
 * Array syncw_sys_active[] is the primary store for each VCPU's SYNCW_SYS active flag.
 *
 * The value of each entry is either 1 or 0.
 *
 * The entries are set, reset and checked using interlocked operations -- in order to avoid unnecessary
 * locking of cpu_database_lock.
 *
 * Entry is:
 *
 *     Set only under the protection of cpu_database_lock, either by corresponsing VCPU or 
 *     by any other VCPU noticing that this VCPU has SYNCLK or IPI interrupts pending 
 *     but is not in SYNCW_SYS yet.
 *
 *     Reset only under the protection of cpu_database_lock by VCPU itself.
 *
 *     Checked with interlocked instructions only; holding cpu_database_lock is not required for checking.
 *
 * There are two slave copies of syncw_sys_active[] kept for cheaper access under special valid only
 * under special restrictive conditions.
 *
 * One copy is replicated to (cpu[].active & SYNCW_SYS) that can be accessed only under the protecton of
 * cpu_database_lock and when accessed under such protection always accurately reflects syncw_sys_active[].
 *
 * Another slave copy is kept in (cpu_unit->syncw_active & SYNCW_SYS). When (cpu_unit->syncw_active & SYNCW_SYS)
 * is set, it is guaranteed that (cpu[].active & SYNCW_SYS) and syncw_sys_active[] are set too.
 * When (cpu_unit->syncw_active & SYNCW_SYS) is cleared, the state of other two flags is unknown and
 * should be consulted using appropriate access procedures: interlocked access for syncw_sys_active[]
 * or access to (cpu[].active & SYNCW_SYS) while holding cpu_database_lock.
 */
extern smp_interlocked_uint32_var syncw_sys_active[SIM_MAX_CPUS];
#define syncw_sys_active_is_set(ix)        smp_interlocked_cas_done_var(& syncw_sys_active[ix], 1, 1)
#define syncw_sys_active_is_cleared(ix)    smp_interlocked_cas_done_var(& syncw_sys_active[ix], 0, 0)
#define syncw_sys_active_do_set(ix)        smp_interlocked_cas_done_var(& syncw_sys_active[ix], 0, 1)
#define syncw_sys_active_do_clear(ix)      smp_interlocked_cas_done_var(& syncw_sys_active[ix], 1, 0)

/*
 * SYNCW_NOSYNC if set in syncw.cpu[].active designates that this CPU is excluded from synchronization.
 * It does not participate in progress scale comparison and cannot be entered in any synchronization window.
 */
#define SYNCW_SYS      (1 << 0)
#define SYNCW_ILK      (1 << 1)
#define SYNCW_NOSYNC   (1 << 2)

#define SYNCW_OVERRIDE_ALL  (1 << 0)
#define SYNCW_DISABLE_CPU   (1 << 1)
#define SYNCW_ENABLE_CPU    (1 << 2)
#define SYNCW_NOLOCK        (1 << 3)

void syncw_init();
void syncw_reset();
void syncw_start(RUN_DECL);

#define syncw_need_enter_sys(cpu_unit)                                                                \
        ((syncw.on & SYNCW_SYS) && !(cpu_unit->syncw_active & SYNCW_SYS) &&                           \
         weak_read(sim_mp_active) && syncw_sys_active_is_cleared((cpu_unit)->cpu_id))

#define syncw_enter_sys(cpu_unit)                                                                     \
        do                                                                                            \
        {                                                                                             \
            if (syncw_need_enter_sys(cpu_unit))                                                       \
                syncw_doenter_sys(cpu_unit);                                                          \
        }                                                                                             \
        while (0)

#define syncw_enter_ilk(cpu_unit)                                                                     \
        do                                                                                            \
        {                                                                                             \
            if ((syncw.on & SYNCW_ILK) && !(cpu_unit->syncw_active & SYNCW_ILK) && weak_read(sim_mp_active))     \
                syncw_doenter_ilk(cpu_unit);                                                          \
        }                                                                                             \
        while (0)

#define syncw_leave_sys(cpu_unit)                                                                     \
        do                                                                                            \
        {                                                                                             \
            if ((cpu_unit->syncw_active & SYNCW_SYS) || syncw_sys_active_is_set((cpu_unit)->cpu_id))  \
                syncw_doleave_sys(cpu_unit);                                                          \
        }                                                                                             \
        while (0)

#define syncw_leave_ilk(cpu_unit)                                                                     \
        do                                                                                            \
        {                                                                                             \
            if (cpu_unit->syncw_active & SYNCW_ILK)                                                   \
                syncw_doleave_ilk(cpu_unit);                                                          \
        }                                                                                             \
        while (0)

void syncw_doenter_sys(RUN_DECL);
void syncw_doleave_sys(RUN_DECL);

t_bool syncw_doenter_ilk(RUN_DECL);
void syncw_doleave_ilk(RUN_DECL);

void syncw_leave_all(RUN_DECL, uint32 flags);

void syncw_wakeup_all_cpus(uint32 flags);

t_stat syncw_checkinterval(RUN_DECL, t_bool resuming);

void syncw_reinit_cpu(RUN_DECL, uint32 flags = 0);

void syncw_enable_cpu(RUN_DECL, uint32 flags = 0);

void syncw_reeval_sys(RUN_DECL);

void syncw_resuming();

void syncw_process_syncwsys_interrupt(RUN_DECL);

/*
 * Try to enter ILK.
 * If SYNCW_ILK state changed as the result of the call, return TRUE.
 * Otherwise return FALSE.
 */
SIM_INLINE static t_bool syncw_ifenter_ilk(RUN_DECL)
{
    if ((syncw.on & SYNCW_ILK) && !(cpu_unit->syncw_active & SYNCW_ILK) && weak_read(sim_mp_active))
    {
        return syncw_doenter_ilk(RUN_PASS);
    }
    else
    {
        return FALSE;
    }
}

void syncw_display(SMP_FILE* st);

#endif
