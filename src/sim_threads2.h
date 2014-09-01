/*
 * VAX interlocked instructions helper
 */
class InterlockedOpLock : public sim_try_auto_destructable
{
public:
    sim_try_volatile t_bool wmb;

    InterlockedOpLock(RUN_DECL, uint32 flags = 0);
    ~InterlockedOpLock();
    void onDestroy(t_bool unregistered);
    void virt_lock(int32 virt_addr, int32 acc) sim_try_volatile;
    void phys_lock(int32 phys_addr) sim_try_volatile;
    void prio_lock() sim_try_volatile;
    void qxi_busy() sim_try_volatile;
    void unlock() sim_try_volatile;

private:
    sim_try_volatile int lock_nesting_count;
    sim_try_volatile int32 lock_index;
    CPU_UNIT* sim_try_volatile cpu_unit;
    sim_try_volatile t_bool sv_priority_stored;
    sim_try_volatile sim_thread_priority_t sv_priority;
    uint32 flags;
    t_bool entered_temp_ilk;
};

#define IOP_ILK  (1 << 0)