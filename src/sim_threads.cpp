/*
 * Threading support, other than memory barrier functions.
 *
 * Cross-platform code at the top, system-specific below.
 */

/* ============================================  Cross-platofrm part  ============================================ */

#define SIM_THREADS_H_FULL_INCLUDE
#include "sim_defs.h"

#if defined(__linux) || defined(__APPLE__)
#  include <unistd.h>
#  include <signal.h>
#  include <alloca.h>
#  include <sched.h>
#  include <string.h>
#  include <sys/types.h>
#  include <sys/time.h>
#  include <sys/resource.h>
#  include <sys/syscall.h>
#endif

#if defined(__linux)
#  include <sys/prctl.h>
#endif

#if defined(__APPLE__)
#  include <sys/sysctl.h>
#endif

#include <math.h>

void smp_mb_init();

#define InterlockedOpLock_NCS 64
static smp_lock_impl InterlockedOpLock_CS[InterlockedOpLock_NCS];
int smp_ncpus = 0;
int smp_nsmt_per_core = 0;
t_bool smp_smt_factor_set = FALSE;
double smp_smt_factor = 1.0;
uint32 smp_spinwait_min_us = 20;
static t_bool smp_init_info();
static int smp_get_cpu_count();
static int smp_get_smt_per_core();

#if defined(__linux) || defined(__APPLE__)
static void smp_set_thread_priority_init();
#endif

#if defined(__linux)
static t_bool check_nptl();
#endif

#if SMP_NATIVE_INTERLOCKED
static void smp_native_self_test();
#endif

#define CHECK(cond)  do { if (! (cond))  goto cleanup; } while (0)


// Hash function "jenkin32"
// See http://baagoe.org/en/wiki/Avalanche_on_integer_hash_functions
// Good hash function provides close to equal influence of every input bit to each output bit
SIM_INLINE static int32 hash32(int32 key)
{
    key += key << 12;
    key ^= (uint32) key >> 22;
    key += key << 4;
    key ^= (uint32) key >> 9;
    key += key << 10;
    key ^= (uint32) key >> 2;
    key += key << 7;
    key ^= (uint32) key >> 12;
    return key;
}

// Hash function "blend"
// See http://baagoe.org/en/wiki/Avalanche_on_integer_hash_functions
// SIM_INLINE static int32 hash32(int32 key)
// {
//     static const uint32 table[] = 
//     {
//         2945918480, 3438006206, 2844659691, 2655754164, 2149826949, 1264591814, 2178756406, 1108394547, 
//         1403771681, 4048350178, 1677843536, 2048500138, 1297136353, 2024434236, 2509406493, 316199015, 
//         2393194003, 1874105163, 3983215261, 3439390297, 1584819634, 1478947895, 3540658215, 883689796, 
//         719502645, 4247659682, 822370394, 2772803220, 757936898, 1719845855, 3502055134, 656881580, 
//         3374324640, 3124330056, 347211968, 1599773202, 496963709, 3110458943, 523394089, 2497946217, 
//         3335010626, 3404214538, 1888883598, 2976003771, 3939254910, 3003897015, 2182581168, 1671343576, 
//         2348764985, 4084950097, 4178729868, 3317058044, 2046252615, 422121877, 3563333935, 1337552614, 
//         1104797690, 2548923623, 451955747, 1164810435, 2927935617, 2554484595, 2619888961, 2857541770, 
//         1795365133, 2811945112, 1649331294, 1450688335, 671939448, 902285340, 3681429487, 2447096243, 
//         483718424, 2421517520, 3091631718, 605345797, 1410878973, 220137172, 865307884, 1182838007, 
//         253388584, 3277440191, 3371578720, 1916241651, 1728483986, 989513374, 456916081, 92768634, 
//         3206340126, 4202615644, 1531539748, 396485535, 4026082661, 1231485340, 2199300060, 4157798005, 
//         3777044224, 2287933036, 3595694247, 1551094359, 543261433, 1500897547, 66453467, 2872504971, 
//         300503158, 1128807093, 1014568889, 1947989870, 3587123916, 1980336113, 1762764873, 2133813317, 
//         3411418623, 820424661, 409564493, 2275920411, 3347611752, 936714661, 1064327833, 2468481999, 
//         1631358254, 2007544357, 4005061284, 912950047, 2910109411, 3474310600, 1899265939, 1973156029, 
//         2318969741, 2892813695, 2713929178, 3242518237, 16594153, 171984074, 591497231, 954543444, 
//         1354554019, 3235356530, 1476151471, 842338040, 118293459, 2224653319, 101169288, 2759980018, 
//         4116230445, 2992490774, 1324564348, 3868192635, 3517975959, 1818706027, 1432941044, 1213399497, 
//         2380031540, 2822674933, 1362982528, 3182785362, 1082775942, 3191502910, 2733095942, 1622141137, 
//         1518776481, 3745984642, 2104654362, 3774118182, 3265330586, 1700027821, 4073411392, 3085708053, 
//         3918811649, 726486672, 2965415373, 384468241, 1840089642, 3907032068, 1387173604, 68103926, 
//         3034885741, 1568156840, 2359606024, 1931753687, 1144194828, 3833410097, 4035832398, 2413482260, 
//         187017860, 3660353746, 3718880698, 149932000, 2670783638, 2310246279, 798536622, 3887744889, 
//         3961328305, 3951836255, 2613399833, 157182278, 326923407, 3536393481, 2097043788, 2589739568, 
//         21642499, 4287699182, 2748556443, 3653154024, 2073938870, 3048969709, 366991318, 968877428, 
//         3731838026, 2532488498, 2244928169, 239328683, 1204032763, 3632622590, 44459504, 4272061216, 
//         513128292, 1791826543, 995878081, 4213921898, 653353816, 2572115655, 282075229, 2257250062, 
//         1855746134, 1277086999, 688409481, 212966434, 1242468837, 1745001301, 3302366320, 2119695939, 
//         4176389798, 1056449218, 3065488184, 3808772962, 3692233925, 2793013347, 4139938488, 577855961, 
//         3458902478, 1032048172, 3616243041, 3141528977, 3855022780, 780134715, 752775146, 568410459, 
//         631440003, 2687590202, 2458928836, 3170567485, 4241558647, 2646717995, 3798169427, 4106894283
//     };
//     key = 3783973111 * key;
//     key ^= table[(uint32) key >> 24];
//     return key ^ table[key & 255];
// }

// Hash function by Thomas Wang
// http://www.concentric.net/~ttwang/tech/inthash.htm
// SIM_INLINE static int32 hash32(int32 key)
// {
//   key = ~key + (key << 15);  // key = (key << 15) - key - 1;
//   key = key ^ ((uint32) key >> 12);
//   key = key + (key << 2);
//   key = key ^ ((uint32) key >> 4);
//   key = key * 2057;          // key = (key + (key << 3)) + (key << 11);
//   key = key ^ ((uint32) key >> 16);
//   return key;
// }

/*
 * InterlockedOpLock_SpinCount logics:
 *
 *   When USE_SIMH_SMP_LOCK is not defined on Windows CriticalSection is used instead, spin-wait loop is 5 instructions,
 *   one of which is PAUSE:
 *
 *       loop: pause
 *             cmp     dword ptr own_count[edx], -1
 *             je      try_acquire
 *             dec     dword ptr [esp]
 *             jne     loop
 *
 *   with little refernces to memory and a delay introduced by pause, on some processors.
 *
 *   When USE_SIMH_SMP_LOCK is used, spin-wait loop compiles to 11 instructions (both by MSVC and GCC) one of which is PAUSE.
 *
 *   Latency of PAUSE instruction can range widely, from virtualy no latency (NOP) on old processors to a latency equivalent
 *   to roughly about 50 regular instructions or so on newer processors.
 *
 *   By comparison, a locked core of single VAX interlocked instructuion such as INSQHI can translate into few hundred
 *   instructions on the host system, with many references to memory for PTE, TLB, local variables etc.
 *
 *   This suggests a rather high value for InterlockedOpLock_SpinCount.
 *
 *   ToDo: Consider implementing self-learning auto-adaptive spin-wait critical section that
 *
 *             1) takes certain number of initial cycles to sample, limitng these initial waits by rather high maximum;
 *
 *             2) after the sampling throws away outliers and sets spin-wait cycle count based on resulting sampled vaues]
 *                (e.g. average multipled by 2);
 *
 *             3) and then adjusts spinwait dynamically based on success/failure, based by growing by a small margin
 *                on failure and shrinking on success, within max and min limits. 
 *
 *         More specfically, possible heurstics of general-purpose self-learnig auto-adaptable lock can be:
 *
 *             - smp_lock can collect statistics over certain number of lock cycles, similar to what smp_lock
 *               does now for PERF command, but delta-statistics, rather than continuously running one. For example,
 *               statistics can be kept over recent 1000 and/or 10000 lock cycles.
 *
 *             - Based on acquired statistics, adjust spin count to target very low (but non-zero) percentage of lock
 *               acquisitions resulting in going to OS-level blocking wait, such as 0.1% or 0.01%.
 *
 *             - If recent blocking wait rate was too low (well below the target), then tighten spin count.
 *               If it was too high (well above the target), then loosen spin count.
 *
 *             - Tighten slow, loosen fast.
 *               Tighten spin count by a small amount, gradually, based on delta-statistics taken over many acquisitions.
 *               However if a contention burst comes in and blocking wait rate goes high, detect it quickly (based on smaller
 *               sampling interval, i.e. number of recent acquisitions) and loosen spin count quickly by larger amount.
 *
 *             - If lock contention has bursty pattern, detect it and cope with it. After having to back the tightening off 
 *               by large amount for several times, give up on tightening and allow spin count to stay large.
 *
 *             - Never increase spin count beyond ~ 1/2 of the cost of rescheduling wait.
 *
 *             - Never drop spin count too low. Account for variations in cache/main memory access times etc.
 *               Specifically, it does not make sense to set spin count below 10.
 *               Also it does not make sense to set spin count below 1-5% of rescheduling cost.
 *
 */

static inline uint32 ROUNDUP(uint32 n, uint32 r)
{
    return ((n + r - 1) / r) * r;
}

/* give it a plenty of margin */
#define InterlockedOpLock_SpinCount 4000

#if defined(_WIN32)
   static DWORD run_scope_key = -1;
#elif (defined(__linux) || defined(__APPLE__)) && (defined(__x86_32__) || defined(__x86_64__))
   static pthread_key_t run_scope_key;
#endif

/*
 * Routines to verify memory alignment of variables used for interlocked and atomic
 * operations. If these variables are misplaced somehow, e.g. due to a bug in the
 * compiler or linker, and their misplacement goes unnoticed, it could be a silent killer.
 */

t_bool check_aligned(void* p, uint32 alignment, t_bool dothrow)
{
    if (0 == ((t_addr_val)(alignment - 1) & (t_addr_val) p))
    {
        return TRUE;
    }
    else if (dothrow)
    {
        panic("Internal error: Data alignment check failed");
        never_returns_bool_t;
    }
    else
    {
        return FALSE;
    }
}

t_bool smp_check_aligned(const smp_interlocked_uint32* p, t_bool dothrow)
{
    if (0 == (3 & (t_addr_val) p))
    {
        return TRUE;
    }
    else if (dothrow)
    {
        panic("Internal error: Data alignment check failed");
        never_returns_bool_t;
    }
    else
    {
        return FALSE;
    }
}

t_bool smp_check_aligned(const smp_interlocked_int32* p, t_bool dothrow)
{
    if (0 == (3 & (t_addr_val) p))
    {
        return TRUE;
    }
    else if (dothrow)
    {
        panic("Internal error: Data alignment check failed");
        never_returns_bool_t;
    }
    else
    {
        return FALSE;
    }
}

#if 0
// from the compiler's viewpoint, atomic_int32 is the same as smp_interlocked_int32
t_bool smp_check_aligned(const atomic_int32* p, t_bool dothrow)
{
    if (0 == (3 & (t_addr_val) p))
    {
        return TRUE;
    }
    else if (dothrow)
    {
        panic("Internal error: Data alignment check failed");
        never_returns_bool_t;
    }
    else
    {
        return FALSE;
    }
}
#endif

t_bool smp_check_aligned(const smp_interlocked_uint32_var* p, t_bool dothrow)
{
    if ((SMP_MAXCACHELINESIZE - 1) & (t_addr_val) p)
    {
        if (dothrow)
        {
            panic("Internal error: Data alignment check failed");
            never_returns_bool_t;
        }
        else
        {
            return FALSE;
        }
    }

    return smp_check_aligned(smp_var_p(p), dothrow);
}

t_bool smp_check_aligned(const smp_interlocked_int32_var* p, t_bool dothrow)
{
    if ((SMP_MAXCACHELINESIZE - 1) & (t_addr_val) p)
    {
        if (dothrow)
        {
            panic("Internal error: Data alignment check failed");
            never_returns_bool_t;
        }
        else
        {
            return FALSE;
        }
    }

    return smp_check_aligned(smp_var_p(p), dothrow);
}

t_bool smp_check_aligned(const atomic_int32_var* p, t_bool dothrow)
{
    if ((SMP_MAXCACHELINESIZE - 1) & (t_addr_val) p)
    {
        if (dothrow)
        {
            panic("Internal error: Data alignment check failed");
            never_returns_bool_t;
        }
        else
        {
            return FALSE;
        }
    }

    return smp_check_aligned(atomic_var_p(p), dothrow);
}

t_bool smp_check_aligned(const atomic_uint32_var* p, t_bool dothrow)
{
    if ((SMP_MAXCACHELINESIZE - 1) & (t_addr_val) p)
    {
        if (dothrow)
        {
            panic("Internal error: Data alignment check failed");
            never_returns_bool_t;
        }
        else
        {
            return FALSE;
        }
    }

    return smp_check_aligned(atomic_var_p(p), dothrow);
}

#if defined (__x86_64__)
/* not truly cross-platform, but for any processor with 64-bit interlocked operations */
t_bool smp_check_aligned(const smp_interlocked_uint64* p, t_bool dothrow)
{
    if (0 == (7 & (t_addr_val) p))
    {
        return TRUE;
    }
    else if (dothrow)
    {
        panic("Internal error: Data alignment check failed");
        never_returns_bool_t;
    }
    else
    {
        return FALSE;
    }
}

t_bool smp_check_aligned(const smp_interlocked_uint64_var* p, t_bool dothrow)
{
    if ((SMP_MAXCACHELINESIZE - 1) & (t_addr_val) p)
    {
        if (dothrow)
        {
            panic("Internal error: Data alignment check failed");
            never_returns_bool_t;
        }
        else
        {
            return FALSE;
        }
    }

    return smp_check_aligned(smp_var_p(p), dothrow);
}
#endif

smp_semaphore* smp_semaphore::create(int initial_open_count, t_bool dothrow)
{
    smp_semaphore_impl* sem = new smp_semaphore_impl();
    if (! sem->init(initial_open_count, dothrow))
    {
        delete sem;
        sem = NULL;
    }
    return sem;
}

smp_barrier* smp_barrier::create(int initial_count, t_bool dothrow)
{
    smp_barrier_impl* bar = new smp_barrier_impl();
    if (! bar->init(initial_count, dothrow))
    {
        delete bar;
        bar = NULL;
    }
    return bar;
}

smp_mutex* smp_mutex::create(t_bool dothrow)
{
    smp_mutex_impl* mtx = new smp_mutex_impl();
    if (! mtx->init(dothrow))
    {
        delete mtx;
        mtx = NULL;
    }
    return mtx;
}

smp_condvar* smp_condvar::create(t_bool dothrow)
{
    smp_condvar_impl* cv = new smp_condvar_impl();
    if (! cv->init(dothrow))
    {
        delete cv;
        cv = NULL;
    }
    return cv;
}

smp_event* smp_event::create(t_bool dothrow)
{
    smp_event_impl* ev = new smp_event_impl();
    if (! ev->init(dothrow))
    {
        delete ev;
        ev = NULL;
    }
    return ev;
}

#if SMP_NATIVE_INTERLOCKED == 0
int32 smp_native_adawi(volatile void* memory, int32 pa_sum, uint16 addend)
{
    panic("Native support for interlocked instructions is unavailable");
    return 0;
}

t_bool smp_native_bb(volatile void* memory, int32 pa, int32 bit, t_bool set)
{
    panic("Native support for interlocked instructions is unavailable");
    return 0;
}
#endif

/**********************************  init_threads  **********************************/

#if defined(__x86_32__) || defined(__x86_64__)
extern t_bool have_x86_xaddl;
extern t_bool have_x86_cmpxchgl;
extern t_bool have_pentium;
#endif

void init_threads_core()
{
#if defined(_WIN32)
    // assumptions by interlocked primitives
    if (sizeof(LONG) != sizeof(uint32))
        panic("Broken assumption: LONG != uint32");
    if (sizeof(long) != sizeof(smp_interlocked_uint32))
        panic("Broken assumption: sizeof(long) != sizeof(smp_interlocked_uint32)");
#endif

#if defined(__linux)
    if (! check_nptl())
        panic("System is not configured to use NPTL (New Posix Threads) library");
#endif

    if (! smp_init_info())
        panic("Unable to acquire information about processors on the host system");
    smp_ncpus = smp_get_cpu_count();
    smp_nsmt_per_core = smp_get_smt_per_core();

    smp_mb_init();

#if defined(__GNUC__) && (defined(__x86_32__) || defined(__x86_64__))
    // many gcc __sync_* builtins require x86 instructions not available before i486
    if (!have_x86_xaddl || !have_x86_cmpxchgl)
        panic("Requires i486 or later processor");
#elif defined(__WIN32__)
    // WIN32 code uses CMPXCHG
    if (! have_x86_cmpxchgl)
        panic("Requires i486 or later processor");
#endif

#if defined(_WIN32)
    if ((run_scope_key = TlsAlloc()) == 0xFFFFFFFF)
        panic("Unable to initialize thread-local storage");
#elif defined(__linux) || defined(__APPLE__)
    if (pthread_key_create(& run_scope_key, NULL))
        panic("Unable to initialize thread-local storage");
#else
#   error Unimplemented
#endif

#if defined(__GNUC__) && defined(__x86_32__) && !defined(__tune_i386__) && !defined(__tune_i486__)
    // If the code was not compiled for i386/i486, and current processor is not Pentium, exit
    if (! have_pentium)
        panic("Requires Pentium or later processor");
#endif

#if defined(__linux) || defined(__APPLE__)
    smp_set_thread_priority_init();
#endif

#if SMP_NATIVE_INTERLOCKED
    // self-test primitives used for native-mode implementation of VAX interlocked instructions
    smp_native_self_test();
#endif

    run_scope_context::set_current(NULL);
}

void init_threads_ext()
{
    if (smp_ncpus > 1)
        smp_lock::calibrate();

    for (int k = 0;  k < InterlockedOpLock_NCS;  k++)
    {
        /* initialize the lock */
        InterlockedOpLock_CS[k].init(InterlockedOpLock_SpinCount);

        /* register the lock as managed performance object */
        char lock_name[50];
        sprintf(lock_name, "interlock_%d", k);
        perf_register_object(dupstr(lock_name), & InterlockedOpLock_CS[k]);
    }

    smp_wmb();
}

/**********************************  InterlockedOpLock  **********************************/

InterlockedOpLock::InterlockedOpLock(RUN_DECL, uint32 flags)
{
    lock_nesting_count = 0;
    lock_index = -1;
    wmb = FALSE;
    this->cpu_unit = cpu_unit;
    sv_priority_stored = FALSE;
    this->flags = flags;
    entered_temp_ilk = FALSE;
    onConstructor();
}

InterlockedOpLock::~InterlockedOpLock()
{
    onDestroy(FALSE);
}

void InterlockedOpLock::onDestroy(t_bool unregistered)
{
    if (onDestructor(unregistered))
    {
        if (lock_nesting_count > 0)
        {
            lock_nesting_count = 1;
            unlock();
        }
    }
}

void InterlockedOpLock::virt_lock(int32 vaddr, int32 acc) sim_try_volatile
{
    /* 
     * We rely here on design for device handlers (XQ, RQ and TQ) that access shared areas,
     * such as UQSSP COMM area and XQ BDL, that performs such access only within the context
     * of VCPU threads and not IOP threads. Otherwise MB would have been always required
     * both here and in unlock(), regardless of whether sim_mp_active is TRUE or not.
     */
    if (unlikely(weak_read(sim_mp_active) == FALSE))
    {
        // smp_mb();
        return;
    }

    int32 pa_ilock = TestMark (RUN_PASS, vaddr, WA, NULL);
    phys_lock(pa_ilock);
}

void InterlockedOpLock::phys_lock(int32 addr) sim_try_volatile
{
    /* 
     * We rely here on design for device handlers (XQ, RQ and TQ) that access shared areas,
     * such as UQSSP COMM area and XQ BDL, that performs such access only within the context
     * of VCPU threads and not IOP threads. Otherwise MB would have been always required
     * both here and in unlock(), regardless of whether sim_mp_active is TRUE or not.
     */
    if (unlikely(weak_read(sim_mp_active) == FALSE))
    {
        // smp_mb();
        return;
    }

    if (lock_nesting_count++ == 0)
    {
        cpu_begin_interlocked(RUN_PASS, & sv_priority, & sv_priority_stored);

        if (flags & IOP_ILK)
            entered_temp_ilk = syncw_ifenter_ilk(RUN_PASS);

        // interlock on base longword of the address
        addr &= ~0x3;

        // compute hash function on addr and select critical section from the array
        lock_index = hash32((uint32) addr >> 2) & (InterlockedOpLock_NCS - 1);

        InterlockedOpLock_CS[lock_index].lock();

        // do not need to execute smp_mb because "lock" above executes full memory barrier
        // smp_mb();
    }
}

void InterlockedOpLock::prio_lock() sim_try_volatile
{
    if (unlikely(weak_read(sim_mp_active) == FALSE))
        return;

    if (lock_nesting_count++ == 0)
    {
        cpu_begin_interlocked(RUN_PASS, & sv_priority, & sv_priority_stored);

        if (flags & IOP_ILK)
            entered_temp_ilk = syncw_ifenter_ilk(RUN_PASS);
    }
}

void InterlockedOpLock::qxi_busy() sim_try_volatile
{
    /* 
     * INSQHI, INSQTI, REMQHI or REMQTI tried to acquire secondary interlock,
     * but queue header was busy: stay in SYNCW-ILK is IPL >= RESCHED
     */

    if (entered_temp_ilk && PSL_GETIPL(PSL) >= syncw.ipl_resched)
        entered_temp_ilk = FALSE;
}

void InterlockedOpLock::unlock() sim_try_volatile
{
    if (lock_nesting_count && 0 == --lock_nesting_count)
    {
        /*
         * do not need to execute smp_mb because the "unlock" below executes full memory barrier for virt_lock
         * and phys_lock cases; whereas for prio_lock case a barrier is not required since the code calling prio_lock
         * (native-mode implementations of interlocked instructions) takes care about issuing barriers itself
         */
        // if (wmb)  smp_mb();

        if (entered_temp_ilk)
        {
            syncw_leave_ilk(RUN_PASS);
            entered_temp_ilk = FALSE;
        }

        if (lock_index != -1)
        {
            InterlockedOpLock_CS[lock_index].unlock();
            lock_index = -1;
            // wmb = FALSE;
        }

        cpu_end_interlocked(RUN_PASS, sv_priority, sv_priority_stored);
        sv_priority_stored = FALSE;
    }
}

/**************************  smp_lock  **************************/

smp_lock* smp_lock::create(uint32 cycles, t_bool dothrow)
{
    smp_lock* cs = new smp_lock_impl();
    if (! cs->init(cycles, dothrow))
    {
        delete cs;
        cs = NULL;
    }
    return cs;
}

smp_lock* smp_lock::create(uint32 us, uint32 min_cycles, uint32 max_cycles, t_bool dothrow)
{
    smp_lock* cs = new smp_lock_impl();
    if (! cs->init(us, min_cycles, max_cycles, dothrow))
    {
        delete cs;
        cs = NULL;
    }
    return cs;
}

/**********************  InterruptRegister  **********************/

InterruptRegister::InterruptRegister()
{
    lo_ipl = 0;
    hi_ipl = 0;
    devs_per_ipl = NULL;
    smp_var(changed) = TRUE;
    irqs = NULL;
    local_irqs = NULL;
}

InterruptRegister::~InterruptRegister()
{
    if (irqs)
        free_aligned((void*) irqs);
    if (local_irqs)
        free(local_irqs);
}

void InterruptRegister::init(uint32 lo_ipl, uint32 hi_ipl, const uint32* devs_per_ipl)
{
    check_aligned(this, SMP_MAXCACHELINESIZE);
    smp_check_aligned(& changed);
    if (lo_ipl > hi_ipl)
        panic("Unable to initialize InterruptRegister: invalid parameters");
    for (uint32 k = 0;  k < hi_ipl - lo_ipl + 1;  k++)
    {
        if (devs_per_ipl[k] > 32)
            panic("Unable to initialize InterruptRegister: invalid parameters");
    }
    if (irqs)
        free_aligned((void*) irqs);
    if (local_irqs)
        free(local_irqs);

    uint32 alloc_size = (hi_ipl - lo_ipl + 1) * sizeof(uint32);
    if (NULL == (local_irqs = (uint32*) malloc(alloc_size)))
        panic("Unable to initialize InterruptRegister: memory allocation failed");

    /*
     * It may be possible to allocate a mask for each interrupt level in its own cache line container,
     * but is probably not worth it
     */
    alloc_size = (hi_ipl - lo_ipl + 1) * sizeof(smp_interlocked_uint32);
    alloc_size = ROUNDUP(alloc_size, SMP_MAXCACHELINESIZE);
    if (NULL == (irqs = (smp_interlocked_uint32*) malloc_aligned(alloc_size, SMP_MAXCACHELINESIZE)))
        panic("Unable to initialize InterruptRegister: aligned memory allocation failed");

    this->lo_ipl = lo_ipl;
    this->hi_ipl = hi_ipl;
    this->devs_per_ipl = devs_per_ipl;

    reset();
}

/* reset all pending interrupts */
void InterruptRegister::reset()
{
    for (uint32 ipl = lo_ipl;  ipl <= hi_ipl;  ipl++)
    {
        irqs[ipl - lo_ipl] = 0;
        local_irqs[ipl - lo_ipl] = 0;
    }
    smp_var(changed) = TRUE;
}

/* raise pending interrupt */
t_bool InterruptRegister::set_int(uint32 ipl, uint32 dev, t_bool toself)
{
    if (! toself)  smp_pre_interlocked_wmb();

    t_bool res = smp_test_set_bit(& irqs[ipl - lo_ipl], dev);

#if !defined(__x86_32__) && !defined(__x86_64__)
    if (! toself)  smp_post_interlocked_wmb();
#endif

    smp_interlocked_cas_done_var(& changed, 0, 1);    // can be just xchg(1) as well

    if (toself)
        local_irqs[ipl - lo_ipl] |= (1 << dev);

    return res;
}

/* clear pending interrupt */
t_bool InterruptRegister::clear_int(uint32 ipl, uint32 dev, t_bool toself)
{
    if (! toself)  smp_pre_interlocked_wmb();

    t_bool res = smp_test_clear_bit(& irqs[ipl - lo_ipl], dev);

#if !defined(__x86_32__) && !defined(__x86_64__)
    if (! toself)  smp_post_interlocked_wmb();
#endif

    smp_interlocked_cas_done_var(& changed, 0, 1);    // can be just xchg(1) as well

    if (toself)
        local_irqs[ipl - lo_ipl] &= ~(1 << dev);

    return res;
}

/* check if interrupt is marked pending in local buffer */
t_bool InterruptRegister::is_local_int(uint32 ipl, uint32 dev)
{
    return 0 != (local_irqs[ipl - lo_ipl] & (1 << dev));
}

/* dismiss interrupt on local processor */
void InterruptRegister::dismiss_int(RUN_DECL, uint32 ipl, uint32 dev)
{
    if (interrupt_reeval_syncw_sys[ipl - lo_ipl] & (1 << dev))
        syncw_enter_sys(RUN_PASS);
    smp_test_clear_bit(& irqs[ipl - lo_ipl], dev);
    local_irqs[ipl - lo_ipl] &= ~(1 << dev);
}

/* copy irqs to local_irqs, usually will be executed after memory barrier */
void InterruptRegister::copy_irqs_to_local()
{
    for (uint32 ipl = lo_ipl;  ipl <= hi_ipl;  ipl++)
    {
        local_irqs[ipl - lo_ipl] = weak_read(irqs[ipl - lo_ipl]);
    }
}

/* find highest irql pending in local_irqs */
int32 InterruptRegister::highest_local_irql()
{
    for (uint32 ipl = hi_ipl;  ipl >= lo_ipl;  ipl--)
    {
        if (local_irqs[ipl - lo_ipl])
            return (int32) ipl;
    }
    return 0;
}

void InterruptRegister::query_local_clk_ipi(t_bool* is_active_clk_interrupt, t_bool* is_active_ipi_interrupt)
{
    *is_active_clk_interrupt = (local_irqs[IPL_CLK] & INT_CLK) ? TRUE : FALSE;
    *is_active_ipi_interrupt = (local_irqs[IPL_IPINTR] & INT_IPINTR) ? TRUE : FALSE;
}

/* examine if device interrupts is pending, called from console with all CPUs paused and memory barriers already executed */
t_bool InterruptRegister::examine_int(uint32 ipl, uint32 dev)
{
    return 0 != (weak_read(irqs[ipl - lo_ipl]) & (1 << dev));
}

/*
 * Check if there is any interrupt pending at exact "ipl" level using previously read
 * local_irqs.
 *
 * If yes, return requesting device index to "*int_dev" and return TRUE as method value,
 * and clear the request bit both in irqs and local_irqs. If the bit in irqs had been
 * already cleared, consider interrupt request as cancelled and disregard it.
 *
 * If there are no interrupts pending at "ipl", method returns FALSE.
 *
 * Must be called by local processor only.
 */
t_bool InterruptRegister::check_int_atipl_clr(RUN_DECL, uint32 ipl, uint32* int_dev)
{
    if (ipl < lo_ipl || ipl > hi_ipl)
        return FALSE;

    uint32 dev;

    smp_interlocked_uint32* pintr = & irqs[ipl - lo_ipl];
    uint32* plocal = & local_irqs[ipl - lo_ipl];
    uint32 local = *plocal;

    for (dev = 0;  dev < devs_per_ipl[ipl - lo_ipl];  dev++)
    {
        if (local & (1 << dev))
        {
            if (interrupt_reeval_syncw_sys[ipl - lo_ipl] & (1 << dev))
                syncw_enter_sys(RUN_PASS);
            *plocal &= ~(1 << dev);
            if (smp_test_clear_bit(pintr, dev))
            {
                *int_dev = dev;
                return TRUE;
            }
        }
    }

    return FALSE;
}

/* check if any of syncw_sys relevant interrupts are pending */
#if defined(VM_VAX_MP) && (IPL_CLK == IPL_SYNCLK) && (IPL_CLK == IPL_IPINTR)
SIM_INLINE t_bool InterruptRegister::query_syncw_sys()
{
    return 0 != (interrupt_reeval_syncw_sys[IPL_CLK] & weak_read(irqs[IPL_CLK]));
}
#else
t_bool InterruptRegister::query_syncw_sys()
{
    for (uint32 ipl = lo_ipl;  ipl <= hi_ipl;  ipl++)
    {
        if (interrupt_reeval_syncw_sys[ipl - lo_ipl] & weak_read(irqs[ipl - lo_ipl]))
            return TRUE;
    }

    return FALSE;
}
#endif

#if 0
/* 
 * This routine is not currently used.
 *
 * Examine if interrupts at level "ipl" are pending, 
 * intended to be called from console with all CPUs paused and memory barriers already executed.
 */
t_bool InterruptRegister::examine_int(uint32 ipl)
{
    return 0 != weak_read(irqs[ipl - lo_ipl]);
}

/*
 * This routine is not currently used.
 *
 * Check if there is any interrupt pending that can be delivered at "ipl" level.
 * Clear "changed" flag, execute memory barrier, but do not reset the pending interrupt.
 */
t_bool InterruptRegister::check_int(uint32 ipl, uint32* int_ipl)
{
    smp_interlocked_cas_var_done(& changed, 1, 0);       // can be just xchg(0) as well
    smp_post_interlocked_rmb();

    if (ipl >= hi_ipl)
        return FALSE;

    uint32 llpl = imax(lo_ipl, ipl + 1);

    smp_interlocked_uint32* pintr = & irqs[hi_ipl - lo_ipl];

    for (uint32 cpl = hi_ipl; cpl >= llpl;  cpl--, pintr--)
    {
        if (0 != weak_read(*pintr))
        {
            *int_ipl = cpl;
            return TRUE;
        }
    }

    return FALSE;
}

/*
 * This routine is not currently used.
 *
 *   Check if any interrupt is pending that will interrupt executiion at level "ipl" and if so, retrive and clear it.
 *
 *   This method begins by weak (lazy) reading of interrupt bits data, without using memory barriers.
 *   This means that even if other threads set the interrupt, it may not be propagated to the cache memory
 *   of the CPU executing current thread yet.
 *   In virtual terms, this interrupt may not be visible right away to current processor.
 *
 *   This is fine, since interrupt reguster is intended for asynchronous interrupts, such as external device interrupts or
 *   interprocessor (IPI) interrupts. Being asynchronous by their nature, and not timed exactly to instructuion stream, it is fine if
 *   they get noticed by the target CPU with slight delay, such as a delay caused by cache coherence / update propagation protocol.
 *
 *   In practical terms, it may take about one or two VAX instructions in the simulator for the host memory cache coherence protocol
 *   to complete and update to be propagated. This is perfectly fine.
 *
 *   On the other hand, set_int() executed by local processor _will_ be noticed immediatelly, since local cache is always self-coherent.
 *   Therefore IPI interrupt sent by the processor to itself or any interrupt cause by per-CPU device or any other interrupt-generating
 *   code executing within the context of current emulator thread _will_ be visible to that thread immediatelly.
 *
 *   Thus, logically asynchronous interrupts may get noticed at very slight delay, but logically synchronous interrupts are always
 *   noticed right away.
 *
 */

t_bool InterruptRegister::check_int(uint32 ipl, uint32* int_ipl, uint32* int_dev)
{
#if (IPL_HMAX - IPL_HMIN != 3)
#  error change shortcut code below
#endif

    // NB: may need to reset "changed", and synchronize with SET_IRQL and eval_int

    if (hi_ipl - lo_ipl == 3)
    {
        // most usual case: no pending interrupts at all
        if (weak_read(irqs[0]) == 0 &&
            weak_read(irqs[1]) == 0 &&
            weak_read(irqs[2]) == 0 &&
            weak_read(irqs[3]) == 0)
        {
            return FALSE;
        }
    }

    if (ipl >= hi_ipl)
        return FALSE;

    uint32 cpl, dev;

    uint32 llpl = imax(lo_ipl, ipl + 1);

    smp_interlocked_uint32* pintr = & irqs[hi_ipl - lo_ipl];

    for (cpl = hi_ipl; cpl >= llpl;  cpl--, pintr--)
    {
        if (weak_read(*pintr))
        {
            for (dev = 0;  dev < devs_per_ipl[cpl - lo_ipl];  dev++)
            {
                if (smp_test_clear_bit(pintr, dev))
                {
                    *int_ipl = cpl;
                    *int_dev = dev;
                    return TRUE;
                }
            }
        }
    }

    return FALSE;
}
#endif

/* ============================================  Windows ============================================ */

#if defined(_WIN32)

/**********************  Windows -- create thread  **********************/

t_bool smp_create_thread(smp_thread_routine_t start_routine, void* arg, smp_thread_t* thread_handle, t_bool dothrow)
{
    if (thread_handle)
        *thread_handle = SMP_THREAD_NULL;

    if (sizeof(smp_thread_t) != sizeof(HANDLE))
        panic("Internal error: smp_thread_t misdeclared");

    uintptr_t res = _beginthreadex(NULL, 0, start_routine, arg, thread_handle ? CREATE_SUSPENDED : 0, NULL);

    if (res == 0)
    {
        if (dothrow)
            panic("Unable to create thread");
        return FALSE;
    }

    if (thread_handle)
    {
        *thread_handle = (HANDLE) res;
        if (ResumeThread(*thread_handle) != 1)
        {
            *thread_handle = SMP_THREAD_NULL;
            if (dothrow)
                panic("Unable to create thread");
            return FALSE;
        }
    }

    return TRUE;
}

/**********************  Windows -- wait thread  **********************/

t_bool smp_wait_thread(smp_thread_t thread_handle, t_value* exitcode, t_bool dothrow)
{
    if (WaitForSingleObject(thread_handle, INFINITE) == WAIT_OBJECT_0)
    {
        DWORD dw = 0;
        if (GetExitCodeThread(thread_handle, & dw))
        {
            if (exitcode)
                *exitcode = dw;
            return TRUE;
        }
    }

    if (dothrow)
        panic("Internal error: unable to complete worker thread");

    return FALSE;
}

/**********************  Windows -- set thread priority **********************/

t_bool smp_set_thread_priority(sim_thread_priority_t prio)
{
    run_scope_context::get_current()->setting_priority(prio);
    return smp_set_thread_priority(GetCurrentThread(), prio);
}

int smp_get_thread_os_priority(smp_thread_t thread_th)
{
    DWORD pclass = GetPriorityClass(GetCurrentProcess());
    int tprio = GetThreadPriority(thread_th);

    if (pclass == 0 || tprio == THREAD_PRIORITY_ERROR_RETURN)
        return -1;

    /*
     * See priorities table in "Scheduling priorities"
     * http://msdn.microsoft.com/en-us/library/windows/desktop/ms685100%28v=VS.85%29.aspx
     */

    switch (pclass)
    {
    case IDLE_PRIORITY_CLASS:
        switch (tprio)
        {
        case THREAD_PRIORITY_IDLE:            return 1;
        case THREAD_PRIORITY_LOWEST:          return 2;
        case THREAD_PRIORITY_BELOW_NORMAL:    return 3;
        case THREAD_PRIORITY_NORMAL:          return 4;
        case THREAD_PRIORITY_ABOVE_NORMAL:    return 5;
        case THREAD_PRIORITY_HIGHEST:         return 6;
        case THREAD_PRIORITY_TIME_CRITICAL:   return 15;
        }
        break;

    case BELOW_NORMAL_PRIORITY_CLASS:
        switch (tprio)
        {
        case THREAD_PRIORITY_IDLE:            return 1;
        case THREAD_PRIORITY_LOWEST:          return 4;
        case THREAD_PRIORITY_BELOW_NORMAL:    return 5;
        case THREAD_PRIORITY_NORMAL:          return 6;
        case THREAD_PRIORITY_ABOVE_NORMAL:    return 7;
        case THREAD_PRIORITY_HIGHEST:         return 8;
        case THREAD_PRIORITY_TIME_CRITICAL:   return 15;
        }
        break;

    case NORMAL_PRIORITY_CLASS:
        switch (tprio)
        {
        case THREAD_PRIORITY_IDLE:            return 1;
        case THREAD_PRIORITY_LOWEST:          return 6;
        case THREAD_PRIORITY_BELOW_NORMAL:    return 7;
        case THREAD_PRIORITY_NORMAL:          return 8;
        case THREAD_PRIORITY_ABOVE_NORMAL:    return 9;
        case THREAD_PRIORITY_HIGHEST:         return 10;
        case THREAD_PRIORITY_TIME_CRITICAL:   return 15;
        }
        break;

    case ABOVE_NORMAL_PRIORITY_CLASS:
        switch (tprio)
        {
        case THREAD_PRIORITY_IDLE:            return 1;
        case THREAD_PRIORITY_LOWEST:          return 8;
        case THREAD_PRIORITY_BELOW_NORMAL:    return 9;
        case THREAD_PRIORITY_NORMAL:          return 10;
        case THREAD_PRIORITY_ABOVE_NORMAL:    return 11;
        case THREAD_PRIORITY_HIGHEST:         return 12;
        case THREAD_PRIORITY_TIME_CRITICAL:   return 15;
        }
        break;

    case HIGH_PRIORITY_CLASS:
        switch (tprio)
        {
        case THREAD_PRIORITY_IDLE:            return 1;
        case THREAD_PRIORITY_LOWEST:          return 11;
        case THREAD_PRIORITY_BELOW_NORMAL:    return 12;
        case THREAD_PRIORITY_NORMAL:          return 13;
        case THREAD_PRIORITY_ABOVE_NORMAL:    return 14;
        case THREAD_PRIORITY_HIGHEST:         return 15;
        case THREAD_PRIORITY_TIME_CRITICAL:   return 15;
        }
        break;

    case REALTIME_PRIORITY_CLASS:
        switch (tprio)
        {
        case THREAD_PRIORITY_IDLE:            return 16;
        case THREAD_PRIORITY_LOWEST:          return 22;
        case THREAD_PRIORITY_BELOW_NORMAL:    return 23;
        case THREAD_PRIORITY_NORMAL:          return 24;
        case THREAD_PRIORITY_ABOVE_NORMAL:    return 25;
        case THREAD_PRIORITY_HIGHEST:         return 26;
        case THREAD_PRIORITY_TIME_CRITICAL:   return 31;
        }
        break;
    }

    return -1;
}

t_bool smp_set_thread_priority(smp_thread_t thread_th, sim_thread_priority_t prio)
{
    int nPriority;
    BOOL bDisablePriorityBoost = FALSE;

    switch (prio)
    {
    /* CPU thread -- regular instruction stream execution */
    case SIMH_THREAD_PRIORITY_CPU_RUN:
        nPriority = THREAD_PRIORITY_BELOW_NORMAL;
        bDisablePriorityBoost = TRUE;
        break;

    /* console thread -- while CPUs are paused */
    case SIMH_THREAD_PRIORITY_CONSOLE_PAUSED:
        nPriority = THREAD_PRIORITY_NORMAL;
        bDisablePriorityBoost = FALSE;
        break;

    /* console thread -- while CPUs are running */
    case SIMH_THREAD_PRIORITY_CONSOLE_RUN:
        // nPriority = THREAD_PRIORITY_ABOVE_NORMAL;
        // bDisablePriorityBoost = FALSE;
        /* make it the same as OS_HI, so console could preempt VCPU threads,
           could even make it higher than OS_HI */
        nPriority = THREAD_PRIORITY_HIGHEST;
        bDisablePriorityBoost = TRUE;
        break;

    /*
     * IO processing thread: make thread priority identical with CRITICAL_OS to avoid assigning criticality
     * setting to locks for IOP structures, so locking can be done without the overhead of elevating/demoting thread
     * priority, but at the same time also avoiding priority inversion when CPU's thread running at CRITIAL_OS
     * (a priority at which OS will typically access IO device) would have to wait for the lock held by IOP thread
     * executing at a lower priority and possibly preempted. Downside of this choice being that high IO rate may
     * also saturate host system computationally and take precedence over host UI enviroment both in terms of
     * high-priority computational load and IO requests queued at high-priority.
     */
    case SIMH_THREAD_PRIORITY_IOP:
        /* fall through to CRITICAL_OS */

    /* CPU thread -- kernel is in a critical section (holding spinlocks etc.) */
    case SIMH_THREAD_PRIORITY_CPU_CRITICAL_OS:
        nPriority = THREAD_PRIORITY_ABOVE_NORMAL;
        bDisablePriorityBoost = TRUE;
        break;

    /* CPU thread -- kernel is processing CLK or IPI interrupt,
       or any of these interrupts is pending */
    case SIMH_THREAD_PRIORITY_CPU_CRITICAL_OS_HI:
        nPriority = THREAD_PRIORITY_HIGHEST;
        bDisablePriorityBoost = TRUE;
        break;

    /* CPU thread -- VM is in a critical section (holding lock etc.) */
    case SIMH_THREAD_PRIORITY_CPU_CRITICAL_VM:
        nPriority = THREAD_PRIORITY_TIME_CRITICAL;
        bDisablePriorityBoost = TRUE;
        break;

    /* CPU thread -- processor loops calibration using SSC clock */
    case SIMH_THREAD_PRIORITY_CPU_CALIBRATION:
        nPriority = THREAD_PRIORITY_TIME_CRITICAL;
        bDisablePriorityBoost = TRUE;
        break;

    /* clock strobing thread */
    case SIMH_THREAD_PRIORITY_CLOCK:
        if (sim_vsmp_active)
        {
            nPriority = THREAD_PRIORITY_TIME_CRITICAL;
            bDisablePriorityBoost = TRUE;
        }
        else
        {
            nPriority = THREAD_PRIORITY_HIGHEST;
            bDisablePriorityBoost = TRUE;
        }
        break;

    default:
        return FALSE;
    }

    HANDLE hThread = thread_th;
    t_bool res = (FALSE != SetThreadPriority(hThread, nPriority));
    if (res == FALSE && nPriority == THREAD_PRIORITY_TIME_CRITICAL)
    {
        nPriority = THREAD_PRIORITY_HIGHEST;
        bDisablePriorityBoost = FALSE;
        res = (FALSE != SetThreadPriority(hThread, nPriority));
    }
    res = res && (0 != SetThreadPriorityBoost(hThread, bDisablePriorityBoost));
    return res;
}

/* per-thread initializaion */
t_bool smp_thread_init()
{
    return TRUE;
}

/* print thread priority allocation etc. */
void smp_show_thread_priority_info(SMP_FILE* st)
{
}

/**********************  Windows -- set thread name **********************/

const DWORD MS_VC_EXCEPTION=0x406D1388;

#pragma pack(push,8)
typedef struct tagTHREADNAME_INFO
{
   DWORD dwType; // Must be 0x1000.
   LPCSTR szName; // Pointer to name (in user addr space).
   DWORD dwThreadID; // Thread ID (-1=caller thread).
   DWORD dwFlags; // Reserved for future use, must be zero.
} THREADNAME_INFO;
#pragma pack(pop)

void smp_set_thread_name(const char* name)
{
    THREADNAME_INFO info;
    info.dwType = 0x1000;
    info.szName = name;
    info.dwThreadID = (DWORD) -1;
    info.dwFlags = 0;

    __try
    {
        RaiseException(MS_VC_EXCEPTION, 0, sizeof(info) / sizeof(ULONG_PTR), (ULONG_PTR*) &info);
    }
    __except(EXCEPTION_EXECUTE_HANDLER)
    {
    }
}

/**********************  Windows -- TLS  **********************/

t_bool smp_tls_alloc(smp_tls_key* key)
{
    uint32 /*DWORD*/ dw = TlsAlloc();
    return (*key = dw) != 0xFFFFFFFF;
}

void tls_set_value(const smp_tls_key& key, void* value)
{
    TlsSetValue(key, value);
}

void* tls_get_value(const smp_tls_key& key)
{
    return TlsGetValue(key);
}

/************************  Windows -- smp_lock  ************************/

#if !defined(USE_SIMH_SMP_LOCK)
smp_lock_impl::smp_lock_impl()
{
    criticality = SIM_LOCK_CRITICALITY_NONE;
    m_inited = FALSE;
}

t_bool smp_lock_impl::init(uint32 cycles, t_bool dothrow)
{
    if (m_inited)  return TRUE;

    if (! smp_check_aligned((smp_interlocked_uint32*) & m_cs, dothrow))
        return FALSE;

    if (cycles == 0 || smp_ncpus <= 1)
    {
        InitializeCriticalSection(& m_cs);
        m_inited = TRUE;
    }
    else
    {
        m_inited = InitializeCriticalSectionAndSpinCount(& m_cs, cycles);
    }

    if (!m_inited && dothrow)
        panic("Unable to initialize critical section");

    return m_inited;
}

smp_lock_impl::~smp_lock_impl()
{
    if (m_inited)
        DeleteCriticalSection(& m_cs);
}

void smp_lock_impl::set_spin_count(uint32 cycles)
{
    SetCriticalSectionSpinCount(&m_cs, (smp_ncpus > 1) ? cycles : 0);
}

void smp_lock_impl::lock()
{
    if (criticality != SIM_LOCK_CRITICALITY_NONE)
        critical_lock(criticality);

    EnterCriticalSection(& m_cs);
}

void smp_lock_impl::unlock()
{
    LeaveCriticalSection(& m_cs);

    if (criticality != SIM_LOCK_CRITICALITY_NONE)
        critical_unlock(criticality);
}
#endif

/**********************  Windows -- smp_semaphore  **********************/

smp_semaphore_impl::smp_semaphore_impl()
{
    hSemaphore = NULL;
    if (sizeof(smp_pollable_handle_t) != sizeof(HANDLE))
        panic("Internal error: smp_pollable_handle_t misdeclared");
}

smp_semaphore_impl::~smp_semaphore_impl()
{
    if (hSemaphore != NULL)
        CloseHandle(hSemaphore);
}

t_bool smp_semaphore_impl::init(int initial_open_count, t_bool dothrow)
{
    const int max_count = 0x3FFFFFFF;
    if (NULL != (hSemaphore = CreateSemaphore(NULL, initial_open_count, max_count, NULL)))
    {
        smp_wmb();
        return TRUE;
    }
    else if (dothrow)
    {
        panic("Unable to initialize semaphore");
        never_returns_bool_t;
    }
    else
    {
        return FALSE;
    }
}

smp_pollable_handle_t smp_semaphore_impl::pollable_handle()
{
    return hSemaphore;
}

const char* smp_semaphore_impl::pollable_handle_op()
{
    return "h";
}

void smp_semaphore_impl::release(int count)
{
    if (! ReleaseSemaphore(hSemaphore, count, NULL))
        panic("Unable to release semaphore");
}

void smp_semaphore_impl::clear()
{
    // could do "binary search" if count was expected to be high, but in SIMH it is usually just zero
    for (;;)
    {
        switch (WaitForSingleObject(hSemaphore, 0))
        {
        case WAIT_OBJECT_0:
            break;
        case WAIT_TIMEOUT:
            return;
        default:
            panic("Unexpected failure trying to clear semaphore");
        }
    }
}

void smp_semaphore_impl::wait()
{
    if (WAIT_OBJECT_0 != WaitForSingleObject(hSemaphore, INFINITE))
        panic("Unable to acquire semaphore");
}

t_bool smp_semaphore_impl::trywait()
{
    switch (WaitForSingleObject(hSemaphore, 0))
    {
    case WAIT_OBJECT_0:
        return TRUE;
    case WAIT_TIMEOUT:
        return FALSE;
    default:
        panic("Unexpected failure trying to acquire semaphore");
        never_returns_bool_t;
    }
}

/**********************  Windows -- smp_barrier  **********************/

smp_barrier_impl::smp_barrier_impl()
{
    hMutex = NULL;
    hEvent = NULL;
    barrier_count = 0;
    waiting_count = 0;
    for (int k = 0;  k < SMP_BARRIER_CYCLES;  k++)
        hOldEvents[k] = NULL;
}

smp_barrier_impl::~smp_barrier_impl()
{
    dealloc();
}

void smp_barrier_impl::dealloc()
{
    if (hMutex != NULL)
    {
        CloseHandle(hMutex);
        hMutex = NULL;
    }

    if (hEvent != NULL)
    {
        CloseHandle(hEvent);
        hEvent = NULL;
    }

    for (int k = 0;  k < SMP_BARRIER_CYCLES;  k++)
    {
        if (hOldEvents[k] != NULL)
            CloseHandle(hOldEvents[k]);
        hOldEvents[k] = NULL;
    }
}

t_bool smp_barrier_impl::init(int initial_count, t_bool dothrow)
{
    barrier_count = initial_count;

    hMutex = CreateMutex(NULL, FALSE, NULL);
    t_bool ok = hMutex != NULL;

    hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    ok = ok && (hEvent != NULL);

    for (int k = 0;  k < SMP_BARRIER_CYCLES;  k++)
    {
        hOldEvents[k] = CreateEvent(NULL, TRUE, FALSE, NULL);
        ok = ok && (hOldEvents[k] != NULL);
    }

    if (ok)
    {
        smp_wmb();
        return TRUE;
    }
    else
    {
        dealloc();
        smp_wmb();

        if (dothrow)
        {
            panic("Unable to initialize barrier");
            never_returns_bool_t;
        }
        else
        {
            return FALSE;
        }
    }
}

void smp_barrier_impl::wait()
{
    if (WAIT_OBJECT_0 != WaitForSingleObject(hMutex, INFINITE))
        panic("Unable to acquire barrier mutex");

    /*
     * Note that we cannot ResetEvent here. Othwerwise if a waiting thread is removed from the wait state
     * by kernel APC and re-enters wait state after the event is cleared, the thread will remain stalled.
     *
     * SetEvent followed by ResetEvent, and likewise PulseEvent are inherently unreliable in Windows.
     * If waiting thread is removed from wait state by kernel APC and re-enters wait state after the event
     * is cleared, the thread will remain stalled.
     *
     * This is fundamental design flaw in Windows. See documentation for PulseEvent for more details.
     *
     * Windows should have linked any thread inside SetEvent to the event, so once the thread returns
     * from kernel APC it would know the event had been signalled in the meanwhile and treat the event
     * as signalled. But Windows does not, according to the note in PulseEvent documentation.
     *
     * The workaround is to keep a rotating list of events for several generations of the barrier use.
     *
     * The application should _not_ re-create the whole barrier object, as it would destroy the old event
     * and make the thread resuming after KAPC to possibly fail in waiting on the event -- unless
     * SignalObjectAndWait internally bumps reference count and keeps it across post-KAPC restarts,
     * which we do not know if it does.
     */

    // if (waiting_count == 0)
    //  cannot ResetEvent(hEvent);

    if (++waiting_count >= barrier_count)
    {
        waiting_count = 0;
        SetEvent(hEvent);
        rotate_events();
        ReleaseMutex(hMutex);
    }
    else
    {
        if (WAIT_OBJECT_0 != SignalObjectAndWait(hMutex, hEvent, INFINITE, FALSE))
            panic("Unable to wait on the barrier");
    }
}

t_bool smp_barrier_impl::trywait()
{
    if (WAIT_OBJECT_0 != WaitForSingleObject(hMutex, INFINITE))
        panic("Unable to acquire barrier mutex");
    /* See the note above for wait() */
    if (waiting_count >= barrier_count - 1)
    {
        waiting_count = 0;
        SetEvent(hEvent);
        rotate_events();
        ReleaseMutex(hMutex);
        return TRUE;
    }
    else
    {
        ReleaseMutex(hMutex);
        return FALSE;
    }
}

void smp_barrier_impl::rotate_events()
{
    HANDLE hOldestEvent = hOldEvents[SMP_BARRIER_CYCLES - 1];
    for (int k = SMP_BARRIER_CYCLES - 1;  k >= 1;  k--)
        hOldEvents[k] = hOldEvents[k - 1];
    hOldEvents[0] = hEvent;
    hEvent = hOldestEvent;
    ResetEvent(hEvent);
}

t_bool smp_barrier_impl::set_count(int count, t_bool dothrow)
{
    if (WAIT_OBJECT_0 != WaitForSingleObject(hMutex, INFINITE))
        panic("Unable to acquire barrier mutex");

    barrier_count = count;

    if (waiting_count >= barrier_count)
    {
        waiting_count = 0;
        SetEvent(hEvent);
        rotate_events();
    }

    ReleaseMutex(hMutex);
    return TRUE;
}

void smp_barrier_impl::clear()
{
    panic("Operation undefined: barrier.clear");
}

void smp_barrier_impl::release(int count)
{
    panic("Operation undefined: barrier.release");
}

/**********************  Windows -- smp_mutex  **********************/

smp_mutex_impl::smp_mutex_impl()
{
    hMutex = NULL;
    criticality = SIM_LOCK_CRITICALITY_NONE;
}

smp_mutex_impl::~smp_mutex_impl()
{
    if (hMutex)
        CloseHandle(hMutex);
}

t_bool smp_mutex_impl::init(t_bool dothrow)
{
    if (NULL != (hMutex = CreateMutex(NULL, FALSE, NULL)))
    {
        return TRUE;
    }
    else if (dothrow)
    {
        panic("Unable to create mutex");
        never_returns_bool_t;
    }
    else
    {
        return FALSE;
    }
}

void smp_mutex_impl::set_criticality(sim_lock_criticality_t criticality)
{
    this->criticality = criticality;
}

void smp_mutex_impl::lock()
{
    if (criticality != SIM_LOCK_CRITICALITY_NONE)
        critical_lock(criticality);

    if (WAIT_OBJECT_0 != WaitForSingleObject(hMutex, INFINITE))
        panic("Unable to acquire mutex");
}

void smp_mutex_impl::unlock()
{
    if (! ReleaseMutex(hMutex))
        panic("Unable to release mutex");

    if (criticality != SIM_LOCK_CRITICALITY_NONE)
        critical_unlock(criticality);
}

t_bool smp_mutex_impl::trylock()
{
    if (criticality != SIM_LOCK_CRITICALITY_NONE)
        critical_lock(criticality);

    switch (WaitForSingleObject(hMutex, 0))
    {
    case WAIT_OBJECT_0:
        return TRUE;

    case WAIT_TIMEOUT:
        if (criticality != SIM_LOCK_CRITICALITY_NONE)
            critical_unlock(criticality);
        return FALSE;

    default:
        panic("Unable to acquire mutex");
        never_returns_bool_t;
    }
}

/**********************  Windows -- smp_condvar  **********************/

smp_condvar_impl::smp_condvar_impl()
{
    hEvent = NULL;
}

smp_condvar_impl::~smp_condvar_impl()
{
    if (hEvent)
        CloseHandle(hEvent);
}

t_bool smp_condvar_impl::init(t_bool dothrow)
{
    if (NULL != (CreateEvent(NULL, TRUE, FALSE, NULL)))
    {
        return TRUE;
    }
    else if (dothrow)
    {
        panic("Unable to create condition variable");
        never_returns_bool_t;
    }
    else
    {
        return FALSE;
    }
}

void smp_condvar_impl::prepareForWait()
{
    if (! ResetEvent(hEvent))
        panic("Unable to reset condition variable");
}

void smp_condvar_impl::wait(smp_mutex* mutex, t_bool reacquire_mutex)
{
    smp_mutex_impl* mi = (smp_mutex_impl*) mutex;
    if (WAIT_OBJECT_0 != SignalObjectAndWait(mi->hMutex, hEvent, INFINITE, FALSE))
        panic("Unable to wait on condition variable");
    if (reacquire_mutex)
        mutex->lock();
}

void smp_condvar_impl::signal(smp_mutex* mutex)
{
    if (! SetEvent(hEvent))
        panic("Unable to signal condition variable");
}

/**********************  Windows -- smp_event  **********************/

smp_event_impl::smp_event_impl()
{
    delta_timer = NULL;
    hEvent = NULL;
}

smp_event_impl::~smp_event_impl()
{
    if (delta_timer)
        delete delta_timer;
    if (hEvent)
        CloseHandle(hEvent);
}

t_bool smp_event_impl::init(t_bool dothrow)
{
    if (NULL != (delta_timer = sim_delta_timer::create(dothrow)) &&
        NULL != (hEvent = CreateEvent(NULL, TRUE, FALSE, NULL)))
    {
        return TRUE;
    }
    else if (dothrow)
    {
        panic("Unable to create event variable");
        never_returns_bool_t;
    }
    else
    {
        return FALSE;
    }
}

void smp_event_impl::set()
{
    SetEvent(hEvent);
}

void smp_event_impl::clear()
{
    ResetEvent(hEvent);
}

void smp_event_impl::wait()
{
    switch (WaitForSingleObject(hEvent, INFINITE))
    {
    case WAIT_OBJECT_0:
        return;
    default:
        panic("Unexpected failure waiting for event variable");
    }
}

t_bool smp_event_impl::trywait()
{
    switch (WaitForSingleObject(hEvent, 0))
    {
    case WAIT_OBJECT_0:
        return TRUE;
    case WAIT_TIMEOUT:
        return FALSE;
    default:
        panic("Unexpected failure waiting for event variable");
        never_returns_bool_t;
    }
}

t_bool smp_event_impl::timed_wait(uint32 usec, uint32* p_actual_usec)
{
    t_bool res;

    RUN_SCOPE_RSCX;

    if (rscx->thread_type != SIM_THREAD_TYPE_CPU)
        cpu_unit = NULL;

    if (p_actual_usec)
        delta_timer->begin(RUN_PASS);

    uint32 msec = (usec + 500) / 1000;
    switch (WaitForSingleObject(hEvent, msec))
    {
    case WAIT_OBJECT_0:
        res = TRUE;
        break;
    case WAIT_TIMEOUT:
        res = FALSE;
        break;
    default:
        panic("Unexpected failure waiting for event variable");
        never_returns_bool_t;
    }

    if (p_actual_usec)
    {
        delta_timer->sample(RUN_PASS);
        *p_actual_usec = delta_timer->us_since_start(RUN_PASS);
    }

    return res;
}

void smp_event_impl::wait_and_clear()
{
    wait();
    clear();
}

/**********************  Windows -- run_scope_context  **********************/

void run_scope_context::set_current()
{
    TlsSetValue(run_scope_key, this);
}

void run_scope_context::set_current(run_scope_context* rscx)
{
    TlsSetValue(run_scope_key, rscx);
}

run_scope_context* run_scope_context::get_current()
{
    return (run_scope_context*) TlsGetValue(run_scope_key);
}

/**********************  Windows -- smp_get_cpu_count  **********************/

static int smp_get_cpu_count()
{
    SYSTEM_INFO sysinfo;
    GetSystemInfo(& sysinfo);
    return (int) sysinfo.dwNumberOfProcessors;
}

/**********************  Windows -- smp_init_info  **********************/

static int smp_ncores = 0;
static DWORD_PTR smp_core_cpu_mask = 0;
static DWORD_PTR smp_all_cpu_mask = 0;
static int smp_smt_per_core = -1;

/*
 * Gather host system multiprocessor configuration.
 *
 * ToDo: may later want to switch to Portable Hardware Locality (hwloc) library.
 *       http://www.open-mpi.org/projects/hwloc
 */
static t_bool smp_init_info()
{
    PSYSTEM_LOGICAL_PROCESSOR_INFORMATION pBuffer = NULL;
    DWORD length;

    /* reset data to the "unknown" state */
    smp_ncores = 0;
    smp_core_cpu_mask = 0;
    smp_smt_per_core = -1;
    smp_all_cpu_mask = 0xFFFFFFFF;
    if (sizeof(smp_all_cpu_mask) > 32)
    {
        /* do it weird way to relax false compiler warning */
        smp_all_cpu_mask = smp_all_cpu_mask << 16;
        smp_all_cpu_mask = (smp_all_cpu_mask << 16) | 0xFFFFFFFF;
    }

    /* keep enlarging buffer until processor information fits into it */
    for (int k = 32 ;; k = k * 2)
    {
        size_t size = sizeof(SYSTEM_LOGICAL_PROCESSOR_INFORMATION) * k;
        length = (DWORD) size;
        pBuffer = (PSYSTEM_LOGICAL_PROCESSOR_INFORMATION) malloc(size);
        if (pBuffer == NULL)
        {
            return FALSE;
        }

        if (GetLogicalProcessorInformation(pBuffer, & length))
        {
            break;
        }
        else
        {
            free(pBuffer);
            pBuffer = NULL;
            if (GetLastError() != ERROR_INSUFFICIENT_BUFFER)
            {
                return FALSE;
            }
        }
    }

    /* parse retrieved data */
    int max_per_core = 0;
    int nentries = (int) (length / sizeof(SYSTEM_LOGICAL_PROCESSOR_INFORMATION));

    for (int k = 0;  k < nentries;  k++)
    {
        PSYSTEM_LOGICAL_PROCESSOR_INFORMATION p = pBuffer + k;
        if (p->Relationship == RelationProcessorCore && p->ProcessorCore.Flags == 1)
        {
            /* per-core record */
            int masksize = sizeof(p->ProcessorMask) * 8;
            ULONG_PTR mask = 1;
            int per_core = 0;
            t_bool found_core_cpu = FALSE;

            for (int i = 0;  i < masksize;  i++)
            {
                if (p->ProcessorMask & mask)
                {
                    per_core++;

                    if (! found_core_cpu)
                    {
                        found_core_cpu = TRUE;
                        smp_ncores++;
                        smp_core_cpu_mask |= mask;
                    }
                }
                mask = mask << 1;
            }

            if (per_core > max_per_core)
                max_per_core = per_core;
        }
    }

    free(pBuffer);

    smp_smt_per_core = max_per_core;

    return TRUE;
}

/**********************  Windows -- smp_get_smt_per_core  **********************/

/*
 * get number of SMT/HyperThreading processor units per core, 
 * in case of error return -1
 */
static int smp_get_smt_per_core()
{
    return smp_smt_per_core;
}

/**********************  Windows -- smp_set_affinity  **********************/

t_bool smp_can_alloc_per_core(int nthreads)
{
    return smp_core_cpu_mask && smp_ncores >= nthreads;
}

void smp_set_affinity(smp_thread_t thread_th, smp_affinity_kind_t how)
{
    if (how == SMP_AFFINITY_PER_CORE)
    {
        SetThreadAffinityMask(GetCurrentThread(), smp_core_cpu_mask);
    }
    else
    {
        SetThreadAffinityMask(GetCurrentThread(), smp_all_cpu_mask);
    }
}

/**********************  Windows -- smp_wait_xxx  **********************/

int smp_wait(smp_pollable_synch_object* object, int ms)
{
    return smp_wait_any(&object, 1, ms);
}

int smp_wait_any(smp_pollable_synch_object** objects, int nobjects, int ms)
{
    if (nobjects < 0)
        return -1;

    if (nobjects == 0)
        return 0;

    HANDLE* handles;
    HANDLE hh[64];

    if (nobjects <= 64)
    {
        handles = hh;
    }
    else
    {
        handles = (HANDLE*) _alloca(sizeof(HANDLE) * nobjects);
        if (handles == NULL)  return -1;
    }

    for (int k = 0;  k < nobjects;  k++)
        handles[k] = objects[k]->pollable_handle();

    DWORD dwMilliseconds = (ms < 0) ? INFINITE : (DWORD) ms;
    DWORD dwRes = WaitForMultipleObjects((DWORD) nobjects, handles, FALSE, dwMilliseconds);

    if (dwRes == WAIT_TIMEOUT)
        return 0;

    if (dwRes >= WAIT_OBJECT_0 && dwRes <= WAIT_OBJECT_0 + (DWORD) (nobjects - 1))
        return (int) (dwRes - WAIT_OBJECT_0 + 1);

    if (dwRes >= WAIT_ABANDONED_0 && dwRes <= WAIT_ABANDONED_0 + (DWORD) (nobjects - 1))
        return (int) (dwRes - WAIT_OBJECT_0 + 1);

    return -1;
}

/**********************  Windows -- debug helper  **********************/

void debug_out(const char* x)
{
    OutputDebugStringA(x);
}

#endif

/* =========================================  Linux/OSX OS level semaphores  ========================================= */

/*
 * OS X allows only named semaphores.
 * sem_init and sem_destroy are declared in header files but return error "Function is not implemented".
 */
#if defined(__linux)
int os_sem_init(sem_t** ppsem, sem_t* holder, unsigned int initial_value)
{
    int rc = sem_init(holder, 0, initial_value);
    *ppsem = (rc == 0) ? holder : NULL;
    return rc;
}

int os_sem_destroy(sem_t** ppsem, sem_t* holder)
{
    int rc = 0;
    if (*ppsem)
    {
        rc = sem_destroy(*ppsem);
        if (rc == 0)  *ppsem = NULL;
    }
    return rc;
}
#elif defined(__APPLE__)
int os_sem_init(sem_t** ppsem, char* name, unsigned int initial_value)
{
    /* OS X limits semaphore name to 30 characters max */
    static smp_interlocked_uint32 seq = 0;
    sprintf(name, "SIMH_%08X_%08X_%07X", (int) geteuid(), (int) getpid(), smp_interlocked_increment(&seq));

    (void) sem_unlink(name);  /* delete persistent semaphore e.g. after system crash-reboot */
    sem_t* sem = sem_open(name, O_CREAT|O_EXCL, S_IRUSR|S_IWUSR, initial_value);

    if (sem == SEM_FAILED)
    {
        perror("\nSIMH error: sem_open faled");
        *ppsem = NULL;
        return -1;
    }
    else
    {
        *ppsem = sem;
        return 0;
    }
}

int os_sem_destroy(sem_t** ppsem, char* name)
{
    int rc = 0;
    if (*ppsem)
    {
        if (name[0])
        {
            (void) sem_unlink(name);
            name[0] = 0;
        }
        rc = sem_close(*ppsem);
    }
    return rc;
}
#endif

/* ==========================================  Linux/OSX all architectures  ========================================== */

#if defined(__linux) || defined(__APPLE__)

static int rest_setpriority(int which, int who, int prio)
{
    DECL_RESTARTABLE(rc);
    DO_RESTARTABLE(rc, setpriority(which, who, prio));
    return rc;
}

static int rest_setrlimit(int resource, const struct rlimit *rlim)
{
    DECL_RESTARTABLE(rc);
    DO_RESTARTABLE(rc, setrlimit(resource, rlim));
    return rc;
}

static int rest_pthread_create(pthread_t *thread_id, const pthread_attr_t *attr, void *(*start_routine) (void *), void *arg)
{
    DECL_RESTARTABLE(rc);
    DO_RVAL_RESTARTABLE(rc, pthread_create(thread_id, attr, start_routine, arg));
    return rc;
}

static int rest_pthread_join(pthread_t thread_id, void **retval)
{
    DECL_RESTARTABLE(rc);
    DO_RVAL_RESTARTABLE(rc, pthread_join(thread_id, retval));
    return rc;
}

static int rest_pthread_mutex_init(pthread_mutex_t* __restrict mutex, const pthread_mutexattr_t* __restrict attr)
{
    DECL_RESTARTABLE(rc);
    DO_RVAL_RESTARTABLE(rc, pthread_mutex_init(mutex, attr));
    return rc;
}

static int rest_pthread_mutex_lock(pthread_mutex_t *mutex)
{
    DECL_RESTARTABLE(rc);
    DO_RVAL_RESTARTABLE(rc, pthread_mutex_lock(mutex));
    return rc;
}

static int rest_pthread_mutex_trylock(pthread_mutex_t *mutex)
{
    DECL_RESTARTABLE(rc);
    DO_RVAL_RESTARTABLE(rc, pthread_mutex_trylock(mutex));
    return rc;
}

static int rest_pthread_mutex_unlock(pthread_mutex_t *mutex)
{
    DECL_RESTARTABLE(rc);
    DO_RVAL_RESTARTABLE(rc, pthread_mutex_unlock(mutex));
    return rc;
}

static int rest_pthread_setschedparam(pthread_t thread_id, int policy, const struct sched_param *param)
{
    DECL_RESTARTABLE(rc);
    DO_RVAL_RESTARTABLE(rc, pthread_setschedparam(thread_id, policy, param));
    return rc;
}

static int rest_pthread_getschedparam(pthread_t thread_id, int *policy, struct sched_param *param)
{
    DECL_RESTARTABLE(rc);
    DO_RVAL_RESTARTABLE(rc, pthread_getschedparam(thread_id, policy, param));
    return rc;
}
                      
static int rest_pthread_cond_init(pthread_cond_t* __restrict cond, const pthread_condattr_t* __restrict attr)
{
    DECL_RESTARTABLE(rc);
    DO_RVAL_RESTARTABLE(rc, pthread_cond_init(cond, attr));
    return rc;
}

static int rest_pthread_cond_wait(pthread_cond_t* __restrict cond, pthread_mutex_t* __restrict mutex)
{
    DECL_RESTARTABLE(rc);
    DO_RVAL_RESTARTABLE(rc, pthread_cond_wait(cond, mutex));
    return rc;
}

static int rest_pthread_cond_timedwait(pthread_cond_t* __restrict cond, pthread_mutex_t* __restrict mutex, const struct timespec* __restrict abstime)
{
    DECL_RESTARTABLE(rc);
    DO_RVAL_RESTARTABLE(rc, pthread_cond_timedwait(cond, mutex, abstime));
    return rc;
}

static int rest_pthread_cond_signal(pthread_cond_t *cond)
{
    DECL_RESTARTABLE(rc);
    DO_RVAL_RESTARTABLE(rc, pthread_cond_signal(cond));
    return rc;
}

static int rest_pthread_cond_broadcast(pthread_cond_t *cond)
{
    DECL_RESTARTABLE(rc);
    DO_RVAL_RESTARTABLE(rc, pthread_cond_broadcast(cond));
    return rc;
}

/**********************  Linux/OSX -- create thread  **********************/

static SMP_THREAD_ROUTINE_DECL smp_create_thread_starter(void* pxargs);

t_bool smp_create_thread(smp_thread_routine_t start_routine, void* arg, smp_thread_t* thread_handle, t_bool dothrow)
{
    smp_event* event = NULL;
    smp_event* event_ack = NULL;
    void* xargs[4];
    int rc;

    if (thread_handle)
        *thread_handle = SMP_THREAD_NULL;

    if (sizeof(smp_thread_t) != sizeof(pthread_t))
        panic("Internal error: smp_thread_t misdeclared");

    pthread_t thrd;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setscope(&attr, PTHREAD_SCOPE_SYSTEM);
    if (thread_handle)
    {
        event = smp_event::create();
        event->clear();
        event_ack = smp_event::create();
        event_ack->clear();
        xargs[0] = (void*) start_routine;
        xargs[1] = arg;
        xargs[2] = event;
        xargs[3] = event_ack;
        rc = rest_pthread_create(&thrd, &attr, smp_create_thread_starter, xargs);
    }
    else
    {
        rc = rest_pthread_create(&thrd, &attr, start_routine, arg);
    }
    pthread_attr_destroy(&attr);

    if (rc)
    {
        if (dothrow)
            panic("Unable to create thread");
        return FALSE;
    }

    if (thread_handle)
    {
        *thread_handle = thrd;
        event_ack->wait();  
        event->set();
    }

    return TRUE;
}

static SMP_THREAD_ROUTINE_DECL smp_create_thread_starter(void* pxargs)
{
    void** xargs = (void**) pxargs;
    smp_thread_routine_t start_routine = (smp_thread_routine_t) xargs[0];
    void* arg = xargs[1];
    smp_event* event = (smp_event*) xargs[2];
    smp_event* event_ack = (smp_event*) xargs[3];
    event_ack->set();
    event->wait();
    delete event;
    delete event_ack;
    return (*start_routine)(arg);
}

/**********************  Linux/OSX -- wait thread  **********************/

t_bool smp_wait_thread(smp_thread_t thread_handle, t_value* exitcode, t_bool dothrow)
{
    void* rv = NULL;
    if (0 == rest_pthread_join(thread_handle, &rv))
    {
        if (exitcode)
            *exitcode = (t_value) rv;
        return TRUE;
    }

    if (dothrow)
        panic("Internal error: unable to complete worker thread");

    return FALSE;
}

/**********************  Linux/OSX -- smp_semaphore  **********************/

smp_semaphore_impl::smp_semaphore_impl()
{
    fd[0] = fd[1] = -1;
}

smp_semaphore_impl::~smp_semaphore_impl()
{
    DECL_RESTARTABLE(rc);
    if (fd[0] != -1)  DO_RESTARTABLE(rc, close(fd[0]));
    if (fd[1] != -1)  DO_RESTARTABLE(rc, close(fd[1]));
}

t_bool smp_semaphore_impl::init(int initial_open_count, t_bool dothrow)
{
    DECL_RESTARTABLE(rc);
    DO_RESTARTABLE(rc, pipe(fd));
    if (rc == 0)
    {
        smp_wmb();
        if (initial_open_count)
            release(initial_open_count);
        return TRUE;
    }
    else if (dothrow)
    {
        panic("Unable to initialize semaphore");
        never_returns_bool_t;
    }
    else
    {
        return FALSE;
    }
}

smp_pollable_handle_t smp_semaphore_impl::pollable_handle()
{
    return fd[PIPE_RD];
}

const char* smp_semaphore_impl::pollable_handle_op()
{
    return "R";
}

void smp_semaphore_impl::release(int count)
{
    DECL_RESTARTABLE(rc);
    while (count)
    {
        char buf[256];
        size_t wrsz = ((size_t) count < sizeof(buf)) ? (size_t) count : sizeof(buf);
        DO_RESTARTABLE(rc, write(fd[PIPE_WR], buf, wrsz));
        if (rc == -1)
            panic("Unable to release semaphore");
        count -= rc;
    }
}

void smp_semaphore_impl::clear()
{
    DECL_RESTARTABLE(rc);
    int flags;

    DO_RESTARTABLE(rc, fcntl(fd[PIPE_RD], F_GETFL));
    if (rc == -1)
        panic("Unable to clear semaphore");
    flags = rc;

    DO_RESTARTABLE(rc, fcntl(fd[PIPE_RD], F_SETFL, flags | O_NONBLOCK));
    if (rc == -1)
        panic("Unable to clear semaphore");

    for (;;)
    {
        char buf[256];
        size_t rdsz = sizeof(buf);
        DO_RESTARTABLE(rc, read(fd[PIPE_RD], buf, rdsz));
        if (rc == 0)  break;
        if (rc == -1 && errno == EWOULDBLOCK)  break;
        if (rc == -1)
            panic("Unable to clear semaphore");
    }

    DO_RESTARTABLE(rc, fcntl(fd[PIPE_RD], F_SETFL, flags & ~O_NONBLOCK));
    if (rc == -1)
        panic("Unable to clear semaphore");
}

void smp_semaphore_impl::wait()
{
    DECL_RESTARTABLE(rc);
    for (;;)
    {
        char c;
        DO_RESTARTABLE(rc, read(fd[PIPE_RD], &c, 1));
        if (rc == 0)  continue;  /* spurious */
        if (rc == 1)  break;
        panic("Unable to acquire semaphore");
    }
}

t_bool smp_semaphore_impl::trywait()
{
    panic("Unimplemented: semaphore.trywait");
    never_returns_bool_t;
}

/******************  Linux/OSX -- smp_simple_semaphore  ******************/

smp_simple_semaphore* smp_simple_semaphore::create(int initial_open_count, t_bool dothrow)
{
    smp_simple_semaphore_impl* sem = new smp_simple_semaphore_impl();
    if (! sem->init(initial_open_count, dothrow))
    {
        delete sem;
        sem = NULL;
    }
    return sem;
}

smp_simple_semaphore_impl::smp_simple_semaphore_impl()
{
    inited = FALSE;
    os_sem_decl_init(semaphore);
}

smp_simple_semaphore_impl::~smp_simple_semaphore_impl()
{
    if (inited)
       os_sem_destroy(os_sem_ptr(semaphore));
}

t_bool smp_simple_semaphore_impl::init(int initial_open_count, t_bool dothrow)
{
    if (! inited)
    {
        DECL_RESTARTABLE(rc);
        DO_RESTARTABLE(rc, os_sem_init(os_sem_ptr(semaphore), initial_open_count));
        if (rc == -1)  goto cleanup;
        smp_wmb();
        inited = TRUE;
    }

cleanup:

    if (! inited && dothrow)
        panic("Unable to initialize semaphore");

    return inited;
}

void smp_simple_semaphore_impl::clear()
{
    while (trywait()) ;
}

void smp_simple_semaphore_impl::wait()
{
    DECL_RESTARTABLE(rc);
    DO_RESTARTABLE(rc, sem_wait(semaphore));
    if (rc != 0)  panic("Semaphore error");
}

t_bool smp_simple_semaphore_impl::trywait()
{
    DECL_RESTARTABLE(rc);
    DO_RESTARTABLE(rc, sem_trywait(semaphore));
    if (rc == -1 && errno == EAGAIN)  return FALSE;
    if (rc != 0)  panic("Semaphore error");
    return TRUE;
}

void smp_simple_semaphore_impl::release(int count)
{
    while (count-- > 0)
    {
        DECL_RESTARTABLE(rc);
        DO_RESTARTABLE(rc, sem_post(semaphore));
        if (rc != 0)  panic("Semaphore error");
    }
}

/**********************  Linux/OSX -- smp_barrier  **********************/

smp_barrier_impl::smp_barrier_impl()
{
    mutex_inited = FALSE;
    for (int k = 0;  k < SMP_BARRIER_CYCLES;  k++)
        cond_inited[k] = FALSE;
    wait_index = 0;
    barrier_count = 0;
    waiting_count = 0;
    wseq = 0;
}

smp_barrier_impl::~smp_barrier_impl()
{
    dealloc();
}

void smp_barrier_impl::dealloc()
{
    if (mutex_inited)
    {
        pthread_mutex_destroy(& mutex);
        mutex_inited = FALSE;
    }

    for (int k = 0;  k < SMP_BARRIER_CYCLES;  k++)
    {
        if (cond_inited[k])
            pthread_cond_destroy(& cond[k]);
        cond_inited[k] = FALSE;
    }
}

t_bool smp_barrier_impl::init(int initial_count, t_bool dothrow)
{
    barrier_count = initial_count;

    t_bool ok = TRUE;

    if (0 == rest_pthread_mutex_init(& mutex, NULL))
        mutex_inited = TRUE;
    else
        ok = FALSE;

    for (int k = 0;  k < SMP_BARRIER_CYCLES;  k++)
    {
        if (0 == rest_pthread_cond_init(& cond[k], NULL))
            cond_inited[k] = TRUE;
        else
            ok = FALSE;
    }

    if (ok)
    {
        smp_wmb();
        return TRUE;
    }

    dealloc();
    smp_wmb();

    if (dothrow)
    {
        panic("Unable to initialize barrier");
        never_returns_bool_t;
    }
    else
    {
        return FALSE;
    }
}

void smp_barrier_impl::wait()
{
    if (rest_pthread_mutex_lock(& mutex))
        panic("Unable to acquire barrier mutex");

    if (++waiting_count >= barrier_count)
    {
        waiting_count = 0;
        wseq++;
        if (rest_pthread_cond_broadcast(& cond[wait_index]))
            panic("Unable to wakeup thread waiting on the barrier");
        wait_index = (wait_index + 1) % SMP_BARRIER_CYCLES;
        rest_pthread_mutex_unlock(& mutex);
    }
    else
    {
        /* loop for spurious wakeups */
        uint32 sv_wseq = wseq;
        do
        {
            if (rest_pthread_cond_wait(& cond[wait_index], & mutex))
                panic("Unable to wait on the barrier");
        }
        while (sv_wseq == wseq);
        /*
         * Is not pthreads design great? Not only it is impossible to change pthread_barrier height,
         * but when pthread_cond_broadcast signals condition variable, all waiting threads are made to cram
         * to reacquire the mutex, whether they need it or not, just so they could release it right away,
         * and there is no way to specifiy the intent to the contrary. Two pieces of cake for the price of one package.
         * Good it is not a performance-critical use in VAX MP.
         */
        rest_pthread_mutex_unlock(& mutex);
    }
}

t_bool smp_barrier_impl::trywait()
{
    if (rest_pthread_mutex_lock(& mutex))
        panic("Unable to acquire barrier mutex");

    if (waiting_count >= barrier_count - 1)
    {
        waiting_count = 0;
        if (rest_pthread_cond_broadcast(& cond[wait_index]))
            panic("Unable to wakeup thread waiting on the barrier");
        wait_index = (wait_index + 1) % SMP_BARRIER_CYCLES;
        rest_pthread_mutex_unlock(& mutex);
        return TRUE;
    }
    else
    {
        rest_pthread_mutex_unlock(& mutex);
        return FALSE;
    }
}

t_bool smp_barrier_impl::set_count(int count, t_bool dothrow)
{
    if (rest_pthread_mutex_lock(& mutex))
        panic("Unable to acquire barrier mutex");

    barrier_count = count;

    if (waiting_count >= barrier_count)
    {
        waiting_count = 0;
        wseq++;
        if (rest_pthread_cond_broadcast(& cond[wait_index]))
            panic("Unable to wakeup thread waiting on the barrier");
        wait_index = (wait_index + 1) % SMP_BARRIER_CYCLES;
    }

    rest_pthread_mutex_unlock(& mutex);
    return TRUE;
}

void smp_barrier_impl::clear()
{
    panic("Operation undefined: barrier.clear");
}

void smp_barrier_impl::release(int count)
{
    panic("Operation undefined: barrier.release");
}

/**********************  Linux/OSX -- smp_mutex  **********************/

smp_mutex_impl::smp_mutex_impl()
{
    criticality = SIM_LOCK_CRITICALITY_NONE;
    inited = FALSE;
}

smp_mutex_impl::~smp_mutex_impl()
{
    if (inited)
        pthread_mutex_destroy(& mutex);
}

t_bool smp_mutex_impl::init(t_bool dothrow)
{
    t_bool ok = TRUE;
    t_bool attr_inited = FALSE;
    pthread_mutexattr_t attr;

    if (ok && 0 == pthread_mutexattr_init(& attr))
        attr_inited = TRUE;
    else
        ok = FALSE;

    if (ok && 0 != pthread_mutexattr_settype(& attr, PTHREAD_MUTEX_RECURSIVE))
        ok = FALSE;

    if (ok && 0 != rest_pthread_mutex_init(& mutex, & attr))
        ok = FALSE;

    if (attr_inited)
        pthread_mutexattr_destroy(& attr);

    if (ok)
    {
        inited = TRUE;
        return TRUE;
    }
    else if (dothrow)
    {
        panic("Unable to create mutex");
        never_returns_bool_t;
    }
    else
    {
        return FALSE;
    }
}

void smp_mutex_impl::set_criticality(sim_lock_criticality_t criticality)
{
    this->criticality = criticality;
}

void smp_mutex_impl::lock()
{
    if (criticality != SIM_LOCK_CRITICALITY_NONE)
        critical_lock(criticality);

    if (pthread_mutex_lock(& mutex))
        panic("Unable to acquire mutex");
}

void smp_mutex_impl::unlock()
{
    if (pthread_mutex_unlock(& mutex))
        panic("Unable to acquire mutex");

    if (criticality != SIM_LOCK_CRITICALITY_NONE)
        critical_unlock(criticality);
}

t_bool smp_mutex_impl::trylock()
{
    if (criticality != SIM_LOCK_CRITICALITY_NONE)
        critical_lock(criticality);

    switch (rest_pthread_mutex_trylock(& mutex))
    {
    case 0:
        return TRUE;

    case EBUSY:
        if (criticality != SIM_LOCK_CRITICALITY_NONE)
            critical_unlock(criticality);
        return FALSE;

    default:
        panic("Unable to acquire mutex");
        never_returns_bool_t;
    }
}

/**********************  Linux/OSX -- smp_condvar  **********************/

smp_condvar_impl::smp_condvar_impl()
{
    wseq = 0;
    inited = FALSE;
}

smp_condvar_impl::~smp_condvar_impl()
{
    if (inited)
        pthread_cond_destroy(& cond);
}

t_bool smp_condvar_impl::init(t_bool dothrow)
{
    if (0 == rest_pthread_cond_init(& cond, NULL))
    {
        return TRUE;
    }
    else if (dothrow)
    {
        panic("Unable to create condition variable");
        never_returns_bool_t;
    }
    else
    {
        return FALSE;
    }
}

void smp_condvar_impl::prepareForWait()
{
    /* nothing to do: pthreads condition variables are edge-triggered */
}

void smp_condvar_impl::wait(smp_mutex* mutex, t_bool reacquire_mutex)
{
    smp_mutex_impl* mi = (smp_mutex_impl*) mutex;
    uint32 sv_wseq = wseq;

    /* loop for spurious wakeups */
    do
    {
        if (rest_pthread_cond_wait(& cond, & mi->mutex))
            panic("Unable to wait on condition variable");
    }
    while (sv_wseq = wseq);

    if (! reacquire_mutex)
        mutex->unlock();
}

void smp_condvar_impl::signal(smp_mutex* mutex)
{
    if (mutex) mutex->lock();

    wseq++;
    if (rest_pthread_cond_signal(& cond))
    {
        if (mutex) mutex->unlock();
        panic("Unable to signal condition variable");
    }

    if (mutex) mutex->unlock();
}

/**********************  Linux/OSX -- smp_event  **********************/

smp_event_impl::smp_event_impl()
{
    inited = FALSE;
    state = FALSE;
    set_wseq = 0;
}

smp_event_impl::~smp_event_impl()
{
    if (inited)
    {
        pthread_cond_destroy(& cond);
        pthread_mutex_destroy(& mutex);
    }
}

t_bool smp_event_impl::init(t_bool dothrow)
{
    t_bool inited_mutex = FALSE;
    pthread_condattr_t* p_cond_attr = NULL;
    pthread_condattr_t cond_attr;

    state = FALSE;

    if (0 == rest_pthread_mutex_init(& mutex, NULL))
        inited_mutex = TRUE;

#if defined(HAVE_POSIX_CLOCK_ID)
    if (sim_posix_have_clock_id)
    {
        if (pthread_condattr_init(& cond_attr))
            goto cleanup;
        p_cond_attr = & cond_attr;
        if (pthread_condattr_setclock(p_cond_attr, sim_posix_clock_id))
        {
            pthread_condattr_destroy(p_cond_attr);
            p_cond_attr = NULL;
        }
    }
#endif

    if (0 == rest_pthread_cond_init(& cond, p_cond_attr))
    {
        if (p_cond_attr)
            pthread_condattr_destroy(p_cond_attr);
        inited = TRUE;
        return TRUE;
    }

#if defined(HAVE_POSIX_CLOCK_ID)
cleanup:
#endif

    if (p_cond_attr)
        pthread_condattr_destroy(p_cond_attr);

    if (inited_mutex)
        pthread_mutex_destroy(& mutex);

    if (dothrow)
    {
        panic("Unable to create event object");
        never_returns_bool_t;
    }
    else
    {
        return FALSE;
    }
}

void smp_event_impl::set()
{
    if (rest_pthread_mutex_lock(& mutex))
        panic("Unable to acquire mutex");

    state = TRUE;
    set_wseq++;

    if (rest_pthread_cond_broadcast(& cond))
    {
        rest_pthread_mutex_unlock(& mutex);
        panic("Unable to signal condition variable");
    }

    if (rest_pthread_mutex_unlock(& mutex))
        panic("Unable to acquire mutex");
}

void smp_event_impl::clear()
{
    if (rest_pthread_mutex_lock(& mutex))
        panic("Unable to acquire mutex");

    state = FALSE;

    if (rest_pthread_mutex_unlock(& mutex))
        panic("Unable to acquire mutex");
}

void smp_event_impl::wait()
{
    if (rest_pthread_mutex_lock(& mutex))
        panic("Unable to acquire mutex");

    uint32 start_wseq = set_wseq;

    while (state == FALSE && set_wseq == start_wseq)
    {
        /*
         * Wakeups can be spurious, per pthreads specification. 
         */
        if (rest_pthread_cond_wait(& cond, & mutex))
            panic("Unable to wait on condition variable");
    }

    if (rest_pthread_mutex_unlock(& mutex))
        panic("Unable to acquire mutex");
}

t_bool smp_event_impl::trywait()
{
    if (rest_pthread_mutex_lock(& mutex))
        panic("Unable to acquire mutex");

    t_bool res = state;

    if (rest_pthread_mutex_unlock(& mutex))
        panic("Unable to acquire mutex");

    return res;
}

static void get_now(struct timespec* now)
{
#if defined(HAVE_POSIX_CLOCK_ID)
    if (sim_posix_have_clock_id)
    {
        clock_gettime(sim_posix_clock_id, now);
        return;
    }
#endif

    /*
     * Note: On current versions of OS X gettimeofday result is computed based on combination
     *       of real-time clock data and RDTSC counter progress accumulated since the clock's
     *       recent tick and provides progress resolution down to 1 usec.
     *
     *       For details see
     *       http://www.opensource.apple.com/source/Libc/Libc-763.13/x86_64/sys/i386_gettimeofday_asm.s
     *       http://www.opensource.apple.com/source/Libc/Libc-763.13/x86_64/sys/nanotime.s
     *       http://www.opensource.apple.com/source/xnu/xnu-1699.26.8/osfmk/i386/commpage/commpage.h
     *       http://www.opensource.apple.com/source/xnu/xnu-1699.26.8/osfmk/i386/commpage/commpage.c
     *       http://www.opensource.apple.com/source/xnu/xnu-1699.26.8/osfmk/i386/rtclock.c
     *       http://www.opensource.apple.com/source/xnu/xnu-1699.26.8/osfmk/x86_64/pal_routines_asm.s
     *       http://www.opensource.apple.com/source/xnu/xnu-1699.26.8/osfmk/x86_64/machine_routines_asm.s
     *
     *       Note that TSC base data is currently kept in per-system area of commpage, rather than
     *       in per-processor area. As Intel processors with invariant TSC capability appear to have
     *       TSC reading synchrnonized across the cores (google: tsc across cores, e.g.
     *       http://software.intel.com/en-us/forums/showthread.php?t=77730), this should not be
     *       a problem with current Macs that use such processors and use only one socket.
     *
     *       If Macs were to ever go multi-socket, they could still maintain composite RTC/TSC timer
     *       capability by extending comm area with per-processor pages, keeping time data in per-processor
     *       area and updating thread cpu migration count in thread's context every time the thread
     *       migrates across the processors, so read-time logics can detect migration in the middle of
     *       the function call and redo data fetching.
     *       
     *       For now, the best we can do anyway is a general sanity check for time jumping backwards.
     *       
     */

    struct timeval tv;
    gettimeofday(& tv, NULL);
    now->tv_sec = tv.tv_sec;
    now->tv_nsec = tv.tv_usec * 1000;
}

t_bool smp_event_impl::timed_wait(uint32 usec, uint32* p_actual_usec)
{
    struct timespec start;
    struct timespec target;
    t_bool res = TRUE;

    static const long million = 1000 * 1000;
    static const long billion = 1000 * 1000 * 1000;

    /* get current time */
    get_now(& start);

    /* calculate wait end absolute time */
    target = start;
    target.tv_sec += usec / million;
    target.tv_nsec += (usec % million) * 1000;
    target.tv_sec += target.tv_nsec / billion;
    target.tv_nsec = target.tv_nsec % billion;

    if (rest_pthread_mutex_lock(& mutex))
        panic("Unable to acquire mutex");

    uint32 start_wseq = set_wseq;

    while (state == FALSE && set_wseq == start_wseq)
    {
        /*
         * Wakeups can be spurious, per pthreads specification. 
         */
        int tw = rest_pthread_cond_timedwait(& cond, & mutex, & target);
        if (tw == ETIMEDOUT)
        {
            if (state == FALSE && set_wseq == start_wseq)
                res = FALSE;
            break;
        }
        else if (tw)
        {
            panic("Unable to wait on condition variable");
        }
    }

    if (pthread_mutex_unlock(& mutex))
        panic("Unable to acquire mutex");

    if (p_actual_usec)
    {
        struct timespec now;
        get_now(& now);

        double delta = 0;

        if (now.tv_sec >= start.tv_sec)
        {
            delta = (double) (now.tv_sec - start.tv_sec) * 1000 * 1000;
            delta += (double) (now.tv_nsec - start.tv_nsec) / 1000;
        }

        /* time jumped backwards? */
        if (delta < 0)  delta = 0;

        /* time jumped forward way too much? */
        const double delta_max = 0.9 * (double) UINT32_MAX;
        if (delta > 0.9 * delta_max)
            delta = delta_max;

        *p_actual_usec =  (uint32) delta;
    }

    return res;
}

void smp_event_impl::wait_and_clear()
{
    if (rest_pthread_mutex_lock(& mutex))
        panic("Unable to acquire mutex");

    uint32 start_wseq = set_wseq;

    while (state == FALSE && set_wseq == start_wseq)
    {
        /*
         * Wakeups can be spurious, per pthreads specification. 
         */
        if (rest_pthread_cond_wait(& cond, & mutex))
            panic("Unable to wait on condition variable");
    }

    state = FALSE;

    if (rest_pthread_mutex_unlock(& mutex))
        panic("Unable to acquire mutex");
}

/**********************  Linux/OSX -- run_scope_context  **********************/

void run_scope_context::set_current()
{
    pthread_setspecific(run_scope_key, this);
}

void run_scope_context::set_current(run_scope_context* rscx)
{
    pthread_setspecific(run_scope_key, rscx);
}

run_scope_context* run_scope_context::get_current()
{
    return (run_scope_context*) pthread_getspecific(run_scope_key);
}

/**********************  Linux/OSX -- smp_wait_xxx  **********************/

int smp_wait(smp_pollable_synch_object* object, int ms)
{
    return smp_wait_any(&object, 1, ms);
}

int smp_wait_any(smp_pollable_synch_object** objects, int nobjects, int ms)
{
    if (nobjects < 0)
        return -1;

    if (nobjects == 0)
        return 0;

    struct pollfd* handles;
    struct pollfd  hh[64];

    if (nobjects <= 64)
    {
        handles = hh;
    }
    else
    {
        handles = (struct pollfd*) alloca(sizeof(struct pollfd) * nobjects);
        if (handles == NULL)  return -1;
    }

    for (int k = 0;  k < nobjects;  k++)
    {
        handles[k].fd = objects[k]->pollable_handle();
        handles[k].events = POLLIN;
    }

    for (;;)
    {
        for (;;)
        {
            for (int k = 0;  k < nobjects;  k++)
                handles[k].revents = 0;
            int rc = poll(handles, nobjects, ms);
            if (rc == -1)
            {
                if (errno == EINTR)
                    continue;
                else
                    return -1;
            }
            else if (rc == 0)
            {
                return 0;
            }
            else
            {
                break;
            }
        }

        for (int k = 0;  k < nobjects;  k++)
        {
            if (handles[k].revents & POLLIN)
            {
                const char* op = objects[k]->pollable_handle_op();
                if (0 == strcmp(op, "r"))
                {
                    return k + 1;
                }
                else if (0 == strcmp(op, "R"))
                {
                    objects[k]->wait();
                    return k + 1;
                }
            }
        }
    }
}
#endif

/* ==========================================  Linux all architectures  ========================================== */

#if defined(__linux)

/**********************  Linux -- set thread priority **********************/

static int cpu_calibration_priority = -1;
static int cpu_kernel_critical_vm_priority = -1;
static int cpu_kernel_critical_os_hi_priority = -1;
static int cpu_kernel_critical_os_priority = -1;
static int cpu_clock_hi_priority = -1;
static int cpu_clock_lo_priority = -1;

static pid_t gettid(void)
{
    return syscall(__NR_gettid);
}

t_bool smp_set_thread_priority(sim_thread_priority_t prio)
{
    run_scope_context::get_current()->setting_priority(prio);
    return smp_set_thread_priority(pthread_self(), prio);
}

int smp_get_thread_os_priority(smp_thread_t thread_th)
{
    int policy = SCHED_OTHER;
    struct sched_param sc_param;
    if (0 != rest_pthread_getschedparam(thread_th, & policy, & sc_param))
        return -1;
    if (policy == SCHED_RR || policy == SCHED_FIFO)
        return sc_param.sched_priority;
    else
        return 0;
}

t_bool smp_set_thread_priority(smp_thread_t thread_th, sim_thread_priority_t prio)
{
    int priority, xnice = 0;
    int policy;

    switch (prio)
    {
    /* CPU thread -- regular instruction stream execution */
    case SIMH_THREAD_PRIORITY_CPU_RUN:
        policy = SCHED_OTHER;
        priority = 0;
        xnice = 10;
        break;

    /* console thread -- while CPUs are paused */
    case SIMH_THREAD_PRIORITY_CONSOLE_PAUSED:
        policy = SCHED_OTHER;
        priority = 0;
        xnice = 0;
        break;

    /* console thread -- while CPUs are running */
    case SIMH_THREAD_PRIORITY_CONSOLE_RUN:
        // policy = SCHED_OTHER;
        // priority = 0;
        // xnice = -10;
        /* make it the same as OS_HI, so console could preempt VCPU threads,
           could even make it higher than OS_HI */
        if (cpu_kernel_critical_os_hi_priority < 0)
            return FALSE;
        policy = SCHED_RR;
        priority = cpu_kernel_critical_os_hi_priority;
        break;

    /*
     * IO processing thread: make thread priority identical with CRITICAL_OS to avoid assigning criticality
     * setting to locks for IOP structures, so locking can be done without the overhead of elevating/demoting thread
     * priority, but at the same time also avoiding priority inversion when CPU's thread running at CRITIAL_OS
     * (a priority at which OS will typically access IO device) would have to wait for the lock held by IOP thread
     * executing at a lower priority and possibly preempted. Downside of this choice being that high IO rate may
     * also saturate host system computationally and take precedence over host UI enviroment both in terms of
     * high-priority computational load and IO requests queued at high-priority.
     */
    case SIMH_THREAD_PRIORITY_IOP:
        /* fall through to CRITICAL_OS */

    /* CPU thread -- kernel is in a critical section (holding spinlocks etc.) */
    case SIMH_THREAD_PRIORITY_CPU_CRITICAL_OS:
        if (cpu_kernel_critical_os_priority < 0)
            return FALSE;
        policy = SCHED_RR;
        priority = cpu_kernel_critical_os_priority;
        break;

    /* CPU thread -- kernel is processing CLK or IPI interrupt,
       or any of these interrupts is pending */
    case SIMH_THREAD_PRIORITY_CPU_CRITICAL_OS_HI:
        if (cpu_kernel_critical_os_hi_priority < 0)
            return FALSE;
        policy = SCHED_RR;
        priority = cpu_kernel_critical_os_hi_priority;
        break;

    /* CPU thread -- VM is in a critical section (holding lock etc.) */
    case SIMH_THREAD_PRIORITY_CPU_CRITICAL_VM:
        if (cpu_kernel_critical_vm_priority < 0)
            return FALSE;
        policy = SCHED_RR;  /* cannot be SCHED_OTHER, see below */
        priority = cpu_kernel_critical_vm_priority;
        break;

    /* CPU thread -- processor loops calibration using SSC clock */
    case SIMH_THREAD_PRIORITY_CPU_CALIBRATION:
        if (cpu_calibration_priority < 0)
            return FALSE;
        policy = SCHED_FIFO;
        priority = cpu_calibration_priority;
        break;

    /* clock strobing thread */
    case SIMH_THREAD_PRIORITY_CLOCK:
        if (sim_vsmp_active)
        {
            if (cpu_clock_hi_priority < 0)
                return FALSE;
            policy = SCHED_FIFO;
            priority = cpu_clock_hi_priority;
        }
        else
        {
            if (cpu_clock_lo_priority < 0)
                return FALSE;
            policy = SCHED_FIFO;
            priority = cpu_clock_lo_priority;
        }
        break;

    default:
        return FALSE;
    }

    struct sched_param sc_param;
    sc_param.sched_priority = priority;

    if (0 != rest_pthread_setschedparam(thread_th, policy, & sc_param))
        return FALSE;

    if (policy == SCHED_OTHER)
    {
        /*
         * The only VAX MP cross-thread invocations are for CRITICAL_VM, CRITICAL_OS and CRITICAL_OS_HI,
         * which are not SCHED_OTHER. All SCHED_OTHER priorities are always set for the current thread only.
         *
         * Note that we rely here on Linux NPTL (and LinuxThreads) feature non-compliant to
         * pthreads specification: NPTL threads (contrary to pthreads specification) not sharing nice value.
         *
         * Unfortunately, all SCHED_OTHER processes have sched priority of 0 and the only way to
         * tweak their real priority is via per-thread nice value (at least on Linux, can be different on
         * other systems). Neither portable nor semi-portable access to nice value are not exposed by pthreads
         * specification.
         * An old good case of "Unix is snake oil".
         */
        if (! pthread_equal(thread_th, pthread_self()))
            return FALSE;
        pid_t tid = gettid();
        if (rest_setpriority(PRIO_PROCESS, tid, xnice))
            return FALSE;
    }

    return TRUE;
}

static void smp_set_thread_priority_init()
{
    struct rlimit rlim;
    int prio, pmin, pmax;
    int pmin_fifo, pmax_fifo;

    // some pthreads implementatons may want to call
    // pthread_setconcurrency(...)

    if (0 == getrlimit(RLIMIT_RTPRIO, & rlim))
    {
        rlim.rlim_cur = rlim.rlim_max;
        setrlimit(RLIMIT_RTPRIO, & rlim); 
    }

    if (0 == getrlimit(RLIMIT_NICE, & rlim))
    {
        rlim.rlim_cur = rlim.rlim_max;
        setrlimit(RLIMIT_NICE, & rlim); 
    }
    
    pmin = sched_get_priority_min(SCHED_RR);
    pmax = sched_get_priority_max(SCHED_RR);
    if (pmin < 0 || pmax - pmin < 31)
    {
        /* range is below Unix specification */
        return;
    }

    pmin_fifo = sched_get_priority_min(SCHED_FIFO);
    pmax_fifo = sched_get_priority_max(SCHED_FIFO);
    if (pmin_fifo < 0 || pmax_fifo - pmin_fifo < 31)
    {
        /* range is below Unix specification */
        return;
    }

    if (pmin_fifo != pmin || pmax_fifo != pmax)
    {
        fprintf(stderr, "\n%s INTERNAL ERROR !!! SCHED_FIFO range (%d ... %d) is distinct from SCHED_RR range (%d ... %d)\n",
                sim_name, pmin_fifo, pmax_fifo, pmin, pmax);
        return;
    }

    /* 
     * calculare thread priorities starting with SIMH_THREAD_PRIORITY_CPU_CRITICAL_OS:
     * put it just above 20 (highest nice)
     */
    prio = 21;
    if (prio < pmin)  prio = pmin;
    if (prio > pmax)  return;
    cpu_kernel_critical_os_priority = prio;

    /*
     * assign other priorities, in this increasing order:
     *
     *    cpu_kernel_critical_os_priority
     *    cpu_kernel_critical_os_hi_priority
     *    cpu_kernel_critical_vm_priority
     *    cpu_clock_hi_priority = cpu_clock_lo_priority
     *    cpu_calibration_priority
     *
     */

    if (++prio > pmax)  return;
    cpu_kernel_critical_os_hi_priority = prio;

    if (++prio > pmax)  return;
    cpu_kernel_critical_vm_priority = prio;

    if (++prio > pmax)  return;
    cpu_clock_hi_priority = cpu_clock_lo_priority = prio;

    if (++prio > pmax)  return;
    cpu_calibration_priority = prio;

    /*
     * we have performed minmalist allocation
     *
     * now try to separate priority range from timesharing and also separate defined priorities
     * from each other, if possible, to force scheduler perform cross-CPU thread migration and
     * thus respect priority difference
     */

    if (prio + 4 <= pmax)
    {
        cpu_kernel_critical_os_priority += 4;
        cpu_kernel_critical_os_hi_priority += 4;
        cpu_kernel_critical_vm_priority += 4;
        cpu_clock_lo_priority += 4;
        cpu_clock_hi_priority = cpu_clock_lo_priority;
        cpu_calibration_priority += 4;
        prio += 4;
    }
    else
    {
        return;
    }

    int delta = 0;
    if (prio + 4 * 2 <= pmax)
        delta = 2;
    else if (prio + 4 * 1 <= pmax)
        delta = 1;

    if (delta)
    {
        cpu_kernel_critical_os_priority += delta * 0;
        cpu_kernel_critical_os_hi_priority += delta * 1;
        cpu_kernel_critical_vm_priority += delta * 2;
        cpu_clock_lo_priority += delta * 3;
        cpu_clock_hi_priority = cpu_clock_lo_priority;
        cpu_calibration_priority += delta * 4;
    }
}

/* print thread priority allocation etc. */
void smp_show_thread_priority_info(SMP_FILE* st)
{
    fprintf(st, "Using thread priority range up to %d\n", cpu_calibration_priority);
}

/* per-thread initializaion */
t_bool smp_thread_init()
{
    t_bool res = TRUE;
    struct rlimit rlim;

    /* set priotiry limits for those platforms where limits can be per-thread */
    if (0 == getrlimit(RLIMIT_RTPRIO, & rlim))
    {
        rlim.rlim_cur = rlim.rlim_max;
        if (rest_setrlimit(RLIMIT_RTPRIO, & rlim))
            res = FALSE;
    }

    if (0 == getrlimit(RLIMIT_NICE, & rlim))
    {
        rlim.rlim_cur = rlim.rlim_max;
        if (rest_setrlimit(RLIMIT_NICE, & rlim))
            res = FALSE;
    }

    return res;
}

/**********************  Linux -- set thread name **********************/

void smp_set_thread_name(const char* name)
{
    /* sanity check for 64-bit case, normally should be ok since 64-bit Linux 
       uses LP64 model with long defined as 64 bits */
    if (name == (const char*) (unsigned long) name)
        prctl(PR_SET_NAME, (unsigned long) name, 0, 0, 0);
}

/**********************  Linux -- smp_get_cpu_count  **********************/

static int smp_get_cpu_count()
{
    /*
     * Also works for Solaris, AIX.
     * For other systems (FreeBSD, MacOS X, NetBSD, OpenBSD) see
     * http://stackoverflow.com/questions/150355/programmatically-find-the-number-of-cores-on-a-machine
     */
     int res = (int) sysconf(_SC_NPROCESSORS_ONLN);
     if (res <= 0)
          panic("Unable to determine the number of CPUs in the host system");
     return res;
}

/**********************  Linux -- smp_init_info  **********************/

static void strip_trailing_blanks(char* s)
{
    if (*s)
    {
        char* p = s + strlen(s) - 1;
        while (p >= s && (*p == ' ' || *p == '\t'))
            *p-- = '\0';
    }
}

static int smp_ncores = 0;
static cpu_set_t smp_core_cpu_set;
static cpu_set_t smp_all_cpu_set;
static int smp_smt_per_core = -1;


/*
 * Gather host system multiprocessor configuration.
 *
 * ToDo: may later want to switch to Portable Hardware Locality (hwloc) library.
 *       http://www.open-mpi.org/projects/hwloc
 */
static t_bool smp_init_info()
{
    /*
     * /proc/cpuinfo displays information only for online CPUs, this is just what we want
     */
    FILE* fd = fopen("/proc/cpuinfo", "r");
    int max_per_core = 0;
    t_bool done = FALSE;
    char buffer[1024];
    char* xp;
    char* key;
    char* value;
    int hi_phys_id = 255;
    int hi_core_id = 31;
    int** p_smt = NULL;
    int cpu_id = -1;
    int phys_id = -1;
    cpu_set_t core_set;
    int k, j;
    int* p_core;

    smp_ncores = 0;
    CPU_ZERO(& smp_core_cpu_set);
    CPU_ZERO(& smp_all_cpu_set);
    CPU_ZERO(& core_set);

    CHECK(fd);

    /* allocate array to count SMT factor (CPUs per core) */
    p_smt = (int**) malloc(sizeof(int*) * (hi_phys_id + 1));
    CHECK(p_smt);
    memset(p_smt, 0, sizeof(int*) * (hi_phys_id + 1));

    while (fgets(buffer, sizeof(buffer), fd))
    {
        buffer[sizeof(buffer) - 1] = '\0';
        if (xp = strchr(buffer, '\n')) *xp = '\0';
        if (xp = strchr(buffer, '\r')) *xp = '\0';
        if (buffer[0] == '\0')
        {
            cpu_id = -1;
            phys_id = -1;
            continue;
        }
        key = buffer;
        xp = strchr(buffer, ':');
        if (! xp)
        {
            cpu_id = -1;
            phys_id = -1;
            continue;
        }
        *xp++ = 0;
        value = xp;
        while (*value == ' ' || *value == 't')  value++;
        strip_trailing_blanks(key);
        strip_trailing_blanks(value);

        if (streqi(key, "processor"))
        {
            cpu_id = -1;
            phys_id = -1;
            CHECK(1 == sscanf(value, "%d", & cpu_id));
            CHECK(cpu_id >= 0);
            CPU_SET(cpu_id, & smp_all_cpu_set);
        }
        else if (streqi(key, "physical id"))
        {
            // may not be reported when executed on hosts that do not have multicore CPUs
            // or under VMWare
            phys_id = -1;
            CHECK(1 == sscanf(value, "%d", & phys_id));
            CHECK(phys_id >= 0);

            if (phys_id > hi_phys_id)
            {
                int** np = (int**) realloc(p_smt, sizeof(int*) * (phys_id + 1));
                CHECK(np);
                memset(np + hi_phys_id + 1, 0, sizeof(int*) * (phys_id - hi_phys_id));
                p_smt = np;
                hi_phys_id = phys_id;
            }
        }
        else if (streqi(key, "core id"))
        {
            // may not be reported when executed on hosts that do not have SMT (hyperthreaded) CPUs
            // or under VMWare
            int core_id = -1;
            CHECK(1 == sscanf(value, "%d", & core_id));
            CHECK(core_id >= 0);
            CHECK(phys_id >= 0);

            if (! p_smt[phys_id])
            {
                p_smt[phys_id] = (int*) malloc(sizeof(int) * (hi_core_id + 1));
                CHECK(p_smt[phys_id]);
                memset(p_smt[phys_id], 0, sizeof(int) * (hi_core_id + 1));
            }

            if (core_id > hi_core_id)
            {
                int new_hi_core_id = hi_core_id * 2;
                if (core_id >= new_hi_core_id)
                    new_hi_core_id = core_id;

                for (k = 0;  k <= hi_phys_id;  k++)
                {
                    int* np = (int*) realloc(p_smt[k], sizeof(int) * (new_hi_core_id + 1));
                    CHECK(np);
                    memset(np + hi_core_id + 1, 0, sizeof(int) * (new_hi_core_id - hi_core_id));
                    p_smt[k] = np;
                }

                hi_core_id = new_hi_core_id;
            }

            p_core = p_smt[phys_id];
            p_core[core_id]++;

            /*
             * currently limited to:
             *   max 32 physical processors, 
             *   max 32 cores per physical processor
             */
            CHECK(cpu_id >= 0);
            CHECK(phys_id >= 0 || phys_id == -1);
            CHECK(core_id < 32);
            int phys_core_id = (phys_id >= 0 ? phys_id * 32 : 0) + core_id;
            CHECK(phys_core_id < __CPU_SETSIZE);

            if (! CPU_ISSET(phys_core_id, & core_set))
            {
                CPU_SET(cpu_id, & smp_core_cpu_set);
                CPU_SET(phys_core_id, & core_set);
                smp_ncores++;
            }
        }
    }

    CHECK(! ferror(fd));

    for (k = 0;  k <= hi_phys_id;  k++)
    {
        if (! (p_core = p_smt[k]))
            continue;

        for (int j = 0;  j <= hi_core_id;  j++)
        {
            if (p_core[j] > max_per_core)
                max_per_core = p_core[j];
        }
    }

    /* in case "core id" is not reported */
    if (max_per_core == 0)
        max_per_core = 1;

    done = TRUE;

cleanup:

    if (done)
    {
        smp_smt_per_core = max_per_core;
    }
    else
    {
        smp_ncores = 0;
        smp_smt_per_core = -1;
    }

    if (fd)
        fclose(fd);

    if (p_smt)
    {
        for (k = 0;  k <= hi_phys_id;  k++)
        {
            if (p_smt[k])
                free(p_smt[k]);
        }

        free(p_smt);
    }


    return done;
}

/**********************  Linux -- smp_get_smt_per_core  **********************/

/*
 * get number of SMT/HyperThreading processor units per core, 
 * in case of error return -1
 */
static int smp_get_smt_per_core()
{
    return smp_smt_per_core;
}

/**********************  Linux -- smp_set_affinity  **********************/

t_bool smp_can_alloc_per_core(int nthreads)
{
    return smp_ncores >= nthreads;
}

void smp_set_affinity(smp_thread_t thread_th, smp_affinity_kind_t how)
{
    if (how == SMP_AFFINITY_PER_CORE)
    {
        pthread_setaffinity_np(thread_th, sizeof smp_core_cpu_set, & smp_core_cpu_set);
    }
    else
    {
        pthread_setaffinity_np(thread_th, sizeof smp_all_cpu_set, & smp_all_cpu_set);
    }
}
#endif

/* ===========================================  OSX all architectures  =========================================== */

#if defined(__APPLE__)

/**********************  OSX -- set thread priority **********************/

static int cpu_calibration_priority = -1;
static int cpu_kernel_critical_vm_priority = -1;
static int cpu_kernel_critical_os_hi_priority = -1;
static int cpu_kernel_critical_os_priority = -1;
static int cpu_clock_hi_priority = -1;
static int cpu_clock_lo_priority = -1;
static int cpu_run_priority = -1;
static int console_paused_priority = -1;
static int console_run_priority = -1;

t_bool smp_set_thread_priority(sim_thread_priority_t prio)
{
    run_scope_context::get_current()->setting_priority(prio);
    return smp_set_thread_priority(pthread_self(), prio);
}

int smp_get_thread_os_priority(smp_thread_t thread_th)
{
    int policy = SCHED_OTHER;
    struct sched_param sc_param;
    if (0 != rest_pthread_getschedparam(thread_th, & policy, & sc_param))
        return -1;
    return sc_param.sched_priority;
}

t_bool smp_set_thread_priority(smp_thread_t thread_th, sim_thread_priority_t prio)
{
    int priority = -1;
    int policy;

    switch (prio)
    {
    /* CPU thread -- regular instruction stream execution */
    case SIMH_THREAD_PRIORITY_CPU_RUN:
        policy = SCHED_OTHER;
        priority = cpu_run_priority;
        break;

    /* console thread -- while CPUs are paused */
    case SIMH_THREAD_PRIORITY_CONSOLE_PAUSED:
        policy = SCHED_OTHER;
        priority = console_paused_priority;
        break;

    /* console thread -- while CPUs are running */
    case SIMH_THREAD_PRIORITY_CONSOLE_RUN:
        policy = SCHED_RR;
        priority = console_run_priority;
        break;

    /*
     * IO processing thread: make thread priority identical with CRITICAL_OS to avoid assigning criticality
     * setting to locks for IOP structures, so locking can be done without the overhead of elevating/demoting thread
     * priority, but at the same time also avoiding priority inversion when CPU's thread running at CRITIAL_OS
     * (a priority at which OS will typically access IO device) would have to wait for the lock held by IOP thread
     * executing at a lower priority and possibly preempted. Downside of this choice being that high IO rate may
     * also saturate host system computationally and take precedence over host UI enviroment both in terms of
     * high-priority computational load and IO requests queued at high-priority.
     */
    case SIMH_THREAD_PRIORITY_IOP:
        /* fall through to CRITICAL_OS */

    /* CPU thread -- kernel is in a critical section (holding spinlocks etc.) */
    case SIMH_THREAD_PRIORITY_CPU_CRITICAL_OS:
        policy = SCHED_RR;
        priority = cpu_kernel_critical_os_priority;
        break;

    /* CPU thread -- kernel is processing CLK or IPI interrupt,
       or any of these interrupts is pending */
    case SIMH_THREAD_PRIORITY_CPU_CRITICAL_OS_HI:
        policy = SCHED_RR;
        priority = cpu_kernel_critical_os_hi_priority;
        break;

    /* CPU thread -- VM is in a critical section (holding lock etc.) */
    case SIMH_THREAD_PRIORITY_CPU_CRITICAL_VM:
        policy = SCHED_FIFO;
        priority = cpu_kernel_critical_vm_priority;
        break;

    /* CPU thread -- processor loops calibration using SSC clock */
    case SIMH_THREAD_PRIORITY_CPU_CALIBRATION:
        policy = SCHED_FIFO;
        priority = cpu_calibration_priority;
        break;

    /* clock strobing thread */
    case SIMH_THREAD_PRIORITY_CLOCK:
        if (sim_vsmp_active)
        {
            policy = SCHED_FIFO;
            priority = cpu_clock_hi_priority;
        }
        else
        {
            policy = SCHED_FIFO;
            priority = cpu_clock_lo_priority;
        }
        break;

    default:
        return FALSE;
    }

    if (priority < 0)
        return FALSE;

    struct sched_param sc_param;
    sc_param.sched_priority = priority;

    if (0 != rest_pthread_setschedparam(thread_th, policy, & sc_param))
        return FALSE;

    return TRUE;
}

static void smp_set_thread_priority_init()
{
    /*
     * Assign CPU priorities in this increasing order:
     *
     *    cpu_run_priority
     *    cpu_kernel_critical_os_priority
     *    cpu_kernel_critical_os_hi_priority
     *    cpu_kernel_critical_vm_priority
     *    cpu_clock_hi_priority = cpu_clock_lo_priority
     *    cpu_calibration_priority
     *
     * Leave console_paused_priority as default level.
     *
     * Set console_run_priority >= cpu_kernel_critical_vm_priority so it could preepmpt CPU threads,
     * but leave cpu_calibration_priority above the console level so that normal console activity
     * (other than Ctrl/E) does not disturb calibration.
     *
     * OS X uses thread priorities in range 0...63 has several bands of priorities:
     *
     *     Real-time threads
     *     Kernel-mode threads
     *     System high priority range
     *     Normal priority range
     *
     * The range available to regular applications is 15...47 (higher number means higher priority).
     * Default priority for console applications both in fore- and back-ground of UI is 31.
     *
     * See:
     *
     *     https://developer.apple.com/library/mac/#documentation/Darwin/Conceptual/KernelProgramming/scheduler/scheduler.html
     *     https://developer.apple.com/library/mac/#documentation/Darwin/Reference/Manpages/man2/setpriority.2.html
     *
     *     http://www.opensource.apple.com/source/Libc/Libc-763.13/pthreads/pthread.c
     *     http://www.opensource.apple.com/source/xnu/xnu-1699.26.8/bsd/kern/kern_resource.c
     *     http://www.opensource.apple.com/source/xnu/xnu-1699.26.8/osfmk/kern/mk_sp.c
     *     http://www.opensource.apple.com/source/xnu/xnu-1699.26.8/osfmk/kern/priority.c
     */

    int pmin, pmax;
    t_bool bad = FALSE;

    pmin = sched_get_priority_min(SCHED_OTHER);
    pmax = sched_get_priority_max(SCHED_OTHER);
    if (pmin != 15 || pmax != 47)
    {
        fprintf(stderr, "Unexpected SCHED_OTHER priority band: %d/%d instead of 15/47\n", pmin, pmax);
        bad = TRUE;
    }

    pmin = sched_get_priority_min(SCHED_RR);
    pmax = sched_get_priority_max(SCHED_RR);
    if (pmin != 15 || pmax != 47)
    {
        fprintf(stderr, "Unexpected SCHED_RR priority band: %d/%d instead of 15/47\n", pmin, pmax);
        bad = TRUE;
    }

    pmin = sched_get_priority_min(SCHED_FIFO);
    pmax = sched_get_priority_max(SCHED_FIFO);
    if (pmin != 15 || pmax != 47)
    {
        fprintf(stderr, "Unexpected SCHED_FIFO priority band: %d/%d instead of 15/47\n", pmin, pmax);
        bad = TRUE;
    }

    if (bad)  return;

    cpu_calibration_priority = 47;
    cpu_clock_hi_priority = cpu_clock_lo_priority = 42;
    cpu_kernel_critical_vm_priority = 42;
    cpu_kernel_critical_os_hi_priority = 38;
    cpu_kernel_critical_os_priority = 34;
    cpu_run_priority = 25;

    console_run_priority = cpu_kernel_critical_vm_priority;
    console_paused_priority = 31;
}

/* print thread priority allocation etc. */
void smp_show_thread_priority_info(SMP_FILE* st)
{
    fprintf(st, "Using thread priority range up to %d\n", cpu_calibration_priority);
}

/* per-thread initializaion */
t_bool smp_thread_init()
{
    return TRUE;
}

/**********************  OSX -- set thread name **********************/

void smp_set_thread_name(const char* name)
{
    pthread_setname_np(name);
}

/**********************  OSX -- smp_get_cpu_count  **********************/

static t_bool osx_sysctl(const char* name, int* pv);

static int smp_get_cpu_count()
{
    int ncpus = 0;
    if (! osx_sysctl("hw.activecpu", & ncpus))
        panic("Unable to determine the number of CPUs in the host system");
    return ncpus;
}

static t_bool osx_sysctl(const char* key, int* pv)
{
    int data = 0;
    size_t dlen = sizeof(data);

    if (sysctlbyname(key, & data, & dlen, NULL, 0))
    {
        char msg[256];
        sprintf(msg, "unable to get system information %s", key);
        perror(msg);
        return FALSE;
    }

    if (dlen != sizeof(int))
    {
        fprintf(stderr, "unable to get system information %s: buffer size mismatch", key);
        return FALSE;
    }

    *pv = data;

    return TRUE;
}

/**********************  OSX -- smp_init_info  **********************/

static int smp_smt_per_core = -1;

/*
 * Gather host system multiprocessor configuration.
 */
static t_bool smp_init_info()
{
    int ncores = 0;
    int nlcpus = 0;

    if (! osx_sysctl("hw.physicalcpu_max", & ncores))
        return FALSE;

    if (! osx_sysctl("hw.logicalcpu_max", & nlcpus))
        return FALSE;

    if (ncores <= 0 || nlcpus <= 0 || nlcpus < ncores || (nlcpus % ncores))
        return FALSE;

    smp_smt_per_core = nlcpus / ncores;

    return TRUE;
}

/**********************  OSX -- smp_get_smt_per_core  **********************/

/*
 * get number of SMT/HyperThreading processor units per core, 
 * in case of error return -1
 */
static int smp_get_smt_per_core()
{
    return smp_smt_per_core;
}

/**********************  OSX -- smp_set_affinity  **********************/

t_bool smp_can_alloc_per_core(int nthreads)
{
    return FALSE;
}

void smp_set_affinity(smp_thread_t thread_th, smp_affinity_kind_t how)
{
    panic("smp_set_affinity: not implemented");
}
#endif

/* =====================================  Common (x86/x64) -- smp_lock_impl ===================================== */

#if defined(USE_SIMH_SMP_LOCK)

#if (defined(__linux) || defined(__APPLE__)) && (defined(__x86_32__) || defined(__x86_64__))
#  define interlocked_incl(var)  __sync_add_and_fetch(& (var), 1)
#  define interlocked_decl(var)  __sync_add_and_fetch(& (var), -1)
#  define interlocked_incl_is_eq(var)  (interlocked_incl(var) == 0)
#  define interlocked_decl_is_geq(var)  (interlocked_decl(var) >= 0)
#  define interlocked_cas(var, oldvalue, newvalue)  __sync_val_compare_and_swap(& (var), (oldvalue), (newvalue))
#elif defined(_WIN32)
#  define interlocked_incl(var)  smp_interlocked_increment(& (var))
#  define interlocked_decl(var)  smp_interlocked_decrement(& (var))
#  define interlocked_incl_is_eq(var)  (interlocked_incl(var) == 0)
#  define interlocked_decl_is_geq(var)  (interlocked_decl(var) >= 0)
#  define interlocked_cas(var, oldvalue, newvalue)  smp_interlocked_cas(& (var), (oldvalue), (newvalue))
#endif

smp_lock_impl::smp_lock_impl()
{
    criticality = SIM_LOCK_CRITICALITY_NONE;
    inited = FALSE;
#if !defined(_WIN32)
    os_sem_decl_init(semaphore);
#endif
    calibrating_spinloop = FALSE;
}

t_bool smp_lock_impl::init(uint32 cycles, t_bool dothrow)
{
    if (! inited)
    {
        if (! check_aligned(this, SMP_MAXCACHELINESIZE, dothrow))
            goto cleanup;
        if (! smp_check_aligned(& lock_count, dothrow))
            goto cleanup;

        /* gcc builtin __sync_add_and_fetch uses xaddl unavailable before i486 */
        /*             __sync_val_compare_and_swap uses cmpxchgl unavailable before i486 */
        if (!have_x86_xaddl || !have_x86_cmpxchgl)
            goto cleanup;
        smp_var(lock_count) = -1;
        recursion_count = 0;
        owning_thread = 0;
        spin_count = (smp_ncpus > 1) ? cycles : 0;
#if defined(_WIN32)
        semaphore = CreateSemaphore(NULL, 0, 0x7FFFFFFF, NULL);
        if (semaphore == NULL)  goto cleanup;
#else
        DECL_RESTARTABLE(rc);
        DO_RESTARTABLE(rc, os_sem_init(os_sem_ptr(semaphore), 0));
        if (rc == -1)  goto cleanup;
#endif
        perf_collect = FALSE;
        perf_reset();
        inited = TRUE;
    }

cleanup:

    if (! inited && dothrow)
        panic("Unable to initialize critical section");

    return inited;
}

t_bool smp_lock_impl::init(uint32 us, uint32 min_cycles, uint32 max_cycles, t_bool dothrow)
{
    t_bool res = init(0, dothrow);
    if (res)  set_spin_count(us, min_cycles, max_cycles);
    return res;
}

smp_lock_impl::~smp_lock_impl()
{
#if defined(_WIN32)
    if (inited)
        CloseHandle(semaphore);
#else
    if (inited)
       os_sem_destroy(os_sem_ptr(semaphore));
#endif
}

void smp_lock_impl::set_spin_count(uint32 cycles)
{
    spin_count = (smp_ncpus > 1) ? cycles : 0;
}

void smp_lock_impl::set_spin_count(uint32 us, uint32 min_cycles, uint32 max_cycles)
{
    uint32 cycles;

    if (smp_ncpus <= 1)
    {
        cycles = 0;
    }
    else if (spins_per_ms == 0 || us == 0)
    {
        cycles = min_cycles;
    }
    else
    {
        cycles = (spins_per_ms * us) / 1000 + 1;
        if (cycles > max_cycles)  cycles = max_cycles;
        if (cycles < min_cycles)  cycles = min_cycles;
    }

    spin_count = cycles;
}

void smp_lock_impl::set_perf_collect(t_bool collect)
{
    perf_collect = collect;
}

void smp_lock_impl::perf_reset()
{
    UINT64_SET_ZERO(perf_lock_count);
    UINT64_SET_ZERO(perf_nowait_count);
    UINT64_SET_ZERO(perf_syswait_count);
    perf_avg_spinwait = 0;
}

void smp_lock_impl::perf_show(SMP_FILE* fp, const char* name)
{
    if (! perf_collect)
    {
        fprintf(fp, "Lock %s: counters disabled\n", name);
        return;
    }

    if (UINT64_IS_ZERO(perf_lock_count))
    {
        fprintf(fp, "Lock %s: unused\n", name);
        return;
    }

#if defined (USE_INT64)
    fprintf(fp, "Lock %s: acquired %" PRIu64 " times\n", name, perf_lock_count);
#else
    fprintf(fp, "Lock %s: acquired %.0f times\n", name, UINT64_TO_DOUBLE(perf_lock_count));
#endif

    fprintf(fp, "    immediately: %g%%\n", 100.0 * UINT64_TO_DOUBLE(perf_nowait_count) / UINT64_TO_DOUBLE(perf_lock_count));
    fprintf(fp, "    blocking wait: %g%%\n", 100.0 * UINT64_TO_DOUBLE(perf_syswait_count) / UINT64_TO_DOUBLE(perf_lock_count));
    fprintf(fp, "    average spinwait (loop cycles): %.1f\n", perf_avg_spinwait);
}

/*
 * There are few chief categories of locks in VAX MP.
 *
 *  1. Device locks are expected to have moderate contention rate.
 *     (Contention for the device can be high, but contenders will likely be spinning
 *     on OS-level interlock represented by locks category 3 below, rather than on
 *     the device lock.)
 *
 *  2. CPU database lock (cpu_database_lock) is expected to have higher
 *     contention rate.
 *
 *  3. Locks for interlocked VAX instructions can have very high contention rate.
 *     They may also be replaced in the future (or partially replaced anyway)
 *     by host-processor-specific native interlocked operations, when host
 *     provides such operations (such as BBSSI and BCCCI replaced by x86/x64
 *     BTS and BTR instructions).
 *
 * Current lock implementation for the initial release of VAX MP is... well, initial.
 *
 * Current implementation of smp_lock_impl locking is intended for low to medium contention
 * levels, as expected for VAX MP device locks, and may be adequate for cpu_database_lock
 * when no synchronization window is used, but may be poor for interlocked VAX instructions
 * locks in portable mode, or for cpu_database_lock when synchronization window is enabled.
 *
 * Once the initial release of VAX MP is stable and we get experience and peformance data
 * running a realistic load (and stress tests), we can address locks implementation to
 * imporve their performance.
 *
 * Present algorithm for the locks is as follows:
 *
 * If a collision is detected (critical section is locked and there is more than
 * one waiting thread), second and subsequent waiters do not try to spin and perform
 * OS-level blocking wait instead, thus further ensuring low spinwait contention levels
 * on the lock structure in memory.
 *
 * Accordingly, the lock is implemented using "test-and-test-and-set" technique as deemed
 * being amply sufficient for this case. 
 *
 * In general, test-and-test-and-set approach avoids executing interlocked instructions
 * in each loop cycle and thus flooding the interconnect with expensive RFO requests, and in
 * addition causing cache line RFO ping-pong between multiple processors if there were
 * multiple spinners (which is however not the case with the present design). Instead
 * the spinning happens on the value held in local cache that is updated via cache coherency
 * protocol after the lock is released by the holder.
 *
 *     See G. Graunke, S. Thakkar, "Synchronization Algorithms for Shared-Memory
 *     Multiprocessors", IEEE Computer, June 1990
 *
 * Even this limited optimization is amply sufficient for the present design that allows
 * one holder and only one active spinner, with other contenders executing a blocking wait,
 * so there is a priori no hogging of the interconnect by ping-pong and cache invalidations
 * between multiple spinners.
 *
 * It would be great if there was a way to check if lock holder is pre-empted, and
 * in this case avoid spin-waiting, however most host OS'es do not provide this functionality.
 * Therefore we start by spin-waiting on the lock in optimistic assumption that the holder
 * is not pre-empted and that the duration of the critical section is short. It generally
 * does not make sense to spin longer than twice the cost of a context switch (combined direct
 * and indirect costs, also accounting for subsequent performance hit due to possible TLB flushing)
 * or much longer than the duration of the critical section itself, as the latter would indicate
 * that the holder is likely to be pre-empted. Once spin cycles limit is reached, this is regarded
 * as an indication that the holder is likely to be pre-empted and that the contenter is better
 * to switch to OS-level locking.
 *
 * Should the assumptions about low contentoon level be ever revised and the lock would
 * have to cope with high spinwait contention level, it would probably would have to be
 * reimplemented using MCS queue locks
 *
 *     See John Mellor-Crummey, Michael Scott, "Algorithms for Scalable Synchronization on
 *     Shared-Memory Multiprocessors", ACM Trans. on Computer Systems, February 1991
 *
 *     John Mellor-Crummey, Michael Scott, "Synchronization Without Contention", 
 *     ASPLOS-IV, Proceedings of the fourth international conference on Architectural support for
 *     programming languages and operating systems; punlished as special issue of
 *     ACM SIGOPS Operating Systems Review, April 1991, ACM SIGARCH Computer Architecture News,
 *     and ACM SIGPLAN Notices.
 *
 *     T. Anderson, "The Performance of Spin Lock Alternatives for Shared-Memory Multiprocessors",
 *     IEEE Transactions on Parallel and Distributed Systems, Jan 1990
 *
 *     John Mellor-Crummey, "Mutual Exclusion: Classical Algorithms for Locks"
 *     (slides for COMP 422, Lecture 17, 29 March 2011)
 *     http://www.clear.rice.edu/comp422/lecture-notes/comp422-2011-Lecture17-ClassicalLocks.pdf
 *
 *     John Mellor-Crummey, "Algorithms for Scalable Lock Synchronization on Shared-memory Multiprocessors"
 *      (slides for COMP 422, Lecture 18 31 March 2011),
 *     http://www.clear.rice.edu/comp422/lecture-notes/comp422-2011-Lecture18-HWLocks.pdf
 *
 *     James H. Anderson, Yong-jik Kim, "Shared-memory mutual exclusion: Major research trends
 *     since 1986", Distributed Computing, vol. 16 (issue 2-3), September 2003
 *
 *     Maurice Herlihy, Nir Shavit, "The Art of Multiprocessor Programming", Elsevir, 2008, ch. 7.5
 *
 * or M-locks
 *
 *     Peter Magnusson, Anders Landin, Erik Hagersten, "Queue Locks on Cache Coherent Multiprocessors",
 *     Proceedings of the 8th International Symposium on Parallel Processing, 1994
 *
 * or composite locks
 *
 *     Maurice Herlihy, Nir Shavit, "The Art of Multiprocessor Programming", Elsevir, 2008, ch. 7.7
 *
 * ideally modified to be priority-aware and preemption-safe (i.e. not releasing the lock
 * to a pre-empted contender if there are other contenders actively spinning for it).
 */

/*
 * Lock and unlock perform memory barriers as follows:
 *
 *     lock:    acquire lock
 *              full MB
 *
 *     unlock:  full MB
 *              release lock
 *
 * In the lock case, full MB cannot be downgraded to merely RMB, since otherwise writes 
 * from protected section could leak to before the lock if processor reorders instructions
 * or memory references.
 *
 * In the unlock case, full MB cannot be downgraded to merely WMB, since otherwise reads 
 * from protected section can leak past the unlock.
 *
 * Neither read nor write references to protected data should be allowed to leak out of
 * protected section bounded by atomic-lock/MB and MB/atomic-unlock.
 *
 */

void smp_lock_impl::lock()
{
#if defined(_WIN32)
    DWORD this_thread = GetCurrentThreadId();
    if (unlikely(this_thread == 0))  panic("Unexpected: ThreadId is 0");
#   define thread_eq(p1, p2)  ((p1) == (p2))
#else
    pthread_t this_thread = pthread_self();
    if (unlikely(this_thread == 0))  panic("Unexpected: thread handle is 0");
#   define thread_eq(p1, p2)  pthread_equal((p1), (p2))
#endif

    /*
     * We have to choose between two evils.
     *
     * Raising thread priority level _before_ actual lock acqusition causes lower-priority
     * threads to be depressed from doing useful work and, in addition, may cause lock holder
     * to be pre-empted by the contender.
     *
     * Whereas raising thread priority level _after_ actual lock acquision may cause lock
     * holder to be pre-empted and left at low priority for extended time. The probability
     * of the second unfavorable development is less, but its consequences are more drastic.
     *
     * On the balance, the implementation below makes choice for the former evil.
     *
     */
    if (criticality != SIM_LOCK_CRITICALITY_NONE)
        critical_lock(criticality);

    uint32 cycles = spin_count;

    if (likely(spin_count != 0))
    {
        if (thread_eq(owning_thread, this_thread))
        {
            interlocked_incl(smp_var(lock_count));
            recursion_count++;
            return;
        }

        for (;;)
        {
            if (smp_var(lock_count) == -1 && interlocked_cas(smp_var(lock_count), -1, 0) == -1)
            {
                smp_post_interlocked_mb();

                owning_thread = this_thread;
                recursion_count = 1;

                /* record performance counters */
                if (unlikely(perf_collect))
                    perf_acquired(cycles);
                return;
            }

            if (smp_var(lock_count) >= 1)
            {
                // the lock is locked, and at least one more thread is already (or also) waiting for it
                // give up on spin-waiting
                break;
            }

            /*
             * Insert PAUSE to prevent generation of multiple (and useless) simultaneous out-of-order read requests.
             * Effective on Pentium 4 and higher processors, NOP on earlier Intel processors and clones.
             * See Intel application note AP-949 "Using Spin-Loops on Intel Pentium 4 Processor and Intel Xeon Processor",
             * P/N 248674-002 (http://software.intel.com/file/25602).
             * May also help when spin-waitng on a Hyper-Threaded processor by releasing shared resources to a thread
             * doing useful work, perhaps a lock holder.
             */
            smp_cpu_relax();

            if (cycles == 0 || --cycles == 0)
            {
                // make one last attempt at checking
                if (smp_var(lock_count) != -1)
                    // give up on spin-waiting
                    break;
            }
        }
    }

    /* comes here for OS blocking wait */
    if (unlikely(calibrating_spinloop))
        return;

    if (interlocked_incl_is_eq(smp_var(lock_count)))
    {
        smp_post_interlocked_mb();
        owning_thread = this_thread;
        recursion_count = 1;

        /* record performance counters */
        if (unlikely(perf_collect))
            perf_acquired(cycles);
    }
    else if (thread_eq(owning_thread, this_thread))
    {
        recursion_count++;
    }
    else
    {
#if defined(_WIN32)
        switch (WaitForSingleObject(semaphore, INFINITE))
        {
        case WAIT_OBJECT_0:
            break;
        default:
            panic("Semaphore error");
        }
#else
        DECL_RESTARTABLE(rc);
        DO_RESTARTABLE(rc, sem_wait(semaphore));
        if (rc != 0)  panic("Semaphore error");
#endif

        /* we do not need explicit MB here since it is expected to be performed
           by OS-level synchronization primitives above */

        owning_thread = this_thread;
        recursion_count = 1;

        /* record performance counters */
        if (unlikely(perf_collect))
        {
            UINT64_INC(perf_lock_count);
            if (UINT64_ISMAX(perf_lock_count))
                perf_collect = FALSE;
            UINT64_INC(perf_syswait_count);
        }
    }
}

void smp_lock_impl::calibrate_spinloop()
{
    smp_var(lock_count) = 0;
    calibrating_spinloop = TRUE;
    lock();
    calibrating_spinloop = FALSE;
    smp_var(lock_count) = -1;
}

void smp_lock_impl::perf_acquired(uint32 cycles)
{
    UINT64_INC(perf_lock_count);
    if (UINT64_ISMAX(perf_lock_count))
        perf_collect = FALSE;

    if (cycles == spin_count)
    {
        /* no-wait case */
        UINT64_INC(perf_nowait_count);
    }
    else
    {
        /* number of previous wait cases */
        double prev_spinwait_count = (UINT64_TO_DOUBLE(perf_lock_count) - 1.0) - UINT64_TO_DOUBLE(perf_nowait_count) - UINT64_TO_DOUBLE(perf_syswait_count);
        perf_avg_spinwait = (prev_spinwait_count * perf_avg_spinwait + (double) (spin_count - cycles)) / (prev_spinwait_count + 1.0);
    }
}

void smp_lock_impl::unlock()
{
    if (0 != --recursion_count)
    {
        interlocked_decl(smp_var(lock_count));
    }
    else
    {
        owning_thread = 0;
        smp_pre_interlocked_mb();
        if (interlocked_decl_is_geq(smp_var(lock_count)))
        {
#if defined(_WIN32)
            if (! ReleaseSemaphore(semaphore, 1, NULL))
                panic("Semaphore error");
#else
            DECL_RESTARTABLE(rc);
            DO_RESTARTABLE(rc, sem_post(semaphore));
            if (rc != 0)  panic("Semaphore error");
#endif
        }
    }

    if (criticality != SIM_LOCK_CRITICALITY_NONE)
        critical_unlock(criticality);
}
#endif

void smp_lock::calibrate()
{
    smp_lock_impl::calibrate();
}

uint32 smp_lock_impl::spins_per_ms = 0;

static t_bool is_calibration_stable(const uint32* v, uint32 nsamples, uint32* pminv);

#define CALIBR_MIN_SAMPLES  25
#define CALIBR_MAX_SAMPLES  200
#define CALIBR_TOLERANCE_RANGE 0.25
#define CALIBR_MIN_WITHIN_TR 5

void smp_lock_impl::calibrate()
{
    smp_lock_impl* lck = (smp_lock_impl*) smp_lock::create();
    sim_delta_timer* tmr = sim_delta_timer::create();
    uint32 spincount;
    uint32 spin_us = 0;      /* initialize to suppress false GCC warning */
    uint32 valid = FALSE;

    smp_set_thread_priority(SIMH_THREAD_PRIORITY_CPU_CALIBRATION);

    /* increase spncount until loop exceeds 5 ms */
    for (spincount = 1000; ; spincount *= 2)
    {
        lck->set_spin_count(spincount);
        tmr->begin(RUN_PASS_NULL);
        lck->calibrate_spinloop();
        tmr->sample(RUN_PASS_NULL);
        if (tmr->us_since_start(RUN_PASS_NULL) > 5000)
            break;
    }

    /*
     * Sometimes calibraton may fail due to spurious reasons, such as external load
     * or cache flush/cold/warm effects. Try thrice before giving up.
     */
    for (int pass = 0;  pass <= 2;  pass++)
    {
        uint32 us[CALIBR_MAX_SAMPLES];
        int nsamples;
        for (nsamples = 0;  nsamples < CALIBR_MAX_SAMPLES;  )
        {
            tmr->begin(RUN_PASS_NULL);
            lck->calibrate_spinloop();
            tmr->sample(RUN_PASS_NULL);
            us[nsamples++] = tmr->us_since_start(RUN_PASS_NULL);
            if (nsamples >= CALIBR_MIN_SAMPLES && is_calibration_stable(us, nsamples, & spin_us))
            {
                valid = TRUE;
                break;
            }
        }
    }

    if (valid)
    {
        double f = 1000.0 * (double) spincount / (double) spin_us;
        spins_per_ms = (uint32) floor(f);
        if (f - spins_per_ms  >= 0.5)  spins_per_ms++;
        /* sanity */
        if (spins_per_ms >= 0x10000000)  spins_per_ms = 0;
    }

    smp_set_thread_priority(SIMH_THREAD_PRIORITY_CONSOLE_PAUSED);
    delete lck;
    delete tmr;
}

static t_bool is_calibration_stable(const uint32* v, uint32 nsamples, uint32* pminv)
{
    uint32 minv = v[0];
    uint32 mink = 0;
    uint32 in_tolerance_range = 0;
    uint32 k;

    /* find minimum */
    for (k = 1;  k < nsamples;  k++)
    {
        if (v[k] < minv)
        {
            *pminv = minv = v[mink = k];
        }
    }

    /* count samples close to minimum */
    for (k = 0;  k < nsamples;  k++)
    {
        if (k != mink && v[k] <= minv * (1.0 + CALIBR_TOLERANCE_RANGE))
        {
            if (++in_tolerance_range >= CALIBR_MIN_WITHIN_TR)
                return TRUE;
        }
    }

    return FALSE;
}

/* ============================================  sim_DebugBreak helper ============================================ */

#if defined(_WIN32) && defined(__x86_32__)
// sim_DebugBreak() is defined inline
#elif defined(__GNUC__) && (defined(__x86_32__) || defined(__x86_64__))
// sim_DebugBreak() is defined inline
#elif defined(_WIN32) && defined(__x86_64__)
void sim_DebugBreak()
{
    DebugBreak();
}
#elif defined(__linux) || defined(__APPLE__)
void sim_DebugBreak()
{
    raise(SIGTRAP);
}
#elif defined(__GNUC__)
t_bool sim_DebugBreak_DoIt = TRUE;
void sim_DebugBreak()
{
    if (sim_DebugBreak_DoIt)
        __builtin_trap();
}
#else
int dzro_res;
int dzro_zero = 0;
void sim_DebugBreak()
{
    dzro_res = 1000 / dzro_zero;
}
#endif

/* ============================================  Linux - specific ============================================ */

#if defined(__linux)
/*
 * Check whether we are running 1x1 NPTL threads, rather than old user-mode LinuxThreads library.
 */
static t_bool check_nptl()
{
    size_t n = confstr(_CS_GNU_LIBPTHREAD_VERSION, NULL, 0);
    if (n > 0)
    {
        char* buf = (char*) alloca(n + 10);
        confstr(_CS_GNU_LIBPTHREAD_VERSION, buf, n);
        if (strstr(buf, "NPTL"))
            return TRUE;
    }
    return FALSE;
}
#endif

/* ====================================  Windows x86 native interlocked ==================================== */

#if defined(__x86_32__) || defined(__x86_64__)
#  define X86_EFLAGS_V_CF    0
#  define X86_EFLAGS_V_ZF    6
#  define X86_EFLAGS_V_SF    7
#  define X86_EFLAGS_V_OF   11
#  define CF_FROM_EFLAGS(cc, eflags)                            \
       cc |= ((eflags >> X86_EFLAGS_V_SF) & 1) << CC_V_N;       \
       cc |= ((eflags >> X86_EFLAGS_V_ZF) & 1) << CC_V_Z;       \
       cc |= ((eflags >> X86_EFLAGS_V_OF) & 1) << CC_V_V;       \
       cc |= ((eflags >> X86_EFLAGS_V_CF) & 1) << CC_V_C;
#endif

#if SMP_NATIVE_INTERLOCKED && defined(_WIN32) && defined(__x86_32__)
/*
 * Adds "addendum" to "*psum" in a manner consistent with ADAWI and returns VAX CC.
 * "*psum" is guaranteed to be in VAX memory space and aligned on a word boundary.
 */
int32 smp_native_adawi(volatile void* memory, int32 pa_sum, uint16 addend)
{
    uint32 eflags;

    volatile t_byte* psum = (volatile t_byte*) memory;
    psum += pa_sum;

    smp_pre_interlocked_mb();

    __asm
    {
        mov    ax, addend
        mov    edx, psum
        lock add word ptr [edx], ax
        pushfd
        pop    eax
        mov    eflags, eax  
    }

    smp_post_interlocked_mb();

    int32 cc = 0;
    CF_FROM_EFLAGS(cc, eflags);

    return cc;
}

/* 
 * interlocked CAS on byte argument:
 *     if (*p == old_value) *p = new_value
 *     in all cases return original value of *p
 */
static t_byte cas_byte(volatile t_byte* p, t_byte old_value, t_byte new_value)
{
    t_byte res = old_value;

    COMPILER_BARRIER;

    __asm
    {
        mov    al, old_value
        mov    bl, new_value
        mov    edx, p
        lock cmpxchg byte ptr [edx], bl
        je     matched
        mov    res, al
matched:
    }

    COMPILER_BARRIER;

    return res;
}
#endif

/* ====================================  Windows x64 native interlocked ==================================== */

#if SMP_NATIVE_INTERLOCKED && defined(_WIN32) && defined(__x86_64__)
extern "C"
{
extern t_uint64 __fastcall x64_native_adawi(volatile uint16* pval, uint16 addend);
extern t_byte __fastcall x64_cas_byte(volatile t_byte* p, t_byte old_value, t_byte new_value);
}

int32 smp_native_adawi(volatile void* memory, int32 pa_sum, uint16 addend)
{
    t_uint64 rflags;
    uint32 eflags;

    volatile t_byte* psum = (volatile t_byte*) memory;
    psum += pa_sum;

    smp_pre_interlocked_mb();
    rflags = x64_native_adawi((volatile uint16*) psum, addend);
    smp_post_interlocked_mb();

    eflags = (uint32) rflags;
    int32 cc = 0;
    CF_FROM_EFLAGS(cc, eflags);

    return cc;
}

static t_byte cas_byte(volatile t_byte* p, t_byte old_value, t_byte new_value)
{
    COMPILER_BARRIER;
    t_byte res = x64_cas_byte(p, old_value, new_value);
    COMPILER_BARRIER;
    return res;
}
#endif

/* =====================================  Linux/OSX x86 native interlocked ===================================== */

#if SMP_NATIVE_INTERLOCKED && (defined(__linux) || defined(__APPLE__)) && defined(__x86_32__)

int32 smp_native_adawi(volatile void* memory, int32 pa_sum, uint16 addend)
{
    uint32 eflags;

    volatile t_byte* psum = (volatile t_byte*) memory;
    psum += pa_sum;

    smp_pre_interlocked_mb();

    __asm__ __volatile__ (
        "lock; addw %1, (%2)\n\t"
        "pushfl\n\t"
        "popl   %0\n\t"
        : "=g" (eflags)
        : "Q" (addend), "r" (psum)
        : "memory", "cc"
    );

    smp_post_interlocked_mb();

    int32 cc = 0;
    CF_FROM_EFLAGS(cc, eflags);

    return cc;
}

static t_byte cas_byte(volatile t_byte* p, t_byte old_value, t_byte new_value)
{
    COMPILER_BARRIER;
    t_byte res = __sync_val_compare_and_swap(p, old_value, new_value);
    COMPILER_BARRIER;
    return res;
}

#endif

/* =====================================  Linux/OSX x64 native interlocked ===================================== */

#if SMP_NATIVE_INTERLOCKED && (defined(__linux) || defined(__APPLE__)) && defined(__x86_64__)

int32 smp_native_adawi(volatile void* memory, int32 pa_sum, uint16 addend)
{
    t_uint64 rflags;

    volatile t_byte* psum = (volatile t_byte*) memory;
    psum += pa_sum;

    smp_pre_interlocked_mb();

    __asm__ __volatile__ (
        "lock; addw %1, (%2)\n\t"
        "pushfq\n\t"
        "popq   %0\n\t"
        : "=g" (rflags)
        : "Q" (addend), "r" (psum)
        : "memory", "cc"
    );

    smp_post_interlocked_mb();

    uint32 eflags = (uint32) rflags;
    int32 cc = 0;
    CF_FROM_EFLAGS(cc, eflags);

    return cc;
}

static t_byte cas_byte(volatile t_byte* p, t_byte old_value, t_byte new_value)
{
    COMPILER_BARRIER;
    t_byte res = __sync_val_compare_and_swap(p, old_value, new_value);
    COMPILER_BARRIER;
    return res;
}

#endif

/* =================================  native interlocked -- all platforms  ================================= */

#if SMP_NATIVE_INTERLOCKED && (defined(__x86_32__) || defined(__x86_64__))
/*
 * Native implementation for BBSSI and BBCCI instructions.
 *
 * On x86/x64 we cannot use BTS and BTR instructions since their smallest argument size is 16 bits,
 * whereas VAX specification says that BBSSI and BBCCI affect only a single byte, therefore we have
 * to use byte-sized CAS (CMPXCHG).
 *
 *     "bit" is the bit number within the byte, 0...7
 *     "set" is TRUE for BBSSI, FALSE for BBCCI
 *
 * Returns TRUE if the bit was originally set, FALSE if it was originally cleared.
 *
 */
t_bool smp_native_bb(volatile void* memory, int32 pa, int32 bit, t_bool set)
{
    volatile t_byte* p = (volatile t_byte*) memory;
    p += pa;

    t_byte old_value;
    t_byte mask = (1u << bit);

    smp_pre_interlocked_mb();

    old_value = *p;

    for (;;)
    {
        t_byte new_value = set ? (old_value | mask) : (old_value & ~mask);

        t_byte ov = cas_byte(p, old_value, new_value);

        if (ov == old_value)
        {
            smp_post_interlocked_mb();
            return (ov & mask) != 0;
        }
        else
        {
            old_value = ov;
        }
    }
}

/*
 * self-test primitives used for native-mode implementation of VAX interlocked instructions
 */
static void smp_native_self_test()
{
    static SIM_ALIGN_32 uint32 tval;

    tval = 0xFFFE;
    CHECK(smp_native_adawi(& tval, 0, 1) == CC_N && tval == 0xFFFF);
    CHECK(smp_native_adawi(& tval, 0, 1) == (CC_Z | CC_C) && tval == 0);
    CHECK(smp_native_adawi(& tval, 0, 1) == 0 && tval == 1);
    CHECK(smp_native_adawi(& tval, 0, 1) == 0 && tval == 2);
    tval = 0x7000;
    CHECK(smp_native_adawi(& tval, 0, 0x7000) == (CC_N | CC_V) && tval == 0xE000);
    tval = 0xF000;
    CHECK(smp_native_adawi(& tval, 0, 0xF000) == (CC_N | CC_C) && tval == 0xE000);

    tval = 0;
    CHECK(0 == smp_native_bb(& tval, 3, 6, TRUE) && tval == 0x40000000);
    CHECK(1 == smp_native_bb(& tval, 3, 6, TRUE) && tval == 0x40000000);
    CHECK(1 == smp_native_bb(& tval, 3, 6, FALSE) && tval == 0);
    CHECK(0 == smp_native_bb(& tval, 3, 6, FALSE) && tval == 0);

    return;

cleanup:
    panic("Self-test of native-mode interlocked primitives failed");
}
#endif

/* ====================================  sim_ws_setup -- all platforms  ==================================== */

#if defined(_WIN32)
void sim_ws_setup()
{
    sim_ws_prefaulted = FALSE;

    /* default working set range from memsize + 40 MB to memsize + 100 MB */
    const uint32 mb = 1024 * 1024;
    uint32 def_ws_min = (uint32) ((cpu_unit_0.capac + mb - 1) / mb) + 40;
    uint32 def_ws_max = (uint32) ((cpu_unit_0.capac + mb - 1) / mb) + 100;

    if (sim_ws_lock && sim_ws_min == 0 && sim_ws_max == 0)
    {
        sim_ws_min = def_ws_min;
        sim_ws_max = def_ws_max;
    }
    else if (sim_ws_min == 0 && sim_ws_max != 0)
    {
        sim_ws_min = imin(def_ws_min, sim_ws_max);
    }
    else if (sim_ws_min != 0 && sim_ws_max == 0)
    {
        sim_ws_max = imax(def_ws_max, sim_ws_min);
    }

    if (sim_ws_min > sim_ws_max)
    {
        sim_ws_max = sim_ws_min;
        smp_printf("Warning: WS_MAX was set below WS_MIN, increasing to %d MB\n", sim_ws_max);
        if (sim_log)
            fprintf(sim_log, "Warning: WS_MAX was set below WS_MIN, increasing to %d MB\n", sim_ws_max);
    }

    if (sim_ws_min && sim_ws_max)
    {
        if (sizeof(void*) == 4 && sim_ws_max >= 3 * 1024 ||
            sizeof(void*) == 8 && sim_ws_max >= 300 * 1024)
        {
            smp_printf("Warning: requested working set size (%d MB) is too high, unable to set it\n", sim_ws_max);
            if (sim_log)
                fprintf(sim_log, "Warning: requested working set size (%d MB) is too high, unable to set it\n", sim_ws_max);
            sim_ws_settings_changed = FALSE;
            return;
        }

        SIZE_T ws_min = sim_ws_min;
        SIZE_T ws_max = sim_ws_max;
        ws_min *= mb;
        ws_max *= mb;

        if (! SetProcessWorkingSetSize(GetCurrentProcess(), ws_min, ws_max))
        {
            smp_printf("Warning: Unable to set requested working set size (%d MB)\n", sim_ws_max);
            if (sim_log)
                fprintf(sim_log, "Warning: Unable to set requested working set size (%d MB)\n", sim_ws_max);
            sim_ws_settings_changed = FALSE;
            return;
        }

        if (! sim_ws_prefaulted)
        {
            sim_prefault_memory();
            sim_ws_prefaulted = TRUE;
        }
    }

    sim_ws_settings_changed = FALSE;
}
#elif defined(__linux)
#  include <sys/mman.h>
void sim_ws_setup()
{
    sim_ws_prefaulted = FALSE;

    if (sim_ws_lock || sim_ws_min || sim_ws_max)
    {
        if (mlockall(MCL_CURRENT))
        {
            smp_printf("Warning: Unable to lock %s in memory (not critical)\n", sim_name);
            if (sim_log)
                fprintf(sim_log, "Warning: Unable to lock %s in memory (not critical)\n", sim_name);
            sim_ws_settings_changed = FALSE;
            return;
        }
    }
    else
    {
        munlockall();
        sim_ws_settings_changed = FALSE;
        return;
    }

    if (! sim_ws_prefaulted)
    {
        sim_prefault_memory();
        sim_ws_prefaulted = TRUE;
    }

    sim_ws_settings_changed = FALSE;
}
#elif defined(__APPLE__)
void sim_ws_setup()
{
    if (sim_ws_min || sim_ws_max || sim_ws_lock)
    {
        if (! sim_ws_prefaulted)
        {
            sim_prefault_memory();
            sim_ws_prefaulted = TRUE;
        }
    }
    else
    {
        sim_ws_prefaulted = FALSE;
    }

    sim_ws_settings_changed = FALSE;
}
#else
void sim_ws_setup()
{
    sim_ws_settings_changed = FALSE;
}
#endif

void sim_prefault_memory()
{
    cpu_prefault_memory();
}
