/* sim_timer.c: simulator timer library

   Copyright (c) 1993-2008, Robert M Supnik

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

   22-Sep-08    RMS     Added "stability threshold" for idle routine
   27-May-08    RMS     Fixed bug in Linux idle routines (from Walter Mueller)
   18-Jun-07    RMS     Modified idle to exclude counted delays
   22-Mar-07    RMS     Added sim_rtcn_init_all
   17-Oct-06    RMS     Added idle support (based on work by Mark Pizzolato)
                        Added throttle support
   16-Aug-05    RMS     Fixed C++ declaration and cast problems
   02-Jan-04    RMS     Split out from SCP

   This library includes the following routines:

   sim_timer_init -     initialize timing system
   sim_rtc_init -       initialize calibration
   sim_rtc_calb -       calibrate clock
   sim_timer_init -     initialize timing system
   sim_idle -           virtual machine idle
   sim_os_msec  -       return elapsed time in msec
   sim_os_sleep -       sleep specified number of seconds
   sim_os_ms_sleep -    sleep specified number of milliseconds

   The calibration, idle, and throttle routines are OS-independent; the _os_
   routines are not.
*/

#if defined (_WIN32)
#  include <windows.h>
#endif

#include "sim_defs.h"
#include <ctype.h>

#if defined(VM_VAX_MP)
#  define SIM_NO_THROTTLING
#endif

t_bool sim_idle_enab = FALSE;                           /* global flag */

#if defined(HAVE_POSIX_CLOCK_ID)
/* Linux and Unix clocks */
t_bool sim_posix_have_clock_id = FALSE;
clockid_t sim_posix_clock_id;
#endif

static uint32 sim_idle_rate_us = 0;
uint32 sim_idle_stable = SIM_IDLE_STDFLT;
static uint32 sim_throt_ms_start = 0;
static uint32 sim_throt_ms_stop = 0;
static uint32 sim_throt_type = SIM_THROT_NONE;
static uint32 sim_throt_val = 0;
static uint32 sim_throt_state = 0;
static int32 sim_throt_wait = 0;
extern int32 sim_switches;
extern SMP_FILE *sim_log;

/* CPU rate (instructions/second) averaging data */
smp_lock* cpu_cycles_per_second_lock = NULL;
atomic_uint32_var cpu_cycles_per_second = atomic_var_init(0);
static cpu_set av_valid;
static uint32 av_checkin[SIM_MAX_CPUS];
static uint32 av_ips[SIM_MAX_CPUS];

t_stat sim_throt_svc (RUN_SVC_DECL, UNIT *uptr);
static int32 sim_rtcn_calb_synclk (RUN_DECL, int32 ticksper, int32 tmr, t_bool* valid, uint32* os_msec);
static int32 sim_rtcn_calb_nosynclk (RUN_DECL, int32 ticksper, int32 tmr, t_bool* valid, uint32* os_msec);

UNIT sim_throt_unit UDATA_SINGLE (&sim_throt_svc, 0, 0);

/* OS-dependent timer and clock routines */

class sim_delta_timer_impl;

class sim_delta_timer_sample
{
    friend class sim_delta_timer_impl;

protected:
    uint32 us_since(RUN_DECL, sim_delta_timer_impl* dtimer, sim_delta_timer_sample* prev);

protected:
    t_bool valid;
    uint32 cycles;
    uint32 min_since;

#if defined(_WIN32)
    LARGE_INTEGER pc;
    uint32 ms;
#elif defined(HAVE_POSIX_CLOCK_ID)
    struct timespec tsv;
#else
    struct timeval tv;
#endif
};

class sim_delta_timer_impl : public sim_delta_timer
{
    friend class sim_delta_timer_sample;
protected:
#if defined(_WIN32)
    LARGE_INTEGER performance_counter_frequency;
#endif
    sim_delta_timer_sample start;
    sim_delta_timer_sample last;
    sim_delta_timer_sample curr;
    void sample(RUN_DECL, sim_delta_timer_sample* sample);

public:
    sim_delta_timer_impl();  
    ~sim_delta_timer_impl();
    t_bool init(t_bool dothrow = TRUE);
    void adjust_cycle_base(uint32 d);
    void begin(RUN_DECL);
    void sample(RUN_DECL);
    uint32 us_since_start(RUN_DECL);
    uint32 us_since_last(RUN_DECL);
};

sim_delta_timer* sim_delta_timer::create(t_bool dothrow)
{
    sim_delta_timer_impl* dt = new sim_delta_timer_impl();
    if (! dt->init(dothrow))
    {
        delete dt;
        dt = NULL;
    }
    return dt;
}

/* VMS */

#if defined (VMS)

#if defined (__VAX)
#define sys$gettim SYS$GETTIM
#endif

#include <starlet.h>
#include <lib$routines.h>
#include <unistd.h>

const t_bool rtc_avail = TRUE;

uint32 sim_os_msec ()
{
    uint32 quo, htod, tod[2];
    int32 i;

    sys$gettim (tod);                                       /* time 0.1usec */

    /* To convert to msec, must divide a 64b quantity by 10000.  This is actually done
       by dividing the 96b quantity 0'time by 10000, producing 64b of quotient, the
       high 32b of which are discarded.  This can probably be done by a clever multiply...
    */

    quo = htod = 0;
    for (i = 0; i < 64; i++)                                /* 64b quo */
    {
        htod = (htod << 1) | ((tod[1] >> 31) & 1);          /* shift divd */
        tod[1] = (tod[1] << 1) | ((tod[0] >> 31) & 1);
        tod[0] = tod[0] << 1;
        quo = quo << 1;                                     /* shift quo */
        if (htod >= 10000)                                  /* divd work? */
        {
            htod = htod - 10000;                            /* subtract */
            quo = quo | 1;                                  /* set quo bit */
        }
    }
    return quo;
}

void sim_os_sleep (unsigned int sec)
{
    sleep (sec);
}

uint32 sim_os_ms_sleep_init (void)
{
#if defined (__VAX)
    return 10;                                          /* VAX/VMS is 10ms */
#else
    return 1;                                           /* Alpha/VMS is 1ms */
#endif
}

uint32 sim_os_ms_sleep (unsigned int msec)
{
    uint32 stime = sim_os_msec ();
    uint32 qtime[2];
    int32 nsfactor = -10000;
    static int32 zero = 0;

    lib$emul (&msec, &nsfactor, &zero, qtime);
    sys$setimr (2, qtime, 0, 0);
    sys$waitfr (2);
    return sim_os_msec () - stime;
}

/* Win32 routines */

#elif defined (_WIN32)

const t_bool rtc_avail = TRUE;
static void sim_os_gettime_vms_init();

uint32 sim_os_msec ()
{
    if (sim_idle_rate_us)
        return timeGetTime ();
    else
        return GetTickCount ();
}

void sim_os_sleep (unsigned int sec)
{
    Sleep (sec * 1000);
}

static void sim_timer_exit (void)
{
    timeEndPeriod (sim_idle_rate_us / 1000);
}

uint32 sim_os_us_sleep_init (void)
{
    TIMECAPS timers;

    sim_os_gettime_vms_init();

    if (timeGetDevCaps (&timers, sizeof (timers)) != TIMERR_NOERROR)
        return 0;
    if ((timers.wPeriodMin == 0) || (timers.wPeriodMin > SIM_IDLE_MAX))
        return 0;
    if (timeBeginPeriod (timers.wPeriodMin) != TIMERR_NOERROR)
        return 0;
    atexit (sim_timer_exit);
    Sleep (1);
    Sleep (1);
    Sleep (1);
    Sleep (1);
    Sleep (1);
    return timers.wPeriodMin * 1000;                       /* sim_idle_rate_us */
}

uint32 sim_os_ms_sleep (unsigned int msec)
{
    uint32 stime = sim_os_msec();
    Sleep (msec);
    return sim_os_msec () - stime;
}

sim_delta_timer_impl::sim_delta_timer_impl()
{
    start.valid = FALSE;
    last.valid = FALSE;
    curr.valid = FALSE;
}

sim_delta_timer_impl::~sim_delta_timer_impl()
{
}

t_bool sim_delta_timer_impl::init(t_bool dothrow)
{
    if (QueryPerformanceFrequency(& performance_counter_frequency))
    {
        return TRUE;
    }
    else if (dothrow)
    {
        panic("Unable to create delta timer");
        never_returns_bool_t;
    }
    else
    {
        return FALSE;
    }
}

#define LARGE_INTEGER_SET_ZERO(a)  do {  (a).LowPart = 0; (a).HighPart = 0; } while (0)
#define LARGE_INTEGER_TO_UINT64(a) ((((t_uint64) (uint32) (a).HighPart) << 32) | (t_uint64) (uint32) (a).LowPart)

void sim_delta_timer_impl::sample(RUN_DECL, sim_delta_timer_sample* sample)
{
    LARGE_INTEGER_SET_ZERO(sample->pc);
    QueryPerformanceCounter(& sample->pc);
    sample->ms = sim_os_msec();
    if (cpu_unit)
        sample->cycles = CPU_CURRENT_CYCLES;
    sample->valid = TRUE;
    sample->min_since = 0;
}

void sim_delta_timer_impl::begin(RUN_DECL)
{
    curr.valid = FALSE;
    last.valid = FALSE;
    sample(RUN_PASS, &start);
}

void sim_delta_timer_impl::sample(RUN_DECL)
{
    if (curr.valid)
        last = curr;
    sample(RUN_PASS, &curr);
}

void sim_delta_timer_impl::adjust_cycle_base(uint32 d)
{
    start.cycles += d;
    last.cycles += d;
    curr.cycles += d;
}

uint32 sim_delta_timer_impl::us_since_start(RUN_DECL)
{
    return curr.us_since(RUN_PASS, this, &start);
}

uint32 sim_delta_timer_impl::us_since_last(RUN_DECL)
{
    if (last.valid)
        return curr.us_since(RUN_PASS, this, &last);
    else
        return curr.us_since(RUN_PASS, this, &start);
}

uint32 sim_delta_timer_sample::us_since(RUN_DECL, sim_delta_timer_impl* dtimer, sim_delta_timer_sample* prev)
{
    uint32 res;
    uint32 x_pc;

    uint32 x_ms = 1000 * (ms - prev->ms);

    t_uint64 pc1 = LARGE_INTEGER_TO_UINT64(pc);
    t_uint64 pc0 = LARGE_INTEGER_TO_UINT64(prev->pc);

    if (pc1 < pc0)
        x_pc = 0;
    else
        x_pc = (uint32) ((double) (pc1 - pc0) * 1000.0 * 1000.0 / 
                         (double) LARGE_INTEGER_TO_UINT64(dtimer->performance_counter_frequency));

    /*
     * Data returned by QueryPerformanceCounter is more precise (has higher temporal resolution) _when_ it is valid,
     * however validity is exactly the problem. QueryPerformanceCounter may utilize, under the hood, variours kinds
     * of clocks, such as local APIC, HPET, CPU (RD)TSC, programmable interval timer (PIT), ACPI PM or good old RTC
     * timers. Depending on the clock used and operating system version, some timers may be unreliable and can behave
     * erratically, for example if QueryPerformanceCounter relies on CPU TSC timer, and the thread is migrated 
     * for execution to another processor, RDTSC value can jump, including jumping backwards; furthermore, 
     * if processor enters power-saving mode, CPU TSC clock can throttle down on earlier CPUs, but not on later models 
     * (that have TSC INVARIANT feature), or likewise can speed up during turbo boost on earlier models, but not
     * TSC INVARIANT models, and when it does slow-down, the amount can depend on the Cx/Px mode and its duration
     * (which we do not have access to in any event), and similarly for speed-ups; futhermore, some versions of Windows
     * may compensate for the frequency drift, whereas others do not.
     *
     * Multimedia clocks utlizied by sim_os_msec (timeGetTime), by comparison, are generally more reliable,
     * but have lower resolution.
     *
     * Hence we use QueryPerformanceCounter readings when they seem to be valid (in rough agreement with sim_os_msec),
     * otherwise fallback to the value provided by sim_os_msec.
     */

    if (x_ms == 0)
    {
        if (x_pc <= 1500)
            res = (uint32) x_pc;
        else
            res = 2;
    }
    else if (x_pc >= x_ms / 10 && x_pc < 2 * x_ms + x_ms / 2)
    {
        res = (uint32) x_pc;
    }
    else
    {
        res = x_ms;
    }

    if (res < prev->min_since)
        res = prev->min_since;

    prev->min_since = res;

    return res;
}

#undef LARGE_INTEGER_SET_ZERO
#undef LARGE_INTEGER_TO_UINT64

/*
 * Get host system local time in VMS format.
 *
 * VMS time format is the number of 100-nanosecond intervals
 * since 00:00 o’clock, November 17, 1858.
 */
static t_bool sim_os_gettime_vms_inited = FALSE;
static t_int64 vms_win_delta;

SIM_INLINE static t_uint64 FileTimeToUInt64(const FILETIME* p)
{
    t_uint64 res = p->dwHighDateTime;
    res = (res << 32) | p->dwLowDateTime;
    return res;
}

static void sim_os_gettime_vms_init()
{
    if (! sim_os_gettime_vms_inited)
    {
        SYSTEMTIME vmsBaseTime;
        FILETIME vmsBaseFile;

        /* November 17, 1858 */
        memset(& vmsBaseTime, 0, sizeof vmsBaseTime);
        vmsBaseTime.wYear = 1858;
        vmsBaseTime.wMonth = 11;
        vmsBaseTime.wDay = 17;
        vmsBaseTime.wDayOfWeek = 3;
        SystemTimeToFileTime(& vmsBaseTime, & vmsBaseFile);

        vms_win_delta = (t_int64) FileTimeToUInt64(& vmsBaseFile);

        sim_os_gettime_vms_inited = TRUE;
    }
}

void sim_os_gettime_vms(uint32* p_vms_time)
{
    SYSTEMTIME localTime;
    FILETIME localFile;
    GetLocalTime(& localTime);
    SystemTimeToFileTime(& localTime, & localFile);
    t_int64 local_time = (t_int64) FileTimeToUInt64(& localFile);
    local_time -= vms_win_delta; 
    t_uint64 vms_time = (t_uint64) local_time;
    p_vms_time[0] = (uint32) (vms_time & 0xFFFFFFFF);
    p_vms_time[1] = (uint32) ((vms_time >> 32) & 0xFFFFFFFF);
}

/* OS/2 routines, from Bruce Ray */

#elif defined (__OS2__)

const t_bool rtc_avail = FALSE;

uint32 sim_os_msec ()
{
    return 0;
}

void sim_os_sleep (unsigned int sec)
{
    return;
}

uint32 sim_os_ms_sleep_init (void)
{
    return FALSE;
}

uint32 sim_os_ms_sleep (unsigned int msec)
{
    return 0;
}

/* Metrowerks CodeWarrior Macintosh routines, from Ben Supnik */

#elif defined (__MWERKS__) && defined (macintosh)

#include <Timer.h>
#include <Mactypes.h>
#include <sioux.h>
#include <unistd.h>
#include <siouxglobals.h>
#define NANOS_PER_MILLI     1000000
#define MILLIS_PER_SEC      1000

const t_bool rtc_avail = TRUE;

uint32 sim_os_msec (void)
{
    unsigned long long micros;
    UnsignedWide macMicros;
    unsigned long millis;

    Microseconds (&macMicros);
    micros = *((unsigned long long *) &macMicros);
    millis = micros / 1000LL;
    return (uint32) millis;
}

void sim_os_sleep (unsigned int sec)
{
    sleep (sec);
}

uint32 sim_os_ms_sleep_init (void)
{
    return 1;
}

uint32 sim_os_ms_sleep (unsigned int milliseconds)
{
    uint32 stime = sim_os_msec ();
    struct timespec treq;

    treq.tv_sec = milliseconds / MILLIS_PER_SEC;
    treq.tv_nsec = (milliseconds % MILLIS_PER_SEC) * NANOS_PER_MILLI;
    (void) nanosleep (&treq, NULL);
    return sim_os_msec () - stime;
}

#else

/* UNIX routines */

#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#define NANOS_PER_MILLI     1000000
#define MILLIS_PER_SEC      1000
#define sleep1Samples       100

const t_bool rtc_avail = TRUE;

uint32 sim_os_msec ()
{
    struct timeval cur;
    struct timezone foo;
    uint32 msec;

    gettimeofday (&cur, &foo);
    msec = ((uint32) cur.tv_sec) * 1000 + ((uint32) cur.tv_usec + 500) / 1000;
    return msec;
}

void sim_os_sleep (unsigned int sec)
{
    sleep (sec);
}

#if defined(HAVE_POSIX_CLOCK_ID)
static void init_insane_resolution(struct timespec* tsv)
{
    tsv->tv_sec = tsv->tv_nsec = INT32_MAX;
}
static t_bool is_sane_resolution(struct timespec* tsv)
{
    return tsv->tv_sec == 0 && tsv->tv_nsec > 0 && tsv->tv_nsec <= (1000 * 1000 * 1000 / 50);
}
#endif

uint32 sim_os_us_sleep_init (void)
{
#if defined(HAVE_POSIX_CLOCK_ID)

    struct timespec tsv;
    struct timespec clkres;
    uint32 msec;

    clkres.tv_nsec = 0;                /* initialize to suppress false GCC warning */

#if defined(CLOCK_MONOTONIC_RAW)
    init_insane_resolution(& tsv);
    if (!sim_posix_have_clock_id && 0 == clock_getres(CLOCK_MONOTONIC_RAW, & tsv) && is_sane_resolution(& tsv))
    {
        clkres = tsv;
        sim_posix_clock_id = CLOCK_MONOTONIC_RAW;
        sim_posix_have_clock_id = TRUE;
    }
#endif

#if defined(CLOCK_MONOTONIC)
    init_insane_resolution(& tsv);
    if (!sim_posix_have_clock_id && 0 == clock_getres(CLOCK_MONOTONIC, & tsv) && is_sane_resolution(& tsv))
    {
        clkres = tsv;
        sim_posix_clock_id = CLOCK_MONOTONIC;
        sim_posix_have_clock_id = TRUE;
    }
#endif

#if defined(CLOCK_REALTIME)
    init_insane_resolution(& tsv);
    if (!sim_posix_have_clock_id && 0 == clock_getres(CLOCK_REALTIME, & tsv) && is_sane_resolution(& tsv))
    {
        clkres = tsv;
        sim_posix_clock_id = CLOCK_REALTIME;
        sim_posix_have_clock_id = TRUE;
    }
#endif

    /* 
     * ToDo: Unix/Linux sleep timer resolution
     *
     * For Linux see
     *
     *     http://www.advenage.com/topics/linux-timer-interrupt-frequency.php
     *     https://rt.wiki.kernel.org/index.php/Cyclictest
     *
     * Specifically, the following Linux kernel config parameters are relevant:
     *
     *     CONFIG_HIGH_RES_TIMERS
     *     CONFIG_HPET_TIMER
     *     CONFIG_X86_PM_TIMER
     *     CONFIG_SCx200HR_TIMER
     *     CONFIG_TIMER_STATS
     *     CONFIG_NO_HZ
     *     CONFIG_HZ
     *     CONFIG_HZ_100
     *     CONFIG_HZ_250
     *     CONFIG_HZ_300
     *     CONFIG_HZ_1000
     *
     */

    if (sim_posix_have_clock_id)
    {
        msec = (clkres.tv_nsec + (NANOS_PER_MILLI - 1)) / NANOS_PER_MILLI;
        if (msec == 0)  msec = 1;
    }
    else
    {
        /* 
         * Default (optimistically) to 1 msec.
         * Benchmarking Linux 2.6.38 on x86 3.2 GHz systems shows minimum sleep delay
         * ranging from 0.25 to 1.5 ms.
         */
        msec = 1;
    }

    if (msec > SIM_IDLE_MAX)
        return 0;

    return msec * 1000;

#elif defined(__APPLE__)

    /*
     * Statistical benchmarking of OS X Lion Server 10.7.4 (using "clocks" test) shows that average minimum
     * sleep request roundtrip is about 7 usec (when using 500ns/1 usec sleep time). Thus sleep delay
     * below 7 usec cannot be satisfied satisfactory. Hence we could use 7 usec as a cut-off time value for
     * sleep/spin decision. Actual average sleep delay when requesting sleep time of 7 usec is 12 usec.
     * In more practical workloads however "7" is likely to be wasteful, so use 30 usec as more practical
     * threshold. This also covers latency of RQ/TQ requests satisfied by IOP thread almost immediatelly
     * (such as via host file system cache) without having to incur rescheduling delays.
     */

    return 30;

#else                                                   /* others */

    uint32 i, t1, t2, tot, tim;

    /* sample sleep times, elevate priority for this */
    smp_set_thread_priority(SIMH_THREAD_PRIORITY_CPU_CALIBRATION);
    for (i = 0, tot = 0; i < sleep1Samples; i++)
    {
        t1 = sim_os_msec ();
        sim_os_ms_sleep (1);
        t2 = sim_os_msec ();
        tot += (t2 - t1);
    }
    smp_set_thread_priority(SIMH_THREAD_PRIORITY_CONSOLE_PAUSED);

    tim = (tot + (sleep1Samples - 1)) / sleep1Samples;

    if (tim == 0)
        tim = 1;
    else if (tim > SIM_IDLE_MAX)
        tim = 0;

    return tim * 1000;

#endif
}

uint32 sim_os_ms_sleep (unsigned int milliseconds)
{
    uint32 stime = sim_os_msec ();
    struct timespec treq;

    treq.tv_sec = milliseconds / MILLIS_PER_SEC;
    treq.tv_nsec = (milliseconds % MILLIS_PER_SEC) * NANOS_PER_MILLI;
    (void) nanosleep (&treq, NULL);
    return sim_os_msec () - stime;
}

sim_delta_timer_impl::sim_delta_timer_impl()
{
    start.valid = FALSE;
    last.valid = FALSE;
    curr.valid = FALSE;
}

sim_delta_timer_impl::~sim_delta_timer_impl()
{
}

t_bool sim_delta_timer_impl::init(t_bool dothrow)
{
    return TRUE;
}

void sim_delta_timer_impl::sample(RUN_DECL, sim_delta_timer_sample* sample)
{
#if defined(HAVE_POSIX_CLOCK_ID)
    if (sim_posix_have_clock_id)
    {
        clock_gettime(sim_posix_clock_id, & sample->tsv);
    }
    else
    {
        struct timeval tv;
        gettimeofday(& tv, NULL);
        sample->tsv.tv_sec = tv.tv_sec;
        sample->tsv.tv_nsec = tv.tv_usec * 1000;
    }
#else
    /*
     * OS X note: under OS X / Mach it may be tempting to use directly
     *
     *    clock_get_attributes
     *    clock_set_attributes
     *    clock_get_time
     *    clock_map_time
     *    host_get_clock_service
     *
     * for get_now and sim_delta_time_impl::sample, however unfortunately OS X does not implement clock_map_time
     * (and even if it did, there would be problems with atomicity and ordering of access), whereas for all other
     * clocks exposed granularity returned by clock_get_attrbutes is 1 sec. Futhermore, composite RTC/TSC based
     * timer provides better granularity.
     *
     * We could also use mach_absolute_time which on older i386 versions was but a wrapper around clock_get_time,
     * but on newer x86 and x64 versions is RTC+RDTSC based, see
     *
     *     http://www.opensource.apple.com/source/Libc/Libc-763.13/i386/sys/mach_absolute_time.c
     *     http://www.opensource.apple.com/source/Libc/Libc-763.13/i386/sys/mach_absolute_time_asm.s
     *     http://www.opensource.apple.com/source/Libc/Libc-763.13/x86_64/sys/nanotime.s
     *
     * i.e. time comes from real-time clock, and is further adjusted by RDTSC cycles accumulated from the recent
     * clock tick, thus giving a highly accurate reading. This would give us nano-range precision.
     *
     * For our purposes however we are all well with microsecond-range precision, which is provided on OS X
     * by gettimeofday (see comment in sim_threads.cpp function get_now).
     */
    gettimeofday(& sample->tv, NULL);
#endif

    if (cpu_unit)
        sample->cycles = CPU_CURRENT_CYCLES;

    sample->valid = TRUE;
    sample->min_since = 0;
}

void sim_delta_timer_impl::begin(RUN_DECL)
{
    curr.valid = FALSE;
    last.valid = FALSE;
    sample(RUN_PASS, &start);
}

void sim_delta_timer_impl::sample(RUN_DECL)
{
    if (curr.valid)
        last = curr;
    sample(RUN_PASS, &curr);
}

void sim_delta_timer_impl::adjust_cycle_base(uint32 d)
{
    start.cycles += d;
    last.cycles += d;
    curr.cycles += d;
}

uint32 sim_delta_timer_impl::us_since_start(RUN_DECL)
{
    return curr.us_since(RUN_PASS, this, &start);
}

uint32 sim_delta_timer_impl::us_since_last(RUN_DECL)
{
    if (last.valid)
        return curr.us_since(RUN_PASS, this, &last);
    else
        return curr.us_since(RUN_PASS, this, &start);
}

uint32 sim_delta_timer_sample::us_since(RUN_DECL, sim_delta_timer_impl* dtimer, sim_delta_timer_sample* prev)
{
    double delta = 0;

#if defined(HAVE_POSIX_CLOCK_ID)
    if (tsv.tv_sec >= prev->tsv.tv_sec)
    {
        delta = (double) (tsv.tv_sec - prev->tsv.tv_sec) * 1000 * 1000;
        delta += (double) (tsv.tv_nsec - prev->tsv.tv_nsec) / 1000;
    }
#else
    if (tv.tv_sec >= prev->tv.tv_sec)
    {
        delta = (tv.tv_sec - prev->tv.tv_sec) * 1000 * 1000;
        delta += tv.tv_usec - prev->tv.tv_usec;
    }
#endif

    /* time jumped backwards? */
    if (delta < 0)  delta = 0;

    /* time jumped forward way too much? */
    const double delta_max = 0.9 * (double) UINT32_MAX;
    if (delta > 0.9 * delta_max)
        delta = delta_max;

    uint32 res = (uint32) delta;
    if (res == 0)  res = 1;

    if (res < prev->min_since)
        res = prev->min_since;

    prev->min_since = res;

    return res;
}

/*
 * Get host system local time in VMS format.
 *
 * VMS time format is the number of 100-nanosecond intervals
 * since 00:00 o’clock, November 17, 1858.
 */
void sim_os_gettime_vms(uint32* p_vms_time)
{
    struct timeval tv;
    struct timezone tz;

    /* Linux gettimeofday returns UTC in tv */
    gettimeofday(&tv, &tz);

    /* local time */
    tv.tv_sec -= tz.tz_minuteswest * 60;

#if defined(__APPLE__)
    /* adjust for DST */
    /* note that Linux does not use tz_dsttime */
    if (tz.tz_dsttime != DST_NONE)
        tv.tv_sec += 60 * 60;
#endif

    /* ToDo: implement for hosts with no 64-bit arithmetics */

    /* number of 100ns units between November 17, 1858 and January 1, 1970 */
    static const t_uint64 ux_vms_delta = 35067168000000000ULL;

    t_uint64 vms_time = tv.tv_sec;
    vms_time *= 1000 * 1000;
    vms_time += tv.tv_usec;
    vms_time *= 10;             /* 100 ns units since local January 1, 1970 */
    vms_time += ux_vms_delta;   /* 100 ns units since November 17, 1858 */

    p_vms_time[0] = (uint32) (vms_time & 0xFFFFFFFF);
    p_vms_time[1] = (uint32) ((vms_time >> 32) & 0xFFFFFFFF);
}

#endif

/* OS independent clock calibration package */

// int32 rtc_ticks[SIM_NTIMERS] = { 0 };            /* ticks */
// int32 rtc_hz[SIM_NTIMERS] = { 0 };               /* tick rate */
// uint32 rtc_rtime[SIM_NTIMERS] = { 0 };           /* real time */
// uint32 rtc_vtime[SIM_NTIMERS] = { 0 };           /* virtual time */
// uint32 rtc_nxintv[SIM_NTIMERS] = { 0 };          /* next interval */
// int32 rtc_based[SIM_NTIMERS] = { 0 };            /* base delay */
// int32 rtc_currd[SIM_NTIMERS] = { 0 };            /* current delay */
// int32 rtc_initd[SIM_NTIMERS] = { 0 };            /* initial delay */
// uint32 rtc_elapsed[SIM_NTIMERS] = { 0 };         /* sec since init */

#define rtc_ticks (cpu_unit->cpu_rtc_ticks)
#define rtc_hz (cpu_unit->cpu_rtc_hz)
#define rtc_rtime (cpu_unit->cpu_rtc_rtime)
#define rtc_vtime (cpu_unit->cpu_rtc_vtime)
#define rtc_nxintv (cpu_unit->cpu_rtc_nxintv)
#define rtc_based (cpu_unit->cpu_rtc_based)
#define rtc_currd (cpu_unit->cpu_rtc_currd)
#define rtc_initd (cpu_unit->cpu_rtc_initd)
#define rtc_elapsed (cpu_unit->cpu_rtc_elapsed)

void sim_rtcn_init_all (RUN_DECL, t_bool keep_stable)
{
    for (uint32 i = 0; i < SIM_NTIMERS; i++)
    {
        /*
         * ToDo: Do not start calibration timer from the very start (thus resetting rtc_elapsed[])
         *       if (keep_stable && rtc_elapsed[i] >= sim_idle_stable). This reset, as it happens now,
         *       causes simulator being unable to go to idle sleep for some time after SIMH console
         *       command "CONTNINUE".
         */
        if (rtc_initd[i] != 0)
            sim_rtcn_init (RUN_PASS, rtc_initd[i], i);
    }

    /* reset rate averaging data */
    if (! keep_stable)
        atomic_var(cpu_cycles_per_second) = 0;
    av_valid.clear_all();
    memzero(av_checkin);
}

/*
 * Initialize timer on the primary processor.
 *
 * For secondaries actual initialization is done by sim_rtcn_init_secondary_cpu when it is started, 
 * but sim_rtcn_init still can be called merely to read the initial clock rate calibration.
 */
int32 sim_rtcn_init (RUN_DECL, int32 time, int32 tmr)
{
    if (! cpu_unit->is_primary_cpu())
        return rtc_currd[tmr];

    if (time == 0)
        time = 1;
    if (tmr < 0 || tmr >= SIM_NTIMERS)
        return time;
    rtc_rtime[tmr] = sim_os_msec ();
    rtc_vtime[tmr] = rtc_rtime[tmr];
    rtc_nxintv[tmr] = 1000;
    rtc_ticks[tmr] = 0;
    rtc_hz[tmr] = 0;
    rtc_based[tmr] = time;
    rtc_currd[tmr] = time;
    rtc_initd[tmr] = time;
    rtc_elapsed[tmr] = 0;

    if (tmr == TMR_CLK)
    {
        if (use_clock_thread)
        {
            cpu_unit->cpu_last_tslice_tick_cycles = CPU_CURRENT_CYCLES;
            cpu_unit->cpu_last_second_tick_cycles = CPU_CURRENT_CYCLES;
            cpu_unit->cpu_idle_sleep_us = 0;
            cpu_unit->cpu_idle_sleep_cycles = 0;
        }

        atomic_var(tmr_poll) = rtc_currd[tmr];                         /* set tmr poll */
        atomic_var(tmxr_poll) = atomic_var(tmr_poll) * TMXR_MULT;      /* set mux poll */
    }

    return rtc_currd[tmr];
}

/* 
 * Initialize timer on the secondary processor (cpu_unit) when starting it
 * using data from originating processir (src).
 */
void sim_rtcn_init_secondary_cpu (CPU_UNIT* cpu_unit, CPU_UNIT* src, int32 tmr, int32 time, t_bool clone)
{
    if (clone)
    {
        rtc_rtime[tmr] = sim_os_msec();
        rtc_vtime[tmr] = rtc_rtime[tmr];
        rtc_nxintv[tmr] = 1000;
        rtc_ticks[tmr] = 0;
        rtc_hz[tmr] = src->cpu_rtc_hz[tmr];
        rtc_based[tmr] = src->cpu_rtc_based[tmr];
        rtc_currd[tmr] = src->cpu_rtc_currd[tmr];
        rtc_initd[tmr] = src->cpu_rtc_initd[tmr];
        rtc_elapsed[tmr] = src->cpu_rtc_elapsed[tmr];
        if (tmr == TMR_CLK && use_clock_thread)
        {
            cpu_unit->cpu_idle_sleep_us = src->cpu_idle_sleep_us;
            cpu_unit->cpu_idle_sleep_cycles = src->cpu_idle_sleep_cycles;
        }
    }
    else
    {
        if (time == 0)
            time = 1;
        rtc_rtime[tmr] = sim_os_msec ();
        rtc_vtime[tmr] = rtc_rtime[tmr];
        rtc_nxintv[tmr] = 1000;
        rtc_ticks[tmr] = 0;
        rtc_hz[tmr] = 0;
        rtc_based[tmr] = time;
        rtc_currd[tmr] = time;
        rtc_initd[tmr] = time;
        rtc_elapsed[tmr] = 0;
        if (tmr == TMR_CLK && use_clock_thread)
        {
            cpu_unit->cpu_idle_sleep_us = 0;
            cpu_unit->cpu_idle_sleep_cycles = 0;
        }
    }

    if (tmr == TMR_CLK)
    {
        if (use_clock_thread)
        {
            cpu_unit->cpu_last_tslice_tick_cycles = CPU_CURRENT_CYCLES;
            cpu_unit->cpu_last_second_tick_cycles = CPU_CURRENT_CYCLES;
        }

        rtc_currd[tmr] = weak_read_var(cpu_cycles_per_second) / rtc_hz[tmr];

        if (rtc_currd[tmr] <= 0)
            rtc_currd[tmr] = 1;
    }
}

/* evaluates expected number of CPU cycles per clock tick */
int32 sim_rtcn_calb (RUN_DECL, int32 ticksper, int32 tmr)
{
    t_bool valid = FALSE;          /* initialize to suppress false GCC warning message */
    uint32 os_msec = 0;            /* ... */
    int32 t;

    if (use_clock_thread)
        t = sim_rtcn_calb_synclk(RUN_PASS, ticksper, tmr, & valid, & os_msec);
    else
        t = sim_rtcn_calb_nosynclk(RUN_PASS, ticksper, tmr, & valid, & os_msec);

    if (tmr == TMR_CLK)
    {
        if (rtc_ticks[tmr] == 0 && rtc_elapsed[tmr] != 0)
        {
            /* average with other VCPUs */
            cpu_update_cycles_per_second(RUN_PASS, (uint32) (rtc_currd[tmr] * rtc_hz[tmr]), valid, os_msec);
            rtc_currd[tmr] = weak_read_var(cpu_cycles_per_second) / rtc_hz[tmr];

            /* never negative or zero! */
            if (rtc_currd[tmr] <= 0)
                rtc_currd[tmr] = 1;

            /* ToDo: rethink the interference of rtc_currd averaging with rtc_based in nosynclk case */

            t = rtc_currd[tmr];

            /* execute per-cpu once-a-second routine */
            cpu_once_a_second(RUN_PASS);
        }
        else if (rtc_elapsed[tmr] == 0 && cpu_unit->is_primary_cpu())
        {
            atomic_var(tmr_poll) = rtc_currd[tmr];                         /* set tmr poll */
            atomic_var(tmxr_poll) = atomic_var(tmr_poll) * TMXR_MULT;      /* set mux poll */
        }
    }

    return t;
}

int32 sim_rtcn_calb_synclk (RUN_DECL, int32 ticksper, int32 tmr, t_bool* valid, uint32* os_msec)
{
    if (unlikely(tmr != TMR_CLK))
        panic("Timer calibration: not TMR_CLK");

    if (unlikely(ticksper != CLK_TPS))
        panic("Timer calibration: unexpected clock frequency");

    /*
     * 1-second interval is sliced into "ntslices" slices,
     * each composed of "sliceticks" clock ticks.
     *
     * At the end of each slice, rtc_currd is adjusted using simple moving average.
     */
    const int ntslices = 10;
    const int sliceticks = CLK_TPS / ntslices;

    if (unlikely(ntslices * sliceticks != CLK_TPS))
        panic("Timer calibration: internal error (invalid timeslice definition)");

    rtc_hz[tmr] = ticksper;
    rtc_ticks[tmr] = rtc_ticks[tmr] + 1;                        /* count ticks */

    if (rtc_ticks[tmr] % sliceticks)                            /* intraslice? */
    {                                                           
        /* do nothing */
    }
    else if (rtc_ticks[tmr] < ticksper)                         /* 1 sec yet? */
    {
        /* 
         * calculate intra-virtual-second rate adjustment using moving average
         * (maters only during the very first second of primary VCPU execution)
         */
        if (unlikely(rtc_elapsed[tmr] == 0) && cpu_unit->is_primary_cpu())
        {
            /* cycles since last slice */
            uint32 dcycles = CPU_CURRENT_CYCLES - cpu_unit->cpu_last_tslice_tick_cycles;
            cpu_unit->cpu_last_tslice_tick_cycles = CPU_CURRENT_CYCLES;

            /* apply the adjustment */
            rtc_currd[tmr] = ((dcycles / sliceticks) + (ntslices - 1) * rtc_currd[tmr]) / ntslices;
        }
    }
    else                                                        /* once a virtual second */
    {
        /* next virtual second */
        rtc_ticks[tmr] = 0;                                     /* reset ticks */
        rtc_elapsed[tmr] = rtc_elapsed[tmr] + 1;                /* count sec */

        /* get elapsed real time */
        uint32 new_rtime = sim_os_msec ();
        uint32 delta_rtime = new_rtime - rtc_rtime[tmr];        /* elapsed wtime */
        rtc_rtime[tmr] = new_rtime;                             /* adv wall time */

        /* time goes backward or not running? */
        if (delta_rtime > 30000 || delta_rtime == 0)
        {
            /* do not apply adjustment, continue using old calibration */
            *valid = FALSE;
        }
        else
        {
            /* cycles since last second */
            uint32 dcycles = CPU_CURRENT_CYCLES - cpu_unit->cpu_last_second_tick_cycles;

            /* check if more than 5% of elapsed real time was spent doing VCPU cycles execution,
               rather than in voluntary sleep, i.e. whether voluntary sleep made less than 95% of the
               interval */
            *valid = cpu_unit->cpu_idle_sleep_us < delta_rtime * 19 * (1000 / 20);

            if (*valid)
            {
                dcycles -= cpu_unit->cpu_idle_sleep_cycles;
                delta_rtime -= cpu_unit->cpu_idle_sleep_us / 1000;
            }

            rtc_currd[tmr] = (int32) (((double) dcycles * 1000) / ((double) delta_rtime * rtc_hz[tmr]));
        }

        // cpu_unit->cpu_last_tslice_tick_cycles = CPU_CURRENT_CYCLES;
        cpu_unit->cpu_last_second_tick_cycles = CPU_CURRENT_CYCLES;
        cpu_unit->cpu_idle_sleep_us = 0;
        cpu_unit->cpu_idle_sleep_cycles = 0;
        *os_msec = new_rtime;
    }

    if (rtc_currd[tmr] <= 0)                                    /* never negative or zero! */
        rtc_currd[tmr] = 1;

    return rtc_currd[tmr];
}

static int32 sim_rtcn_calb_nosynclk (RUN_DECL, int32 ticksper, int32 tmr, t_bool* valid, uint32* os_msec)
{
    uint32 new_rtime, delta_rtime;
    int32 delta_vtime;

    if (tmr < 0 || tmr >= SIM_NTIMERS)
        return 10000;
    rtc_hz[tmr] = ticksper;
    rtc_ticks[tmr] = rtc_ticks[tmr] + 1;                    /* count ticks */
    if (rtc_ticks[tmr] < ticksper)                          /* 1 sec yet? */
        return rtc_currd[tmr];
    rtc_ticks[tmr] = 0;                                     /* reset ticks */
    rtc_elapsed[tmr] = rtc_elapsed[tmr] + 1;                /* count sec */
    *os_msec = new_rtime = sim_os_msec ();                  /* wall time */
    *valid = FALSE;                                         /* assume no valid timing data */
    if (! rtc_avail)                                        /* no timer? */
        return rtc_currd[tmr];
    if (new_rtime < rtc_rtime[tmr])                         /* time running backwards? */
    {
        rtc_rtime[tmr] = new_rtime;                         /* reset wall time */
        return rtc_currd[tmr];                              /* can't calibrate */
    }
    delta_rtime = new_rtime - rtc_rtime[tmr];               /* elapsed wtime */
    rtc_rtime[tmr] = new_rtime;                             /* adv wall time */
    rtc_vtime[tmr] = rtc_vtime[tmr] + 1000;                 /* adv sim time */
    if (delta_rtime > 30000)                                /* gap too big? */
        return rtc_initd[tmr];                              /* can't calibr */
    /*
     * ToDo: integrate the use of cpu_unit->cpu_idle_sleep_cycles and cpu_unit->cpu_idle_sleep_us
     *       similar to sim_rtcn_calb_synclk case.
     */
    *valid = TRUE;                                          /* assume valid timing data can be produced */
    if (delta_rtime == 0)                                   /* gap too small? */
    {
        rtc_based[tmr] = rtc_based[tmr] * ticksper;         /* slew wide */
        *valid = FALSE;
    }
    else
    {
        rtc_based[tmr] = (int32) (((double) rtc_based[tmr] * (double) rtc_nxintv[tmr]) /
                                 ((double) delta_rtime));   /* new base rate */
    }
    delta_vtime = rtc_vtime[tmr] - rtc_rtime[tmr];          /* gap */
    if (delta_vtime > SIM_TMAX)                             /* limit gap */
        delta_vtime = SIM_TMAX;
    else if (delta_vtime < -SIM_TMAX)
        delta_vtime = -SIM_TMAX;
    rtc_nxintv[tmr] = 1000 + delta_vtime;                   /* next wtime */
    rtc_currd[tmr] = (int32) (((double) rtc_based[tmr] * (double) rtc_nxintv[tmr]) /
                             1000.0);                       /* next delay */
    if (rtc_based[tmr] <= 0)                                /* never negative or zero! */
    {
        rtc_based[tmr] = 1;
        *valid = FALSE;
    }
    if (rtc_currd[tmr] <= 0)                                /* never negative or zero! */
    {
        rtc_currd[tmr] = 1;
        *valid = FALSE;
    }

    return rtc_currd[tmr];
}

/* Prior interfaces - default to timer 0 */

int32 sim_rtc_init (RUN_DECL, int32 time)
{
    return sim_rtcn_init (RUN_PASS, time, 0);
}

int32 sim_rtc_calb (RUN_DECL, int32 ticksper)
{
    return sim_rtcn_calb (RUN_PASS, ticksper, 0);
}

/*
 * This routine is called by each running VCPU once a virtual second to average CPU rate data
 * (instructons per second) across all the VCPUs. Bases on the result of this averaging it
 * calculates updated values for:
 *
 *     cpu_cycles_per_second
 *     tmr_poll
 *     tmxr_poll
 *
 * valid = FALSE means that this VCPU spent virtually all time during recent virtual second
 * in a voluntary idle sleep and has no meaningful rate calibration data to contribute
 */
void cpu_update_cycles_per_second(RUN_DECL, uint32 ips, t_bool valid, uint32 os_msec)
{
    uint32 cx = cpu_unit->cpu_id;
    uint32 ix;

    /* sanity check */
    if (ips < CLK_TPS)
        valid = FALSE;

    cpu_cycles_per_second_lock->lock();

    /* record this VCPU's checkin time and rate */
    uint32 prev_checkin = av_checkin[cx];
    av_checkin[cx] = os_msec;
    uint32 dt_checkin = os_msec - prev_checkin;

    /* record whether this VCPU has valid rate adjustment data to contribute */
    if (valid)
    {
        av_valid.set(cx);
        av_ips[cx] = ips;
    }
    else
    {
        av_valid.clear(cx);
    }

    /* mark CPUs that have not checked in for too long as contributing no valid data */
    if (prev_checkin && os_msec - prev_checkin <= 0x10000000)
    {
        for (ix = 0;  ix < sim_ncpus;  ix++)
        {
            if (ix != cx && av_valid.is_set(ix) && os_msec - av_checkin[ix] > dt_checkin + 1500)
                av_valid.clear(ix);
        }
    }

    /*
     * if supplied valid data, recalculate average; otherwise carry on with old average value
     */
    if (valid)
    {
        ips = 0;

        for (ix = 0;  ix < sim_ncpus;  ix++)
        {
            if (av_valid.is_set(ix))
                ips += av_ips[ix];
        }

        ips /= av_valid.count_set(0, sim_ncpus - 1);

        /* must never be made less than CLK_TPS */
        if (ips < CLK_TPS)
            ips = CLK_TPS;

        atomic_var(cpu_cycles_per_second) = ips;

        /* set tmr poll */
        atomic_var(tmr_poll) = ips / CLK_TPS;

        /* set mux poll */
        atomic_var(tmxr_poll) = atomic_var(tmr_poll) * TMXR_MULT;
    }

    /*
     * if simulator had UI, it could display "ips" as current CPU rate indicator
     * (after the unlock below, of course)
     */
    // smp_printf(" \n*** CPU%d: updated rate %d IPS\n", cx, ips);

    cpu_cycles_per_second_lock->unlock();
}

void cpu_stopping_ips_rate_update(RUN_DECL)
{
    cpu_cycles_per_second_lock->lock();
    av_valid.clear(cpu_unit->cpu_id);
    cpu_cycles_per_second_lock->unlock();
}

uint32 cpu_get_cycles_per_second(RUN_DECL)
{
    uint32 cps = atomic_var(cpu_cycles_per_second);
    if (cps == 0)
        cps = rtc_currd[TMR_CLK] * rtc_hz[TMR_CLK];
    return cps;
}

/* sim_timer_init - get minimum sleep time available on this host */

t_bool sim_timer_init (void)
{
    sim_idle_enab = FALSE;                                  /* init idle off */
    sim_idle_rate_us = sim_os_us_sleep_init ();             /* get OS timer rate */
    return sim_idle_rate_us != 0;
}

/* 
 * sim_idle - idle simulator until next event or for specified interval
 *
 * Inputs:
 *
 *     tmr       calibrated timer to use
 *     maxticks  max tick count to sleep (0 = till next clock tick, 1 = till tick after next, 
 *               2 = till second tick after next and so on)
 *
 * Must solve the linear equation
 *
 *      ms_to_wait = w * ms_per_wait
 *
 * Or
 *      w = ms_to_wait / ms_per_wait
 *
 * May return SCPE_OK or SCPE_STOP (if stop_cpus is set). When SMP is active, the latter is ignored
 * by the caller, stop_cpus will be re-checked instead during execution of next VAX instruction,
 * hence under SMP control return status is ignored.
 */

t_stat sim_idle(RUN_DECL, uint32 tmr, t_bool sin_cyc, uint32 maxticks)
{
    /*
     * ToDo: Implement support for multi-tick idle sleeping.
     *
     * Support for multi-tick idle sleeping had been implemented at VMS VSMP layer which calculates
     * and passes down the value of tick-count-to-sleep in VAXMP_API_OP_IDLE call, which is then 
     * propagated here as "maxticks" argument. However sim_idle currently ignores "maxticks" and 
     * always tries to sleep until the next tick only.
     *
     * Note that VSMP requests multi-tick sleep (with duration up to 50 ticks) only for secondary
     * VCPUs, but primary VCPU is always requested to sleeps till next click only (at least when
     * multiple CPUs are active) because the primary is responsible for providing services to other VCPUs,
     * including incrementing time-keeping system variables in the kernel on each tick, so the primary
     * cannot sleep multiple ticks and cannot skip wake up on clock ticks. Secondaries however can.
     *
     * One obvious benefit of multi-tick sleep is reduction of system overhead due to elimination
     * of frequent thread scheduling. However this overhead is already low and its further reduction
     * (let us say, from 1% of system computational resources to 0.3%) won't be a drastic improvement.
     *
     * Another benefit is that host operating system can reduce power consumption by idling cores.
     * (Whereas frequent wakeups and use of cores at 100 Hz frequency can interfere with host OS
     * capability to shift idling cores into lower power state.)
     *
     * Furthermore, with idling cores shifted into lower power state, host CPU thermal management
     * may be able to boost the frequency on active cores.
     *
     * Nevertheless it had been deemed that these benefits are marginal enough and do not
     * justify introduction of additional complexity (required to track multi-tick sleep state)
     * in the initial release of VAX MP.
     *
     * Support for multi-tick sleeping can be added in later versions of VAX MP.
     *
     * When adding such support, sleep time should be constrained by the following factors:
     *
     *     - value of "maxticks"
     *
     *     - pending clock event queue entries for devices co-scheduled with the clock
     *       (min(clk_cosched) where (clk_cosched) != 0)
     *
     *     - pending clock event queue entries for devices not co-scheduled with the clock
     *       (min(time) where clk_cosched == 0 and unit->device != & dev_clk)
     *
     *     - in all cases limited to a reasonable "safety net" value, e.g. 2 seconds
     *
     * On wakeup sim_idle should examine duration of actual sleep and decide how much
     * ticks had the VCPU actually slept (but minimize it with intended sleep time).
     *
     * It should then execute sequence of back-to-back CLK interrupts, as described in
     * "Timer control" section of "VAX MP Technical Overview". On wakeup, sim_idle
     * should advance CPU cycle counter by a value defined a minimum or actual sleep time
     * and time to next tick. It should execute all pending events than fall into this
     * region. If region reaches next clock tick, CLK interrupt should be raised.
     * When CPU subsequently executes REI instruction, cpu_active_clk_interrupt is set
     * and new IPL level is below CLK level, check for next pending sleep tick should
     * be done, and if there is one, CPU cycle counter shoold be advacned, intervening clock 
     * queue events should be processed, and CLK interrupt raised again. SYNCLK protection
     * in this case should be overriden. This sequence should be repeated until all pending
     * ticks are exhausted. After the last tick is executed and corresponding REI is issued,
     * CPU cycle counter should be advanced again by the amount corresponding to partial
     * tick, and clock queue events falling into this range should be processed.
     *
     * SYNCLK during execution of the sequence should be postponed.
     *
     * Ability to execute back-to-back sequence relies on CLK being the highest-priority
     * hardware interrupt in the system passed to VAX code. In particular, it must be
     * processed before IPINTR. Thus if idling CPU is woken up by external interrupt,
     * it will flush all pending CLK interrupts before processing other internal
     * interrupts -- as required to mainatain logical consistent state of the system.
     *
     * Implementation will be slightly different for cases when use_clock_thread is set
     * and not set.
     *
     * CPU resets (including restarts of secondaries) should be propertly handled to
     * reset the sequence.
     *
     * It might be possible for the primary CPU to also sleep more than one tick, but
     * only when no secondary CPUs are active. VSMP currently does not implement such feature
     * (since, after all, VSMP is intended for multiprocessor operations), but such support
     * may be added. If it is added, additional aspect should be handled at the simulator
     * level: when entering ROM console mode such as via Ctrl/P, execution of REI-CLK replay
     * sequence should be suspended when entering ROM console, then resumed on return from it
     * via CONTINUE command.
     */

    UINT64 w_us;
    uint32 cps1, cps2, w32_us;
    uint32 act_cyc = 0;                                                 /* initialize to suppress false GCC warning */
    UINT64 act_cyc64;
    t_bool act_cyc_isvalid = FALSE;

    /*************************************************************************************
    *  End SYNCLK protection period and check if CLK interrupt should be raised          *
    *************************************************************************************/

    if (cpu_unit->cpu_synclk_protect_dev)
        return SCPE_OK;

    cpu_unit->cpu_synclk_protect_os = 0;
    cpu_unit->cpu_synclk_protect = FALSE;

    if (check_synclk_pending(RUN_PASS))
        return SCPE_OK;

    /*************************************************************************************
    *  Remove clock queue entries that had been rescheduled to other CPUs off the head   *
    *  of clock queue, so they do not impact idle wait time calculation                  *
    *************************************************************************************/

    sim_flush_migrated_clock_queue_entries(RUN_PASS);

    /*************************************************************************************
    *  Check if may sleep                                                                *
    *************************************************************************************/

    if (cpu_unit->clock_queue == NULL)
    {
        if (use_clock_thread && cpu_unit->clk_active)
        {
            // clk_unit is active, may sleep
        }
        else
        {
            if (sin_cyc)
            {
                cpu_cycle();
            }
            return SCPE_OK;
        }
    }

    if (cpu_unit->clock_queue && (cpu_unit->clock_queue->uptr->flags & UNIT_IDLE) == 0 ||    /* event not idle-able? */
        rtc_elapsed[tmr] < sim_idle_stable)                                                  /* timer not stable? */
    {
        if (sin_cyc)
        {
            cpu_cycle();
        }
        return SCPE_OK;
    }

    /*************************************************************************************
    *  Calculate maximum microseconds to sleep                                           *
    *************************************************************************************/

    cps1 = cpu_get_cycles_per_second(RUN_PASS);

    if (cpu_unit->clock_queue)
    {
        if (cpu_unit->clock_queue->clk_cosched)
        {
            /* will be awoken by SYNCLK -- request to sleep 1 second */
            UINT64_FROM_UINT32(w_us, 1000 * 1000);
        }
        else
        {
            UINT64_FROM_UINT32(w_us, (uint32) sim_interval);
            UINT64_MUL_UINT32(w_us, 1000 * 1000);
            UINT64_DIV_UINT32(w_us, cps1);
        }
    }
    else
    {
        /* no clock queue entries -- sleep 1 second */
        UINT64_FROM_UINT32(w_us, 1000 * 1000);
    }

    /*************************************************************************************
    *  Is idle sleep time below host system's sleep timer resolution?                    *
    *************************************************************************************/

    if (sim_idle_rate_us == 0 || UINT64_LT_UINT32(w_us, sim_idle_rate_us))
    {
        /* if below, cannot wait, have to spin */
        if (sin_cyc)
        {
            cpu_cycle();
        }
        return SCPE_OK;
    }
    else
    {
        t_bool valid = FALSE;
        UINT64_TO_UINT32(w_us, w32_us, valid);
        if (! valid)  w32_us = 1000 * 1000 * 1000;
    }

    /*************************************************************************************
    *  Bump priority for sleep unless SYNCLK is used                                     *
    *************************************************************************************/

    if (! use_clock_thread)
    {
        /*
         * Since we are in the guest OS idle loop, we should be executing at SIMH_THREAD_PRIORITY_CPU_RUN.
         * Boost current VCPU thread priority so that when sleep interval expires, we get control back
         * ASAP and can process HW clock ticks promptly.
         *
         * Increment "VM-critical" reason count, so priority does not get dropped back if cpu_reevaluate_thread_priority()
         * is called either directy or indirectly between here and skip_sleep_attempt.
         *
         * Note that priority elevation will be suppressed if host is dedicated or if only primary VCPU is active
         * (see must_control_prio()). However in these cases special efforts to secure prompt wake-up are less important;
         * not to mention that all currently supported hosts systems do use clock thread (which boosts VCPU thread
         * priority when sending SYNCLK interrupt to the VCPU). If these cases are deemed important later, we can
         * modify thread priority change control scheme to make it accomodate idle sleep-time boosts.
         */
        vm_critical_lock();
    }

    /*
     * ToDo:
     *
     *   We may want to boost VCPU thread priority even if using clock thread but sleeping for a short time,
     *   less then expected time to next SYNCLK interrupt. This should help VCPU thread to wake up fast and
     *   process pending events. Otherwise if VCPU thread is left running at normal or depressed prioriy and
     *   round-robin policy is in action, VCPU thread may be be placed at the end of scheduling queue. When
     *   thread is waked up, we may want delay downgrading (reeavluating) priority until pending clock queue
     *   events are processed.
     *
     *   Thus condition for priority elevation may be
     *
     *       (!use_clock_thread) || (use_clock_thread && next_non_clk_event_interval <= max(3 * next_clock_interval_estimate, threshold))
     *
     *   where "3" allows for variability in instructions per second calibration rate. If next_clock_interval_estimate
     *   is low, use minimum threshold instead of (3 * next_clock_interval_estimate).
     *
     */

    /*************************************************************************************
    *  Prepare to enter idle sleep                                                       *
    *************************************************************************************/

    uint32 act_us = 0;
    cpu_unit->cpu_wakeup_event->clear();

    if (unlikely(! smp_interlocked_cas_done_var(& cpu_unit->cpu_sleeping, 0, 1)))
        panic("Unexpected state of cpu_sleeping (0->1)");

    /*************************************************************************************
    *  Last check if may enter sleep                                                     *
    *************************************************************************************/

    if (! cpu_may_sleep(RUN_PASS))
    {
        if (unlikely(! smp_interlocked_cas_done_var(& cpu_unit->cpu_sleeping, 1, 0)))
            panic("Unexpected state of cpu_sleeping (1->0 pre sleep)");
        goto skip_sleep_attempt;
    }

    /*************************************************************************************
    *  Do sleep                                                                          *
    *************************************************************************************/

    /* 
     * Note that due to race condition between sim_idle and wakeup_cpu,
     * spurious wakeups can sometimes (infrequently) happen.
     */
    cpu_unit->cpu_wakeup_event->timed_wait(w32_us, & act_us);

    /*************************************************************************************
    *  Leave sleep state                                                                 *
    *************************************************************************************/

    if (unlikely(! smp_interlocked_cas_done_var(& cpu_unit->cpu_sleeping, 1, 0)))
        panic("Unexpected state of cpu_sleeping (1->0 after sleep)");

    /*************************************************************************************
    *  Advance CPU cycle counters                                                        *
    *************************************************************************************/

    if (act_us != UINT32_MAX)
    {
        cps2 = cpu_get_cycles_per_second(RUN_PASS);
        UINT64_FROM_UINT32(act_cyc64, (cps1 + cps2) / 2);
        UINT64_MUL_UINT32(act_cyc64, act_us);
        UINT64_DIV_UINT32(act_cyc64, 1000 * 1000);
        UINT64_TO_UINT32(act_cyc64, act_cyc, act_cyc_isvalid);
        /*
         * if slept for too long, e.g. as the result of host hibernation,
         * ignore this sleep time altogether
         */
        if (act_cyc > 0x7FFFFFFF)  act_cyc_isvalid = FALSE;
    }

    if (act_cyc_isvalid)
    {
        /* ToDo: if overslept, reduce "time" in due clock queue entries */
        if (sim_interval > (int32) act_cyc)
            sim_interval -= (int32) act_cyc;
        else
            sim_interval = 1;

        CPU_CURRENT_CYCLES += act_cyc;
        cpu_unit->cpu_idle_sleep_cycles += act_cyc;

        /* 
         * Register usecs spent in voluntary sleep (does not currently handle maxticks).
         *
         * Estimate of maxvol_us is imperfect if clock thread is used, but fairly close to the best we can do.
         * We could of course try to estimate maxvol_us more precisely based on approximation of starting
         * position for the sleep within a current tick derived from cycles accumulated since the last tick
         * compared vs. cpu_get_cycles_per_second() / CLK_TPS, howerver current approximation seems to be
         * satisfactory for the purposes it is used for.
         */
        uint32 maxvol_us = imin(w32_us, (uint32) (1000 * 1000) / CLK_TPS);
        cpu_unit->cpu_idle_sleep_us += imin(maxvol_us, act_us);
    }

    /*************************************************************************************
    *  Drop priority to original (unless clock event is imminent)                        *
    *************************************************************************************/

skip_sleep_attempt:

    if (! use_clock_thread)
    {
        RUN_SCOPE_RSCX_ONLY;

        /* Decrement "VM-critical" count to compensate for the increment above */
        rscx->vm_critical_locks--;

        /*
         * If clock event is not imminent, drop priority back to what it used to be at the entrance 
         * to sim_idle. However if clock event is imminent, leave the thread at elevated priority.
         * Priority will be dropped then either when clock interrupt is delivered (in case clock
         * interrupts are enabled via CSR_IE) or in clk_svc if clock interrupts are disabled.
         *
         * Define threshold of "imminent" as 5% of a clock tick.
         */
        int32 clk_time = sim_is_active(& clk_unit) - 1;
        if (clk_time >= 0 && (uint32) clk_time <= cpu_get_cycles_per_second(RUN_PASS) / (CLK_TPS * 20) /*&& clk_time == sim_interval*/)
        {
            CPU_CURRENT_CYCLES += sim_interval;
            cpu_unit->cpu_idle_sleep_cycles += sim_interval;
            sim_interval = 0;
        }
        else
        {
            if (rscx->vm_critical_locks == 0)
                cpu_reevaluate_thread_priority(RUN_PASS);
        }
    }

    /*************************************************************************************
    *  Check if CPU stop request is pending                                              *
    *************************************************************************************/

    smp_rmb();
    if (weak_read(stop_cpus))
        return SCPE_STOP;

    return SCPE_OK;
}

/* Set idling - implicitly disables throttling */

t_stat sim_set_idle (UNIT *uptr, int32 val, char *cptr, void *desc)
{
    t_stat r;
    uint32 v;

    if (sim_idle_rate_us == 0)
        return SCPE_NOFNC;

    if (val != 0 && sim_idle_rate_us > (uint32) val)
        return SCPE_NOFNC;

    if (cptr)
    {
        v = (uint32) get_uint (cptr, 10, SIM_IDLE_STMAX, &r);
        if (r != SCPE_OK || v < SIM_IDLE_STMIN)
            return SCPE_ARG;
        sim_idle_stable = v;
    }

    sim_idle_enab = TRUE;

    if (sim_throt_type != SIM_THROT_NONE)
    {
        sim_set_throt (0, NULL);
        smp_printf ("Throttling disabled\n");
        if (sim_log)
            fprintf (sim_log, "Throttling disabled\n");
    }

    return SCPE_OK;
}

/* Clear idling */

t_stat sim_clr_idle (UNIT *uptr, int32 val, char *cptr, void *desc)
{
    sim_idle_enab = FALSE;
    return SCPE_OK;
}

/* Show idling */

t_stat sim_show_idle (SMP_FILE *st, UNIT *uptr, int32 val, void *desc)
{
    if (sim_idle_enab)
        fprintf (st, "idle enabled, stability wait = %ds", sim_idle_stable);
    else
        fputs ("idle disabled", st);
    return SCPE_OK;
}

/* Throttling package */

t_stat sim_set_throt (int32 arg, char *cptr)
{
    if (arg == 0)
    {
        if (cptr && *cptr)
            return SCPE_ARG;
        sim_throt_type = SIM_THROT_NONE;
        sim_throt_cancel ();
    }
#if defined(SIM_NO_THROTTLING)
    else
    {
        return SCPE_NOFNC;
    }
#else
    else if (sim_idle_rate_us == 0)
    {
        return SCPE_NOFNC;
    }
    else
    {
        char *tptr, c;
        t_value val = strtotv (cptr, &tptr, 10);
        if (cptr == tptr)
            return SCPE_ARG;
        c = toupper (*tptr++);
        if (*tptr != 0)
            return SCPE_ARG;
        if (c == 'M') 
            sim_throt_type = SIM_THROT_MCYC;
        else if (c == 'K')
            sim_throt_type = SIM_THROT_KCYC;
        else if ((c == '%') && (val > 0) && (val < 100))
            sim_throt_type = SIM_THROT_PCT;
        else return SCPE_ARG;
        if (sim_idle_enab)
        {
            smp_printf ("Idling disabled\n");
            if (sim_log)
                fprintf (sim_log, "Idling disabled\n");
            sim_clr_idle (NULL, 0, NULL, NULL);
        }
        sim_throt_val = (uint32) val;
    }
#endif

    return SCPE_OK;
}

t_stat sim_show_throt (SMP_FILE *st, DEVICE *dnotused, UNIT *unotused, int32 flag, char *cptr)
{
#if defined(SIM_NO_THROTTLING)
    fprintf (st, "Throttling not available\n");
#else
    if (sim_idle_rate_us == 0)
    {
        fprintf (st, "Throttling not available\n");
    }
    else
    {
        switch (sim_throt_type)
        {
        case SIM_THROT_MCYC:
            fprintf (st, "Throttle = %d megacycles\n", sim_throt_val);
            break;

        case SIM_THROT_KCYC:
            fprintf (st, "Throttle = %d kilocycles\n", sim_throt_val);
            break;

        case SIM_THROT_PCT:
            fprintf (st, "Throttle = %d%%\n", sim_throt_val);
            break;

        default:
            fprintf (st, "Throttling disabled\n");
            break;
        }

        if (sim_switches & SWMASK ('D'))
        {
            fprintf (st, "Wait rate = %d us\n", sim_idle_rate_us);
            if (sim_throt_type != 0)
                fprintf (st, "Throttle interval = %d cycles\n", sim_throt_wait);
        }
    }
#endif
    return SCPE_OK;
}

void sim_throt_sched (void)
{
#if !defined(SIM_NO_THROTTLING)
    sim_throt_state = 0;
    if (sim_throt_type)
        sim_activate (&sim_throt_unit, SIM_THROT_WINIT);
#endif
}

void sim_throt_cancel (void)
{
#if !defined(SIM_NO_THROTTLING)
    sim_cancel (&sim_throt_unit);
#endif
}

/* Throttle service

   Throttle service has three distinct states

   0        take initial measurement
   1        take final measurement, calculate wait values
   2        periodic waits to slow down the CPU
*/

t_stat sim_throt_svc (RUN_SVC_DECL, UNIT *uptr)
{
#if !defined(SIM_NO_THROTTLING)
    // RUN_SVC_CHECK_CANCELLED(uptr);    // not required for per-CPU devices
    uint32 delta_ms;
    double a_cps, d_cps;

    switch (sim_throt_state)
    {
    case 0:                                             /* take initial reading */
        sim_throt_ms_start = sim_os_msec ();
        sim_throt_wait = SIM_THROT_WST;
        sim_throt_state++;                              /* next state */
        break;                                          /* reschedule */

    case 1:                                             /* take final reading */
        sim_throt_ms_stop = sim_os_msec ();
        delta_ms = sim_throt_ms_stop - sim_throt_ms_start;
        if (delta_ms < SIM_THROT_MSMIN) {               /* not enough time? */
            if (sim_throt_wait >= 100000000)            /* too many inst? */
            {
                sim_throt_state = 0;                    /* fails in 32b! */
                return SCPE_OK;
            }
            sim_throt_wait = sim_throt_wait * SIM_THROT_WMUL;
            sim_throt_ms_start = sim_throt_ms_stop;
        }
        else                                            /* long enough */
        {
            a_cps = ((double) sim_throt_wait) * 1000.0 / (double) delta_ms;
            if (sim_throt_type == SIM_THROT_MCYC)       /* calc desired cps */
                d_cps = (double) sim_throt_val * 1000000.0;
            else if (sim_throt_type == SIM_THROT_KCYC)
                d_cps = (double) sim_throt_val * 1000.0;
            else d_cps = (a_cps * ((double) sim_throt_val)) / 100.0;
            if (d_cps >= a_cps) {
                sim_throt_state = 0;
                return SCPE_OK;
                }
            sim_throt_wait = (int32)                    /* time between waits */
                ((a_cps * d_cps * (double) sim_idle_rate_us) / (a_cps - d_cps));
            if (sim_throt_wait < SIM_THROT_WMIN) {      /* not long enough? */
                sim_throt_state = 0;
                return SCPE_OK;
                }
            sim_throt_state++;
//            fprintf (smp_stderr, "Throttle values a_cps = %f, d_cps = %f, wait = %d\n",
//                a_cps, d_cps, sim_throt_wait);
        }
        break;

    case 2:                                             /* throttling */
        sim_os_ms_sleep (1);
        break;
    }

    sim_activate (uptr, sim_throt_wait);                /* reschedule */
#endif

    return SCPE_OK;
}
