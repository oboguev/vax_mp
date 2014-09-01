/*
 * Memory barrier functions. Placed into a separate file so compiler cannot know their content
 * when compilng other modules and will treat them as a compiler barrier too.
 */

#if defined(_WIN32)
#  define _WIN32_WINNT 0x0403
#  include <windows.h>
#  include <intrin.h>
#endif

#if defined(__linux) || defined(__APPLE__)
#  include <signal.h>
#  include <setjmp.h>
#endif

#include "sim_defs.h"

/*
 * For information on x86 and x64 shared memory synchronization see
 *
 *     "Intel 64 and IA-32 Architectures Software Developer's Manual, Volume 3 (3A & 3B): System Programming Guide"
 *     (order number: 325384-039US), sections 8.* and 19.34,
 *     at http://www.intel.com/Assets/PDF/manual/325384.pdf (hereafter P686)
 *
 *     "Intel Architecture Software Developer's Manual, Volume 3: System Programming"
 *     (order number 243192), sections 7.1.2 and 7.2.2,
 *     at http://communities.intel.com/servlet/JiveServlet/downloadBody/5061-102-1-8118/Pentium_SW_Developers_Manual_Vol3_SystemProgramming.pdf
 *     (hereafter P686)
 *
 * P686, section 8.1.2:
 *
 *     "For the P6 family processors, locked operations serialize all outstanding load and store operations (that is,
 *     wait for them to complete). This rule is also true for the Pentium 4 and Intel Xeon processors, with one exception.
 *     Load operations that reference weakly ordered memory types (such as the WC memory type) may not be serialized."
 *
 * P686, section 8.2.2:
 *
 *     The Pentium and Intel486 processors follow the processor-ordered memory model;
 *     however, they operate as strongly-ordered processors under most circumstances.
 *     Reads and writes always appear in programmed order at the system bus—except for
 *     the following situation where processor ordering is exhibited. Read misses are
 *     permitted to go ahead of buffered writes on the system bus when all the buffered
 *     writes are cache hits and, therefore, are not directed to the same address being
 *     accessed by the read miss.
 *
 *     The Intel Core 2 Duo, Intel Atom, Intel Core Duo, Pentium 4, and P6 family processors
 *     also use a processor-ordered memory-ordering model that can be further
 *     defined as “write ordered with store-buffer forwarding.” This model can be characterized
 *     as follows.
 *     In a single-processor system for memory regions defined as write-back cacheable,
 *     the memory-ordering model respects the following principles
 *
 *         - Reads are not reordered with other reads.
 *         - Writes are not reordered with older reads.
 *         - Writes to memory are not reordered with other writes, with the following exceptions [...]
 *
 *     Reads may be reordered with older writes to different locations but not with older
 *     writes to the same location.
 *
 *         - Reads or writes cannot be reordered with I/O instructions, locked instructions, or
 *         serializing instructions.
 *         - Reads cannot pass earlier LFENCE and MFENCE instructions.
 *         - Writes cannot pass earlier LFENCE, SFENCE, and MFENCE instructions.
 *         - LFENCE instructions cannot pass earlier reads.
 *         - SFENCE instructions cannot pass earlier writes.
 *         - MFENCE instructions cannot pass earlier reads or writes.
 *
 *     In a multiple-processor system, the following ordering principles apply:
 *
 *         - Individual processors use the same ordering principles as in a single-processor
 *         system.
 *         - Writes by a single processor are observed in the same order by all processors.
 *         - Writes from an individual processor are NOT ordered with respect to the writes
 *         from other processors.
 *         - Memory ordering obeys causality (memory ordering respects transitive
 *         visibility).
 *         - Any two stores are seen in a consistent order by processors other than those
 *         performing the stores
 *         - Locked instructions have a total order.
 *
 *     The processor-ordering model described in this section is virtually identical to that
 *     used by the Pentium and Intel486 processors. The only enhancements in the Pentium
 *     4, Intel Xeon, and P6 family processors are:
 *
 *         - Added support for speculative reads, while still adhering to the ordering
 *         principles above.
 *         - Store-buffer forwarding, when a read passes a write to the same memory
 *         location.
 *         - Out of order store from long string store and string move operations
 *
 * P686, section 8.2.5:
 *
 *     Locking operations typically operate like I/O operations in that they wait
 *     for all previous instructions to complete and for all buffered writes to drain to memory.
 *
 * On older processors that do not have LFENCE, LOCK prefix stalls reads.
 *
 * P686, section 19.34:
 *
 *     During a locked bus cycle, the Intel486 processor will always access external
 *     memory, it will never look for the location in the on-chip cache. All data pending in
 *     the Intel486 processor's store buffers will be written to memory before a locked cycle
 *     is allowed to proceed to the external bus. Thus, the locked bus cycle can be used for
 *     eliminating the possibility of reordering read cycles on the Intel486 processor. The
 *     Pentium processor does check its cache on a read-modify-write access and, if the
 *     cache line has been modified, writes the contents back to memory before locking the
 *     bus.
 *
 *     The P6 family processors write to their cache on a read-modify-write operation
 *     (if the access does not split across a cache line) and does not write back to system
 *     memory. If the access does split across a cache line, it locks the bus and accesses
 *     system memory.
 *
 * P586, section 7.1.2:
 *
 *     For the P6 family processors, locked operations serialize all outstanding load and store operations
 *     (that is, wait for them to complete).
 *
 * In a nutshell, most x86 writes are strongly ordered and WMB can be no-op except for writes by 
 * non-temporal SSE instructions, and except for some clones that provide out-of-order stroring and that 
 * should use "LOCK; some-instruction" for WMB.
 *
 * However reads can be reorderd on x86, so RMB and MB should expand to "LOCK; some-instuction".
 * CPUs that have SSE and applications that use SSE non-temporal instructions should use lfence, sfence, mfence.
 *
 */

#if defined (__x86_32__) || defined (__x86_64__)
static t_bool have_lfence = FALSE;
static t_bool have_sfence = FALSE;
static t_bool have_mfence = FALSE;
static t_bool have_PentiumPro = FALSE;
static t_bool have_oostore = FALSE;
t_bool have_x86_xaddl = FALSE;
t_bool have_x86_cmpxchgl = FALSE;
t_bool have_pentium = FALSE;
t_bool have_cmpxchg8b = FALSE;
#endif

/*
 * SMP_X86_USING_NONTEMPORAL indicates whether compiler-generated code or runtime libraries might
 * utilize non-temporal SSE/3DNow instructions without proper LFENCE/SFENCE/MFENCE termination,
 * and hence if such termination should be supplied by VAX MP as a part of memory barriers.
 *
 * If application (including runtime libraries) uses SSE/3DNow non-temporal instructions without
 * proper termination, memory barriers should be defined as follows:
 *
 *     rmb = barrier() + lfence
 *     wmb = barrier() + sfence
 *     mb = barrier() + mfence
 *
 * If application does not use SSE/3DNow non-temporal instructions (and assuming processor does not have
 * OOStore mode or it is not enabled and processor is not PentiumPro), memory barriers can be defined
 * in a more lightweight fashion, as follows:
 *
 *     rmb = barrier()
 *     wmb = barrier()
 *     mb = barrier() + lock-addl
 *
 * No additional hardware primitive is required for ordering in wmb since x86/x64 orders regular writes
 * (except for non-temporal instructions and for OOStore case).
 *
 * No additional hardware primitive is required for ordering in rmb since x86/x64 orders regular reads
 * (except for non-temporal instructions and for PentiumPro case).
 *
 * Some x86 clone processors from 3rd party manufacturers (but not Intel nor AMD) used to have OOStore mode
 * enable-able via a setting. For these processors:
 *
 *     wmb = barrier() + lock-addl
 *     mb = barrier() + lock-addl
 *
 * PentiumPro has a bug in cache coherency that can cause it effectively reorder loads in a multiprocessing
 * system. As a workaround for this bug, rmb should be defined on PentiumPro as:
 *
 *     rmb = barrier() + lock-addl
 *
 * Note that MB is _not_ equivalent to the sum of RMB and WMB: the latter order loads-to-loads
 * and stores-to-stores, but not load-to-stores and stores-to-loads. Intel x86/x64 is an example
 * of processor where the distinction matters. OOStore mode and PentiumPro bug aside, x86/x64
 * processors do not locally reorder loads with other loads, and do not locally reorder stores 
 * with other stores, but loads can be reordered with older stores to a different (not same!) location
 * unless memory barrier is issued in between.
 *
 * For a summary see Paul McKenney, "Is Parallel Programming Hard, And, If So, What Can You Do About It?",
 * chapter "Memory-Barrier Instructions for Specific CPUs", subsection on x86.
 *
 * Also see a list of documents on x86-TSO model in "VAX MP Techincal Overview", Appendix A.
 *
 ******************************************************************************************************
 *
 * Linux behavior corresponds to SMP_X86_USING_NONTEMPORAL = TRUE since Linux has to support
 * context switching between user-level applications that may utilize non-temporal instructions.
 * Whether this indicates any possible use of non-temporal instructions in the kernel is unclear.
 *
 * See Linux kernel sources:
 *
 *   arch/x86/include/asm/system.h
 *   arch/um/sys-i386/shared/sysdep/system.h
 *   arch/um/sys-x86_64/shared/sysdep/system.h
 *
 * In Windows DDK, KeMemoryBarrier (full MB) is defined as:
 *
 *    on x86:   [lock] xchg barrier, eax
 *
 *    on x64:   lock or dword ptr [rsp], 0  ; __faststorefence
 *              lfence
 *
 * Microsoft documentation for __faststorefence says:
 *
 *     "On the x64 platform, this routine generates an instruction that is a faster store fence
 *      than the sfence instruction. Use this intrinsic instead of _mm_sfence on the x64 platform."
 *
 */

#if defined (__x86_32__)
   static void smp_mb_init_x86_32();
#endif

#if defined(_WIN32) && defined(__x86_32__)
#  define ASM_MFENCE __asm mfence
#  define ASM_LFENCE __asm lfence
#  define ASM_SFENCE __asm sfence
#elif defined(_WIN32) && defined(__x86_64__)
   void __faststorefence(void);
   void _mm_lfence(void);
   void _mm_mfence(void);
   void _mm_sfence(void);
#  pragma intrinsic(__faststorefence)
#  pragma intrinsic(_mm_lfence)
#  pragma intrinsic(_mm_mfence)
#  pragma intrinsic(_mm_sfence)
#  define ASM_MFENCE _mm_mfence();
#  define ASM_LFENCE _mm_lfence();
#  define ASM_SFENCE _mm_sfence();
#elif defined(__GNUC__)
// http://gcc.gnu.org/onlinedocs/gcc-4.6.1/gcc.pdf, section 6.41 and following
// http://www.ibiblio.org/gferg/ldp/GCC-Inline-Assembly-HOWTO.html
// http://www.delorie.com/djgpp/doc/brennan/brennan_att_inline_djgpp.html
// http://www.osdever.net/tutorials/view/a-brief-tutorial-on-gcc-inline-asm
// http://asm.sourceforge.net//articles/linasm.html
// http://www.cims.nyu.edu/cgi-systems/info2html?%28gcc%29Extended%2520Asm
#  define ASM_MFENCE __asm__ __volatile__ ("mfence":::"memory", "cc")
#  define ASM_LFENCE __asm__ __volatile__ ("lfence":::"memory", "cc")
#  define ASM_SFENCE __asm__ __volatile__ ("sfence":::"memory", "cc")
#endif

#if defined(_WIN32) && defined(__x86_32__)
#  define ASM_LOCKED_BARRIER  __asm lock or dword ptr [esp], 0
#endif

#if defined(_WIN64) && defined(__x86_64__)
#  define ASM_LOCKED_BARRIER  __faststorefence();
#endif

#if defined(__GNUC__) && defined(__x86_32__)
#  define ASM_LOCKED_BARRIER  __asm__ __volatile__ ("lock; orl $0,0(%%esp)":::"memory", "cc")
#endif

#if defined(__GNUC__) && defined(__x86_64__)
#  define ASM_LOCKED_BARRIER  __asm__ __volatile__ ("lock; orq $0,0(%%rsp)":::"memory", "cc")
#endif

#if defined (__x86_32__) || defined (__x86_64__)
static void smp_xmb_noop()
{
    COMPILER_BARRIER;
}
static void smp_xmb_lfence()
{ 
    COMPILER_BARRIER;
    ASM_LFENCE;
    COMPILER_BARRIER;
}
static void smp_xmb_sfence()
{
    COMPILER_BARRIER;
    ASM_SFENCE;
    COMPILER_BARRIER;
}
static void smp_xmb_mfence()
{
    COMPILER_BARRIER;
    ASM_MFENCE;
    COMPILER_BARRIER;
}
static void smp_xmb_locked()
{
    COMPILER_BARRIER;
    ASM_LOCKED_BARRIER;
    COMPILER_BARRIER;
}

void (*smp_rmb_p)() = NULL;
void (*smp_wmb_p)() = NULL;
void (*smp_mb_p)() = NULL;

static void smp_xmb_init_wmb()
{
    if (SMP_X86_USING_NONTEMPORAL && have_sfence)
        smp_wmb_p = smp_xmb_sfence;
    else if (have_oostore)
        smp_wmb_p = smp_xmb_locked;
    else
        smp_wmb_p = smp_xmb_noop;
}

static void smp_xmb_init_rmb()
{
    if (SMP_X86_USING_NONTEMPORAL && have_lfence)
        smp_rmb_p = smp_xmb_lfence;
    else if (have_PentiumPro)
        smp_rmb_p = smp_xmb_locked;
    else
        smp_rmb_p = smp_xmb_noop;
}

static void smp_xmb_init_mb()
{
    if (SMP_X86_USING_NONTEMPORAL && have_mfence)
        smp_mb_p = smp_xmb_mfence;
    else
        smp_mb_p = smp_xmb_locked;
}

void smp_mb_init()
{
#if defined (__x86_64__)
    have_lfence = have_sfence = have_mfence = TRUE;
    have_x86_xaddl = have_x86_cmpxchgl = TRUE;
    have_pentium = TRUE;
#elif defined (__x86_32__)
    smp_mb_init_x86_32();
#else
#  error Unimplemented
#endif

    /* initialize barrier indirection vectors */
    smp_xmb_init_rmb();
    smp_xmb_init_wmb();
    smp_xmb_init_mb();

    /* for single-processor case, reset all to no-op's */
    if (smp_ncpus == 1)
    {
        smp_wmb_p = smp_xmb_noop;
        smp_rmb_p = smp_xmb_noop;
        smp_mb_p = smp_xmb_noop;
    }

    /* if barriers somehow crash, try to catch it early */
    smp_rmb();
    smp_wmb();
    smp_mb();
}
#endif

#if defined (__x86_32__)
/* see for details Intel Architecture Manual (CPUID instruction) as well as:
 *     http://www.amd.com/us-en/Processors/DevelopWithAMD/0,,30_2252_2272_2274,00.html
 *     http://www.codeproject.com/KB/system/camel.aspx
 *     http://msdn.microsoft.com/library/default.asp?url=/library/en-us/vcsample/html/vcsamCPUIDDetermineCPUCapabilities.asp
 */

typedef struct
{
    SIM_ALIGN_32  uint32  eax;
    SIM_ALIGN_32  uint32  ebx;
    SIM_ALIGN_32  uint32  ecx;
    SIM_ALIGN_32  uint32  edx;
}
x86_regs;

#define streq(s1, s2)  (0 == strcmp((s1), (s2)))

#if defined(_WIN32)
// NB: should switch from custom code to MSVC intrinsics _cpuid(), _cpuidex()
static t_bool xcpuid(x86_regs* regs)
{
    // use SEH here so we can disable C++ monitoring for processor exceptions other than explicit throw
    // and use /EHsc for more optimized compilation instead of /EHsca
    BOOL res = FALSE;
    __try
    {
        __asm
        {
            mov  esi, regs
            mov  eax, [esi]x86_regs.eax
            mov  ebx, [esi]x86_regs.ebx
            mov  ecx, [esi]x86_regs.ecx
            mov  edx, [esi]x86_regs.edx
            cpuid
            mov  [esi]x86_regs.eax, eax
            mov  [esi]x86_regs.ebx, ebx
            mov  [esi]x86_regs.ecx, ecx
            mov  [esi]x86_regs.edx, edx
        }
        res = TRUE;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
    }
    return res;
}
#elif defined(__linux) || defined(__APPLE__)
static jmp_buf xcpuid_jmpbuf;
static void xcpuid_sig_handler(int sig)
{
    longjmp(xcpuid_jmpbuf, 1);
}

static void xcpuid_signal_all(sighandler_t handler)
{
    signal(SIGILL, handler);
    // signal(SIGSEGV, handler);
}

static t_bool xcpuid(x86_regs* regs)
{
    volatile t_bool res = FALSE;

    if (setjmp(xcpuid_jmpbuf) == 0)
    {
        xcpuid_signal_all(xcpuid_sig_handler);
        __asm__ __volatile__ ("\n\t"
            "movl  0(%%esi), %%eax\n\t"
            "movl  4(%%esi), %%ebx\n\t"
            "movl  8(%%esi), %%ecx\n\t"
            "movl  12(%%esi), %%edx\n\t"
            "cpuid\n\t"
            "movl  %%eax, 0(%%esi)\n\t"
            "movl  %%ebx, 4(%%esi)\n\t"
            "movl  %%ecx, 8(%%esi)\n\t"
            "movl  %%edx, 12(%%esi)\n\t"
               : /* no outputs */
               : "S" (regs)
               : "eax", "ebx", "ecx", "edx", /*"esi",*/ "cc"
        );
        res = TRUE;
    }
    else
    {
        res = FALSE;
    }
    xcpuid_signal_all(SIG_DFL);
    return res;
}
#endif

static void smp_mb_init_x86_32()
{
    x86_regs regs;
    t_bool have_mmx = FALSE;
    t_bool have_sse = FALSE;
    t_bool have_sse2 = FALSE;
    t_bool have_amd_sfence = FALSE;
    uint32 chipfamily = 0;
    uint32 chipmodel = 0;
    int maxcode;
    char cpuname[13];
    t_bool extsupport = FALSE;
    uint32 maxextlevel = 0;
    t_bool isIntel = FALSE;
    t_bool isAMD = FALSE;

    regs.eax = 0;
    if (xcpuid(& regs))
    {
        maxcode = regs.eax;
        memcpy(cpuname, & regs.ebx, 4);
        memcpy(cpuname + 4, & regs.edx, 4);
        memcpy(cpuname + 8, & regs.ecx, 4);
        cpuname[12] = 0;
    }
    else
    {
        maxcode = 0;
        cpuname[0] = 0;
    }

    if (maxcode == 0)
    {
        // CPUID is unavailable: i386, early versions of i486, Cyrix/IBM M1 and below, NexGen 586.
        // Could try to differentiate between them as in FreeBSD: http://people.freebsd.org/~kato/cpuident.html
        // but probably is not worth the effort. Just assume as safe fallback it can be one of the clones with out-of-order store.
        have_oostore = TRUE;
        return;
    }

    have_x86_xaddl = have_x86_cmpxchgl = TRUE;

    if (maxcode >= 1)
    {
        regs.eax = 1;
        if (! xcpuid(& regs))
            panic("Unexpected failure in execution of CPUID instruction");
        if (regs.edx & (1 << 8))  have_cmpxchg8b = TRUE;
        // if (regs.edx & (1 << 15))  have_cmov = TRUE;
        // if (regs.edx & (1 << 19))  have_clflush = TRUE;
        if (regs.edx & (1 << 23))  have_mmx = TRUE;
        if (regs.edx & (1 << 25))  have_sse = TRUE;
        if (regs.edx & (1 << 26))  have_sse2 = TRUE;
        // if (regs.ecx & (1 << 0))   have_sse3 = TRUE;
        // if (regs.edx & (1 << 28))  have_hyperthreading = TRUE;

        chipfamily = (regs.eax >> 8) & 0xF;
        if (chipfamily == 0xF)  chipfamily |= ((regs.eax >> 20) & 0xFF) << 4;

        chipmodel = (regs.eax >> 4) & 0xF;
        if (chipmodel == 0xF)  chipmodel |= ((regs.eax >> 16) & 0xF) << 4;
    }

    // see "Intel Processor Identification and the CPUID Instruction, Application Note 485"
    // at http://web.archive.org/web/20090205235617/http://intel.com/Assets/PDF/appnote/241618.pdf
    have_pentium = (chipfamily >= 5);

    extsupport = TRUE;

    if (streq(cpuname, "GenuineIntel"))
    {
        isIntel = TRUE;
        extsupport = (chipfamily >= 0xF);
    }
    else if (streq(cpuname, "AuthenticAMD") || streq(cpuname, "AMD ISBETTER"))
    {
        isAMD = TRUE;
        extsupport = (chipfamily > 5) || (chipfamily == 5 && chipmodel >= 6);
    }
    else if (streq(cpuname, "CyrixInstead"))
    {
        extsupport = (chipfamily > 6) || (chipfamily == 5 && chipmodel >= 4) || (chipfamily == 6 && chipmodel >= 5);
    }
    else if (streq(cpuname, "CentaurHauls"))
    {
        // IDT
        extsupport = (chipfamily > 5) || (chipfamily == 5 && chipmodel >= 8);
    }
    else if (streq(cpuname, "TransmetaCPU") || streq(cpuname, "GenuineTMx86"))
    {
        extsupport = (chipfamily >= 5);
    }

    /* Some other known maker codes:
     *
     *    "AMDisbetter!" - early engineering samples of AMD K5 processor
     *    "Geode by NSC" - National Semiconductor
     *    "NexGenDriven" - NexGen
     *    "RiseRiseRise" - Rise
     *    "SiS SiS SiS " - SiS
     *    "UMC UMC UMC " - UMC
     *    "VIA VIA VIA " - VIA
     */

    if (extsupport)
    {
        regs.eax = 0x80000000;
        if (xcpuid(& regs))
        {
            maxextlevel = regs.eax;
        }
        else
        {
            maxextlevel = 0x80000000;
        }
    }

    if (isAMD && have_mmx && maxextlevel >= 0x80000001)
    {
        // "AMD Extensions to the 3DNow! and MMX Instruction Sets Manual"
        // http://support.amd.com/us/Processor_TechDocs/22466.pdf
        regs.eax = 0x80000001;
        if (xcpuid(& regs))
            have_amd_sfence = (0 != (regs.edx & (1 << 22)));
        else
            have_amd_sfence = FALSE;
    }

    if (have_sse || have_amd_sfence)  have_sfence = TRUE;
    if (have_sse2) have_lfence = have_mfence = TRUE;
    have_PentiumPro = isIntel && chipfamily == 6 && chipmodel == 1;

    // some x86 clones, such as Cyrix and Centaur, can do out-of-order stores
    have_oostore = !isIntel && !isAMD;
}
#endif
