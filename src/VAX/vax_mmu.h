/*
 * vax_mmu.h - prototypes and inline basic functions for physical memory access
 */

#define refoff(type, base, offset)  (* (type*) (((t_byte*)(base)) + (offset)))

extern volatile uint32 *M;

int32 ReadIO (RUN_DECL, uint32 pa, int32 lnt);
void WriteIO (RUN_DECL, uint32 pa, int32 val, int32 lnt);

int32 ReadReg (RUN_DECL, uint32 pa, int32 lnt);
void WriteReg (RUN_DECL, uint32 pa, int32 val, int32 lnt);

int32 ReadB_nomem (RUN_DECL, uint32 pa);
int32 ReadW_nonmem (RUN_DECL, uint32 pa);
int32 ReadL_nonmem (RUN_DECL, uint32 pa);
int32 ReadLP_nonmem (RUN_DECL, uint32 pa);

void WriteB_nomem (RUN_DECL, uint32 pa, int32 val);
void WriteW_nonmem (RUN_DECL, uint32 pa, int32 val);
void WriteL_nonmem (RUN_DECL, uint32 pa, int32 val);
void WriteLP_nomem (RUN_DECL, uint32 pa, int32 val);

#if 0
#  define MEM_REF_ASSERT(cond)  do { if (! (cond))  sim_DebugBreak(); } while (0)
#else
#  define MEM_REF_ASSERT(cond)
#endif

/* Read aligned physical (in virtual context, unless indicated)

   Inputs:
        pa      =       physical address, naturally aligned
   Output:
        returned data, right justified in 32b longword
*/

SIM_INLINE static int32 ReadB (RUN_DECL, uint32 pa)
{
    if (ADDR_IS_MEM (pa))
    {
#if defined(__x86_32__) || defined(__x86_64__)
        return (int32) refoff(uint8, M, pa);
#else
        int32 dat = M[pa >> 2];
        return ((dat >> ((pa & 3) << 3)) & BMASK);
#endif
    }
    else
    {
        return ReadB_nomem (RUN_PASS, pa);
    }
}

/*
 * Read word.
 * Assumes aligned address.
 */
SIM_INLINE static int32 ReadW (RUN_DECL, uint32 pa)
{
    MEM_REF_ASSERT((pa & 1) == 0);

    if (ADDR_IS_MEM (pa))
    {
#if defined(__x86_32__) || defined(__x86_64__)
        return (int32) refoff(uint16, M, pa);
#else
        int32 dat = M[pa >> 2];
        return ((dat >> ((pa & 2)? 16: 0)) & WMASK);
#endif
    }
    else
    {
        return ReadW_nonmem (RUN_PASS, pa);
    }
}

/*
 * Read longword.
 * Assumes aligned address.
 */
SIM_INLINE static int32 ReadL (RUN_DECL, uint32 pa)
{
    MEM_REF_ASSERT((pa & 3) == 0);

    if (ADDR_IS_MEM (pa))
    {
#if defined(__x86_32__) || defined(__x86_64__)
        return refoff(uint32, M, pa);
#else
        return M[pa >> 2];
#endif
    }
    else
    {
        return ReadL_nonmem (RUN_PASS, pa);
    }
}

/*
 * Read longword, physical access.
 * Assumes aligned address.
 */
SIM_INLINE static int32 ReadLP (RUN_DECL, uint32 pa)
{
    MEM_REF_ASSERT((pa & 3) == 0);

    if (ADDR_IS_MEM (pa))
    {
#if defined(__x86_32__) || defined(__x86_64__)
        return refoff(uint32, M, pa);
#else
        return M[pa >> 2];
#endif
    }
    else
    {
        return ReadLP_nonmem (RUN_PASS, pa);
    }
}

/* 
 * ReadL, but memory address argument may be pointing inside the longword,
 * ignore its lowest 2 bits
 */
SIM_INLINE static int32 ReadL_NA (RUN_DECL, uint32 pa)
{
    if (ADDR_IS_MEM (pa))
    {
#if defined(__x86_32__) || defined(__x86_64__)
        return refoff(uint32, M, pa & ~0x3);
#else
        return M[pa >> 2];
#endif
    }
    else
    {
        return ReadL_nonmem(RUN_PASS, pa);
    }
}

/* Write aligned physical (in virtual context, unless indicated)

   Inputs:
        pa      =       physical address, naturally aligned
        val     =       data to be written, right justified in 32b longword
   Output:
        none
*/

SIM_INLINE static void WriteB (RUN_DECL, uint32 pa, int32 val)
{
    if (ADDR_IS_MEM (pa))
    {
#if 0
        /*
         * Original uniprocessor SIMH code that works regardless of the host endianness,
         * but only in uniprocessor (single-threaded) case.
         */
        int32 id = pa >> 2;
        int32 sc = (pa & 3) << 3;
        int32 mask = 0xFF << sc;
        M[id] = (M[id] & ~mask) | (val << sc);
#elif defined(__x86_32__) || defined(__x86_64__)
        /* SMP case: atomic access to byte without interfering with neighbor bytes */
        refoff(t_byte, M, pa) = (t_byte) val;
#else
#  error Unimplemented
#endif
    }
    else
    {
        WriteB_nomem (RUN_PASS, pa, val);
    }
}

SIM_INLINE static void WriteW (RUN_DECL, uint32 pa, int32 val)
{
    MEM_REF_ASSERT((pa & 1) == 0);

    if (ADDR_IS_MEM (pa))
    {
#if 0
        /*
         * Original uniprocessor SIMH code that works regardless of the host endianness,
         * but only in uniprocessor (single-threaded) case.
         */
        int32 id = pa >> 2;
        M[id] = (pa & 2)? (M[id] & 0xFFFF) | (val << 16):
                          (M[id] & ~0xFFFF) | val;
#elif defined(__x86_32__) || defined(__x86_64__)
        /* SMP case: atomic access to aligned word without interfering with neighbor words */
        refoff(uint16, M, pa) = (uint16) val;
#else
#  error Unimplemented
#endif
    }
    else
    {
        WriteW_nonmem (RUN_PASS, pa, val);
    }
}

SIM_INLINE static void WriteL (RUN_DECL, uint32 pa, int32 val)
{
    MEM_REF_ASSERT((pa & 3) == 0);

    if (ADDR_IS_MEM (pa))
    {
#if defined(__x86_32__) || defined(__x86_64__)
        refoff(uint32, M, pa) = (uint32) val;
#else
        M[pa >> 2] = val;
#endif
        if (unlikely(PA_MAY_BE_INSIDE_SCB(pa)))
            cpu_scb_written(pa);
    }
    else
    {
        WriteL_nonmem (RUN_PASS, pa, val);
    }
}

SIM_INLINE static void WriteLP (RUN_DECL, uint32 pa, int32 val)
{
    MEM_REF_ASSERT((pa & 3) == 0);

    if (ADDR_IS_MEM (pa))
    {
#if defined(__x86_32__) || defined(__x86_64__)
        refoff(uint32, M, pa) = (uint32) val;
#else
        M[pa >> 2] = val;
#endif
    }
    else
    {
        WriteLP_nomem (RUN_PASS, pa, val);
    }
}

/*
 * WriteL, but memory address argument may be pointing inside the longword,
 * ignore its lowest 2 bits
 */
SIM_INLINE static void WriteL_NA (RUN_DECL, uint32 pa, int32 val)
{
    if (ADDR_IS_MEM (pa))
    {
#if defined(__x86_32__) || defined(__x86_64__)
        refoff(uint32, M, pa & ~0x3) = (uint32) val;
#else
        M[pa >> 2] = val;
#endif
    }
    else
    {
        WriteL_nonmem (RUN_PASS, pa, val);
    }
}

/* 
 * ReadW, but memory address argument may be pointing inside the word,
 * ignore its lowest bit
 */
SIM_INLINE static int32 ReadW_NA (RUN_DECL, uint32 pa)
{
    if (ADDR_IS_MEM (pa))
    {
#if defined(__x86_32__) || defined(__x86_64__)
        return (int32) refoff(uint16, M, pa & ~0x1);
#else
        int32 dat = M[pa >> 2];
        return ((dat >> ((pa & 2)? 16: 0)) & WMASK);
#endif
    }
    else
    {
        return ReadW_nonmem (RUN_PASS, pa);
    }
}

/*
 * WriteW, but memory address argument may be pointing inside the word,
 * ignore its lowest bit
 */
SIM_INLINE static void WriteW_NA (RUN_DECL, uint32 pa, int32 val)
{
    if (ADDR_IS_MEM (pa))
    {
#if 0
        /*
         * Original uniprocessor SIMH code that works regardless of the host endianness,
         * but only in uniprocessor (single-threaded) case.
         */
        int32 id = pa >> 2;
        M[id] = (pa & 2)? (M[id] & 0xFFFF) | (val << 16):
                          (M[id] & ~0xFFFF) | val;
#elif defined(__x86_32__) || defined(__x86_64__)
        /* SMP case: atomic access to aligned word without interfering with neighbor words */
        refoff(uint16, M, pa & ~0x1) = (uint16) val;
#else
#  error Unimplemented
#endif
    }
    else
    {
        WriteW_nonmem (RUN_PASS, pa, val);
    }
}
