/*
 * Primitives used by instruction history recording.
 * Generate 64-bit stamps in multiprocessor case.
 */

#if defined (__x86_64__)
#define HST_MAKE_STAMP_ISBOOL 0
static smp_interlocked_uint64_var hst_stamp = smp_var_init(0);

SIM_INLINE static void hst_reinit_stamp()
{
    smp_var(hst_stamp) = 0;
}

SIM_INLINE static void hst_make_stamp(UINT64* pst)
{
    *pst = smp_interlocked_increment(& smp_var(hst_stamp));
}

#endif

#if defined (__x86_32__)
#define HST_MAKE_STAMP_ISBOOL 1
static smp_interlocked_uint32_var hst_stamp_counter = smp_var_init(0);
static smp_interlocked_uint32_var hst_stamp_epoch = smp_var_init(0);
extern t_bool have_cmpxchg8b;

SIM_INLINE static void hst_reinit_stamp()
{
    if (sizeof(hst_stamp_counter) < 8)
        panic("Internal error: wrong hst_stamp_counter size");
    memset((void*) & hst_stamp_counter, 0, sizeof hst_stamp_counter);
    smp_var(hst_stamp_epoch) = 0;
}

SIM_INLINE static t_bool hst_make_stamp(UINT64* pst)
{
    if (unlikely(! have_cmpxchg8b))
        goto no_cmpxchg8b;

#if defined(_WIN32)
    {
        // COMPILER_BARRIER;
        uint32* pctr = (uint32*) & smp_var(hst_stamp_counter);
        __asm
        {
            mov    edi, pctr
            mov    eax, [edi]
            mov    edx, [edi + 4]
cx_again:
            mov    ebx, eax
            mov    ecx, edx
            add    ebx, 1
            adc    ecx, 0
            lock   cmpxchg8b [edi]
            jnz    cx_again
            mov    edi, pst
            mov    [edi], ebx
            mov    [edi + 4], ecx
        }
        // COMPILER_BARRIER;
    }
    return TRUE;
#elif defined(__linux) || defined(__APPLE__)
    {
        // COMPILER_BARRIER;
        uint32* pctr = (uint32*) & smp_var(hst_stamp_counter);
        __asm__ __volatile__ ("\n\t"
            "movl  (%%edi), %%eax\n\t"
            "movl  4(%%edi), %%edx\n\t"
"1:"
            "movl  %%eax, %%ebx\n\t"
            "movl  %%edx, %%ecx\n\t"
            "addl  $1, %%ebx\n\t"
            "adcl  $0, %%ecx\n\t"
            "lock; cmpxchg8b (%%edi)\n\t"
            "jnz   1b\n\t"
            "movl  %%ebx, (%%esi)\n\t"
            "movl  %%ecx, 4(%%esi)\n\t"
               : /* no outputs */
               : "S" (pst), "D" (pctr)
               : "eax", "ebx", "ecx", "edx", /*"esi", "edi",*/ "cc"
        );
        // COMPILER_BARRIER;
    }
    return TRUE;
#else
#  error Unimplemented
#endif

no_cmpxchg8b:
    uint32 counter = smp_interlocked_increment_var(& hst_stamp_counter);
    uint32 epoch = weak_read_var(hst_stamp_epoch);

    /*
     * check safety zones
     */

    // if (counter >= 0xF0000000 || counter <= 0x10000000)
    // {
    //     if (epoch & 1)
    //     {
    //         __asm int 3
    //     }
    // }
    // else if (counter >= 0x70000000 && counter <= 0x90000000)
    // {
    //     if (! (epoch & 1))
    //     {
    //         __asm int 3
    //     }
    // }

    switch (counter >> 28)
    {
    case 0x0:
    case 0x1:
    case 0xF:
        if (epoch & 1)
            return FALSE;
        break;

    case 0x7:
    case 0x8:
    case 0x9:
        if (! (epoch & 1))
            return FALSE;
        break;
    }

    // process epoch decoding and incrementing

    if (counter < 0x80000000)
    {
        if (counter == 0x40000000)
        {
            epoch = epoch / 2;
            // (epoch, 0) -> (epoch + 1, 1)
            if (! smp_interlocked_cas_done_var(& hst_stamp_epoch,  2 * epoch, 2 * (epoch + 1) + 1))
            {
                return FALSE;
            }
        }
        else
        {
            // if (epoch & 1)
            //     epoch = epoch / 2 - 1;
            // else
            //     epoch = epoch / 2;
            epoch = (epoch >> 1) - (epoch & 1);
        }
    }
    else // if (counter >= 0x80000000)
    {
        if (counter == 0xC0000000)
        {
            epoch = epoch / 2;
            // (epoch + 1, 1) -> (epoch + 1, 0)
            if (! smp_interlocked_cas_done_var(& hst_stamp_epoch,  2 * epoch + 1, 2 * epoch))
            {
                return FALSE;
            }
            epoch--;
        }
        else
        {
            epoch = epoch / 2 - 1;
        }
    }

    // compose the stamp
    
    // t_uint64 stamp = epoch;
    // stamp = (epoch << 32) | counter;
    // *pst = stamp;

    uint32* pst32 = (uint32*) pst;
    pst[0] = counter;
    pst[1] = epoch;

    return TRUE;
}
#endif
