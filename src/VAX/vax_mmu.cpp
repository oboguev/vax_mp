/* vax_mmu.c - VAX memory management

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

   21-Jul-08    RMS     Removed inlining support
   28-May-08    RMS     Inlined physical memory routines
   29-Apr-07    RMS     Added address masking for system page table reads
   22-Sep-05    RMS     Fixed declarations (from Sterling Garwood)
   30-Sep-04    RMS     Comment and formating changes
   19-Sep-03    RMS     Fixed upper/lower case linkage problems on VMS
   01-Jun-03    RMS     Fixed compilation problem with USE_ADDR64

   This module contains the instruction simulators for

        Read            -       read virtual
        Write           -       write virtual
        ReadL(P)        -       read aligned physical longword (physical context)
        WriteL(P)       -       write aligned physical longword (physical context)
        ReadB(W)        -       read aligned physical byte (word)
        WriteB(W)       -       write aligned physical byte (word)
        Test            -       test acccess

        zap_tb          -       clear TB
        zap_tb_ent      -       clear TB entry
        chk_tb_ent      -       check TB entry
        set_map_reg     -       set up working map registers
*/

#include "sim_defs.h"
#include "vax_defs.h"

extern const uint32 align[4];

static const int32 insert[4] = {
    0x00000000, 0x000000FF, 0x0000FFFF, 0x00FFFFFF
};
static const int32 cvtacc[16] = { 0, 0,
    TLB_ACCW (KERN)+TLB_ACCR (KERN),
    TLB_ACCR (KERN),
    TLB_ACCW (KERN)+TLB_ACCW (EXEC)+TLB_ACCW (SUPV)+TLB_ACCW (USER)+
            TLB_ACCR (KERN)+TLB_ACCR (EXEC)+TLB_ACCR (SUPV)+TLB_ACCR (USER),
    TLB_ACCW (KERN)+TLB_ACCW (EXEC)+TLB_ACCR (KERN)+TLB_ACCR (EXEC),
    TLB_ACCW (KERN)+TLB_ACCR (KERN)+TLB_ACCR (EXEC),
    TLB_ACCR (KERN)+TLB_ACCR (EXEC),
    TLB_ACCW (KERN)+TLB_ACCW (EXEC)+TLB_ACCW (SUPV)+
            TLB_ACCR (KERN)+TLB_ACCR (EXEC)+TLB_ACCR (SUPV),
    TLB_ACCW (KERN)+TLB_ACCW (EXEC)+
            TLB_ACCR (KERN)+TLB_ACCR (EXEC)+TLB_ACCR (SUPV),
    TLB_ACCW (KERN)+TLB_ACCR (KERN)+TLB_ACCR (EXEC)+TLB_ACCR (SUPV),
    TLB_ACCR (KERN)+TLB_ACCR (EXEC)+TLB_ACCR (SUPV),
    TLB_ACCW (KERN)+TLB_ACCW (EXEC)+TLB_ACCW (SUPV)+
            TLB_ACCR (KERN)+TLB_ACCR (EXEC)+TLB_ACCR (SUPV)+TLB_ACCR (USER),
    TLB_ACCW (KERN)+TLB_ACCW (EXEC)+
            TLB_ACCR (KERN)+TLB_ACCR (EXEC)+TLB_ACCR (SUPV)+TLB_ACCR (USER),
    TLB_ACCW (KERN)+
            TLB_ACCR (KERN)+TLB_ACCR (EXEC)+TLB_ACCR (SUPV)+TLB_ACCR (USER),
    TLB_ACCR (KERN)+TLB_ACCR (EXEC)+TLB_ACCR (SUPV)+TLB_ACCR (USER)
};

t_stat tlb_ex (t_value *vptr, t_addr addr, UNIT *uptr, int32 sw);
t_stat tlb_dep (t_value val, t_addr addr, UNIT *uptr, int32 sw);
t_stat tlb_reset (DEVICE *dptr);

static TLBENT fill (RUN_DECL, uint32 va, int32 acc, int32 *stat);
static void Write_Uncommon (RUN_DECL, uint32 va, int32 pa, int32 pa1_pagebase, int32 val, int32 lnt, int32 acc);

/* TLB data structures

   tlb_dev      pager device descriptor
   tlb_unit     pager units
   pager_reg    pager register list
*/

UNIT* tlb_unit[] = {
    UDATA (NULL, UNIT_FIX, VA_TBSIZE * 2),
    UDATA (NULL, UNIT_FIX, VA_TBSIZE * 2)
};

REG tlb_reg[] = {
    { NULL }
};

DEVICE tlb_dev = {
    "TLB", tlb_unit, tlb_reg, NULL,
    2, 16, VA_N_TBI * 2, 1, 16, 32,
    &tlb_ex, &tlb_dep, &tlb_reset,
    NULL, NULL, NULL,
    NULL, DEV_PERCPU
};


/* Read and write virtual

   These routines logically fall into three phases:

   1.   Look up the virtual address in the translation buffer, calling
        the fill routine on a tag mismatch or access mismatch (invalid
        tlb entries have access = 0 and thus always mismatch).  The
        fill routine handles all errors.  If the resulting physical
        address is aligned, do an aligned physical read or write.
   2.   Test for unaligned across page boundaries.  If cross page, look
        up the physical address of the second page.  If not cross page,
        the second physical address is the same as the first.
   3.   Using the two physical addresses, do an unaligned read or
        write, with three cases: unaligned long, unaligned word within
        a longword, unaligned word crossing a longword boundary.

   Note that these routines do not handle quad or octa references.
*/

/* Read virtual

   Inputs:
        va      =       virtual address
        lnt     =       length code (BWL)
        acc     =       access code (KESU)
   Output:
        returned data, right justified in 32b longword
*/

int32 Read (RUN_DECL, uint32 va, int32 lnt, int32 acc)
{
    int32 vpn, off, tbi, pa;
    int32 pa1, bo, sc, wl, wh;
    TLBENT xpte;

    mchk_va = va;

    if (mapen)                                              /* mapping on? */
    {
        vpn = VA_GETVPN (va);                               /* get vpn, offset */
        off = VA_GETOFF (va);
        tbi = VA_GETTBI (vpn);
        xpte = (va & VA_S0)? stlb[tbi]: ptlb[tbi];          /* access tlb */
        if (((xpte.pte & acc) == 0) || (xpte.tag != vpn) ||
            ((acc & TLB_WACC) && ((xpte.pte & TLB_M) == 0)))
            xpte = fill (RUN_PASS, va, acc, NULL);          /* fill if needed */
        pa = (xpte.pte & TLB_PFN) | off;                    /* get phys addr */
    }
    else
    {
        pa = va & PAMASK;
        off = VA_GETOFF (va);                               /* unused, only to suppress false GCC warning */
    }

    if ((pa & (lnt - 1)) == 0)                              /* aligned? */
    {
        if (lnt >= L_LONG)                                  /* long, quad? */
            return ReadL (RUN_PASS, pa);
        else if (lnt == L_WORD)                             /* word? */
            return ReadW (RUN_PASS, pa);
        else
            return ReadB (RUN_PASS, pa);                    /* byte */
    }

    if (mapen && (uint32) (off + lnt) > VA_PAGSIZE)         /* cross page? */
    {              
        vpn = VA_GETVPN (va + lnt);                         /* vpn 2nd page */
        tbi = VA_GETTBI (vpn);
        xpte = (va & VA_S0)? stlb[tbi]: ptlb[tbi];          /* access tlb */
        if (((xpte.pte & acc) == 0) || (xpte.tag != vpn) ||
            ((acc & TLB_WACC) && ((xpte.pte & TLB_M) == 0)))
            xpte = fill (RUN_PASS, va + lnt, acc, NULL);         /* fill if needed */
        pa1 = (xpte.pte & TLB_PFN) | VA_GETOFF (va + 4);
    }
    else
    { 
        pa1 = (pa + 4) & PAMASK;                           /* not cross page */
    }

    bo = pa & 3;

    if (lnt >= L_LONG)                                      /* lw unaligned? */
    {
        sc = bo << 3;
        wl = ReadL_NA (RUN_PASS, pa);                                    /* read both lw */
        wh = ReadL_NA (RUN_PASS, pa1);                                   /* extract */
        return ((((wl >> sc) & align[bo]) | (wh << (32 - sc))) & LMASK);
    }
    else if (bo == 1)
    {
        return ((ReadL_NA (RUN_PASS, pa) >> 8) & WMASK);
    }
    else
    {
        wl = ReadL_NA (RUN_PASS, pa);                                    /* word cross lw */
        wh = ReadL_NA (RUN_PASS, pa1);                                   /* read, extract */
        return (((wl >> 24) & 0xFF) | ((wh & 0xFF) << 8));
    }
}

/* Write virtual

   Inputs:
        va      =       virtual address
        val     =       data to be written, right justified in 32b lw
        lnt     =       length code (BWL)
        acc     =       access code (KESU)
   Output:
        none
*/

void Write (RUN_DECL, uint32 va, int32 val, int32 lnt, int32 acc)
{
    int32 vpn, off, tbi, pa, pa1;
    TLBENT xpte;

    mchk_va = va;
    if (mapen)
    {
        vpn = VA_GETVPN (va);
        off = VA_GETOFF (va);
        tbi = VA_GETTBI (vpn);
        xpte = (va & VA_S0)? stlb[tbi]: ptlb[tbi];          /* access tlb */
        if ((xpte.pte & acc) == 0 || xpte.tag != vpn || (xpte.pte & TLB_M) == 0)
        {
            xpte = fill (RUN_PASS, va, acc, NULL);
        }
        pa = (xpte.pte & TLB_PFN) | off;
    }
    else 
    {
        pa = va & PAMASK;
        off = VA_GETOFF (va);                               /* unused, only to suppress false GCC warning */
    }

    /*
     * lnt can be L_WORD or L_LONG; or perhaps L_QUAD/L_OCTA but meaining here L_LONG
     */
    if (lnt > L_LONG)
        lnt = L_LONG;

    if ((pa & (lnt - 1)) == 0)                              /* aligned? */
    {
        if (lnt >= L_LONG)                                  /* long, quad? */
            WriteL (RUN_PASS, pa, val);
        else if (lnt == L_WORD)                             /* word? */
            WriteW (RUN_PASS, pa, val);
        else
            WriteB (RUN_PASS, pa, val);                     /* byte */
        return;
    }

    /*
     * Non-aligned write: lnt can be L_WORD or L_LONG only
     */

#if 0
    /*
     * Original uniprocessor SIMH code that works regardless of the host endianness,
     * but only in uniprocessor (single-threaded) case.
     */

    int32 bo, sc, wl, wh;

    if (mapen && (uint32) (off + lnt) > VA_PAGSIZE)
    {
        vpn = VA_GETVPN (va + 4);
        tbi = VA_GETTBI (vpn);
        xpte = (va & VA_S0)? stlb[tbi]: ptlb[tbi];          /* access tlb */
        if ((xpte.pte & acc) == 0 || xpte.tag != vpn || (xpte.pte & TLB_M) == 0)
        {
            xpte = fill (RUN_PASS, va + lnt, acc, NULL);
        }
        pa1 = (xpte.pte & TLB_PFN) | VA_GETOFF (va + 4);
    }
    else
    {
        pa1 = (pa + 4) & PAMASK;
    }

    bo = pa & 3;
    wl = ReadL_NA (RUN_PASS, pa);

    if (lnt >= L_LONG)
    {
        sc = bo << 3;
        wh = ReadL_NA (RUN_PASS, pa1);
        wl = (wl & insert[bo]) | ((val << sc) & LMASK);
        wh = (wh & ~insert[bo]) | ((val >> (32 - sc)) & insert[bo]);
        WriteL_NA (RUN_PASS, pa, wl);
        WriteL_NA (RUN_PASS, pa1, wh);
    }
    else if (bo == 1)
    {
        wl = (wl & 0xFF0000FF) | (val << 8);
        WriteL_NA (RUN_PASS, pa, wl);
    }
    else
    {
        wh = ReadL_NA (RUN_PASS, pa1);
        wl = (wl & 0x00FFFFFF) | ((val & 0xFF) << 24);
        wh = (wh & 0xFFFFFF00) | ((val >> 8) & 0xFF);
        WriteL_NA (RUN_PASS, pa, wl);
        WriteL_NA (RUN_PASS, pa1, wh);
    }
#elif defined(__x86_32__) || defined(__x86_64__)
    /*
     * SMP case: though this is not formally required by VAX Architecture Standard,
     *           but nevertheless to be on the safe side do our best to
     *           avoid interfering with data adjacent to write region but outside it.
     *           This is what byte masking on SBI and other memory buses actually do.
     *           In x86/x64 case the simplest approach is non-aligned hardware write, wherever possible,
     *           on other machines that do not allow non-aligned access and/or may have
     *           different endianness than VAX, a series of split word/byte writes would be required.
     */
    if (mapen && (uint32) (off + lnt) > VA_PAGSIZE)
    {
        vpn = VA_GETVPN (va + lnt - 1);
        tbi = VA_GETTBI (vpn);
        xpte = (va & VA_S0)? stlb[tbi]: ptlb[tbi];          /* access tlb */
        if ((xpte.pte & acc) == 0 || xpte.tag != vpn || (xpte.pte & TLB_M) == 0)
        {
            xpte = fill (RUN_PASS, va + lnt - 1, acc, NULL);
        }
        pa1 = xpte.pte & TLB_PFN;
        if (!ADDR_IS_MEM(pa) || !ADDR_IS_MEM(pa1))
        {
            Write_Uncommon (RUN_PASS, va, pa, pa1, val, lnt, acc);
            return;
        }
        uint32 xval = (uint32) val;                         // just make it unsigned for less casts and braces below
        if (lnt == L_WORD)
        {
            refoff(t_byte, M, pa) = (uint8) xval;
            refoff(t_byte, M, pa1) = (uint8) (xval >> 8);
        }
        else switch ((off + lnt) & (VA_PAGSIZE - 1))
        {
        /* writing a longword, how many bytes stick out into another page? */
        case 1:
            /* three bytes on this (pa) side, one byte on the other (pa1) side */
            // byte to pa
            // word to pa + 1
            // byte to pa1
            refoff(t_byte, M, pa)  = (t_byte) xval;
            refoff(uint16, M, pa + 1)  = (uint16) (xval >> 8);
            refoff(t_byte, M, pa1)  = (t_byte) (xval >> 24);
            break;
        case 2:
            /* two bytes on this (pa) side, two bytes on the other (pa1) side */
            // word to pa
            // word to pa1
            refoff(uint16, M, pa)  = (uint16) xval;
            refoff(uint16, M, pa1)  = (uint16) (xval >> 16);
            break;
        case 3:
            /* one byte on this (pa) side, three bytes on the other (pa1) side */
            // byte to pa
            // word to pa1
            // byte to pa1 + 2
            refoff(t_byte, M, pa)  = (t_byte) xval;
            refoff(uint16, M, pa1)  = (uint16) (xval >> 8);
            refoff(t_byte, M, pa1 + 2)  = (t_byte) (xval >> 24);
            break;
        }
    }
    else /* datum does not cross page boundary */
    {
        if (ADDR_IS_MEM(pa))
        {
            /* 
             * Datum does not cross virtual page boundary, and hence is contiguous in physical memory.
             * On x86/x64 store it with non-aligned hardware-level write.
             */
            if (lnt == L_WORD)
            {
                refoff(uint16, M, pa)  = (uint16) val;
            }
            else
            {
                refoff(uint32, M, pa) = (uint32) val;
            }
        }
        else
        {
            Write_Uncommon (RUN_PASS, va, pa, -1, val, lnt, acc);
        }
    }
#else
#  error Unimplemented
#endif
}

/* 
 * Handles unaligned writes that are not wholly within memory region.
 */
static void Write_Uncommon (RUN_DECL, uint32 va, int32 pa, int32 pa1_pagebase, int32 val, int32 lnt, int32 acc)
{
    int32 bo, sc, wl, wh, pa1;

    if (mapen && VA_GETOFF(va) + lnt > VA_PAGSIZE)
    {
        pa1 = pa1_pagebase | VA_GETOFF (va + 4);
    }
    else
    {
        pa1 = (pa + 4) & PAMASK;
    }

    bo = pa & 3;
    wl = ReadL_NA (RUN_PASS, pa);

    if (lnt >= L_LONG)
    {
        sc = bo << 3;
        wh = ReadL_NA (RUN_PASS, pa1);
        wl = (wl & insert[bo]) | ((val << sc) & LMASK);
        wh = (wh & ~insert[bo]) | ((val >> (32 - sc)) & insert[bo]);
        WriteL_NA (RUN_PASS, pa, wl);
        WriteL_NA (RUN_PASS, pa1, wh);
    }
    else if (bo == 1)
    {
        wl = (wl & 0xFF0000FF) | (val << 8);
        WriteL_NA (RUN_PASS, pa, wl);
    }
    else
    {
        wh = ReadL_NA (RUN_PASS, pa1);
        wl = (wl & 0x00FFFFFF) | ((val & 0xFF) << 24);
        wh = (wh & 0xFFFFFF00) | ((val >> 8) & 0xFF);
        WriteL_NA (RUN_PASS, pa, wl);
        WriteL_NA (RUN_PASS, pa1, wh);
    }
}

/* 
 * Test access to a byte (VAX PROBEx)
 *
 * If "status" is not NULL, does not raise memory access exceptions (including page or
 * page table not valid faults, and access violation).
 * Instead on exception just returns condition in "*status" and return value as -1.
 * On succes will return physical address corresponding to "va".
 *
 * If "status" is NULL, will raise and throw via ABORT any memory access exceptions
 * encountered during access to memory location.
 * If returns regular way, page is resident, access to "va" requested by "acc" is enabled.
 * Return value is physical address corresponding to "va".
 *
 * Note that this routine may refill TLB entry and evict its previous content.
 * Therefore it is potentially possible for the sequence
 *
 *     Test (va1)
 *     Test (va2)
 *     Read (va1) or Write(va1)
 *
 * to fail if second Test evicts TLB entry for va1, and Read/Write then discovers
 * that PTE for va1 had been modified compared to previously cached in TLB (or is not
 * accessible anymore) and does not allow access to va1.
 */
int32 Test (RUN_DECL, uint32 va, int32 acc, int32 *status)
{
    int32 vpn, off, tbi;
    TLBENT xpte;

    if (status)
        *status = PR_OK;                                    /* assume ok */

    if (mapen)                                              /* mapping on? */
    {
        vpn = VA_GETVPN (va);                               /* get vpn, off */
        off = VA_GETOFF (va);
        tbi = VA_GETTBI (vpn);
        xpte = (va & VA_S0) ? stlb[tbi]: ptlb[tbi];         /* access tlb */
        if ((xpte.pte & acc) && xpte.tag == vpn)            /* TB hit, acc ok? */ 
            return (xpte.pte & TLB_PFN) | off;
        xpte = fill (RUN_PASS, va, acc, status);            /* fill TB */
        if (status == NULL || *status == PR_OK)
            return (xpte.pte & TLB_PFN) | off;
        else
            return -1;
    }
    return va & PAMASK;                                     /* ret phys addr */
}

/*
 * TestMark is identical to Test routine, except it will always mark PTE
 * as Modified if write access is requested, whereas Test may mark
 * sometimes but not always
 */
int32 TestMark (RUN_DECL, uint32 va, int32 acc, int32 *status)
{
    int32 vpn, off, tbi;
    TLBENT xpte;

    if (status)
        *status = PR_OK;                                    /* assume ok */

    if (mapen)                                              /* mapping on? */
    {
        vpn = VA_GETVPN (va);                               /* get vpn, off */
        off = VA_GETOFF (va);
        tbi = VA_GETTBI (vpn);
        xpte = (va & VA_S0) ? stlb[tbi]: ptlb[tbi];         /* access tlb */

        if (acc & TLB_WACC)
        {
            if ((xpte.pte & acc) && xpte.tag == vpn && (xpte.pte & TLB_M))
                return (xpte.pte & TLB_PFN) | off;
        }
        else
        {
            if ((xpte.pte & acc) && xpte.tag == vpn)        /* TB hit, acc ok? */ 
                return (xpte.pte & TLB_PFN) | off;
        }

        xpte = fill (RUN_PASS, va, acc, status);

        if (status == NULL || *status == PR_OK)
            return (xpte.pte & TLB_PFN) | off;
        else
            return -1;
    }

    return va & PAMASK;                                     /* ret phys addr */
}



/* TLB fill

   This routine fills the TLB after a tag or access mismatch, or
   on a write if pte<m> = 0.  It fills the TLB and returns the
   pte to the caller.  On an error, it aborts directly to the
   fault handler in the CPU.

   If called from map (VAX PROBEx), the error status is returned
   to the caller, and no fault occurs.
*/

#define MM_ERR(param) { \
    if (stat) { \
        *stat = param; \
        return zero_pte; \
        } \
    fault_p1 = MM_PARAM (acc & TLB_WACC, param); \
    fault_p2 = va; \
    ABORT ((param & PR_TNV)? ABORT_TNV: ABORT_ACV); }

static TLBENT fill (RUN_DECL, uint32 va, int32 acc, int32 *stat)
{
    int32 ptidx = (((uint32) va) >> 7) & ~03;
    int32 tlbpte, ptead, pte, tbi, vpn;
    static const TLBENT zero_pte = { 0, 0 };

    if (va & VA_S0)                                         /* system space? */
    {
        if (ptidx >= d_slr)                                 /* system */
            MM_ERR (PR_LNV);
        ptead = (d_sbr + ptidx) & PAMASK;
    }
    else
    {
        if (va & VA_P1)                                     /* P1? */
        {
            if (ptidx < d_p1lr)
                MM_ERR (PR_LNV);
            ptead = d_p1br + ptidx;
        }
        else                                                /* P0 */
        {
            if (ptidx >= d_p0lr)
                MM_ERR (PR_LNV);
            ptead = d_p0br + ptidx;
        }
        if ((ptead & VA_S0) == 0)
            ABORT (STOP_PPTE);                              /* ppte must be sys */
        vpn = VA_GETVPN (ptead);                            /* get vpn, tbi */
        tbi = VA_GETTBI (vpn);
        if (stlb[tbi].tag != vpn)                           /* in sys tlb? */
        {
            ptidx = (((uint32) ptead) >> 7) & ~0x3;         /* xlate like sys */
            if (ptidx >= d_slr)
                MM_ERR (PR_PLNV);
            pte = ReadLP (RUN_PASS, (d_sbr + ptidx) & PAMASK);        /* get system pte */
#if defined (VAX_780)
            if ((pte & PTE_ACC) == 0)                       /* spte ACV? */
                MM_ERR (PR_PACV);
#endif
            if ((pte & PTE_V) == 0)                         /* spte TNV? */
                MM_ERR (PR_PTNV);
            stlb[tbi].tag = vpn;                            /* set stlb tag */
            stlb[tbi].pte = cvtacc[PTE_GETACC (pte)] |
                ((pte << VA_N_OFF) & TLB_PFN);              /* set stlb data */
        }
        ptead = (stlb[tbi].pte & TLB_PFN) | VA_GETOFF (ptead);
    }
    pte = ReadL (RUN_PASS, ptead);                          /* read pte */
    tlbpte = cvtacc[PTE_GETACC (pte)] |                     /* cvt access */
        ((pte << VA_N_OFF) & TLB_PFN);                      /* set addr */
    if ((tlbpte & acc) == 0)                                /* chk access */
        MM_ERR (PR_ACV);
    if ((pte & PTE_V) == 0)                                 /* check valid */
        MM_ERR (PR_TNV);
    if (acc & TLB_WACC)                                     /* write? */
    {
        if ((pte & PTE_M) == 0)
        {
            WriteL (RUN_PASS, ptead, pte | PTE_M);
            smp_wmb();
        }
        tlbpte = tlbpte | TLB_M;                            /* set M */
    }
    vpn = VA_GETVPN (va);
    tbi = VA_GETTBI (vpn);
    if ((va & VA_S0) == 0)                                  /* process space? */
    {
        ptlb[tbi].tag = vpn;                                /* store tlb ent */
        ptlb[tbi].pte = tlbpte;
        return ptlb[tbi];
    }
    stlb[tbi].tag = vpn;                                    /* system space */
    stlb[tbi].pte = tlbpte;                                 /* store tlb ent */
    return stlb[tbi];
}

/* Utility routines */

void set_map_reg (RUN_DECL)
{
    d_p0br = P0BR & ~03;
    d_p1br = (P1BR - 0x800000) & ~03;                       /* VA<30> >> 7 */
    d_sbr = (SBR - 0x1000000) & ~03;                        /* VA<31> >> 7 */
    d_p0lr = (P0LR << 2);
    d_p1lr = (P1LR << 2) + 0x800000;                        /* VA<30> >> 7 */
    d_slr = (SLR << 2) + 0x1000000;                         /* VA<31> >> 7 */
}

/* Zap process (0) or whole (1) tb */

void zap_tb (RUN_DECL, int stb, t_bool keep_prefetch)
{
    uint32 i;

    for (i = 0; i < VA_TBSIZE; i++)
    {
        ptlb[i].tag = ptlb[i].pte = -1;
        if (stb)
            stlb[i].tag = stlb[i].pte = -1;
    }

#if VAX_DIRECT_PREFETCH
    /* kludge: invalidate mppc/mppc_rem and ppc/ibcnt */
    if (! keep_prefetch)
    {
        FLUSH_ISTR;
    }
#endif
}

/* Zap single tb entry corresponding to va */

void zap_tb_ent (RUN_DECL, uint32 va)
{
    int32 tbi = VA_GETTBI (VA_GETVPN (va));

    if (va & VA_S0)
        stlb[tbi].tag = stlb[tbi].pte = -1;
    else
        ptlb[tbi].tag = ptlb[tbi].pte = -1;

#if VAX_DIRECT_PREFETCH
    /* kludge: invalidate mppc/mppc_rem and ppc/ibcnt */
    FLUSH_ISTR;
#endif
}

/* Check for tlb entry corresponding to va */

t_bool chk_tb_ent (RUN_DECL, uint32 va)
{
    int32 vpn = VA_GETVPN (va);
    int32 tbi = VA_GETTBI (vpn);
    TLBENT xpte;

    xpte = (va & VA_S0)? stlb[tbi]: ptlb[tbi];
    if (xpte.tag == vpn)
        return TRUE;
    return FALSE;
}

/* TLB examine */

t_stat tlb_ex (t_value *vptr, t_addr addr, UNIT *uptr, int32 sw)
{
    RUN_SCOPE;
    int32 tlbn = sim_unit_index (uptr);
    uint32 idx = (uint32) addr >> 1;

    if (idx >= VA_TBSIZE)
        return SCPE_NXM;
    if (addr & 1)
        *vptr = (uint32) (tlbn ? stlb[idx].pte: ptlb[idx].pte);
    else
        *vptr = (uint32) (tlbn ? stlb[idx].tag: ptlb[idx].tag);
    return SCPE_OK;
}

/* TLB deposit */

t_stat tlb_dep (t_value val, t_addr addr, UNIT *uptr, int32 sw)
{
    RUN_SCOPE;
    int32 tlbn = sim_unit_index (uptr);
    uint32 idx = (uint32) addr >> 1;

    if (idx >= VA_TBSIZE)
        return SCPE_NXM;
    if (addr & 1)
    {
        if (tlbn)
            stlb[idx].pte = (int32) val;
        else
            ptlb[idx].pte = (int32) val;
    }
    else
    {
        if (tlbn)
            stlb[idx].tag = (int32) val;
        else
            ptlb[idx].tag = (int32) val;
    }
    return SCPE_OK;
}

/* TLB reset */

t_stat tlb_reset (DEVICE *dptr)
{
    RUN_SCOPE;
    uint32 i;

    for (i = 0; i < VA_TBSIZE; i++)
        stlb[i].tag = ptlb[i].tag = stlb[i].pte = ptlb[i].pte = -1;
    return SCPE_OK;
}

/*
 * Reading non-memory space
 */

int32 ReadB_nomem (RUN_DECL, uint32 pa)
{
    int32 dat;

    mchk_ref = REF_V;
    if (ADDR_IS_IO (pa))
        dat = ReadIO (RUN_PASS, pa, L_BYTE);
    else
        dat = ReadReg (RUN_PASS, pa, L_BYTE);

    return ((dat >> ((pa & 3) << 3)) & BMASK);
}

int32 ReadW_nonmem (RUN_DECL, uint32 pa)
{
    int32 dat;

    mchk_ref = REF_V;
    if (ADDR_IS_IO (pa))
        dat = ReadIO (RUN_PASS, pa, L_WORD);
    else
        dat = ReadReg (RUN_PASS, pa, L_WORD);

    return ((dat >> ((pa & 2)? 16: 0)) & WMASK);
}

int32 ReadL_nonmem (RUN_DECL, uint32 pa)
{
    mchk_ref = REF_V;
    if (ADDR_IS_IO (pa))
        return ReadIO (RUN_PASS, pa, L_LONG);
    else
        return ReadReg (RUN_PASS, pa, L_LONG);
}

int32 ReadLP_nonmem (RUN_DECL, uint32 pa)
{
    mchk_va = pa;
    mchk_ref = REF_P;
    if (ADDR_IS_IO (pa))
        return ReadIO (RUN_PASS, pa, L_LONG);
    else
        return ReadReg (RUN_PASS, pa, L_LONG);
}

/*
 * Writing non-memory space
 */

void WriteB_nomem (RUN_DECL, uint32 pa, int32 val)
{
    mchk_ref = REF_V;
    if (ADDR_IS_IO (pa))
        WriteIO (RUN_PASS, pa, val, L_BYTE);
    else
        WriteReg (RUN_PASS, pa, val, L_BYTE);
}

void WriteW_nonmem (RUN_DECL, uint32 pa, int32 val)
{
    mchk_ref = REF_V;
    if (ADDR_IS_IO (pa))
        WriteIO (RUN_PASS, pa, val, L_WORD);
    else
        WriteReg (RUN_PASS, pa, val, L_WORD);
}

void WriteL_nonmem (RUN_DECL, uint32 pa, int32 val)
{
    mchk_ref = REF_V;
    if (ADDR_IS_IO (pa))
        WriteIO (RUN_PASS, pa, val, L_LONG);
    else
        WriteReg (RUN_PASS, pa, val, L_LONG);
}

void WriteLP_nomem (RUN_DECL, uint32 pa, int32 val)
{
    mchk_va = pa;
    mchk_ref = REF_P;
    if (ADDR_IS_IO (pa))
        WriteIO (RUN_PASS, pa, val, L_LONG);
    else
        WriteReg (RUN_PASS, pa, val, L_LONG);
}
