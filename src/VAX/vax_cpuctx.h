#ifndef __VAX_MP_CPUCTX_H__
#define __VAX_MP_CPUCTX_H__ 1

// #pragma message ("Loading vax_cpuctx.h")
#include "vax_defs.h"

#define memzero(x) memset(x, 0, sizeof(x))

#define cpu_sim_interval(xcpu) ((xcpu)->cpu_context.r_sim_interval)
#define sim_interval cpu_sim_interval(cpu_unit)
#define R (cpu_unit->cpu_context.r_R)
#define PSL (cpu_unit->cpu_context.r_PSL)
#define STK (cpu_unit->cpu_context.r_STK)
// #ifndef KSP
// #  define KSP  STK[KERN]
// #  define ESP  STK[EXEC]
// #  define SSP  STK[SUPV]
// #  define USP  STK[USER]
// #  define IS   STK[4]
// #endif
#define SCBB (cpu_unit->cpu_context.r_SCBB)
#define PCBB (cpu_unit->cpu_context.r_PCBB)
#define P0BR (cpu_unit->cpu_context.r_P0BR)
#define P0LR (cpu_unit->cpu_context.r_P0LR)
#define P1BR (cpu_unit->cpu_context.r_P1BR)
#define P1LR (cpu_unit->cpu_context.r_P1LR)
#define SBR (cpu_unit->cpu_context.r_SBR)
#define SLR (cpu_unit->cpu_context.r_SLR)
#define WHAMI (cpu_unit->cpu_context.r_WHAMI)
#define SISR (cpu_unit->cpu_context.r_SISR)
#define ASTLVL (cpu_unit->cpu_context.r_ASTLVL)
#define mapen (cpu_unit->cpu_context.r_mapen)
#define pme (cpu_unit->cpu_context.r_pme)
#define crd_err (cpu_unit->cpu_context.r_crd_err)
#define mem_err (cpu_unit->cpu_context.r_mem_err)
#define pcq (cpu_unit->cpu_context.r_pcq)
#define pcq_p (cpu_unit->cpu_context.r_pcq_p)
#define mchk_va (cpu_unit->cpu_context.r_mchk_va)
#define mchk_ref (cpu_unit->cpu_context.r_mchk_ref)
#define d_p0br (cpu_unit->cpu_context.r_d_p0br)
#define d_p0lr (cpu_unit->cpu_context.r_d_p0lr)
#define d_p1br (cpu_unit->cpu_context.r_d_p1br)
#define d_p1lr (cpu_unit->cpu_context.r_d_p1lr)
#define d_sbr (cpu_unit->cpu_context.r_d_sbr)
#define d_slr (cpu_unit->cpu_context.r_d_slr)
#define stlb (cpu_unit->cpu_context.r_stlb)
#define ptlb (cpu_unit->cpu_context.r_ptlb)
#define fault_p1 (cpu_unit->cpu_context.r_fault_p1)
#define fault_p2 (cpu_unit->cpu_context.r_fault_p2)
#define fault_PC (cpu_unit->cpu_context.r_fault_PC)
#define trpirq (cpu_unit->cpu_context.r_trpirq)
#define in_ie (cpu_unit->cpu_context.r_in_ie)
#define recq (cpu_unit->cpu_context.r_recq)
#define recqptr (cpu_unit->cpu_context.r_recqptr)
#if VAX_DIRECT_PREFETCH
#  define mppc (cpu_unit->cpu_context.r_mppc)
#  define mppc_rem (cpu_unit->cpu_context.r_mppc_rem)
#endif
#define ppc (cpu_unit->cpu_context.r_ppc)
#define ibcnt (cpu_unit->cpu_context.r_ibcnt)
#define ibufl (cpu_unit->cpu_context.r_ibufl)
#define ibufh (cpu_unit->cpu_context.r_ibufh)
#define badabo (cpu_unit->cpu_context.r_badabo)
#define cq_scr  (cpu_unit->cpu_context.r_cq_scr)
#define cq_dser  (cpu_unit->cpu_context.r_cq_dser)
#define cq_mear  (cpu_unit->cpu_context.r_cq_mear)
#define cq_sear  (cpu_unit->cpu_context.r_cq_sear)
#define cq_ipc  (cpu_unit->cpu_context.r_cq_ipc)
#define cdg_dat  (cpu_unit->cpu_context.r_cdg_dat)
#define ka_cacr  (cpu_unit->cpu_context.r_ka_cacr)
#define CADR  (cpu_unit->cpu_context.r_CADR)
#define MSER  (cpu_unit->cpu_context.r_MSER)
#define cmctl_reg  (cpu_unit->cpu_context.r_cmctl_reg)
#define ssc_bto  (cpu_unit->cpu_context.r_ssc_bto)
#define ssc_otp  (cpu_unit->cpu_context.r_ssc_otp)
#define tmr_csr  (cpu_unit->cpu_context.r_tmr_csr)
#define tmr_tir  (cpu_unit->cpu_context.r_tmr_tir)
#define tmr_tir_rtstart  (cpu_unit->cpu_context.r_tmr_tir_rtstart)
#define tmr_tnir  (cpu_unit->cpu_context.r_tmr_tnir)
#define tmr_tivr  (cpu_unit->cpu_context.r_tmr_tivr)
#define tmr_inc  (cpu_unit->cpu_context.r_tmr_inc)
#define tmr_sav  (cpu_unit->cpu_context.r_tmr_sav)
#define ssc_adsm  (cpu_unit->cpu_context.r_ssc_adsm)
#define ssc_adsk  (cpu_unit->cpu_context.r_ssc_adsk)
#define tti_csr  (cpu_unit->cpu_context.r_tti_csr)
#define tto_csr  (cpu_unit->cpu_context.r_tto_csr)
#define tto_buf  (cpu_unit->cpu_context.r_tto_buf)
#define clk_csr  (cpu_unit->cpu_context.r_clk_csr)
#define todr_reg  (cpu_unit->cpu_context.r_todr_reg)
#define todr_blow  (cpu_unit->cpu_context.r_todr_blow)

#define primary_todr_reg  (cpu_unit_0.cpu_context.r_todr_reg)
#define primary_todr_blow  (cpu_unit_0.cpu_context.r_todr_blow)

typedef struct
{
    int32       tag;                                    /* tag */
    int32       pte;                                    /* pte */
}
TLBENT;

class CPU_CONTEXT
{
private:
    /* first reset? */
    t_bool initial;

public:
    /* emulator runtime */
    int32 r_sim_interval;

    /* processor registers */
    int32 r_R[16];
    int32 r_PSL;
    int32 r_STK[5];
    int32 r_SCBB;
    int32 r_PCBB;
    int32 r_P0BR;
    int32 r_P0LR;
    int32 r_P1BR;
    int32 r_P1LR;
    int32 r_SBR;
    int32 r_SLR;
    int32 r_WHAMI;
    int32 r_SISR;
    int32 r_ASTLVL;
    int32 r_mapen;
    int32 r_pme;
    int32 r_crd_err;
    int32 r_mem_err;
    int32 r_pcq[PCQ_SIZE];
    uint32 r_pcq_qptr;
    int32 r_pcq_p;

    /* for mcheck */
    int32 r_mchk_va;
    int32 r_mchk_ref;

    /* dynamic copies, altered per ucode */
    int32 r_d_p0br;
    int32 r_d_p0lr;
    int32 r_d_p1br;
    int32 r_d_p1lr;
    int32 r_d_sbr;
    int32 r_d_slr;

    TLBENT r_stlb[VA_TBSIZE];
    TLBENT r_ptlb[VA_TBSIZE];

    /* fault parameters */
    SIM_ALIGN_32
    int32 r_fault_p1;
    int32 r_fault_p2;
    int32 r_fault_PC;

    /* interrupt requests */
    int32 r_trpirq;

    /* in exception or interrupt */
    int32 r_in_ie;

    /* instruction recovery queue */
    int32 r_recq[6];    /* recovery queue */
    SIM_ALIGN_32
    int32 r_recqptr;    /* recq pointer */

#if VAX_DIRECT_PREFETCH
    t_byte* r_mppc;          /* next memory byte fetch pointer (only if ADDR_IS_MEM) */
    int32   r_mppc_rem;      /* remaining memory fetch byte count */
#endif
    int32 r_ppc;             /* instruction prefetch ctl */
    int32 r_ibcnt;           /* instruction prefetch ctl */
    int32 r_ibufl, r_ibufh;  /* instruction prefetch buffer */

    int32 r_badabo;          /* last abort code */

    /* CQBIC registers: SCR, DSER, MEAR, SEAR and IPC */
    int32 r_cq_scr;
    int32 r_cq_dser;
    int32 r_cq_mear;
    int32 r_cq_sear;
    int32 r_cq_ipc;

    int32 r_cdg_dat[CDASIZE >> 2];       /* cache data */
    int32 r_ka_cacr;                     /* CACR, KA655 cache ctl */

    /* 
     * CADR is 1ST LEVEL CACHE STATUS, normal value after operating system startup is comleted:
     *          CADR:  000000FC
     *          _D STREAM ENABLED
     *          _I STREAM ENABLED
     *          _SET 1 ENABLED
     *          _SET 2 ENABLED
     */
    int32 r_CADR;                        /* cache disable reg */
    int32 r_MSER;                        /* mem sys error reg */
    int32 r_cmctl_reg[CMCTLSIZE >> 2];   /* cache memory control registers */

    int32 r_ssc_bto;                     /* SSC timeout */
    int32 r_ssc_otp;                     /* SSC output port */
    int32 r_tmr_csr[2];                  /* SSC timers */
    uint32 r_tmr_tir[2];                 /* curr interval */
    uint32 r_tmr_tir_rtstart[2];         /* starting value  (when delta timer is used) */
    uint32 r_tmr_tnir[2];                /* next interval */
    int32 r_tmr_tivr[2];                 /* vector */
    uint32 r_tmr_inc[2];                 /* tir increment */
    uint32 r_tmr_sav[2];                 /* saved inst cnt */
    int32 r_ssc_adsm[2];                 /* addr strobes */
    int32 r_ssc_adsk[2];

    int32 r_tti_csr;                     /* control/status */
    int32 r_tto_csr;                     /* control/status */
    int32 r_tto_buf;                     /* data buffer */
    int32 r_clk_csr;                     /* control/status */
    atomic_int32 r_todr_reg;             /* TODR register */
    int32 r_todr_blow;                   /* TODR battery low */

    uint32 scb_range_pamask;             /* auxiliary variable for fast checks by PA_MAY_BE_INSIDE_SCB */

    int32 highest_irql;                  /* highest IRQL (IPL) of a pending interrupt */

    CPU_CONTEXT();

    void reset(CPU_UNIT* cpu_unit);
};
#endif

#ifdef VAX_MP_CPUCTX_METHODS
CPU_CONTEXT::CPU_CONTEXT()
{
    initial = TRUE;
    CPU_UNIT* cpu_unit = CPU_UNIT::getBy(this);
    cqbic_reset_percpu(RUN_PASS, TRUE);
}

void CPU_CONTEXT::reset(CPU_UNIT* cpu_unit)
{
    sim_interval = 0;
    memzero(R);
    PSL = PSL_IS | PSL_IPL1F;
    KSP = 0;
    ESP = 0;
    SSP = 0;
    USP = 0;
    IS = 0;
    SCBB = 0;
    PCBB = 0;
    P0BR = 0;
    P0LR = 0;
    P1BR = 0;
    P1LR = 0;
    SBR = 0;
    SLR = 0;
    WHAMI = 0;
    SISR = 0;
    ASTLVL = 4;
    mapen = 0;
    pme = 0;
    crd_err = 0;
    mem_err = 0;
    memzero(pcq);
    r_pcq_qptr = 0;
    pcq_p = 0;
    mchk_va = 0;
    mchk_ref = 0;
    d_p0br = 0;
    d_p0lr = 0;
    d_p1br = 0;
    d_p1lr = 0;
    d_sbr = 0;
    d_slr = 0;
    memzero(stlb);
    memzero(ptlb);
    fault_p1 = 0;
    fault_p2 = 0;
    fault_PC = 0;
    trpirq = 0;
    in_ie = 0;
    recqptr = 0;
#if VAX_DIRECT_PREFETCH
    mppc = NULL;
    mppc_rem = 0;
#endif
    ppc = -1;
    ibcnt = 0;
    ibufl = ibufh = 0;
    badabo = 0;

    cqbic_reset_percpu(RUN_PASS, FALSE);

    memzero(cdg_dat);
    ka_cacr = 0;
    CADR = 0;
    MSER = 0;
    memzero(r_cmctl_reg);

    ssc_bto = 0;
    ssc_otp = 0;

    memzero(tmr_csr);
    memzero(tmr_tir);
    memzero(tmr_tir_rtstart);
    memzero(tmr_tnir);
    memzero(tmr_tivr);
    memzero(tmr_inc);
    memzero(tmr_sav);
    memzero(ssc_adsm);
    memzero(ssc_adsk);

    tti_csr = 0;
    tto_csr = 0;
    tto_buf = 0;
    clk_csr = 0;
    todr_reg = 0;
    todr_blow = 1;

    scb_range_pamask = 0;

    highest_irql = 0;

    initial = FALSE;

    cpu_on_clear_mapen(RUN_PASS);
}
#endif
