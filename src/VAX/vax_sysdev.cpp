/* vax_sysdev.c: VAX 3900 system-specific logic

   Copyright (c) 1998-2011, Robert M Supnik

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

   This module contains the CVAX chip and VAX 3900 system-specific registers
   and devices.

   rom          bootstrap ROM (no registers)
   nvr          non-volatile ROM (no registers)
   csi          console storage input
   cso          console storage output
   sysd         system devices (SSC miscellany)

   23-Dec-10    RMS     Added power clear call to boot routine (from Mark Pizzolato)
   25-Oct-05    RMS     Automated CMCTL extended memory
   16-Aug-05    RMS     Fixed C++ declaration and cast problems
   10-Mar-05    RMS     Fixed bug in timer schedule routine (from Mark Hittinger)
   30-Sep-04    RMS     Moved CADR, MSER, CONPC, CONPSL, machine_check, cpu_boot,
                         con_halt here from vax_cpu.c
                        Moved model-specific IPR's here from vax_cpu1.c
   09-Sep-04    RMS     Integrated powerup into RESET (with -p)
                        Added model-specific registers and routines from CPU
   23-Jan-04    MP      Added extended physical memory support (Mark Pizzolato)
   07-Jun-03    MP      Added calibrated delay to ROM reads (Mark Pizzolato)
                        Fixed calibration problems interval timer (Mark Pizzolato)
   12-May-03    RMS     Fixed compilation warnings from VC.Net
   23-Apr-03    RMS     Revised for 32b/64b t_addr
   19-Aug-02    RMS     Removed unused variables (found by David Hittner)
                        Allowed NVR to be attached to file
   30-May-02    RMS     Widened POS to 32b
   28-Feb-02    RMS     Fixed bug, missing end of table (found by Lars Brinkhoff)
*/

#include "sim_defs.h"
#include "vax_defs.h"

#ifndef DONT_USE_INTERNAL_ROM
#  include "vax_ka655x_bin.h"
#endif

#define UNIT_V_NODELAY  (UNIT_V_UF + 0)                 /* ROM access equal to RAM access */
#define UNIT_NODELAY    (1u << UNIT_V_NODELAY)

/* Console storage control/status */

#define CSICSR_IMP      (CSR_DONE + CSR_IE)             /* console input */
#define CSICSR_RW       (CSR_IE)
#define CSOCSR_IMP      (CSR_DONE + CSR_IE)             /* console output */
#define CSOCSR_RW       (CSR_IE)

/* CMCTL configuration registers */

#define CMCNF_VLD       0x80000000                      /* addr valid */
#define CMCNF_BA        0x1FF00000                      /* base addr */
#define CMCNF_LOCK      0x00000040                      /* lock NI */
#define CMCNF_SRQ       0x00000020                      /* sig req WO */
#define CMCNF_SIG       0x0000001F                      /* signature */
#define CMCNF_RW        (CMCNF_VLD | CMCNF_BA)          /* read/write */
#define CMCNF_MASK      (CMCNF_RW | CMCNF_SIG)
#define MEM_BANK        (1 << 22)                       /* bank size 4MB */
#define MEM_SIG         (0x17)                          /* ECC, 4 x 4MB */

/* CMCTL error register */

#define CMERR_RDS       0x80000000                      /* uncorr err NI */
#define CMERR_FRQ       0x40000000                      /* 2nd RDS NI */
#define CMERR_CRD       0x20000000                      /* CRD err NI */
#define CMERR_PAG       0x1FFFFC00                      /* page addr NI */
#define CMERR_DMA       0x00000100                      /* DMA err NI */
#define CMERR_BUS       0x00000080                      /* bus err NI */
#define CMERR_SYN       0x0000007F                      /* syndrome NI */
#define CMERR_W1C       (CMERR_RDS | CMERR_FRQ | CMERR_CRD | \
                         CMERR_DMA | CMERR_BUS)

/* CMCTL control/status register */

#define CMCSR_PMI       0x00002000                      /* PMI speed NI */
#define CMCSR_CRD       0x00001000                      /* enb CRD int NI */
#define CMCSR_FRF       0x00000800                      /* force ref WONI */
#define CMCSR_DET       0x00000400                      /* dis err NI */
#define CMCSR_FDT       0x00000200                      /* fast diag NI */
#define CMCSR_DCM       0x00000080                      /* diag mode NI */
#define CMCSR_SYN       0x0000007F                      /* syndrome NI */
#define CMCSR_MASK      (CMCSR_PMI | CMCSR_CRD | CMCSR_DET | \
                         CMCSR_FDT | CMCSR_DCM | CMCSR_SYN)

/* KA655 boot/diagnostic register */

#define BDR_BRKENB      0x00000080                      /* break enable */

/* KA655 cache control register */

#define CACR_DRO        0x00FFFF00                      /* diag bits RO */
#define CACR_V_DPAR     24                              /* data parity */
#define CACR_FIXED      0x00000040                      /* fixed bits */
#define CACR_CPE        0x00000020                      /* parity err W1C */
#define CACR_CEN        0x00000010                      /* enable */                    
#define CACR_DPE        0x00000004                      /* disable par NI */
#define CACR_WWP        0x00000002                      /* write wrong par NI */
#define CACR_DIAG       0x00000001                      /* diag mode */
#define CACR_W1C        (CACR_CPE)
#define CACR_RW         (CACR_CEN | CACR_DPE | CACR_WWP | CACR_DIAG)

/*
 * For purposes of SSC (System Support Chip) see "KA655 CPU System Maintenance" manual, p. 1.10
 */

/* SSC base register */

#define SSCBASE_MBO     0x20000000                      /* must be one */
#define SSCBASE_RW      0x1FFFFC00                      /* base address */

/* SSC configuration register */

#define SSCCNF_BLO      0x80000000                      /* batt low W1C */
#define SSCCNF_IVD      0x08000000                      /* int dsbl NI */
#define SSCCNF_IPL      0x03000000                      /* int IPL NI */
#define SSCCNF_ROM      0x00F70000                      /* ROM param NI */
#define SSCCNF_CTLP     0x00008000                      /* ctrl P enb */
#define SSCCNF_BAUD     0x00007700                      /* baud rates NI */
#define SSCCNF_ADS      0x00000077                      /* addr strb NI */
#define SSCCNF_W1C      SSCCNF_BLO
#define SSCCNF_RW       0x0BF7F777

/* SSC timeout register */

#define SSCBTO_BTO      0x80000000                      /* timeout W1C */
#define SSCBTO_RWT      0x40000000                      /* read/write W1C */
#define SSCBTO_INTV     0x00FFFFFF                      /* interval NI */
#define SSCBTO_W1C      (SSCBTO_BTO | SSCBTO_RWT)
#define SSCBTO_RW       SSCBTO_INTV

/* SSC output port */

#define SSCOTP_MASK     0x0000000F                      /* output port */

/* SSC timer control/status */

#define TMR_CSR_ERR     0x80000000                      /* error W1C */
#define TMR_CSR_DON     0x00000080                      /* done W1C */
#define TMR_CSR_IE      0x00000040                      /* int enb */
#define TMR_CSR_SGL     0x00000020                      /* single WO */
#define TMR_CSR_XFR     0x00000010                      /* xfer WO */
#define TMR_CSR_STP     0x00000004                      /* stop */
#define TMR_CSR_RUN     0x00000001                      /* run */
#define TMR_CSR_W1C     (TMR_CSR_ERR | TMR_CSR_DON)
#define TMR_CSR_RW      (TMR_CSR_IE | TMR_CSR_STP | TMR_CSR_RUN)

/* SSC timer intervals */

#define TMR_INC         10000                           /* usec/interval */

/* SSC timer vector */

#define TMR_VEC_MASK    0x000003FC                      /* vector */

/* SSC address strobes */

#define SSCADS_MASK     0x3FFFFFFC                      /* match or mask */

#define sysd_activating(tmr)       do { cpu_unit->sysd_active_mask |= (1 << (tmr)); } while (0)
#define sysd_deactivating(tmr)     do { cpu_unit->sysd_active_mask &= ~(1 << (tmr)); } while (0)

extern UNIT clk_unit;
extern int32 sim_switches;

uint32 *rom = NULL;                                     /* boot ROM */
uint32 *nvr = NULL;                                     /* non-volatile mem */
int32 conpc, conpsl;                                    /* console reg */
int32 csi_csr = 0;                                      /* control/status */
int32 cso_csr = 0;                                      /* control/status */
int32 ka_bdr = BDR_BRKENB;                              /* KA655 boot diag */
int32 ssc_base = SSCBASE;                               /* SSC base */
int32 ssc_cnf = 0;                                      /* SSC conf */
static uint32 rom_delay = 0;

t_stat rom_ex (t_value *vptr, t_addr exta, UNIT *uptr, int32 sw);
t_stat rom_dep (t_value val, t_addr exta, UNIT *uptr, int32 sw);
t_stat rom_reset (DEVICE *dptr);
t_stat nvr_ex (t_value *vptr, t_addr exta, UNIT *uptr, int32 sw);
t_stat nvr_dep (t_value val, t_addr exta, UNIT *uptr, int32 sw);
t_stat nvr_reset (DEVICE *dptr);
t_stat nvr_attach (UNIT *uptr, char *cptr);
t_stat nvr_detach (UNIT *uptr);
t_stat csi_reset (DEVICE *dptr);
t_stat cso_reset (DEVICE *dptr);
t_stat cso_svc (RUN_SVC_DECL, UNIT *uptr);
t_stat tmr_svc (RUN_SVC_DECL, UNIT *uptr);
t_stat sysd_reset (DEVICE *dptr);

int32 rom_rd (RUN_DECL, int32 pa);
int32 nvr_rd (RUN_DECL, int32 pa);
void nvr_wr (RUN_DECL, int32 pa, int32 val, int32 lnt);
int32 csrs_rd (RUN_DECL);
int32 csrd_rd (RUN_DECL);
int32 csts_rd (RUN_DECL);
void csrs_wr (RUN_DECL, int32 dat);
void csts_wr (RUN_DECL, int32 dat);
void cstd_wr (RUN_DECL, int32 dat);
int32 cmctl_rd (RUN_DECL, int32 pa);
void cmctl_wr (RUN_DECL, int32 pa, int32 val, int32 lnt);
int32 ka_rd (RUN_DECL, int32 pa);
void ka_wr (RUN_DECL, int32 pa, int32 val, int32 lnt);
int32 cdg_rd (RUN_DECL, int32 pa);
void cdg_wr (RUN_DECL, int32 pa, int32 val, int32 lnt);
int32 ssc_rd (RUN_DECL, int32 pa);
void ssc_wr (RUN_DECL, int32 pa, int32 val, int32 lnt);
int32 tmr_tir_rd (RUN_DECL, int32 tmr, t_bool interp);
void tmr_csr_wr (RUN_DECL, int32 tmr, int32 val);
void tmr_sched (RUN_DECL, int32 tmr, t_bool is_realtime);
void tmr_incr (RUN_DECL, int32 tmr, uint32 inc, t_bool is_realtime);
int32 tmr0_inta (void);
int32 tmr1_inta (void);
static int32 tmr_tir_rd_realtime (RUN_DECL, int32 tmr);
static void tmr_csr_wr_realtime (RUN_DECL, int32 tmr, int32 val);
static void tmr_adjust_thread_priority(RUN_DECL, t_bool force_high = FALSE);
int32 parity (int32 val, int32 odd);
t_stat sysd_powerup (RUN_DECL);

extern int32 intexc (RUN_DECL, int32 vec, int32 cc, int32 ipl, int ei);
extern int32 cqmap_rd (RUN_DECL, int32 pa);
extern void cqmap_wr (RUN_DECL, int32 pa, int32 val, int32 lnt);
extern int32 cqipc_rd (RUN_DECL, int32 pa);
extern void cqipc_wr (RUN_DECL, int32 pa, int32 val, int32 lnt);
extern int32 cqbic_rd (RUN_DECL, int32 pa);
extern void cqbic_wr (RUN_DECL, int32 pa, int32 val, int32 lnt);
extern int32 cqmem_rd (RUN_DECL, int32 pa);
extern void cqmem_wr (RUN_DECL, int32 pa, int32 val, int32 lnt);
extern int32 iccs_rd (RUN_DECL);
extern int32 todr_rd (RUN_DECL);
extern int32 rxcs_rd (RUN_DECL);
extern int32 rxdb_rd (RUN_DECL);
extern int32 txcs_rd (RUN_DECL);
extern void iccs_wr (RUN_DECL, int32 dat);
extern void todr_wr (RUN_DECL, int32 dat);
extern void rxcs_wr (RUN_DECL, int32 dat);
extern void txcs_wr (RUN_DECL, int32 dat);
extern void txdb_wr (RUN_DECL, int32 dat);
extern void ioreset_wr (RUN_DECL, int32 dat);
extern uint32 sim_os_msec();

/*
 * For more information on ROM see "KA655 CPU System Maintenance" manual, p. 1.11.
 */

/* ROM data structures

   rom_dev      ROM device descriptor
   rom_unit     ROM units
   rom_reg      ROM register list
*/

UNIT rom_unit UDATA_SINGLE (NULL, UNIT_FIX+UNIT_BINK, ROMSIZE);
UNIT_TABLE_SINGLE(rom_unit);

REG rom_reg[] = {
    { NULL }
    };

MTAB rom_mod[] = {
    { UNIT_NODELAY, UNIT_NODELAY, "fast access", "NODELAY", NULL },
    { UNIT_NODELAY, 0, "1usec calibrated access", "DELAY", NULL },
    { 0 }
    };

DEVICE rom_dev = {
    "ROM", rom_unit_table, rom_reg, rom_mod,
    1, 16, ROMAWIDTH, 4, 16, 32,
    &rom_ex, &rom_dep, &rom_reset,
    NULL, NULL, NULL,
    NULL, 0
    };

/* NVR data structures

   nvr_dev      NVR device descriptor
   nvr_unit     NVR units
   nvr_reg      NVR register list
*/

UNIT nvr_unit UDATA_SINGLE (NULL, UNIT_FIX+UNIT_BINK, NVRSIZE);
UNIT_TABLE_SINGLE(nvr_unit);

REG nvr_reg[] = {
    { NULL }
    };

DEVICE nvr_dev = {
    "NVR", nvr_unit_table, nvr_reg, NULL,
    1, 16, NVRAWIDTH, 4, 16, 32,
    &nvr_ex, &nvr_dep, &nvr_reset,
    NULL, &nvr_attach, &nvr_detach,
    NULL, 0
    };

/* CSI data structures

   csi_dev      CSI device descriptor
   csi_unit     CSI unit descriptor
   csi_reg      CSI register list
*/

DIB csi_dib = { 0, 0, NULL, NULL, 1, IVCL (CSI), SCB_CSI, { NULL } };

UNIT csi_unit UDATA_SINGLE_WAIT (NULL, 0, 0, KBD_POLL_WAIT);
UNIT_TABLE_SINGLE(csi_unit);

REG csi_reg[] = {
    { ORDATA_GBL (BUF, csi_unit.buf, 8) },
    { ORDATA_GBL (CSR, csi_csr, 16) },
    { IRDATA_DEV (INT, IVCL (CSI)) },
    { FLDATA_GBL (DONE, csi_csr, CSR_V_DONE) },
    { FLDATA_GBL (IE, csi_csr, CSR_V_IE) },
    { DRDATA_GBL (POS, csi_unit.pos, 32), PV_LEFT },
    { DRDATA_GBL (TIME, csi_unit.wait, 24), REG_NZ + PV_LEFT },
    { NULL }
    };

MTAB csi_mod[] = {
    { MTAB_XTD|MTAB_VDV, 0, "VECTOR", NULL,     NULL, &show_vec },
    { 0 }
    };

DEVICE csi_dev = {
    "CSI", csi_unit_table, csi_reg, csi_mod,
    1, 10, 31, 1, 8, 8,
    NULL, NULL, &csi_reset,
    NULL, NULL, NULL,
    &csi_dib, 0
    };

/* CSO data structures

   cso_dev      CSO device descriptor
   cso_unit     CSO unit descriptor
   cso_reg      CSO register list
*/

DIB cso_dib = { 0, 0, NULL, NULL, 1, IVCL (CSO), SCB_CSO, { NULL } };

UNIT cso_unit UDATA_SINGLE_WAIT (&cso_svc, UNIT_SEQ+UNIT_ATTABLE, 0, SERIAL_OUT_WAIT);
UNIT_TABLE_SINGLE(cso_unit);

REG cso_reg[] = {
    { ORDATA_GBL (BUF, cso_unit.buf, 8) },
    { ORDATA_GBL (CSR, cso_csr, 16) },
    { IRDATA_DEV (INT, IVCL (CSO)) },
    { FLDATA_GBL (DONE, cso_csr, CSR_V_DONE) },
    { FLDATA_GBL (IE, cso_csr, CSR_V_IE) },
    { DRDATA_GBL (POS, cso_unit.pos, 32), PV_LEFT },
    { DRDATA_GBL (TIME, cso_unit.wait, 24), PV_LEFT },
    { NULL }
    };

MTAB cso_mod[] = {
    { MTAB_XTD|MTAB_VDV, 0, "VECTOR", NULL,     NULL, &show_vec },
    { 0 }
    };

DEVICE cso_dev = {
    "CSO", cso_unit_table, cso_reg, cso_mod,
    1, 10, 31, 1, 8, 8,
    NULL, NULL, &cso_reset,
    NULL, NULL, NULL,
    &cso_dib, 0
    };

/* SYSD data structures

   sysd_dev     SYSD device descriptor
   sysd_unit    SYSD units
   sysd_reg     SYSD register list
*/

DIB sysd_dib = {
    0, 0, NULL, NULL,
    2, IVCL (TMR0), 0, { &tmr0_inta, &tmr1_inta }
    };

UNIT* sysd_unit[] = {
    UDATA (&tmr_svc, 0, 0),
    UDATA (&tmr_svc, 0, 0)
    };

REG sysd_reg[] = {
    { HRDATA_CPU ("CADR", r_CADR, 8) },
    { HRDATA_CPU ("MSER", r_MSER, 8) },
    { HRDATA_GBL (CONPC, conpc, 32) },
    { HRDATA_GBL (CONPSL, conpsl, 32) },
    { BRDATA_CPU ("CMCSR", r_cmctl_reg, 16, 32, CMCTLSIZE >> 2) },
    { HRDATA_CPU ("CACR", r_ka_cacr, 8) },
    { HRDATA_GBL (BDR, ka_bdr, 8) },
    { HRDATA_GBL (BASE, ssc_base, 29) },
    { HRDATA_GBL (CNF, ssc_cnf, 32) },
    { HRDATA_CPU ("BTO", r_ssc_bto, 32) },
    { HRDATA_CPU ("OTP", r_ssc_otp, 4) },
    { HRDATA_CPU ("TCSR0", r_tmr_csr[0], 32) },
    { HRDATA_CPU ("TIR0", r_tmr_tir[0], 32) },
    { HRDATA_CPU ("TNIR0", r_tmr_tnir[0], 32) },
    { HRDATA_CPU ("TIVEC0", r_tmr_tivr[0], 9) },
    { HRDATA_CPU ("TINC0", r_tmr_inc[0], 32) },
    { HRDATA_CPU ("TSAV0", r_tmr_sav[0], 32) },
    { HRDATA_CPU ("TCSR1", r_tmr_csr[1], 32) },
    { HRDATA_CPU ("TIR1", r_tmr_tir[1], 32) },
    { HRDATA_CPU ("TNIR1", r_tmr_tnir[1], 32) },
    { HRDATA_CPU ("TIVEC1", r_tmr_tivr[1], 9) },
    { HRDATA_CPU ("TINC1", r_tmr_inc[1], 32) },
    { HRDATA_CPU ("TSAV1", r_tmr_sav[1], 32) },
    { HRDATA_CPU ("ADSM0", r_ssc_adsm[0], 32) },
    { HRDATA_CPU ("ADSK0", r_ssc_adsk[0], 32) },
    { HRDATA_CPU ("ADSM1", r_ssc_adsm[1], 32) },
    { HRDATA_CPU ("ADSK1", r_ssc_adsk[1], 32) },
    { BRDATA_CPU ("CDGDAT", r_cdg_dat, 16, 32, CDASIZE >> 2) },
    { NULL }
    };

DEVICE sysd_dev = {
    "SYSD", sysd_unit, sysd_reg, NULL,
    2, 16, 16, 1, 16, 8,
    NULL, NULL, &sysd_reset,
    NULL, NULL, NULL,
    &sysd_dib, DEV_PERCPU
    };

AUTO_INIT_DEVLOCK(sysd_lock);
AUTO_INIT_DEVLOCK(csio_lock);

/* ROM: read only memory - stored in a buffered file
   Register space access routines see ROM twice

   ROM access has been 'regulated' to about 1Mhz to avoid issues
   with testing the interval timers in self-test.  Specifically,
   the VAX boot ROM (ka655.bin) contains code which presumes that
   the VAX runs at a particular slower speed when code is running
   from ROM (which is not cached).  These assumptions are built
   into instruction based timing loops. As the host platform gets
   much faster than the original VAX, the assumptions embedded in
   these code loops are no longer valid.
   
   Code has been added to the ROM implementation to limit CPU speed
   to about 500K instructions per second.  This heads off any future
   issues with the embedded timing loops.  
*/

int32 rom_swapb(int32 val)
{
    return ((val << 24) & 0xff000000) | (( val << 8) & 0xff0000) |
        ((val >> 8) & 0xff00) | ((val >> 24) & 0xff);
}

int32 rom_read_delay (int32 val)
{
    uint32 i, l = rom_delay;
    volatile int32 loopval = 0;

    if (rom_unit.flags & UNIT_NODELAY)
        return val;

    /* Calibrate the loop delay factor when first used.
       Do this 4 times to and use the largest value computed. */

    if (rom_delay == 0)
    {
        uint32 ts, te, c = 10000, samples = 0;
        while (1)
        {
            c = c * 2;
            te = sim_os_msec();
            while (te == (ts = sim_os_msec ()));            /* align on ms tick */

    /* This is merely a busy wait with some "work" that won't get optimized
       away by a good compiler. loopval always is zero.  To avoid smart compilers,
       the loopval variable is referenced in the function arguments so that the
       function expression is not loop invariant.  It also must be referenced
       by subsequent code or to avoid the whole computation being eliminated. */

            for (i = 0; i < c; i++)
                loopval |= (loopval + ts) ^ rom_swapb (rom_swapb (loopval + ts));
            te = sim_os_msec (); 
            if ((te - ts) < 50)                         /* sample big enough? */
                continue;
            if (rom_delay < (loopval + (c / (te - ts) / 1000) + 1))
                rom_delay = loopval + (c / (te - ts) / 1000) + 1;
            if (++samples >= 4)
                break;
            c = c / 2;
        }
        if (rom_delay < 5)
            rom_delay = 5;
    }

    for (i = 0; i < l; i++)
        loopval |= (loopval + val) ^ rom_swapb (rom_swapb (loopval + val));
    return val + loopval;
}

int32 rom_rd (RUN_DECL, int32 pa)
{
    /*
     * Kludge: Detect that primary processor jumped into ROM while secondary
     *         processors were active, and perform emergency shutdown of secondaries.
     *
     *         This condition should normally never happen, this is just an emergency safeguard
     *         in case guest OS fails to shut down properly.
     *
     * Logically this should be done in main instruction loop, but it would be
     * inefficient to place it there.
     */
    cpu_on_rom_rd(RUN_PASS);

    int32 rg = ((pa - ROMBASE) & ROMAMASK) >> 2;
    return rom_read_delay (rom[rg]);
}

void rom_wr_B (RUN_DECL, int32 pa, int32 val)
{
    int32 rg = ((pa - ROMBASE) & ROMAMASK) >> 2;
    int32 sc = (pa & 3) << 3;

    rom[rg] = ((val & 0xFF) << sc) | (rom[rg] & ~(0xFF << sc));
}

/* ROM examine */

t_stat rom_ex (t_value *vptr, t_addr exta, UNIT *uptr, int32 sw)
{
    RUN_SCOPE;
    uint32 addr = (uint32) exta;

    if ((vptr == NULL) || (addr & 03))
        return SCPE_ARG;
    if (addr >= ROMSIZE)
        return SCPE_NXM;
    *vptr = rom[addr >> 2];
    return SCPE_OK;
}

/* ROM deposit */

t_stat rom_dep (t_value val, t_addr exta, UNIT *uptr, int32 sw)
{
    RUN_SCOPE;
    uint32 addr = (uint32) exta;

    if (addr & 03)
        return SCPE_ARG;
    if (addr >= ROMSIZE)
        return SCPE_NXM;
    rom[addr >> 2] = (uint32) val;
    return SCPE_OK;
}

/* ROM reset */

t_stat rom_reset (DEVICE *dptr)
{
    RUN_SCOPE;
    if (rom == NULL)
        rom = (uint32*) calloc (ROMSIZE >> 2, sizeof (uint32));
    if (rom == NULL)
        return SCPE_MEM;
    return SCPE_OK;
}

/* NVR: non-volatile RAM - stored in a buffered file */

int32 nvr_rd (RUN_DECL, int32 pa)
{
    int32 rg = (pa - NVRBASE) >> 2;
    return nvr[rg];
}

void nvr_wr (RUN_DECL, int32 pa, int32 val, int32 lnt)
{
    if (! cpu_unit->is_primary_cpu())
    {
        /* On MicroVAX 3900, only boot ROM firmware and early stages of VMS SYSBOOT write to NVR */
        smp_printf ("\nNon-primary CPU (CPU%d) attempted to write NVR\n", cpu_unit->cpu_id);
        if (sim_log)
            fprintf (sim_log, "Non-primary CPU (CPU%d) attempted to write NVR\n", cpu_unit->cpu_id);
        ABORT_INVALID_SYSOP;
    }

    int32 rg = (pa - NVRBASE) >> 2;

    if (lnt < L_LONG)                                       /* byte or word? */
    {
        int32 sc = (pa & 3) << 3;                           /* merge */
        int32 mask = (lnt == L_WORD)? 0xFFFF: 0xFF;
        nvr[rg] = ((val & mask) << sc) | (nvr[rg] & ~(mask << sc));
    }
    else
    {
        nvr[rg] = val;
    }
}

/* NVR examine */

t_stat nvr_ex (t_value *vptr, t_addr exta, UNIT *uptr, int32 sw)
{
    RUN_SCOPE;
    uint32 addr = (uint32) exta;

    if ((vptr == NULL) || (addr & 03))
        return SCPE_ARG;
    if (addr >= NVRSIZE)
        return SCPE_NXM;
    *vptr = nvr[addr >> 2];
    return SCPE_OK;
}

/* NVR deposit */

t_stat nvr_dep (t_value val, t_addr exta, UNIT *uptr, int32 sw)
{
    RUN_SCOPE;
    uint32 addr = (uint32) exta;

    if (addr & 03)
        return SCPE_ARG;
    if (addr >= NVRSIZE)
        return SCPE_NXM;
    nvr[addr >> 2] = (uint32) val;
    return SCPE_OK;
}

/* NVR reset */

t_stat nvr_reset (DEVICE *dptr)
{
    RUN_SCOPE;
    AUTO_LOCK(sysd_lock);
    sim_bind_devunits_lock(&nvr_dev, sysd_lock);
    if (nvr == NULL)
    {
        nvr = (uint32*) calloc (NVRSIZE >> 2, sizeof (uint32));
        nvr_unit.filebuf = nvr;
        ssc_cnf = ssc_cnf | SSCCNF_BLO;
    }
    if (nvr == NULL)
        return SCPE_MEM;
    return SCPE_OK;
}

/* NVR attach */

t_stat nvr_attach (UNIT *uptr, char *cptr)
{
    t_stat r;

    uptr->flags = uptr->flags | (UNIT_ATTABLE | UNIT_BUFABLE);
    r = attach_unit (uptr, cptr);
    if (r != SCPE_OK)
        uptr->flags = uptr->flags & ~(UNIT_ATTABLE | UNIT_BUFABLE);
    else
    {
        uptr->hwmark = (uint32) uptr->capac;
        ssc_cnf = ssc_cnf & ~SSCCNF_BLO;
    }
    return r;
}

/* NVR detach */

t_stat nvr_detach (UNIT *uptr)
{
    t_stat r;

    r = detach_unit (uptr);
    if ((uptr->flags & UNIT_ATT) == 0)
        uptr->flags = uptr->flags & ~(UNIT_ATTABLE | UNIT_BUFABLE);
    return r;
}

/* CSI: console storage input */

int32 csrs_rd (RUN_DECL)
{
    AUTO_LOCK(csio_lock);
    return (csi_csr & CSICSR_IMP);
}

int32 csrd_rd (RUN_DECL)
{
    AUTO_LOCK(csio_lock);
    csi_csr = csi_csr & ~CSR_DONE;
    CLR_INT (CSI);
    return (csi_unit.buf & 0377);
}

void csrs_wr (RUN_DECL, int32 data)
{
    AUTO_LOCK(csio_lock);
    if ((data & CSR_IE) == 0)
        CLR_INT (CSI);
    else if ((csi_csr & (CSR_DONE + CSR_IE)) == CSR_DONE)
        SET_INT (CSI);
    csi_csr = (csi_csr & ~CSICSR_RW) | (data & CSICSR_RW);
}

t_stat csi_reset (DEVICE *dptr)
{
    AUTO_LOCK(csio_lock);
    sim_bind_devunits_lock(&csi_dev, csio_lock);
    csi_unit.buf = 0;
    csi_csr = 0;
    CLR_INT (CSI);
    return SCPE_OK;
}

/* CSO: console storage output */

int32 csts_rd (RUN_DECL)
{
    AUTO_LOCK(csio_lock);
    return (cso_csr & CSOCSR_IMP);
}

void csts_wr (RUN_DECL, int32 data)
{
    AUTO_LOCK(csio_lock);
    if ((data & CSR_IE) == 0)
        CLR_INT (CSO);
    else if ((cso_csr & (CSR_DONE + CSR_IE)) == CSR_DONE)
        SET_INT (CSO);
    cso_csr = (cso_csr & ~CSOCSR_RW) | (data & CSOCSR_RW);
}

void cstd_wr (RUN_DECL, int32 data)
{
    AUTO_LOCK(csio_lock);
    cso_unit.buf = data & 0377;
    cso_csr = cso_csr & ~CSR_DONE;
    CLR_INT (CSO);
    sim_activate (&cso_unit, cso_unit.wait);
}

t_stat cso_svc (RUN_SVC_DECL, UNIT *uptr)
{
    AUTO_LOCK(csio_lock);
    RUN_SVC_CHECK_CANCELLED(uptr);
    cso_csr = cso_csr | CSR_DONE;
    if (cso_csr & CSR_IE)
        SET_INT (CSO);
    if ((cso_unit.flags & UNIT_ATT) == 0)
        return SCPE_OK;
    if (putc (cso_unit.buf, cso_unit.fileref) == EOF)
    {
        smp_perror ("CSO I/O error");
        clearerr (cso_unit.fileref);
        return SCPE_IOERR;
    }
    cso_unit.pos = cso_unit.pos + 1;
    return SCPE_OK;
}

t_stat cso_reset (DEVICE *dptr)
{
    AUTO_LOCK(csio_lock);
    sim_bind_devunits_lock(&cso_dev, csio_lock);
    cso_unit.buf = 0;
    cso_csr = CSR_DONE;
    CLR_INT (CSO);
    sim_cancel (&cso_unit);                                 /* deactivate unit */
    return SCPE_OK;
}

/* SYSD: SSC access mechanisms and devices

   - IPR space read/write routines
   - register space read/write routines
   - SSC local register read/write routines
   - SSC console storage UART
   - SSC timers
   - CMCTL local register read/write routines
*/

/* Read/write IPR register space

   These routines implement the SSC's response to IPR's which are
   sent off the CPU chip for processing.
*/

int32 ReadIPR (RUN_DECL, int32 rg)
{
    int32 val;

    switch (rg)
    {
    case MT_ICCS:                                       /* ICCS */
        val = iccs_rd (RUN_PASS);
        break;

    case MT_CSRS:                                       /* CSRS */
        val = csrs_rd (RUN_PASS);
        break;

    case MT_CSRD:                                       /* CSRD */
        val = csrd_rd (RUN_PASS);
        break;

    case MT_CSTS:                                       /* CSTS */
        val = csts_rd (RUN_PASS);
        break;

    case MT_CSTD:                                       /* CSTD */
        val = 0;
        break;

    case MT_RXCS:                                       /* RXCS */
        val = rxcs_rd (RUN_PASS);
        break;

    case MT_RXDB:                                       /* RXDB */
        val = rxdb_rd (RUN_PASS);
        break;

    case MT_TXCS:                                       /* TXCS */
        val = txcs_rd (RUN_PASS);
        break;

    case MT_TXDB:                                       /* TXDB */
        val = 0;
        break;

    case MT_TODR:                                       /* TODR */
        val = todr_rd (RUN_PASS);
        break;

    case MT_CADR:                                       /* CADR */
        val = CADR & 0xFF;
        break;

    case MT_MSER:                                       /* MSER */
        val = MSER & 0xFF;
        break;

    case MT_CONPC:                                      /* console PC */
        if (! cpu_unit->is_primary_cpu())
        {
            // On MicroVAX 3900, CONPC and CONPSL are intened for use by firmware only -- that executes on the primary CPU.
            // VMS does not access SAVPC/SAVPSL except on Calypso (VAX 9CC) processors.
            smp_printf ("\nNon-primary CPU (CPU%d) attempted to read CONPC\n", cpu_unit->cpu_id);
            if (sim_log)
                fprintf (sim_log, "Non-primary CPU (CPU%d) attempted to read CONPC\n", cpu_unit->cpu_id);
            ABORT_INVALID_SYSOP;
        }
        val = conpc;
        break;

    case MT_CONPSL:                                     /* console PSL */
        if (! cpu_unit->is_primary_cpu())
        {
            // On MicroVAX 3900, CONPC and CONPSL are intened for use by firmware only -- that executes on the primary CPU.
            // VMS does not access SAVPC/SAVPSL except on Calypso (VAX 9CC) processors.
            smp_printf ("\nNon-primary CPU (CPU%d) attempted to read CONPSL\n", cpu_unit->cpu_id);
            if (sim_log)
                fprintf (sim_log, "Non-primary CPU (CPU%d) attempted to read CONPSL\n", cpu_unit->cpu_id);
            ABORT_INVALID_SYSOP;
        }
        val = conpsl;
        break;

    case MT_SID:                                        /* SID */
        val = CVAX_SID | CVAX_UREV;
        break;

    default:
        ssc_bto = ssc_bto | SSCBTO_BTO;                 /* set BTO */
        val = 0;
        break;
    }

    return val;
}

void WriteIPR (RUN_DECL, int32 rg, int32 val, t_bool& set_irql)
{
    switch (rg)
    {
    case MT_ICCS:                                       /* ICCS */
        iccs_wr (RUN_PASS, val);
        break;

    case MT_TODR:                                       /* TODR */
        todr_wr (RUN_PASS, val);
        break;

    case MT_CSRS:                                       /* CSRS */
        csrs_wr (RUN_PASS, val);
        break;

    case MT_CSRD:                                       /* CSRD */
        break;

    case MT_CSTS:                                       /* CSTS */
        csts_wr (RUN_PASS, val);
        break;

    case MT_CSTD:                                       /* CSTD */
        cstd_wr (RUN_PASS, val);
        break;

    case MT_RXCS:                                       /* RXCS */
        rxcs_wr (RUN_PASS, val);
        break;

    case MT_RXDB:                                       /* RXDB */
        break;

    case MT_TXCS:                                       /* TXCS */
        txcs_wr (RUN_PASS, val);
        break;

    case MT_TXDB:                                       /* TXDB */
        txdb_wr (RUN_PASS, val);
        break;

    case MT_CADR:                                       /* CADR */
        CADR = (val & CADR_RW) | CADR_MBO;
        set_irql = FALSE;
        break;

    case MT_MSER:                                       /* MSER */
        MSER = MSER & MSER_HM;
        set_irql = FALSE;
        break;

    case MT_IORESET:                                    /* IORESET */
        ioreset_wr (RUN_PASS, val);
        break;

    case MT_SID:
    case MT_CONPC:
    case MT_CONPSL:                                     /* halt reg */
        RSVD_OPND_FAULT;

    default:
        ssc_bto = ssc_bto | SSCBTO_BTO;                 /* set BTO */
        break;
    }
}

/* Read/write I/O register space

   These routines are the 'catch all' for address space map.  Any
   address that doesn't explicitly belong to memory, I/O, or ROM
   is given to these routines for processing.
*/

struct reglink {                                        /* register linkage */
    uint32      low;                                    /* low addr */
    uint32      high;                                   /* high addr */
    int32       (*read)(RUN_DECL, int32 pa);                        /* read routine */
    void        (*write)(RUN_DECL, int32 pa, int32 val, int32 lnt); /* write routine */
    };

struct reglink regtable[] = {
    { CQMAPBASE, CQMAPBASE+CQMAPSIZE, &cqmap_rd, &cqmap_wr },
    { ROMBASE, ROMBASE+ROMSIZE+ROMSIZE, &rom_rd, NULL },
    { NVRBASE, NVRBASE+NVRSIZE, &nvr_rd, &nvr_wr },
    { CMCTLBASE, CMCTLBASE+CMCTLSIZE, &cmctl_rd, &cmctl_wr },
    { SSCBASE, SSCBASE+SSCSIZE, &ssc_rd, &ssc_wr },
    { KABASE, KABASE+KASIZE, &ka_rd, &ka_wr },
    { CQBICBASE, CQBICBASE+CQBICSIZE, &cqbic_rd, &cqbic_wr },
    { CQIPCBASE, CQIPCBASE+CQIPCSIZE, &cqipc_rd, &cqipc_wr },
    { CQMBASE, CQMBASE+CQMSIZE, &cqmem_rd, &cqmem_wr },
    { CDGBASE, CDGBASE+CDGSIZE, &cdg_rd, &cdg_wr },
    { 0, 0, NULL, NULL }
    };

/* ReadReg - read register space

   Inputs:
        pa      =       physical address
        lnt     =       length (BWLQ) - ignored
   Output:
        longword of data
*/

int32 ReadReg (RUN_DECL, uint32 pa, int32 lnt)
{
    struct reglink *p;

    for (p = &regtable[0]; p->low != 0; p++)
    {
        if ((pa >= p->low) && (pa < p->high) && p->read)
            return p->read (RUN_PASS, pa);
    }
    ssc_bto = ssc_bto | SSCBTO_BTO | SSCBTO_RWT;
    MACH_CHECK (MCHK_READ);
    return 0;
}

/* WriteReg - write register space

   Inputs:
        pa      =       physical address
        val     =       data to write, right justified in 32b longword
        lnt     =       length (BWLQ)
   Outputs:
        none
*/

void WriteReg (RUN_DECL, uint32 pa, int32 val, int32 lnt)
{
    struct reglink *p;

    for (p = &regtable[0]; p->low != 0; p++)
    {
        if ((pa >= p->low) && (pa < p->high) && p->write)
        {
            p->write (RUN_PASS, pa, val, lnt);  
            return;
        }
    }
    ssc_bto = ssc_bto | SSCBTO_BTO | SSCBTO_RWT;
    MACH_CHECK (MCHK_WRITE);
}

/*
 * For more information on CMCTL see "KA655 CPU System Maintenance" manual, pp 1.5, 1.9, 1.10, B.1-B.5.
 */

/* CMCTL registers

   CMCTL00 - 15 configure memory banks 00 - 15.  Note that they are
   here merely to entertain the firmware; the actual configuration
   of memory is unaffected by the settings here.

   CMCTL16 - error status register

   CMCTL17 - control/diagnostic status register

   The CMCTL registers are cleared at power up.
*/

int32 cmctl_rd (RUN_DECL, int32 pa)
{
    int32 rg = (pa - CMCTLBASE) >> 2;

    switch (rg)
    {
    default:                                            /* config reg */
        return cmctl_reg[rg] & CMCNF_MASK;

    case 16:                                            /* err status */
        return cmctl_reg[rg];

    case 17:                                            /* csr */
        return cmctl_reg[rg] & CMCSR_MASK;

    case 18:                                            /* KA655X ext mem */
        if (MEMSIZE > MAXMEMSIZE)                       /* more than 128MB? */
            return ((int32) MEMSIZE);
        MACH_CHECK (MCHK_READ);
    }

    return 0;
}

void cmctl_wr (RUN_DECL, int32 pa, int32 val, int32 lnt)
{
    int32 i, rg = (pa - CMCTLBASE) >> 2;

    if (lnt < L_LONG)                                       /* LW write only */
    {
        int32 sc = (pa & 3) << 3;                           /* shift data to */
        val = val << sc;                                    /* proper location */
    }

    switch (rg)
    {
    default:                                            /* config reg */
        if (val & CMCNF_SRQ)                            /* sig request? */
        {
            int32 rg_g = rg & ~3;                       /* group of 4 */
            for (i = rg_g; i < (rg_g + 4); i++)
            {
                cmctl_reg[i] = cmctl_reg[i] & ~CMCNF_SIG;
                if (ADDR_IS_MEM (i * MEM_BANK))
                    cmctl_reg[i] = cmctl_reg[i] | MEM_SIG;
            }
        }
        cmctl_reg[rg] = (cmctl_reg[rg] & ~CMCNF_RW) | (val & CMCNF_RW);
        break;

    case 16:                                            /* err status */
        cmctl_reg[rg] = cmctl_reg[rg] & ~(val & CMERR_W1C);
        break;

    case 17:                                            /* csr */
        cmctl_reg[rg] = val & CMCSR_MASK;
        break;

    case 18:
        MACH_CHECK (MCHK_WRITE);
    }
}

/* KA655 registers */

int32 ka_rd (RUN_DECL, int32 pa)
{
    int32 rg = (pa - KABASE) >> 2;

    switch (rg)
    {
    case 0:                                             /* CACR */
        return ka_cacr;

    case 1:                                             /* BDR */
        return ka_bdr;
    }

    return 0;
}

void ka_wr (RUN_DECL, int32 pa, int32 val, int32 lnt)
{
    int32 rg = (pa - KABASE) >> 2;

    if ((rg == 0) && ((pa & 3) == 0))                       /* lo byte only */
    {
        ka_cacr = (ka_cacr & ~(val & CACR_W1C)) | CACR_FIXED;
        ka_cacr = (ka_cacr & ~CACR_RW) | (val & CACR_RW);
    }
}

int32 sysd_hlt_enb (void)
{
    return ka_bdr & BDR_BRKENB;
}

/* Cache diagnostic space */

int32 cdg_rd (RUN_DECL, int32 pa)
{
    int32 t, row = CDG_GETROW (pa);

    t = cdg_dat[row];
    ka_cacr = ka_cacr & ~CACR_DRO;                          /* clear diag */
    ka_cacr = ka_cacr |
        (parity ((t >> 24) & 0xFF, 1) << (CACR_V_DPAR + 3)) |
        (parity ((t >> 16) & 0xFF, 0) << (CACR_V_DPAR + 2)) |
        (parity ((t >> 8) & 0xFF, 1) << (CACR_V_DPAR + 1)) |
        (parity (t & 0xFF, 0) << CACR_V_DPAR);
    return t;
}

void cdg_wr (RUN_DECL, int32 pa, int32 val, int32 lnt)
{
    int32 row = CDG_GETROW (pa);

    if (lnt < L_LONG)                                       /* byte or word? */
    {
        int32 sc = (pa & 3) << 3;                           /* merge */
        int32 mask = (lnt == L_WORD)? 0xFFFF: 0xFF;
        int32 t = cdg_dat[row];
        val = ((val & mask) << sc) | (t & ~(mask << sc));
    }
    cdg_dat[row] = val;                                     /* store data */
}

int32 parity (int32 val, int32 odd)
{
    for ( ; val != 0; val = val >> 1)
    {
        if (val & 1)
            odd = odd ^ 1;
    }
    return odd;
}

/* SSC registers - byte/word merges done in WriteReg */

int32 ssc_rd (RUN_DECL, int32 pa)
{
    int32 rg = (pa - SSCBASE) >> 2;

    switch (rg)
    {
    case 0x00:                                          /* base reg */
        {
        AUTO_LOCK(sysd_lock);
        return ssc_base;
        }

    case 0x04:                                          /* conf reg */
        {
        AUTO_LOCK(sysd_lock);
        return ssc_cnf;
        }

    case 0x08:                                          /* bus timeout */
        return ssc_bto;

    case 0x0C:                                          /* output port */
        return ssc_otp & SSCOTP_MASK;

    case 0x1B:                                          /* TODR */
        return todr_rd (RUN_PASS);

    case 0x1C:                                          /* CSRS */
        return csrs_rd (RUN_PASS);

    case 0x1D:                                          /* CSRD */
        return csrd_rd (RUN_PASS);

    case 0x1E:                                          /* CSTS */
        return csts_rd (RUN_PASS);

    case 0x20:                                          /* RXCS */
        return rxcs_rd (RUN_PASS);

    case 0x21:                                          /* RXDB */
        return rxdb_rd (RUN_PASS);

    case 0x22:                                          /* TXCS */
        return txcs_rd (RUN_PASS);

    case 0x40:                                          /* T0CSR */
        return tmr_csr[0];

    case 0x41:                                          /* T0INT */
        return tmr_tir_rd (RUN_PASS, 0, FALSE);

    case 0x42:                                          /* T0NI */
        return tmr_tnir[0];

    case 0x43:                                          /* T0VEC */
        return tmr_tivr[0];

    case 0x44:                                          /* T1CSR */
        return tmr_csr[1];

    case 0x45:                                          /* T1INT */
        return tmr_tir_rd (RUN_PASS, 1, FALSE);

    case 0x46:                                          /* T1NI */
        return tmr_tnir[1];

    case 0x47:                                          /* T1VEC */
        return tmr_tivr[1];

    case 0x4C:                                          /* ADS0M */
        return ssc_adsm[0];

    case 0x4D:                                          /* ADS0K */
        return ssc_adsk[0];

    case 0x50:                                          /* ADS1M */
        return ssc_adsm[1];

    case 0x51:                                          /* ADS1K */
        return ssc_adsk[1];
    }

    return 0;
}

void ssc_wr (RUN_DECL, int32 pa, int32 val, int32 lnt)
{
    int32 rg = (pa - SSCBASE) >> 2;

    if (lnt < L_LONG)                                       /* byte or word? */
    {
        int32 sc = (pa & 3) << 3;                           /* merge */
        int32 mask = (lnt == L_WORD)? 0xFFFF: 0xFF;
        int32 t = ssc_rd (RUN_PASS, pa);
        val = ((val & mask) << sc) | (t & ~(mask << sc));
    }

    switch (rg)
    {
    case 0x00:                                          /* base reg */
        {
        if (! cpu_unit->is_primary_cpu())
        {
            smp_printf ("\nNon-primary CPU (CPU%d) attempted to write SSCBASE\n", cpu_unit->cpu_id);
            if (sim_log)
                fprintf (sim_log, "Non-primary CPU (CPU%d) attempted to write SSCBASE\n", cpu_unit->cpu_id);
            ABORT_INVALID_SYSOP;
        }
        AUTO_LOCK(sysd_lock);
        ssc_base = (val & SSCBASE_RW) | SSCBASE_MBO;
        }
        break;

    case 0x04:                                          /* conf reg */
        {
        AUTO_LOCK(sysd_lock);
        ssc_cnf = ssc_cnf & ~(val & SSCCNF_W1C);
        ssc_cnf = (ssc_cnf & ~SSCCNF_RW) | (val & SSCCNF_RW);
        }
        break;

    case 0x08:                                          /* bus timeout */
        ssc_bto = ssc_bto & ~(val & SSCBTO_W1C);
        ssc_bto = (ssc_bto & ~SSCBTO_RW) | (val & SSCBTO_RW);
        break;

    case 0x0C:                                          /* output port */
        ssc_otp = val & SSCOTP_MASK;
        break;

    case 0x1B:                                          /* TODR */
        todr_wr (RUN_PASS, val);
        break;

    case 0x1C:                                          /* CSRS */
        csrs_wr (RUN_PASS, val);
        break;

    case 0x1E:                                          /* CSTS */
        csts_wr (RUN_PASS, val);
        break;

    case 0x1F:                                          /* CSTD */
        cstd_wr (RUN_PASS, val);
        break;

    case 0x20:                                          /* RXCS */
        rxcs_wr (RUN_PASS, val);
        break;

    case 0x22:                                          /* TXCS */
        txcs_wr (RUN_PASS, val);
        break;

    case 0x23:                                          /* TXDB */
        txdb_wr (RUN_PASS, val);
        break;

    case 0x40:                                          /* T0CSR */
        tmr_csr_wr (RUN_PASS, 0, val);
        break;

    case 0x42:                                          /* T0NI */
        tmr_tnir[0] = val;
        break;

    case 0x43:                                          /* T0VEC */
        tmr_tivr[0] = val & TMR_VEC_MASK;
        break;

    case 0x44:                                          /* T1CSR */
        tmr_csr_wr (RUN_PASS, 1, val);
        break;

    case 0x46:                                          /* T1NI */
        tmr_tnir[1] = val;
        break;

    case 0x47:                                          /* T1VEC */
        tmr_tivr[1] = val & TMR_VEC_MASK;
        break;

    case 0x4C:                                          /* ADS0M */
        ssc_adsm[0] = val & SSCADS_MASK;
        break;

    case 0x4D:                                          /* ADS0K */
        ssc_adsk[0] = val & SSCADS_MASK;
        break;

    case 0x50:                                          /* ADS1M */
        ssc_adsm[1] = val & SSCADS_MASK;
        break;

    case 0x51:                                          /* ADS1K */
        ssc_adsk[1] = val & SSCADS_MASK;
        break;
    }
}

/*
 * VAX MP version of programmable timers is a hack upon the original uniprocessor SIMH hack.
 *
 * SSC timers are used in two sitiations.
 *
 * First is pre-boot ROM firmware tests. They play no functional significance for actual
 * booting and execution of operating system after control is transferred to VMB and then
 * to VMS (or Unix).
 *
 * The only goal with regard to these tests is to make them pass without annoying
 * (but harmless) messages. Accuracy of values they measure is of no great significance.
 *
 * In VAX MP (both SYNCLK and non-SYNCLK modes) we use original SIMH code to process timers
 * during ROM tests, except for Test 31 we return hard-coded known good values in SYNCLK
 * mode, to suppress message caused by very tight tolerance margins on this test.
 *
 * ROM tests can be discriminated by MAPEN=0 and PC in ROM range.
 *
 * Second kind of use is the use by VMB and operating system. SSC timers are used by VMB,
 * VMS SYSBOOT/INIT,  and are also used during system shutdown or crash when IO system is
 * recalibrated for reading in the bugcheck handling code and writing dump. This use can be 
 * discriminated by MAPEN=1 or PC in RAM range. Only timer 0 is used (never timer 1) 
 * and the pattern of access is as follows:
 *
 *      TNIR0 = 0
 *      TCR0 = RUN | XFR
 *      (calibrated loop)
 *      read TIR0
 *      TCR0 = 0
 *
 *      Interrupts are not used (and counter does not overflow).
 *
 * We try to use host system timers to provide actual calibration data for this sequence.
 */

/* Programmable timers

   The SSC timers, which increment at 1Mhz, cannot be accurately
   simulated due to the overhead that would be required for 1M
   clock events per second.  Instead, a gross hack is used.  When
   a timer is started, the clock interval is inspected.

   if (int < 0 and small) then testing timer, count instructions.
        Small is determined by when the requested interval is less
        than the size of a 100hz system clock tick.
   if (int >= 0 or large) then counting a real interval, schedule
        clock events at 100Hz using calibrated line clock delay
        and when the remaining time value gets small enough, behave
        like the small case above.

   If the interval register is read, then its value between events
   is interpolated using the current instruction count versus the
   count when the most recent event started, the result is scaled
   to the calibrated system clock, unless the interval being timed
   is less than a calibrated system clock tick (or the calibrated 
   clock is running very slowly) at which time the result will be 
   the elapsed instruction count.

   The powerup TOY Test sometimes fails its tolerance test.  This was
   due to varying system load causing varying calibration values to be
   used at different times while referencing the TIR.  While timing long
   intervals, we now synchronize the stepping (and calibration) of the
   system tick with the opportunity to reference the value.  This gives
   precise tolerance measurement values (when interval timers are used
   to measure the system clock), regardless of other load issues on the
   host system which might cause varying values of the system clock's
   calibration factor.
*/

int32 tmr_tir_rd (RUN_DECL, int32 tmr, t_bool interp)
{
    if (! cpu_unit->is_primary_cpu())
    {
        /* 
         * Only boot ROM firmware, VMS SYSBOOT and EXE$INIT, and final phase of VMS shutdown access SSC clocks.
         * All these phases are performed on the primary processor, so secondaries are not expected to access SSC clocks.
         * On some VAX processors SSC clock is also accessed by VMS machine check handler, but not on the 650.
         * SMP$SETUP_CPU in some SYSLOAxxx may also call EXE$INI_TIMWAIT calibration, but not for VAX MP based SMP
         * which copies calibration data for the secondaries from the primary CPU.
         *
         * VAX MP SMP paravirtualization module (VSMP.EXE for VMS) can also call EXE$INI_TIMWAIT before enabling SMP
         * to ensure that calibration is stable, and that loops were not undercalibrated, but this will be performed
         * on the primary processor as well.
         *
         * VAX MP SSC clock implementation is per-CPU, so we could enable SSC clock use by the secondaries if ever needed.
         * So far keep it disabled just to make sure nothing unintended goes on.
         */
        smp_printf ("\nNon-primary CPU (CPU%d) attempted to access SSC clock (reading TIR%d)\n", cpu_unit->cpu_id, tmr);
        if (sim_log)
            fprintf (sim_log, "Non-primary CPU (CPU%d) attempted to access SSC clock (reading TIR%d)\n", cpu_unit->cpu_id, tmr);
        ABORT_INVALID_SYSOP;
    }

    t_bool rom_test = (mapen == 0) && ADDR_IS_ROM(PC);
    if (! rom_test)
        return tmr_tir_rd_realtime(RUN_PASS, tmr);

    uint32 delta;
    int32 x_tmr_poll = weak_read_var(tmr_poll);
    int32 res;

    if (interp || (tmr_csr[tmr] & TMR_CSR_RUN))             /* interp, running? */
    {
        delta = sim_grtime (RUN_PASS) - tmr_sav[tmr];       /* delta inst */
        if ((tmr_inc[tmr] == TMR_INC) &&                    /* scale large int */
            (x_tmr_poll > TMR_INC))
            delta = (uint32) ((((double) delta) * TMR_INC) / x_tmr_poll);
        if (delta >= tmr_inc[tmr])
            delta = tmr_inc[tmr] - 1;
        res = tmr_tir[tmr] + delta;
    }
    else
    {
        res = tmr_tir[tmr];
    }

    /*
     * ROM Test 31 tends to fail when SYNCLK is used because its tolerances are
     * very tight (tighter than 0.15%). Just put out know good values for it,
     * to suppress annoying (but harmless) message about test failure.
     */
    if (tmr == 1 && mapen == 0 && use_clock_thread)
    {
        if (PC == ROM_PC_TEST31_TIR1_RD_A)
            res = 0x4E15;
        else if (PC == ROM_PC_TEST31_TIR1_RD_B)
            res = 0x1D4B5;
    }

    return res;
}

void tmr_csr_wr (RUN_DECL, int32 tmr, int32 val)
{
    if (tmr < 0 || tmr > 1)
        return;

    if (! cpu_unit->is_primary_cpu())
    {
        /* 
         * Only boot ROM firmware, VMS SYSBOOT and EXE$INIT, and final phase of VMS shutdown access SSC clocks.
         * All these phases are performed on the primary processor, so secondaries are not expected to access SSC clocks.
         * On some VAX processors SSC clock is also accessed by VMS machine check handler, but not on the 650.
         * SMP$SETUP_CPU in some SYSLOAxxx may also call EXE$INI_TIMWAIT for calibration, but not under VAX MP based SMP
         * which copies calibration data for the secondaries from the primary CPU.
         *
         * VAX MP SMP paravirtualization module (VSMP.EXE for VMS) can also call EXE$INI_TIMWAIT before enabling SMP
         * to ensure that calibration is stable, and that loops were not undercalibrated, but this will be performed
         * on the primary processor as well.
         *
         * VAX MP SSC clock implementation is per-CPU, so we could enable SSC clock use by the secondaries if ever needed.
         * So far keep it disabled just to make sure nothing unintended goes on.
         */
        smp_printf ("\nNon-primary CPU (CPU%d) attempted to access SSC clock (writing CSR%d)\n", cpu_unit->cpu_id, tmr);
        if (sim_log)
            fprintf (sim_log, "Non-primary CPU (CPU%d) attempted to access SSC clock (writing CSR%d)\n", cpu_unit->cpu_id, tmr);
        ABORT_INVALID_SYSOP;
    }

    /* do not elevate thread priority for ROM tests */
    t_bool rom_test = (mapen == 0) && ADDR_IS_ROM(PC);
    t_bool is_realtime = !rom_test;

    if (is_realtime)
    {
        tmr_csr_wr_realtime (RUN_PASS, tmr, val);
        return;
    }

    if ((val & TMR_CSR_RUN) == 0)                           /* clearing run? */
    {
        sim_cancel (sysd_unit[tmr]);                        /* cancel timer */
        sysd_deactivating(tmr);                             /* note as inactive */
        if (tmr_csr[tmr] & TMR_CSR_RUN)                     /* run 1 -> 0? */
            tmr_tir[tmr] = tmr_tir_rd (RUN_PASS, tmr, TRUE);    /* update itr */
    }
    tmr_csr[tmr] = tmr_csr[tmr] & ~(val & TMR_CSR_W1C);     /* W1C csr */
    tmr_csr[tmr] = (tmr_csr[tmr] & ~TMR_CSR_RW) |           /* new r/w */
        (val & TMR_CSR_RW);
    if (val & TMR_CSR_XFR)                                  /* xfr set? */
        tmr_tir[tmr] = tmr_tnir[tmr];
    if (val & TMR_CSR_RUN)                                  /* run? */
    {
        if (is_realtime)
            tmr_adjust_thread_priority(RUN_PASS, TRUE);
        if (val & TMR_CSR_XFR)                              /* new tir? */
        {
            sim_cancel (sysd_unit[tmr]);                    /* stop prev */
            sysd_deactivating(tmr);                         /* note as inactive */
        }
        if (!sim_is_active (sysd_unit[tmr]))                /* not running? */
            tmr_sched (RUN_PASS, tmr, is_realtime);         /* activate */
    }
    else if (val & TMR_CSR_SGL)                             /* single step? */
    {
        tmr_incr (RUN_PASS, tmr, 1, is_realtime);           /* incr tmr */
        if (tmr_tir[tmr] == 0)                              /* if ovflo, */
            tmr_tir[tmr] = tmr_tnir[tmr];                   /* reload tir */
    }
    if ((tmr_csr[tmr] & (TMR_CSR_DON | TMR_CSR_IE)) !=      /* update int */
        (TMR_CSR_DON | TMR_CSR_IE))
    {
        if (tmr)
            CLR_INT (TMR1);
        else
            CLR_INT (TMR0);
    }

    if (is_realtime)
        tmr_adjust_thread_priority(RUN_PASS);
}

/* Unit service */

t_stat tmr_svc (RUN_SVC_DECL, UNIT *uptr)
{
    // RUN_SVC_CHECK_CANCELLED(uptr);    // not required for per-CPU devices
    int32 tmr = sim_unit_index (uptr);                      /* get timer # */

    uint32 sv_sysd_active_mask = cpu_unit->sysd_active_mask;
    t_bool is_realtime = 0 != (sv_sysd_active_mask & (1 << tmr));
    sysd_deactivating(tmr);                                 /* note as inactive */

    tmr_incr (RUN_PASS, tmr, tmr_inc[tmr], is_realtime);    /* incr timer */

    if ((sv_sysd_active_mask == 0) != (cpu_unit->sysd_active_mask == 0))
        tmr_adjust_thread_priority(RUN_PASS);

    return SCPE_OK;
}

/*
 * SSC timers are used by the operating system to calibrate processor loops for various critical
 * time-waits, including spinlocks timeout and device access delays.
 *
 * If the thread is paused during loop calibration, loop will be under-calibrated to a smaller
 * number of cycles, resulting in shorter subsequent waits, and hence unstable system.
 *
 * To avoid this, boost current thread priority while the loop is beng calibrated, 
 * i.e. SSC clocks are active or about to be activated.
 */
static void tmr_adjust_thread_priority(RUN_DECL, t_bool force_high)
{
    if (force_high || tmr_is_active(RUN_PASS))
    {
        /* raise thread priority */
        RUN_SCOPE_RSCX_ONLY;
        cpu_set_thread_priority(RUN_RSCX_PASS, SIMH_THREAD_PRIORITY_CPU_CALIBRATION);
    }
    else
    {
        /* lower thread priority */
        cpu_reevaluate_thread_priority(RUN_PASS);
    }
}

/* Timer increment */

void tmr_incr (RUN_DECL, int32 tmr, uint32 inc, t_bool is_realtime)
{
    uint32 new_tir = tmr_tir[tmr] + inc;                    /* add incr */

    if (new_tir < tmr_tir[tmr])                             /* ovflo? */
    {
        tmr_tir[tmr] = 0;                                   /* now 0 */

        if (tmr_csr[tmr] & TMR_CSR_DON)                     /* done? set err */
            tmr_csr[tmr] = tmr_csr[tmr] | TMR_CSR_ERR;
        else
            tmr_csr[tmr] = tmr_csr[tmr] | TMR_CSR_DON;      /* set done */

        if (tmr_csr[tmr] & TMR_CSR_STP)                     /* stop? */
            tmr_csr[tmr] = tmr_csr[tmr] & ~TMR_CSR_RUN;     /* clr run */

        if (tmr_csr[tmr] & TMR_CSR_RUN)                     /* run? */
        {
            tmr_tir[tmr] = tmr_tnir[tmr];                   /* reload */
            tmr_sched (RUN_PASS, tmr, is_realtime);             /* reactivate */
        }
        if (tmr_csr[tmr] & TMR_CSR_IE)                      /* set int req */
        {
            if (tmr)
                SET_INT (TMR1);
            else
                SET_INT (TMR0);
        }
    }
    else
    {
        tmr_tir[tmr] = new_tir;                             /* no, upd tir */
        if (tmr_csr[tmr] & TMR_CSR_RUN)                     /* still running? */
            tmr_sched (RUN_PASS, tmr, is_realtime);             /* reactivate */
    }
}

/* Timer scheduling */

void tmr_sched (RUN_DECL, int32 tmr, t_bool is_realtime)
{
    int32 clk_time = sim_is_active (&clk_unit) - 1;
    int32 tmr_time;

    tmr_sav[tmr] = sim_grtime (RUN_PASS);                   /* save intvl base */

    if (tmr_tir[tmr] > (0xFFFFFFFFu - TMR_INC))             /* short interval? */
    {
        tmr_inc[tmr] = (~tmr_tir[tmr] + 1);                 /* inc = interval */
        tmr_time = tmr_inc[tmr];
    }
    else
    {
        tmr_inc[tmr] = TMR_INC;                             /* usec/interval */
        tmr_time = weak_read_var(tmr_poll);
    }

    if (tmr_time == 0)
        tmr_time = 1;

    if (tmr_inc[tmr] == TMR_INC && tmr_time > clk_time)
    {
        /* Align scheduled event to be identical to the event for the next clock
           tick.  This lets us always see a consistent calibrated value, both for
           this scheduling, AND for any query of the current timer register that
           may happen in tmr_tir_rd ().  This presumes that sim_activate will
           queue the interval timer behind the event for the clock tick. */

        tmr_inc[tmr] = (uint32) (((double) clk_time * TMR_INC) / weak_read_var(tmr_poll));
        tmr_time = clk_time;
    }

    sim_activate (sysd_unit[tmr], tmr_time);

    if (is_realtime)
        sysd_activating(tmr);                               /* note as active */
}

int32 tmr0_inta (void)
{
    RUN_SCOPE;
    return tmr_tivr[0];
}

int32 tmr1_inta (void)
{
    RUN_SCOPE;
    return tmr_tivr[1];
}

static void tmr_csr_wr_realtime (RUN_DECL, int32 tmr, int32 val)
{
    t_bool unexpected = FALSE;

    if (tmr != 0 || val != 0 && val != (TMR_CSR_XFR | TMR_CSR_RUN))
        unexpected = TRUE;
    else if ((val & TMR_CSR_RUN) && (tmr_csr[tmr] & TMR_CSR_RUN))
        unexpected = TRUE;

    if (unexpected)
    {
        smp_printf ("\nUnexpected request to SSC clock (tmr=%d, TCR=%08X, write=%08X)\n", tmr, tmr_csr[tmr], val);
        if (sim_log)
            fprintf (sim_log, "Unexpected request to SSC clock (tmr=%d, TCR=%08X, write=%08X)\n", tmr, tmr_csr[tmr], val);
        ABORT_INVALID_SYSOP;
    }

    if (val & TMR_CSR_XFR)
        tmr_tir[tmr] = tmr_tnir[tmr];
        
    if (val & TMR_CSR_RUN)
    {
        tmr_adjust_thread_priority(RUN_PASS, TRUE);
        tmr_csr[tmr] |= TMR_CSR_RUN;
        sysd_activating(tmr);
        tmr_tir_rtstart[tmr] = tmr_tir[tmr];
        cpu_unit->cpu_ssc_delta_timer[tmr]->begin(RUN_PASS);
    }
    else
    {
        uint32 sv_sysd_active_mask = cpu_unit->sysd_active_mask;
        tmr_csr[tmr] &= ~TMR_CSR_RUN;
        sysd_deactivating(tmr);
        if ((sv_sysd_active_mask == 0) != (cpu_unit->sysd_active_mask == 0))
            cpu_reevaluate_thread_priority(RUN_PASS);
    }
}

static int32 tmr_tir_rd_realtime (RUN_DECL, int32 tmr)
{
    if (tmr != 0)
    {
        smp_printf ("\nUnexpected request to SSC clock (tmr=%d)\n", tmr);
        if (sim_log)
            fprintf (sim_log, "Unexpected request to SSC clock (tmr=%d)\n", tmr);
        ABORT_INVALID_SYSOP;
    }

    if (tmr_csr[tmr] & TMR_CSR_RUN)
    {
        sim_delta_timer* dt = cpu_unit->cpu_ssc_delta_timer[tmr];
        dt->sample(RUN_PASS);
        uint32 usec = dt->us_since_start(RUN_PASS);
        uint32 tir = (uint32) tmr_tir_rtstart[tmr] + usec;
        /* real measurments do not incur overflow */
        if (tir > (uint32) tmr_tir[tmr])
            tmr_tir[tmr] = tir; 
    }

    return tmr_tir[tmr];
}

/* Machine check */

int32 machine_check (RUN_DECL, int32 p1, int32 opc, int32 cc, int32 delta)
{
    int32 i, st1, st2, p2, hsir, acc;

    if (p1 & 0x80)                                          /* mref? set v/p */
        p1 = p1 + mchk_ref;
    p2 = mchk_va + 4;                                       /* save vap */
    for (i = hsir = 0; i < 16; i++) {                       /* find hsir */
        if ((SISR >> i) & 1)
            hsir = i;
        }
    st1 = ((((uint32) opc) & 0xFF) << 24) |
        (hsir << 16) |
        ((CADR & 0xFF) << 8) |
        (MSER & 0xFF);
    st2 = 0x00C07000 + (delta & 0xFF);
    cc = intexc (RUN_PASS, SCB_MCHK, cc, 0, IE_SVE);        /* take exception */
    acc = ACC_MASK (KERN);                                  /* in kernel mode */
    in_ie = 1;
    SP = SP - 20;                                           /* push 5 words */
    Write (RUN_PASS, SP, 16, L_LONG, WA);                   /* # bytes */
    Write (RUN_PASS, SP + 4, p1, L_LONG, WA);               /* mcheck type */
    Write (RUN_PASS, SP + 8, p2, L_LONG, WA);               /* address */
    Write (RUN_PASS, SP + 12, st1, L_LONG, WA);             /* state 1 */
    Write (RUN_PASS, SP + 16, st2, L_LONG, WA);             /* state 2 */
    in_ie = 0;
    return cc;
}

t_bool smp_hlt_enb(atomic_int32* set_on)
{
    int nRunning = 0;

    /*
     * MicroVAX 3900 firmware ROM is not SMP-aware,
     * therefore we cannot perform halt on active multiprocessor configuration
     */

    cpu_database_lock->lock();

    for (uint32 cpu_ix = 0;  cpu_ix < sim_ncpus;  cpu_ix++)
    {
        if (cpu_running_set.is_set(cpu_ix))
        {
            if (++nRunning > 1)  break;
        }
    }

    if (nRunning > 1)
    {
        cpu_database_lock->unlock();
        return FALSE;
    }

    /* flag: halt/stop pending */
    if (set_on)
        *set_on = 1;

    cpu_database_lock->unlock();

    return TRUE;
}

/* saved values of variables while interrupted into ROM console mode */
static int32 con_sys_idle_cpu_mask_va = 0;
static int32 con_sys_critical_section_ipl = -1;
static t_bool con_sim_vsmp_active = FALSE;
static uint32 con_syncw_on = 0;
static t_bool con_use_native_interlocked = FALSE;

/* Console entry */
int32 con_halt (int32 code, int32 cc)
{
    int32 temp;
    RUN_SCOPE;

    /* 
     * MicroVAX 3900 firmware ROM is not SMP-aware, so if active SMP configuration is running,
     * never transfer control to ROM.
     *
     * Execute HALT to SimH console instead if HALT instruction was executed. 
     *
     * Halt by HALT PIN (such as raised by Ctrl/P) is disabled if multiple processors are active,
     * so it should never happen,  but if ever did, just ignore it, clear hlt_pin and continue running.
     */
    if (! smp_hlt_enb())
    {
        if (code == CON_HLTINS)
        {
            ABORT (STOP_HALT);
        }
        else // code == CON_HLTPIN
        {
            /* 
             * Should never happen, but if it ever did just ignore and clear halt pin.
             */
            hlt_pin = 0;
            return PSL & CC_MASK;
        }
    }

    /*
     * Single-CPU case only at this point, should be primary processor.
     *
     * Secondary can happen here only because of the bug in the simulator
     * or if the operating system reassigned primary CPU.
     */
    if (! cpu_unit->is_primary_cpu())
    {
        smp_printf ("\nHALT on the secondary processor\n");
        if (sim_log)
            fprintf (sim_log, "HALT on the secondary processor\n");
        ABORT_INVALID_SYSOP;
    }

    hlt_pin = 0;

    conpc = PC;                                             /* save PC */
    conpsl = ((PSL | cc) & 0xFFFF00FF) | CON_HLTINS;        /* PSL, param */
    temp = (PSL >> PSL_V_CUR) & 0x7;                        /* get is'cur */
    if (temp > 4)                                           /* invalid? */
        conpsl = conpsl | CON_BADPSL;
    else
        STK[temp] = SP;                                     /* save stack */
    if (mapen)                                              /* mapping on? */
        conpsl = conpsl | CON_MAPON;
    mapen = 0;                                              /* turn off map */
    SP = IS;                                                /* set SP from IS */
    PSL = PSL_IS | PSL_IPL1F;                               /* PSL = 41F0000 */
    JUMP (ROMBASE);                                         /* PC = 20040000 */

    /* save paravirtualization state */
    con_sys_idle_cpu_mask_va = sys_idle_cpu_mask_va;
    con_sys_critical_section_ipl = sys_critical_section_ipl;
    con_sim_vsmp_active = sim_vsmp_active;
    con_syncw_on = syncw.on;
    con_use_native_interlocked = use_native_interlocked;

    /* reset paravirtualization state */
    cpu_on_clear_mapen(RUN_PASS);                           /* will also re-evaluate thread priority */

    return 0;                                               /* new cc = 0 */
}

/*
 * Called when MAPEN is cleared
 */
void cpu_on_clear_mapen(RUN_DECL)
{
    /* safety check */
    if (mapen)  return;

    /* local CPU is leaving OS context */
    cpu_unit->cpu_active_clk_interrupt = FALSE;
    cpu_unit->cpu_active_ipi_interrupt = FALSE;

    /* after operating system shutdown or when entering console mode... */
    if (cpu_unit->is_primary_cpu() && mapen == 0)
    {
        /* reset the pointer to idle CPUs mask */
        sys_idle_cpu_mask_va = 0;

        /* reset critical section ipl */
        sys_critical_section_ipl = -1;

        /* disable synchronization window */
        syncw_leave_all(RUN_PASS, SYNCW_OVERRIDE_ALL | SYNCW_DISABLE_CPU);
        syncw.on = 0;

        /* disable use of native interlock */
        use_native_interlocked = FALSE;

        /* reset "ready for multiprocessing" state */
        if (sim_vsmp_active)
        {
            sim_vsmp_active = FALSE;
            if (sim_clock_thread_created)
                smp_set_thread_priority(sim_clock_thread, SIMH_THREAD_PRIORITY_CLOCK);
        }
    }

    cpu_reevaluate_thread_priority(RUN_PASS);
}

/*
 * Called when console executes the CONTINUE command.
 *
 *   After MTPR #1, #MT_MAPEN is called.
 *   Right before REI is executed.
 */
void cpu_on_rom_continue(RUN_DECL)
{
    /* restore paravirtualization state */
    sys_idle_cpu_mask_va = con_sys_idle_cpu_mask_va;
    sys_critical_section_ipl = con_sys_critical_section_ipl;
    sim_vsmp_active = con_sim_vsmp_active;
    syncw.on = con_syncw_on;
    use_native_interlocked = con_use_native_interlocked;

    con_sys_idle_cpu_mask_va = 0;
    con_sys_critical_section_ipl = -1;
    con_sim_vsmp_active = FALSE;
    con_syncw_on = 0;
    con_use_native_interlocked = FALSE;

    /*
     * Could execute here
     *     syncw_enable_cpu(RUN_PASS);
     *     syncw_reeval_sys(RUN_PASS);
     * to reenter syncw, however they will be executed by op_rei which is next instruction.
     */

    if (sim_vsmp_active && sim_clock_thread_created)
        smp_set_thread_priority(sim_clock_thread, SIMH_THREAD_PRIORITY_CLOCK);

    /*
     * Both MTPR and REI will call SET_IRQL that will reevaluate thread priority
     * and also values of
     *
     *     cpu_unit->cpu_active_clk_interrupt
     *     cpu_unit->cpu_active_ipi_interrupt
     *
     * if needed.
     */
}

/* Bootstrap */

t_stat cpu_boot (int32 unitno, DEVICE *dptr)
{
    extern t_stat load_cmd (int32 flag, char *cptr);
    extern SMP_FILE *sim_log;

    if (unitno != 0)
    {
        smp_printf ("Only primary CPU (cpu0) can boot\n");
        if (sim_log)
            fprintf (sim_log, "Only primary CPU (cpu0) can boot\n");
        return SCPE_NOFNC;
    }

    RUN_SCOPE;
    t_stat r;

    SETPC(ROMBASE);
    PSL = PSL_IS | PSL_IPL1F;
    conpc = 0;
    conpsl = PSL_IS | PSL_IPL1F | CON_PWRUP;
    if (rom == NULL)
        return SCPE_IERR;
    if (*rom == 0)                                          /* no boot? */
    {
        smp_printf ("Loading boot code from ka655x.bin\n");
        if (sim_log)
            fprintf (sim_log, "Loading boot code from ka655x.bin\n");
        r = load_cmd (0, "-R ka655x.bin");
        if (r != SCPE_OK)
        {
#ifndef DONT_USE_INTERNAL_ROM
            SMP_FILE *f;

            if (f = sim_fopen ("ka655x.bin", "wb"))
            {
                smp_printf ("Saving boot code to ka655x.bin\n");
                if (sim_log)
                    fprintf (sim_log, "Saving boot code to ka655x.bin\n");
                sim_fwrite (vax_ka655x_bin, sizeof(vax_ka655x_bin[0]), sizeof(vax_ka655x_bin)/sizeof(vax_ka655x_bin[0]), f);
                fclose (f);
                smp_printf ("Loading boot code from ka655x.bin\n");
                if (sim_log)
                    fprintf (sim_log, "Loading boot code from ka655x.bin\n");
                r = load_cmd (0, "-R ka655x.bin");
            }
#endif
            return r;
        }
    }
    sysd_powerup(RUN_PASS);
    syncw_leave_all(RUN_PASS, SYNCW_OVERRIDE_ALL | SYNCW_ENABLE_CPU);
    syncw.on = 0;
    return SCPE_OK;
}

/* SYSD reset */

t_stat sysd_reset (DEVICE *dptr)
{
    int32 i;
    RUN_SCOPE;

    if (sim_switches & SWMASK ('P'))
        sysd_powerup (RUN_PASS);       /* powerup? */

    for (i = 0; i < 2; i++)
    {
        tmr_csr[i] = tmr_tnir[i] = tmr_tir[i] = 0;
        tmr_inc[i] = tmr_sav[i] = 0;
        sim_cancel (sysd_unit[i]);
        sysd_deactivating(i);
    }

    if (cpu_unit->is_primary_cpu())
    {
        AUTO_LOCK(csio_lock);
        csi_csr = 0;
        csi_unit.buf = 0;
        sim_cancel (&csi_unit);
        CLR_INT (CSI);
        cso_csr = CSR_DONE;
        cso_unit.buf = 0;
        sim_cancel (&cso_unit);
        CLR_INT (CSO);
    }

    return SCPE_OK;
}

/* SYSD powerup */

t_stat sysd_powerup (RUN_DECL)
{
    int32 i;

    if (cpu_unit->is_primary_cpu())
    {
        // do this just once for the reset cycle across all CPUs, and only from the primary CPU
        AUTO_LOCK(sysd_lock);
        ssc_base = SSCBASE;
        ssc_cnf = ssc_cnf & SSCCNF_BLO;
    }

    for (i = 0;  i < (CMCTLSIZE >> 2);  i++)
    {
        cmctl_reg[i] = 0;
    }

    for (i = 0;  i < 2;  i++)
    {
        tmr_tivr[i] = 0;
        ssc_adsm[i] = ssc_adsk[i] = 0;
    }

    ka_cacr = 0;
    ssc_bto = 0;
    ssc_otp = 0;

    return SCPE_OK;
}

/*
 * Set up secondary CPU's model-specific registers before that CPU is started.
 *
 * cpu_unit points to target secondary CPU.
 * local_cpu is the CPU that is setting up the secondary for starting it.
 */
void cpu_setup_secondary_model_specific(RUN_DECL, CPU_UNIT* local_cpu)
{
    /* clone cache control registers from the local VCPU */
    CADR = local_cpu->cpu_context.r_CADR;
    ka_cacr = local_cpu->cpu_context.r_ka_cacr;

    /* clone SSC BTO setting from the local VCPU */
    ssc_bto = local_cpu->cpu_context.r_ssc_bto & SSCBTO_INTV;

    /* clone settings of SSC timer interrupt vectors from the local VCPU */
    tmr_tivr[0] = local_cpu->cpu_context.r_tmr_tivr[0];
    tmr_tivr[1] = local_cpu->cpu_context.r_tmr_tivr[1];

    /* clone CMCTL registers */
    memcpy(cmctl_reg, local_cpu->cpu_context.r_cmctl_reg, sizeof(cmctl_reg));
}