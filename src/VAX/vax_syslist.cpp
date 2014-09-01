/* vax_syslist.c: VAX simulator interface

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

   17-Oct-06    RMS     Re-ordered device list
   17-May-06    RMS     Added CR11/CD11 support (from John Dundas)
   01-Oct-2004  RMS     Cloned from vax_sys.c
*/

#include "sim_defs.h"
#include "vax_defs.h"

char sim_name[] = "VAX MP";

extern int32 sim_switches;
extern void WriteB (RUN_DECL, uint32 pa, int32 val);
extern void rom_wr_B (RUN_DECL, int32 pa, int32 val);

/* when adding extra QBus device to sim_devices,
   be sure to also include it in the Qbus interrupt reset list
   in qba_reset */
DEVICE *sim_devices[] =
{ 
    &cpu_dev,
    &tlb_dev,
    &rom_dev,
    &nvr_dev,
    &sysd_dev,
    &qba_dev,
    &clk_dev,
    &tti_dev,
    &tto_dev,
    &csi_dev,
    &cso_dev,
    &dz_dev,
    &vh_dev,
    &cr_dev,
    &lpt_dev,
    &rl_dev,
    &rq_dev,
    &rqb_dev,
    &rqc_dev,
    &rqd_dev,
    &ry_dev,
    &ts_dev,
    &tq_dev,
    &xq_dev,
    &xqb_dev,
    NULL
};

int sim_device_index (DEVICE* dptr)
{
    for (int k = 0; ; k++)
    {
        if (sim_devices[k] == NULL)
            panic("Unable to find requested device in sim_dev_index");
        if (sim_devices[k] == dptr)
            return k;
    }
}

/* Binary loader

   The binary loader handles absolute system images, that is, system
   images linked /SYSTEM.  These are simply a byte stream, with no
   origin or relocation information.

   -r           load ROM
   -n           load NVR
   -o           for memory, specify origin
*/

t_stat sim_load (RUN_DECL, SMP_FILE *fileref, char *cptr, char *fnam, int flag)
{
t_stat r;
int32 i;
uint32 origin, limit;
extern int32 ssc_cnf;
#define SSCCNF_BLO      0x80000000

if (flag)                                               /* dump? */
    return SCPE_ARG;
if (sim_switches & SWMASK ('R')) {                      /* ROM? */
    origin = ROMBASE;
    limit = ROMBASE + ROMSIZE;
    }
else if (sim_switches & SWMASK ('N')) {                 /* NVR? */
    origin = NVRBASE;
    limit = NVRBASE + NVRSIZE;
    ssc_cnf = ssc_cnf & ~SSCCNF_BLO;
    }
else {
    origin = 0;                                         /* memory */
    limit = (uint32) cpu_unit->capac;
    if (sim_switches & SWMASK ('O')) {                  /* origin? */
        origin = (int32) get_uint (cptr, 16, 0xFFFFFFFF, &r);
        if (r != SCPE_OK)
            return SCPE_ARG;
        }
    }
while ((i = getc (fileref)) != EOF) {                   /* read byte stream */
    if (origin >= limit)                                /* NXM? */
        return SCPE_NXM;
    if (sim_switches & SWMASK ('R'))                    /* ROM? */
        rom_wr_B (RUN_PASS, origin, i);                           /* not writeable */
    else WriteB (RUN_PASS, origin, i);                            /* store byte */
    origin = origin + 1;
    }
return SCPE_OK;
}

