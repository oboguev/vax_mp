/* sim_disk.c: simulator disk support library

   Copyright (c) 2011, Mark Pizzolato

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

   Except as contained in this notice, the names of Mark Pizzolato shall not be
   used in advertising or otherwise to promote the sale, use or other dealings 
   in this Software without prior written authorization from Mark Pizzolato.



   This is the place which hides processing of various disk formats,
   as well as OS-specific direct hardware access.

   25-Jan-11    MP      Initial Implemementation

Public routines:

   sim_disk_attach           attach disk unit
   sim_disk_detach           detach disk unit
   sim_disk_rdsect           read disk sectors
   sim_disk_rdsect_a         read disk sectors asynchronously
   sim_disk_wrsect           write disk sectors
   sim_disk_wrsect_a         write disk sectors asynchronously
   sim_disk_unload           unload or detach a disk as needed
   sim_disk_reset            reset device
   sim_disk_wrp              TRUE if write protected
   sim_disk_isavailable      TRUE if available for I/O
   sim_disk_size             get disk size
   sim_disk_set_fmt          set disk format
   sim_disk_show_fmt         show disk format
   sim_disk_set_capac        set disk capacity
   sim_disk_show_capac       show disk capacity
   sim_disk_set_async        enable asynchronous operation
   sim_disk_clr_async        disable asynchronous operation
   sim_disk_data_trace       debug support

Internal routines:

   sim_os_disk_open_raw      platform specific open raw device
   sim_os_disk_close_raw     platform specific close raw device
   sim_os_disk_size_raw      platform specific raw device size
   sim_os_disk_unload_raw    platform specific disk unload/eject
   sim_os_disk_rdsect        platform specific read sectors
   sim_os_disk_wrsect        platform specific write sectors

   sim_vhd_disk_open         platform independent open virtual disk file
   sim_vhd_disk_create       platform independent create virtual disk file
   sim_vhd_disk_create_diff  platform independent create differencing virtual disk file
   sim_vhd_disk_close        platform independent close virtual disk file
   sim_vhd_disk_size         platform independent virtual disk size
   sim_vhd_disk_rdsect       platform independent read virtual disk sectors
   sim_vhd_disk_wrsect       platform independent write virtual disk sectors


*/

#if defined(_WIN32)
#  include <windows.h>
#endif

#include "sim_defs.h"
#include "sim_disk.h"

#include <ctype.h>
#include <sys/stat.h>

extern SMP_FILE* sim_log;                               /* log file */
extern int32 sim_switches;
extern int32 sim_quiet;
extern int32 sim_end;

#define DOP_DONE  0             /* close */
#define DOP_RSEC  1             /* sim_disk_rdsect_a */
#define DOP_WSEC  2             /* sim_disk_wrsect_a */
#define DOP_IAVL  3             /* sim_disk_isavailable_a */


class disk_context : public aio_context
{
public:
    disk_context(UNIT* uptr) : aio_context(uptr)
    {
        io_dop = DOP_DONE;
        callback = NULL;
    }
    void perform_flush();
    static void perform_flush(UNIT* uptr);
    t_bool has_request() { return io_dop != DOP_DONE; }
    void perform_request();

public:
    uint32              sector_size;        /* Disk Sector Size (of the pseudo disk) */
    uint32              xfer_element_size;  /* Disk Bus Transfer size (1 - byte, 2 - word, 4 - longword) */
    uint32              storage_sector_size;/* Sector size of the containing storage */
    uint32              removable;          /* Removable device flag */
    uint32              auto_format;        /* Format determined dynamically */
#if defined _WIN32
    HANDLE              disk_handle;        /* OS specific Raw device handle */
#endif
    int                 io_dop;
    uint8               *buf;
    t_seccnt            *rsects;
    t_seccnt            sects;
    t_lba               lba;
    DISK_PCALLBACK      callback;
};

#define disk_ctx up8                        /* Field in Unit structure which points to the disk_context */

static void aio_panic();
#define AIO_CALLSETUP                                               \
disk_context* ctx = (disk_context*) uptr->disk_ctx;                 \
                                                                    \
if ((!callback) || !ctx->asynch_io)

/* caller of AIO_CALL will be holding uptr->lock */
#define AIO_CALL(op, _lba, _buf, _rsects, _sects,  _callback)   \
    if (ctx->asynch_io)                                         \
    {                                                           \
        disk_context* ctx = (disk_context*) uptr->disk_ctx;     \
                                                                \
        sim_debug (ctx->dbit, ctx->dptr,                        \
      "sim_disk AIO_CALL(op=%d, unit=%d, lba=0x%X, sects=%d)\n",\
                   op, sim_unit_index(uptr), _lba, _sects);     \
                                                                \
        if (unlikely(NULL != ctx->callback))                    \
            aio_panic();      /* gross error */                 \
        ctx->lba = _lba;                                        \
        ctx->buf = _buf;                                        \
        ctx->sects = _sects;                                    \
        ctx->rsects = _rsects;                                  \
        ctx->callback = _callback;                              \
        ctx->io_reset_count = uptr->device->a_reset_count;      \
        smp_wmb();                                              \
        ctx->io_dop = op;                                       \
        ctx->io_event_signal();                                 \
    }                                                           \
    else                                                        \
        if (_callback)                                          \
            (_callback) (uptr, r);


SMP_THREAD_ROUTINE_DECL _disk_io(void* arg)
{
    UNIT* volatile uptr = (UNIT*)arg;
    disk_context* ctx = (disk_context*) uptr->disk_ctx;
    char tname[16];

    sim_try
    {
        sim_debug (ctx->dbit, ctx->dptr, "_disk_io(unit=%d) starting\n", sim_unit_index(uptr));

        smp_thread_init();

        run_scope_context* rscx = new run_scope_context(NULL, SIM_THREAD_TYPE_IOP, ctx->io_thread);
        rscx->set_current();

        smp_set_thread_priority(SIMH_THREAD_PRIORITY_IOP);
        sprintf(tname, "IOP_%s%d", uptr->device->name, sim_unit_index(uptr));
        smp_set_thread_name(tname);

        ctx->thread_loop();
    }
    sim_catch (sim_exception_SimError, exc)
    {
        fprintf(smp_stderr, "\nFatal error in %s simulator, unexpected exception while executing disk IOP thread\n", sim_name);
        fprintf(smp_stderr, "Exception cause: %s\n", exc->get_message());
        fprintf(smp_stderr, "Terminating the simulator abnormally...\n");
        exit(1);
    }
    sim_end_try

    sim_debug (ctx->dbit, ctx->dptr, "_disk_io(unit=%d) exiting\n", sim_unit_index(uptr));

    SMP_THREAD_ROUTINE_END;
}

void disk_context::perform_request()
{
    switch (io_dop)
    {
    case DOP_RSEC:
        io_status = sim_disk_rdsect (uptr, lba, buf, rsects, sects);
        break;
    case DOP_WSEC:
        io_status = sim_disk_wrsect (uptr, lba, buf, rsects, sects);
        break;
    case DOP_IAVL:
        io_status = sim_disk_isavailable (uptr);
        break;
    }
    io_dop = DOP_DONE;
    sim_async_post_io_event(uptr);
}

/* This routine is called in the context of the main simulator thread before 
   processing events for any unit. It is only called when an asynchronous 
   thread has called sim_activate() to activate a unit.  The job of this 
   routine is to put the unit in proper condition to digest what may have
   occurred in the asynchrcondition thread.
   
   Since disk processing only handles a single I/O at a time to a 
   particular disk device (due to using stdio for the SimH Disk format
   and stdio doesn't have an atomic seek+(read|write) operation), 
   we have the opportunity to possibly detect improper attempts to 
   issue multiple concurrent I/O requests. */
static void _disk_completion_dispatch (UNIT *uptr)
{
    disk_context* ctx = (disk_context*) uptr->disk_ctx;
    DISK_PCALLBACK callback = ctx->callback;

    sim_debug (ctx->dbit, ctx->dptr, "_disk_completion_dispatch(unit=%d, dop=%d, callback=%p)\n", sim_unit_index(uptr), ctx->io_dop, ctx->callback);

    if (ctx->io_dop != DOP_DONE)
        aio_panic();                                    /* horribly wrong, stop */

    if (ctx->callback && ctx->io_dop == DOP_DONE)
    {
        ctx->callback = NULL;
        if (ctx->io_reset_count == uptr->device->a_reset_count)
            callback (uptr, ctx->io_status);
    }
}

static void aio_panic()
{
    panic("Unexpected fatal error in disk AIO subsystem");
}

/* Forward declarations */

static t_stat sim_vhd_disk_implemented (void);
static SMP_FILE* sim_vhd_disk_open (const char *rawdevicename, const char *openmode);
static SMP_FILE* sim_vhd_disk_create (const char *szVHDPath, t_addr desiredsize);
static SMP_FILE* sim_vhd_disk_create_diff (const char *szVHDPath, const char *szParentVHDPath);
static int sim_vhd_disk_close (SMP_FILE* f);
static void sim_vhd_disk_flush (SMP_FILE* f);
static t_addr sim_vhd_disk_size (SMP_FILE* f);
static t_stat sim_vhd_disk_rdsect (UNIT *uptr, t_lba lba, uint8 *buf, t_seccnt *sectsread, t_seccnt sects);
static t_stat sim_vhd_disk_wrsect (UNIT *uptr, t_lba lba, uint8 *buf, t_seccnt *sectswritten, t_seccnt sects);
static t_stat sim_vhd_disk_set_dtype (SMP_FILE* f, const char *dtype);
static const char *sim_vhd_disk_get_dtype (SMP_FILE* f);
static t_stat sim_os_disk_implemented_raw (void);
static SMP_FILE* sim_os_disk_open_raw (const char *rawdevicename, const char *openmode);
static int sim_os_disk_close_raw (SMP_FILE* f);
static void sim_os_disk_flush_raw (SMP_FILE* f);
static t_addr sim_os_disk_size_raw (SMP_FILE* f);
static t_stat sim_os_disk_unload_raw (SMP_FILE* f);
static t_bool sim_os_disk_isavailable_raw (SMP_FILE* f);
static t_stat sim_os_disk_rdsect (UNIT *uptr, t_lba lba, uint8 *buf, t_seccnt *sectsread, t_seccnt sects);
static t_stat sim_os_disk_wrsect (UNIT *uptr, t_lba lba, uint8 *buf, t_seccnt *sectswritten, t_seccnt sects);
static t_stat sim_os_disk_info_raw (SMP_FILE* f, uint32 *sector_size, uint32 *removable);
static t_stat sim_disk_pdp11_bad_block (UNIT *uptr, int32 sec);

struct sim_disk_fmt {
    char                *name;                          /* name */
    int32               uflags;                         /* unit flags */
    int32               fmtval;                         /* Format type value */
    t_stat              (*impl_fnc)(void);              /* Implemented Test Function */
    };

static struct sim_disk_fmt fmts[DKUF_N_FMT] = {
    { "SIMH", 0, DKUF_F_STD, NULL},
    { "RAW",  0, DKUF_F_RAW, sim_os_disk_implemented_raw},
    { "VHD",  0, DKUF_F_VHD, sim_vhd_disk_implemented},
    { NULL,   0, 0}
    };

/* Set disk format */

t_stat sim_disk_set_fmt (UNIT *uptr, int32 val, char *cptr, void *desc)
{
    uint32 f;

    if (uptr == NULL)
        return SCPE_IERR;
    if (cptr == NULL)
        return SCPE_ARG;
    for (f = 0; f < DKUF_N_FMT && fmts[f].name; f++) {
        if (fmts[f].name && (strcmp (cptr, fmts[f].name) == 0)) {
            if ((fmts[f].impl_fnc) && (fmts[f].impl_fnc() != SCPE_OK))
                return SCPE_NOFNC;
            uptr->flags = (uptr->flags & ~DKUF_FMT) |
                (fmts[f].fmtval << DKUF_V_FMT) | fmts[f].uflags;
            return SCPE_OK;
            }
        }
    return SCPE_ARG;
}

/* Show disk format */

t_stat sim_disk_show_fmt (SMP_FILE* st, UNIT *uptr, int32 val, void *desc)
{
    int32 i, f = DK_GET_FMT (uptr);

    for (i = 0; i < (int32) DKUF_N_FMT; i++)
        if (fmts[i].fmtval == f) {
            fprintf (st, "%s format", fmts[i].name);
            return SCPE_OK;
            }
    fprintf (st, "invalid format");
    return SCPE_OK;
}

/* Set disk capacity */

t_stat sim_disk_set_capac (UNIT *uptr, int32 val, char *cptr, void *desc)
{
    t_addr cap;
    t_stat r;

    if ((cptr == NULL) || (*cptr == 0))
        return SCPE_ARG;
    if (uptr->flags & UNIT_ATT)
        return SCPE_ALATT;
    cap = (t_addr) get_uint (cptr, 10, sim_taddr_64? 2000000: 2000, &r);
    if (r != SCPE_OK)
        return SCPE_ARG;
    uptr->capac = cap * ((t_addr) 1000000);
    return SCPE_OK;
}

/* Show disk capacity */

t_stat sim_disk_show_capac (SMP_FILE* st, UNIT *uptr, int32 val, void *desc)
{
    if (uptr->capac) {
        if (uptr->capac >= (t_addr) 1000000)
            fprintf (st, "capacity=%dMB", (uint32) (uptr->capac / ((t_addr) 1000000)));
        else if (uptr->capac >= (t_addr) 1000)
            fprintf (st, "capacity=%dKB", (uint32) (uptr->capac / ((t_addr) 1000)));
        else fprintf (st, "capacity=%dB", (uint32) uptr->capac);
        }
    else fprintf (st, "undefined capacity");
    return SCPE_OK;
}

/* Test for available */

t_bool sim_disk_isavailable (UNIT *uptr)
{
if (!(uptr->flags & UNIT_ATT))                          /* attached? */
    return FALSE;
switch (DK_GET_FMT (uptr)) {                            /* case on format */
    case DKUF_F_STD:                                    /* SIMH format */
        return TRUE;
    case DKUF_F_VHD:                                    /* VHD format */
        return TRUE;
        break;
    case DKUF_F_RAW:                                    /* Raw Physical Disk Access */
        return sim_os_disk_isavailable_raw (uptr->fileref);
        break;
    default:
        return FALSE;
    }
}

t_bool sim_disk_isavailable_a (UNIT *uptr, DISK_PCALLBACK callback)
{
t_bool r = FALSE;
AIO_CALLSETUP
    r = sim_disk_isavailable (uptr);
AIO_CALL(DOP_IAVL, 0, NULL, NULL, 0, callback);
return r;
}

/* Test for write protect */

t_bool sim_disk_wrp (UNIT *uptr)
{
return (uptr->flags & DKUF_WRP)? TRUE: FALSE;
}

/* Get Disk size */

t_addr sim_disk_size (UNIT *uptr)
{
switch (DK_GET_FMT (uptr)) {                            /* case on format */
    case DKUF_F_STD:                                    /* SIMH format */
        return sim_fsize_ex (uptr->fileref);
    case DKUF_F_VHD:                                    /* VHD format */
        return sim_vhd_disk_size (uptr->fileref);
        break;
    case DKUF_F_RAW:                                    /* Raw Physical Disk Access */
        return sim_os_disk_size_raw (uptr->fileref);
        break;
    default:
        return (t_addr)-1;
    }
}

/* Enable asynchronous operation */

t_stat sim_disk_set_async (UNIT *uptr, int latency)
{
    disk_context* ctx = (disk_context*) uptr->disk_ctx;

    if (ctx->asynch_io = sim_asynch_enabled)
    {
        uptr->a_check_completion = _disk_completion_dispatch;
        ctx->asynch_io = FALSE;
        ctx->asynch_init(_disk_io, (void*) uptr);
        ctx->asynch_io = TRUE;
    }
    return SCPE_OK;
}

/* Disable asynchronous operation */

t_stat sim_disk_clr_async (UNIT *uptr)
{
    disk_context* ctx = (disk_context*) uptr->disk_ctx;

    /* make sure device exists */
    if (!ctx) return SCPE_UNATT;

    if (ctx->asynch_io)
        ctx->asynch_uninit();

    return SCPE_OK;
}

/* Read Sectors */

static t_stat _sim_disk_rdsect (UNIT *uptr, t_lba lba, uint8 *buf, t_seccnt *sectsread, t_seccnt sects)
{
t_addr da;
uint32 err, tbc;
size_t i;
disk_context* ctx = (disk_context*) uptr->disk_ctx;

sim_debug (ctx->dbit, ctx->dptr, "_sim_disk_rdsect(unit=%d, lba=0x%X, sects=%d)\n", sim_unit_index(uptr), lba, sects);

da = ((t_addr)lba) * ctx->sector_size;
tbc = sects * ctx->sector_size;
if (sectsread)
    *sectsread = 0;
err = sim_fseek (uptr->fileref, da, SEEK_SET);          /* set pos */
if (!err) {
    i = sim_fread (buf, ctx->xfer_element_size, tbc/ctx->xfer_element_size, uptr->fileref);
    if (i < tbc/ctx->xfer_element_size)                 /* fill */
        memset (&buf[i*ctx->xfer_element_size], 0, tbc-(i*ctx->xfer_element_size));
    err = ferror (uptr->fileref);
    if ((!err) && (sectsread))
        *sectsread = (t_seccnt)((i*ctx->xfer_element_size+ctx->sector_size-1)/ctx->sector_size);
    }
return err;
}

t_stat sim_disk_rdsect (UNIT *uptr, t_lba lba, uint8 *buf, t_seccnt *sectsread, t_seccnt sects)
{
t_stat r;
disk_context* ctx = (disk_context*) uptr->disk_ctx;
t_seccnt sread;

sim_debug (ctx->dbit, ctx->dptr, "sim_disk_rdsect(unit=%d, lba=0x%X, sects=%d)\n", sim_unit_index(uptr), lba, sects);

if ((sects == 1) &&                                     /* Single sector reads */
    (lba >= uptr->capac/ctx->sector_size)) {            /* beyond the end of the disk */
    memset (buf, '\0', ctx->sector_size);               /* are bad block management efforts - zero buffer */
    if (sectsread)
        *sectsread = 1;
    return SCPE_OK;                                     /* return success */
    }

if ((0 == (ctx->sector_size & (ctx->storage_sector_size - 1))) ||   /* Sector Aligned & whole sector transfers */
    ((0 == ((lba*ctx->sector_size) & (ctx->storage_sector_size - 1))) &&
     (0 == ((sects*ctx->sector_size) & (ctx->storage_sector_size - 1))))) {
    switch (DK_GET_FMT (uptr)) {                        /* case on format */
        case DKUF_F_STD:                                /* SIMH format */
            return _sim_disk_rdsect (uptr, lba, buf, sectsread, sects);
        case DKUF_F_VHD:                                /* VHD format */
            r = sim_vhd_disk_rdsect (uptr, lba, buf, &sread, sects);
            break;
        case DKUF_F_RAW:                                /* Raw Physical Disk Access */
            r = sim_os_disk_rdsect (uptr, lba, buf, &sread, sects);
            break;
        default:
            return SCPE_NOFNC;
        }
    if (sectsread)
        *sectsread = sread;
    if (r != SCPE_OK)
        return r;
    sim_buf_swap_data (buf, ctx->xfer_element_size, (sread * ctx->sector_size) / ctx->xfer_element_size);
    return r;
    }
else { /* Unaligned and/or partial sector transfers */
    uint8 *tbuf = (uint8*) malloc (sects*ctx->sector_size + 2*ctx->storage_sector_size);
    t_lba sspsts = ctx->storage_sector_size/ctx->sector_size; /* sim sectors in a storage sector */
    t_lba tlba = lba & ~(sspsts - 1);
    t_seccnt tsects = sects + (lba - tlba);

    tsects = (tsects + (sspsts - 1)) & ~(sspsts - 1);
    if (sectsread)
        *sectsread = 0;
    if (tbuf == NULL)
        return SCPE_MEM;
    switch (DK_GET_FMT (uptr)) {                        /* case on format */
        case DKUF_F_STD:                                /* SIMH format */
            r = _sim_disk_rdsect (uptr, tlba, tbuf, &sread, tsects);
            break;
        case DKUF_F_VHD:                                /* VHD format */
            r = sim_vhd_disk_rdsect (uptr, tlba, tbuf, &sread, tsects);
            if (r == SCPE_OK)
                sim_buf_swap_data (tbuf, ctx->xfer_element_size, (sread * ctx->sector_size) / ctx->xfer_element_size);
            break;
        case DKUF_F_RAW:                                /* Raw Physical Disk Access */
            r = sim_os_disk_rdsect (uptr, tlba, tbuf, &sread, tsects);
            if (r == SCPE_OK)
                sim_buf_swap_data (tbuf, ctx->xfer_element_size, (sread * ctx->sector_size) / ctx->xfer_element_size);
            break;
        default:
            free (tbuf);
            return SCPE_NOFNC;
        }
    if (r == SCPE_OK) {
        memcpy (buf, tbuf + ((lba - tlba) * ctx->sector_size), sects * ctx->sector_size);
        if (sectsread) {
            *sectsread = sread - (lba - tlba);
            if (*sectsread > sects)
                *sectsread = sects;
            }
        }
    free (tbuf);
    return r;
    }
}

t_stat sim_disk_rdsect_a (UNIT *uptr, t_lba lba, uint8 *buf, t_seccnt *sectsread, t_seccnt sects, DISK_PCALLBACK callback)
{
t_stat r = SCPE_OK;
AIO_CALLSETUP
    r = sim_disk_rdsect (uptr, lba, buf, sectsread, sects);
AIO_CALL(DOP_RSEC, lba, buf, sectsread, sects, callback);
return r;
}

/* Write Sectors */

static t_stat _sim_disk_wrsect (UNIT *uptr, t_lba lba, uint8 *buf, t_seccnt *sectswritten, t_seccnt sects)
{
t_addr da;
uint32 err, tbc;
size_t i;
disk_context* ctx = (disk_context*) uptr->disk_ctx;

sim_debug (ctx->dbit, ctx->dptr, "_sim_disk_wrsect(unit=%d, lba=0x%X, sects=%d)\n", sim_unit_index(uptr), lba, sects);

da = ((t_addr)lba) * ctx->sector_size;
tbc = sects * ctx->sector_size;
if (sectswritten)
    *sectswritten = 0;
err = sim_fseek (uptr->fileref, da, SEEK_SET);          /* set pos */
if (!err) {
    i = sim_fwrite (buf, ctx->xfer_element_size, tbc/ctx->xfer_element_size, uptr->fileref);
    err = ferror (uptr->fileref);
    if ((!err) && (sectswritten))
        *sectswritten = (t_seccnt)((i*ctx->xfer_element_size+ctx->sector_size-1)/ctx->sector_size);
    }
return err;
}

t_stat sim_disk_wrsect (UNIT *uptr, t_lba lba, uint8 *buf, t_seccnt *sectswritten, t_seccnt sects)
{
disk_context* ctx = (disk_context*) uptr->disk_ctx;
uint32 f = DK_GET_FMT (uptr);
t_stat r;
uint8 *tbuf = NULL;

sim_debug (ctx->dbit, ctx->dptr, "sim_disk_wrsect(unit=%d, lba=0x%X, sects=%d)\n", sim_unit_index(uptr), lba, sects);

if (f == DKUF_F_STD)
    return _sim_disk_wrsect (uptr, lba, buf, sectswritten, sects);
if ((0 == (ctx->sector_size & (ctx->storage_sector_size - 1))) ||   /* Sector Aligned & whole sector transfers */
    ((0 == ((lba*ctx->sector_size) & (ctx->storage_sector_size - 1))) &&
     (0 == ((sects*ctx->sector_size) & (ctx->storage_sector_size - 1))))) {

    if (sim_end || (ctx->xfer_element_size == sizeof (char)))
        switch (DK_GET_FMT (uptr)) {                            /* case on format */
            case DKUF_F_VHD:                                    /* VHD format */
                return sim_vhd_disk_wrsect  (uptr, lba, buf, sectswritten, sects);
            case DKUF_F_RAW:                                    /* Raw Physical Disk Access */
                return sim_os_disk_wrsect  (uptr, lba, buf, sectswritten, sects);
            default:
                return SCPE_NOFNC;
            }

    tbuf = (uint8*) malloc (sects * ctx->sector_size);
    if (NULL == tbuf)
        return SCPE_MEM;
    sim_buf_copy_swapped (tbuf, buf, ctx->xfer_element_size, (sects * ctx->sector_size) / ctx->xfer_element_size);

    switch (DK_GET_FMT (uptr)) {                            /* case on format */
        case DKUF_F_VHD:                                    /* VHD format */
            r = sim_vhd_disk_wrsect (uptr, lba, tbuf, sectswritten, sects);
            break;
        case DKUF_F_RAW:                                    /* Raw Physical Disk Access */
            r = sim_os_disk_wrsect (uptr, lba, tbuf, sectswritten, sects);
            break;
        default:
            r = SCPE_NOFNC;
            break;
        }
    }
else { /* Unaligned and/or partial sector transfers */
    t_lba sspsts = ctx->storage_sector_size/ctx->sector_size; /* sim sectors in a storage sector */
    t_lba tlba = lba & ~(sspsts - 1);
    t_seccnt tsects = sects + (lba - tlba);

    tbuf = (uint8*) malloc (sects*ctx->sector_size + 2*ctx->storage_sector_size);
    tsects = (tsects + (sspsts - 1)) & ~(sspsts - 1);
    if (sectswritten)
        *sectswritten = 0;
    if (tbuf == NULL)
        return SCPE_MEM;
    /* Partial Sector writes require a read-modify-write sequence for the partial sectors */
    if ((lba & (sspsts - 1)) ||
        (sects < sspsts))
        switch (DK_GET_FMT (uptr)) {                            /* case on format */
            case DKUF_F_VHD:                                    /* VHD format */
                sim_vhd_disk_rdsect (uptr, tlba, tbuf, NULL, sspsts);
                break;
            case DKUF_F_RAW:                                    /* Raw Physical Disk Access */
                sim_os_disk_rdsect (uptr, tlba, tbuf, NULL, sspsts);
                break;
            default:
                r = SCPE_NOFNC;
                break;
            }
    if ((tsects > sspsts) &&
        ((sects + lba - tlba) & (sspsts - 1)))
        switch (DK_GET_FMT (uptr)) {                            /* case on format */
            case DKUF_F_VHD:                                    /* VHD format */
                sim_vhd_disk_rdsect (uptr, tlba + tsects - sspsts, 
                                     tbuf + (tsects - sspsts) * ctx->sector_size, 
                                     NULL, sspsts);
                break;
            case DKUF_F_RAW:                                    /* Raw Physical Disk Access */
                sim_os_disk_rdsect (uptr, tlba + tsects - sspsts, 
                                    tbuf + (tsects - sspsts) * ctx->sector_size, 
                                    NULL, sspsts);
                break;
            default:
                r = SCPE_NOFNC;
                break;
            }
    sim_buf_copy_swapped (tbuf + (lba & (sspsts - 1)) * ctx->sector_size, 
                          buf, ctx->xfer_element_size, (sects * ctx->sector_size) / ctx->xfer_element_size);
    switch (DK_GET_FMT (uptr)) {                            /* case on format */
        case DKUF_F_VHD:                                    /* VHD format */
            r = sim_vhd_disk_wrsect (uptr, tlba, tbuf, sectswritten, tsects);
            break;
        case DKUF_F_RAW:                                    /* Raw Physical Disk Access */
            r = sim_os_disk_wrsect (uptr, tlba, tbuf, sectswritten, tsects);
            break;
        default:
            r = SCPE_NOFNC;
            break;
        }
    if ((r == SCPE_OK) && sectswritten) {
        *sectswritten -= (lba - tlba);
        if (*sectswritten > sects)
            *sectswritten = sects;
        }
    }
free (tbuf);
return r;
}

t_stat sim_disk_wrsect_a (UNIT *uptr, t_lba lba, uint8 *buf, t_seccnt *sectswritten, t_seccnt sects, DISK_PCALLBACK callback)
{
t_stat r = SCPE_OK;
AIO_CALLSETUP
    r =  sim_disk_wrsect (uptr, lba, buf, sectswritten, sects);
AIO_CALL(DOP_WSEC, lba, buf, sectswritten, sects, callback);
return r;
}

t_stat sim_disk_unload (UNIT *uptr)
{
switch (DK_GET_FMT (uptr)) {                            /* case on format */
    case DKUF_F_STD:                                    /* Simh */
    case DKUF_F_VHD:                                    /* VHD format */
        return sim_disk_detach (uptr);
    case DKUF_F_RAW:                                    /* Raw Physical Disk Access */
        return sim_os_disk_unload_raw (uptr->fileref);  /* remove/eject disk */
        break;
    default:
        return SCPE_NOFNC;
    }
}

static void _sim_disk_io_flush (UNIT *uptr)
{
    disk_context* ctx = (disk_context*) uptr->disk_ctx;
    if (ctx)
        ctx->flush();
    else
        disk_context::perform_flush(uptr);
}

void disk_context::perform_flush()
{
    perform_flush(uptr);
}

void disk_context::perform_flush(UNIT* uptr)
{
    switch (DK_GET_FMT (uptr))                     /* case on format */
    {
    case DKUF_F_STD:                                    /* Simh */
        fflush (uptr->fileref);
        break;
    case DKUF_F_VHD:                                    /* Virtual Disk */
        sim_vhd_disk_flush (uptr->fileref);
        break;
    case DKUF_F_RAW:                                    /* Physical */
        sim_os_disk_flush_raw (uptr->fileref);
        break;
    }
}

static t_stat _err_return (UNIT *uptr, t_stat stat)
{
    free (uptr->filename);
    uptr->filename = NULL;

    delete (disk_context*) uptr->disk_ctx;
    uptr->disk_ctx = NULL;

    return stat;
}

/* 
 * Reset disk:
 *
 * Caller must be either console thread or VCPU thread holding the lock for the device.
 * It is assumed that all units on the device share the same lock.
 */
t_stat sim_disk_reset (DEVICE* dptr)
{
    RUN_SCOPE_RSCX;
    UNIT* uptr;
    t_bool any_async = FALSE;
    uint32 k;

    if (rscx->thread_type == SIM_THREAD_TYPE_CONSOLE)
    {
        /* console thread can reset devices (all VCPUs are paused) */
    }
    else if (rscx->thread_type == SIM_THREAD_TYPE_CPU)
    {
        for (k = 0; k < dptr->numunits; k++)
        {
            uptr = dptr->units[k];
            if (uptr->flags & UNIT_ATT)
            {
                disk_context* ctx = (disk_context*) uptr->disk_ctx;
                if (ctx && ctx->asynch_io)
                {
                    any_async = TRUE;
                    break;
                }
            }
        }

        /*
         * On a multiprocessor VAX with asynchronous IO enabled, asynchronous IO completion is handled
         * by the primary processor that fetches units with AIO events from AIO event queue. Resetting
         * controller requires flushing all entries pending in async queue. To do it on a secondary CPU,
         * we'd have to send IPI to the primary and wait for the response. However primary may already
         * being stopped by the console, so console code responsible for pausing VCPUs would have to
         * check for pending flushing request and execute it (in fact it does, but we'd have to wait
         * either for the primary VCPU response or AIO queue going empty).
         *
         * More seriously, we are holding device lock, so the primary may go deadlocked with us if we
         * try to wait for it. On the other hand, we cannot release the lock (which may even have
         * acquisition depth > 1), even temporarily. Also, primary can already be right at this point
         * blocked inside uptr->lock().
         *
         * It may be possible to design a scheme to handle this situation, however it appears that
         * resetting controller by the secondary CPU is an exremely unlikely event in the first place.
         * We may implement handling of this case if it ever becomes a problem. 
         * For now just abort the simulator if it is encountered.  ToDo.
         */
        if (any_async && !cpu_unit->is_primary_cpu())
            panic("Disk controller device reset attempted by a secondary CPU");
    }
    else
    {
        panic("sim_disk_reset: invalid thread type");
    }

    dptr->a_reset_count++;

    for (k = 0; k < dptr->numunits; k++)
    {
        uptr = dptr->units[k];
        if ((uptr->flags & UNIT_ATT) && !(uptr->flags & UNIT_BUF) && uptr->fileref)
            if (uptr->io_flush)
                uptr->io_flush(uptr);
    }

    if (any_async)
    {
        if (rscx->thread_type == SIM_THREAD_TYPE_CONSOLE)
            sim_async_process_io_events_for_console();
        else
            sim_async_process_io_events(RUN_PASS, NULL, TRUE);
    }

    for (k = 0; k < dptr->numunits; k++)
    {
        uptr = dptr->units[k];
        sim_cancel(uptr);
    }

    return SCPE_OK;
}


t_stat sim_disk_attach (UNIT *uptr, char *cptr, size_t sector_size, size_t xfer_element_size, t_bool dontautosize, uint32 dbit, const char *dtype, uint32 pdp11tracksize)
{
disk_context* ctx;
DEVICE *dptr;
SMP_FILE* (*open_function)(const char *filename, const char *mode) = sim_fopen;
SMP_FILE* (*create_function)(const char *filename, t_addr desiredsize) = NULL;
t_addr (*size_function)(SMP_FILE* file);
t_stat (*storage_function)(SMP_FILE* file, uint32 *sector_size, uint32 *removable) = NULL;
t_bool created = FALSE;
t_bool auto_format = FALSE;
t_addr capac;

if (uptr->flags & UNIT_DIS)                             /* disabled? */
    return SCPE_UDIS;
if (!(uptr->flags & UNIT_ATTABLE))                      /* not attachable? */
    return SCPE_NOATT;
if ((dptr = find_dev_from_unit (uptr)) == NULL)
    return SCPE_NOATT;
if (sim_switches & SWMASK ('F')) {                      /* format spec? */
    char gbuf[CBUFSIZE];
    cptr = get_glyph (cptr, gbuf, 0);                   /* get spec */
    if (*cptr == 0)                                     /* must be more */
        return SCPE_2FARG;
    if (sim_disk_set_fmt (uptr, 0, gbuf, NULL) != SCPE_OK)
        return SCPE_ARG;
    }
if (sim_switches & SWMASK ('D')) {                      /* create difference disk? */
    char gbuf[CBUFSIZE];
    SMP_FILE* vhd;

    sim_switches = sim_switches & ~(SWMASK ('D'));
    cptr = get_glyph_nc (cptr, gbuf, 0);                /* get spec */
    if (*cptr == 0)                                     /* must be more */
        return SCPE_2FARG;
    vhd = sim_vhd_disk_create_diff (gbuf, cptr);
    if (vhd) {
        sim_vhd_disk_close (vhd);
        return sim_disk_attach (uptr, gbuf, sector_size, xfer_element_size, dontautosize, dbit, dtype, pdp11tracksize);
        }
    return SCPE_ARG;
    }
if (sim_switches & SWMASK ('C')) {                      /* create vhd disk & copy contents? */
    char gbuf[CBUFSIZE];
    SMP_FILE* vhd;
    int saved_sim_switches = sim_switches;
    int32 saved_sim_quiet = sim_quiet;
    t_stat r;

    sim_switches = sim_switches & ~(SWMASK ('C'));
    cptr = get_glyph_nc (cptr, gbuf, 0);                /* get spec */
    if (*cptr == 0)                                     /* must be more */
        return SCPE_2FARG;
    sim_switches |= SWMASK ('R') | SWMASK ('E');
    sim_quiet = TRUE;
    /* First open the source of the copy operation */
    r = sim_disk_attach (uptr, cptr, sector_size, xfer_element_size, dontautosize, dbit, dtype, pdp11tracksize);
    sim_quiet = saved_sim_quiet;
    if (r != SCPE_OK) {
        sim_switches = saved_sim_switches;
        return r;
        }
    if (!sim_quiet) 
        smp_printf ("%s%d: creating new virtual disk '%s'\n", sim_dname (dptr), sim_unit_index(uptr), gbuf);
    vhd = sim_vhd_disk_create (gbuf, uptr->capac);
    if (!vhd) {
        if (!sim_quiet)
            smp_printf ("%s%d: can't create virtual disk '%s'\n", sim_dname (dptr), sim_unit_index(uptr), gbuf);
        return SCPE_OPENERR;
        }
    else {
        uint8 *copy_buf = (uint8*) malloc (1024*1024);
        t_lba lba;
        t_seccnt sectors_per_buffer = (t_seccnt)((1024*1024)/sector_size);
        t_lba total_sectors = (t_lba)(uptr->capac/sector_size);
        t_seccnt sects = sectors_per_buffer;

        if (!copy_buf) {
            remove (gbuf);
            return SCPE_MEM;
            }
        for (lba = 0; (lba < total_sectors) && (r == SCPE_OK); lba += sects) {
            if (!sim_quiet)
                smp_printf ("%s%d: Copied %dMB.  %d%% complete.\r", sim_dname (dptr), sim_unit_index(uptr), (int)(((t_addr)lba*sector_size)/1000000), (int)((lba*100)/total_sectors));
            sects = sectors_per_buffer;
            if (lba + sects > total_sectors)
                sects = total_sectors - lba;
            r = sim_disk_rdsect (uptr, lba, copy_buf, NULL, sects);
            if (r == SCPE_OK) {
                uint32 saved_unit_flags = uptr->flags;
                SMP_FILE* save_unit_fileref = uptr->fileref;

                sim_disk_set_fmt (uptr, 0, "VHD", NULL);
                uptr->fileref = vhd;
                r = sim_disk_wrsect (uptr, lba, copy_buf, NULL, sects);
                uptr->fileref = save_unit_fileref;
                uptr->flags = saved_unit_flags;
                }
            }
        if (!sim_quiet)
            smp_printf ("\n%s%d: Copied %dMB. Done.\n", sim_dname (dptr), sim_unit_index(uptr), (int)(((t_addr)lba*sector_size)/1000000));
        free (copy_buf);
        created = TRUE;
        sim_vhd_disk_close (vhd);
        sim_disk_detach (uptr);
        strcpy (cptr, gbuf);
        sim_disk_set_fmt (uptr, 0, "VHD", NULL);
        sim_switches = saved_sim_switches;
        /* fall through and open/return the newly created & copied vhd */
        }
    }
switch (DK_GET_FMT (uptr)) {                            /* case on format */
    case DKUF_F_STD:                                    /* SIMH format */
        if (NULL == (uptr->fileref = sim_vhd_disk_open (cptr, "rb"))) {
            open_function = sim_fopen;
            size_function = sim_fsize_ex;
            break;
            }
        sim_disk_set_fmt (uptr, 0, "VHD", NULL);        /* set file format to VHD */
        sim_vhd_disk_close (uptr->fileref);             /* close vhd file*/
        auto_format = TRUE;
        uptr->fileref = NULL;
        /* Fall through to normal VHD processing */
    case DKUF_F_VHD:                                    /* VHD format */
        open_function = sim_vhd_disk_open;
        create_function = sim_vhd_disk_create;
        size_function = sim_vhd_disk_size;
        break;
    case DKUF_F_RAW:                                    /* Raw Physical Disk Access */
        open_function = sim_os_disk_open_raw;
        size_function = sim_os_disk_size_raw;
        storage_function = sim_os_disk_info_raw;
        break;
    default:
        return SCPE_IERR;
    }
uptr->filename = (char *) calloc (CBUFSIZE, sizeof (char));/* alloc name buf */
uptr->disk_ctx = ctx = new disk_context(uptr);
if (uptr->filename == NULL || uptr->disk_ctx == NULL)
    return _err_return (uptr, SCPE_MEM);
strncpy (uptr->filename, cptr, CBUFSIZE);               /* save name */
ctx->sector_size = (uint32)sector_size;                 /* save sector_size */
ctx->xfer_element_size = (uint32)xfer_element_size;     /* save xfer_element_size */
ctx->dptr = dptr;                                       /* save DEVICE pointer */
ctx->dbit = dbit;                                       /* save debug bit */
ctx->auto_format = auto_format;                         /* save that we auto selected format */
ctx->storage_sector_size = (uint32)sector_size;         /* Default */
if (sim_switches & SWMASK ('R')) {                      /* read only? */
    if ((uptr->flags & UNIT_ROABLE) == 0)               /* allowed? */
        return _err_return (uptr, SCPE_NORO);           /* no, error */
    uptr->fileref = open_function (cptr, "rb");         /* open rd only */
    if (uptr->fileref == NULL)                          /* open fail? */
        return _err_return (uptr, SCPE_OPENERR);        /* yes, error */
    uptr->flags = uptr->flags | UNIT_RO;                /* set rd only */
    if (!sim_quiet)
        smp_printf ("%s%d: unit is read only\n", sim_dname (dptr), sim_unit_index(uptr));
    }
else {                                                  /* normal */
    uptr->fileref = open_function (cptr, "rb+");        /* open r/w */
    if (uptr->fileref == NULL) {                        /* open fail? */
        if ((errno == EROFS) || (errno == EACCES)) {    /* read only? */
            if ((uptr->flags & UNIT_ROABLE) == 0)       /* allowed? */
                return _err_return (uptr, SCPE_NORO);   /* no error */
            uptr->fileref = open_function (cptr, "rb"); /* open rd only */
            if (uptr->fileref == NULL)                  /* open fail? */
                return _err_return (uptr, SCPE_OPENERR);/* yes, error */
            uptr->flags = uptr->flags | UNIT_RO;        /* set rd only */
            if (!sim_quiet)
                smp_printf ("%s%d: unit is read only\n", sim_dname (dptr), sim_unit_index(uptr));
            }
        else {                                          /* doesn't exist */
            if (sim_switches & SWMASK ('E'))            /* must exist? */
                return _err_return (uptr, SCPE_OPENERR); /* yes, error */
            if (create_function)
                uptr->fileref = create_function (cptr, uptr->capac);/* create new file */
            else
                uptr->fileref = open_function (cptr, "wb+");/* open new file */
            if (uptr->fileref == NULL)                  /* open fail? */
                return _err_return (uptr, SCPE_OPENERR);/* yes, error */
            if (!sim_quiet) 
                smp_printf ("%s%d: creating new file\n", sim_dname (dptr), sim_unit_index(uptr));
            created = TRUE;
            }
        }                                               /* end if null */
    }                                                   /* end else */
if (DK_GET_FMT (uptr) == DKUF_F_VHD) {
    if ((created) && dtype)
        sim_vhd_disk_set_dtype (uptr->fileref, dtype);
    if (dtype && strcmp (dtype, sim_vhd_disk_get_dtype (uptr->fileref))) {
        char cmd[32];

        sprintf (cmd, "%s%d %s", dptr->name, sim_unit_index(uptr), sim_vhd_disk_get_dtype (uptr->fileref));
        set_cmd (0, cmd);
        }
    }
uptr->flags = uptr->flags | UNIT_ATT;
uptr->pos = 0;

/* Get Device attributes if they are available */
if (storage_function)
    storage_function (uptr->fileref, &ctx->storage_sector_size, &ctx->removable);

if (created) {
    t_stat r = SCPE_OK;
    uint8 *secbuf = (uint8*) calloc (1, ctx->sector_size);       /* alloc temp sector buf */

    /* 
       On a newly created disk, we write a zero sector to the last and the 
       first sectors.  This serves 3 purposes: 
         1) it avoids strange allocation delays writing newly allocated 
            storage at the end of the disk during simulator operation
         2) it allocates storage for the whole disk at creation time to 
            avoid strange failures which may happen during simulator execution
            if the containing disk is full
         3) it leaves a Sinh Format disk at the intended size so it may 
            subsequently be autosized with the correct size.
    */
    if (secbuf == NULL)
        r = SCPE_MEM;
    if (r == SCPE_OK)
        r = sim_disk_wrsect (uptr, (t_lba)((uptr->capac - ctx->sector_size)/ctx->sector_size), secbuf, NULL, 1); /* Write Last Sector */
    if (r == SCPE_OK)
        r = sim_disk_wrsect (uptr, (t_lba)(0), secbuf, NULL, 1); /* Write First Sector */
    free (secbuf);
    if (r != SCPE_OK) {
        sim_disk_detach (uptr);                         /* report error now */
        remove (cptr);                                  /* remove the create file */
        return SCPE_OPENERR;
        }
    if (pdp11tracksize)
        sim_disk_pdp11_bad_block (uptr, pdp11tracksize);
    }

capac = size_function (uptr->fileref);
if (capac && (capac != (t_addr)-1))
    if (dontautosize) {
        if ((capac < uptr->capac) && (DKUF_F_STD != DK_GET_FMT (uptr))) {
            if (!sim_quiet) {
                smp_printf ("%s%d: non expandable disk %s is smaller than simulated device (", sim_dname (dptr), sim_unit_index(uptr), cptr);
                fprint_val (smp_stdout, capac, 10, T_ADDR_W, PV_LEFT);
                smp_printf (" < ");
                fprint_val (smp_stdout, uptr->capac, 10, T_ADDR_W, PV_LEFT);
                smp_printf (")\n");
                }
            }
        }
    else
        if ((capac > uptr->capac) || (DKUF_F_STD != DK_GET_FMT (uptr)))
            uptr->capac = capac;

sim_disk_set_async (uptr, 0);
uptr->io_flush = _sim_disk_io_flush;

return SCPE_OK;
}

t_stat sim_disk_detach (UNIT *uptr)
{
if (uptr == NULL)
    return SCPE_IERR;

disk_context* ctx = (disk_context*) uptr->disk_ctx;
int (*close_function)(SMP_FILE* f) = NULL;
SMP_FILE* fileref = uptr->fileref;
DEVICE *dptr;
t_bool auto_format;

switch (DK_GET_FMT (uptr)) {                            /* case on format */
    case DKUF_F_STD:                                    /* Simh */
        close_function = fclose;
        break;
    case DKUF_F_VHD:                                    /* Virtual Disk */
        close_function = sim_vhd_disk_close;
        break;
    case DKUF_F_RAW:                                    /* Physical */
        close_function = sim_os_disk_close_raw;
        break;
        }
if (!(uptr->flags & UNIT_ATTABLE))                      /* attachable? */
    return SCPE_NOATT;
if (!(uptr->flags & UNIT_ATT))                          /* attached? */
    return SCPE_OK;
if ((dptr = find_dev_from_unit (uptr)) == NULL)
    return SCPE_OK;
auto_format = ctx->auto_format;

if (uptr->io_flush)
    uptr->io_flush (uptr);                              /* flush buffered data */

sim_disk_clr_async (uptr);

uptr->flags = uptr->flags & ~(UNIT_ATT | UNIT_RO | UNIT_RAW);
free (uptr->filename);
uptr->filename = NULL;
uptr->fileref = NULL;
free (uptr->disk_ctx);
uptr->disk_ctx = NULL;
uptr->io_flush = NULL;
if (auto_format)
    sim_disk_set_fmt (uptr, 0, "SIMH", NULL);           /* restore file format */
if (close_function (fileref) == EOF)
    return SCPE_IOERR;
return SCPE_OK;
}

/* Factory bad block table creation routine

   This routine writes a DEC standard 044 compliant bad block table on the
   last track of the specified unit.  The bad block table consists of 10
   repetitions of the same table, formatted as follows:

        words 0-1       pack id number
        words 2-3       cylinder/sector/surface specifications
         :
        words n-n+1     end of table (-1,-1)

   Inputs:
        uptr    =       pointer to unit
        sec     =       number of sectors per surface
   Outputs:
        sta     =       status code
*/

t_stat sim_disk_pdp11_bad_block (UNIT *uptr, int32 sec)
{
disk_context* ctx = (disk_context*) uptr->disk_ctx;
DEVICE *dptr;
int32 i;
t_addr da;
int32 wds = ctx->sector_size/sizeof (uint16);
uint16 *buf;

if ((sec < 2) || (wds < 16))
    return SCPE_ARG;
if ((uptr->flags & UNIT_ATT) == 0)
    return SCPE_UNATT;
if (uptr->flags & UNIT_RO)
    return SCPE_RO;
if ((dptr = find_dev_from_unit (uptr)) == NULL)
    return SCPE_NOATT;
if ((dptr->dwidth / dptr->aincr) <= 8)                  /* Must be Word oriented Capacity */
    return SCPE_IERR;
if (!get_yn ("Overwrite last track? [N]", FALSE))
    return SCPE_OK;
if ((buf = (uint16 *) malloc (wds * sizeof (uint16))) == NULL)
    return SCPE_MEM;
buf[0] = buf[1] = 012345u;
buf[2] = buf[3] = 0;
for (i = 4; i < wds; i++)
    buf[i] = 0177777u;
da = uptr->capac - (sec * wds);
for (i = 0; (i < sec) && (i < 10); i++, da += wds)
    if (sim_disk_wrsect (uptr, (t_lba)(da/wds), (uint8*)buf, NULL, 1)) {
        free (buf);
        return SCPE_IOERR;
        }
free (buf);
return SCPE_OK;
}

void sim_disk_data_trace(UNIT *uptr, const uint8 *data, size_t lba, size_t len, const char* txt, int detail, uint32 reason)
{
disk_context* ctx = (disk_context*) uptr->disk_ctx;

if (ctx->dptr->dctrl & reason) {
    sim_debug (reason, ctx->dptr, "%s%d %s lbn: %08X len: %08X\n", ctx->dptr->name, sim_unit_index(uptr), txt, lba, len);
    if (detail) {
        size_t i, same, group, sidx, oidx;
        char outbuf[80], strbuf[18];
        static const char hex[] = "0123456789ABCDEF";

        for (i=same=0; i<len; i += 16) {
            if ((i > 0) && (0 == memcmp (&data[i], &data[i-16], 16))) {
                ++same;
                continue;
            }
            if (same > 0) {
                sim_debug (reason, ctx->dptr, "%04X thru %04X same as above\n", i-(16*same), i-1);
                same = 0;
            }
            group = (((len - i) > 16) ? 16 : (len - i));
            for (sidx=oidx=0; sidx<group; ++sidx) {
                outbuf[oidx++] = ' ';
                outbuf[oidx++] = hex[(data[i+sidx]>>4)&0xf];
                outbuf[oidx++] = hex[data[i+sidx]&0xf];
                if (isprint (data[i+sidx]))
                    strbuf[sidx] = data[i+sidx];
                else
                    strbuf[sidx] = '.';
            }
            outbuf[oidx] = '\0';
            strbuf[sidx] = '\0';
            sim_debug (reason, ctx->dptr, "%04X%-48s %s\n", i, outbuf, strbuf);
          }
          if (same > 0)
              sim_debug (reason, ctx->dptr, "%04X thru %04X same as above\n", i-(16*same), len-1);
        }
    }
}


/* OS Specific RAW Disk I/O support */

#if defined _WIN32

static void _set_errno_from_status (DWORD dwStatus)
{
switch (dwStatus) {
    case ERROR_FILE_NOT_FOUND:    case ERROR_PATH_NOT_FOUND:
    case ERROR_INVALID_DRIVE:     case ERROR_NO_MORE_FILES:
    case ERROR_BAD_NET_NAME:      case ERROR_BAD_NETPATH:
    case ERROR_BAD_PATHNAME:      case ERROR_FILENAME_EXCED_RANGE:
        errno = ENOENT;
        return;
    case ERROR_INVALID_ACCESS:    case ERROR_INVALID_DATA:
    case ERROR_INVALID_FUNCTION:  case ERROR_INVALID_PARAMETER:
    case ERROR_NEGATIVE_SEEK:
        errno = EINVAL;
        return;
    case ERROR_ARENA_TRASHED:     case ERROR_NOT_ENOUGH_MEMORY:
    case ERROR_INVALID_BLOCK:     case ERROR_NOT_ENOUGH_QUOTA:
        errno = ENOMEM;
        return;
    case ERROR_TOO_MANY_OPEN_FILES:
        errno = EMFILE;
        return;
    case ERROR_ACCESS_DENIED:     case ERROR_CURRENT_DIRECTORY:
    case ERROR_LOCK_VIOLATION:    case ERROR_NETWORK_ACCESS_DENIED:
    case ERROR_CANNOT_MAKE:       case ERROR_FAIL_I24:
    case ERROR_DRIVE_LOCKED:      case ERROR_SEEK_ON_DEVICE:
    case ERROR_NOT_LOCKED:        case ERROR_LOCK_FAILED:
        errno = EACCES;
        return;
    case ERROR_ALREADY_EXISTS:    case ERROR_FILE_EXISTS:
        errno = EEXIST;
        return;
    case ERROR_INVALID_HANDLE:    case ERROR_INVALID_TARGET_HANDLE:
    case ERROR_DIRECT_ACCESS_HANDLE:
        errno = EBADF;
        return;
    case ERROR_DIR_NOT_EMPTY:
        errno = ENOTEMPTY;
        return;
    case ERROR_BAD_ENVIRONMENT:
        errno = E2BIG;
        return;
    case ERROR_BAD_FORMAT:
        errno = ENOEXEC;
        return;
    case ERROR_NOT_SAME_DEVICE:
        errno = EXDEV;
        return;
    case ERROR_BROKEN_PIPE:
        errno = EPIPE;
        return;
    case ERROR_DISK_FULL:
        errno = ENOSPC;
        return;
    case ERROR_WAIT_NO_CHILDREN:  case ERROR_CHILD_NOT_COMPLETE:
        errno = ECHILD;
        return;
    case ERROR_NO_PROC_SLOTS:     case ERROR_MAX_THRDS_REACHED:
    case ERROR_NESTING_NOT_ALLOWED:
        errno = EAGAIN;
        return;
    }
if ((dwStatus >= ERROR_WRITE_PROTECT) && (dwStatus <= ERROR_SHARING_BUFFER_EXCEEDED)) {
    errno = EACCES;
    return;
    }
if ((dwStatus >= ERROR_INVALID_STARTING_CODESEG) && (dwStatus <= ERROR_INFLOOP_IN_RELOC_CHAIN)) {
    errno = ENOEXEC;
    return;
    }
errno = EINVAL;
}
#include <winioctl.h>
struct _device_type {
    int32 Type;
    char *desc;
    } DeviceTypes[] = {
        {FILE_DEVICE_8042_PORT,             "8042_PORT"},
        {FILE_DEVICE_ACPI,                  "ACPI"},
        {FILE_DEVICE_BATTERY,               "BATTERY"},
        {FILE_DEVICE_BEEP,                  "BEEP"},
#ifdef FILE_DEVICE_BLUETOOTH
        {FILE_DEVICE_BLUETOOTH,             "BLUETOOTH"},
#endif
        {FILE_DEVICE_BUS_EXTENDER,          "BUS_EXTENDER"},
        {FILE_DEVICE_CD_ROM,                "CD_ROM"},
        {FILE_DEVICE_CD_ROM_FILE_SYSTEM,    "CD_ROM_FILE_SYSTEM"},
        {FILE_DEVICE_CHANGER,               "CHANGER"},
        {FILE_DEVICE_CONTROLLER,            "CONTROLLER"},
#ifdef FILE_DEVICE_CRYPT_PROVIDER
        {FILE_DEVICE_CRYPT_PROVIDER,        "CRYPT_PROVIDER"},
#endif
        {FILE_DEVICE_DATALINK,              "DATALINK"},
        {FILE_DEVICE_DFS,                   "DFS"},
        {FILE_DEVICE_DFS_FILE_SYSTEM,       "DFS_FILE_SYSTEM"},
        {FILE_DEVICE_DFS_VOLUME,            "DFS_VOLUME"},
        {FILE_DEVICE_DISK,                  "DISK"},
        {FILE_DEVICE_DISK_FILE_SYSTEM,      "DISK_FILE_SYSTEM"},
        {FILE_DEVICE_DVD,                   "DVD"},
        {FILE_DEVICE_FILE_SYSTEM,           "FILE_SYSTEM"},
#ifdef FILE_DEVICE_FIPS
        {FILE_DEVICE_FIPS,                  "FIPS"},
#endif
        {FILE_DEVICE_FULLSCREEN_VIDEO,      "FULLSCREEN_VIDEO"},
#ifdef FILE_DEVICE_INFINIBAND
        {FILE_DEVICE_INFINIBAND,            "INFINIBAND"},
#endif
        {FILE_DEVICE_INPORT_PORT,           "INPORT_PORT"},
        {FILE_DEVICE_KEYBOARD,              "KEYBOARD"},
        {FILE_DEVICE_KS,                    "KS"},
        {FILE_DEVICE_KSEC,                  "KSEC"},
        {FILE_DEVICE_MAILSLOT,              "MAILSLOT"},
        {FILE_DEVICE_MASS_STORAGE,          "MASS_STORAGE"},
        {FILE_DEVICE_MIDI_IN,               "MIDI_IN"},
        {FILE_DEVICE_MIDI_OUT,              "MIDI_OUT"},
        {FILE_DEVICE_MODEM,                 "MODEM"},
        {FILE_DEVICE_MOUSE,                 "MOUSE"},
        {FILE_DEVICE_MULTI_UNC_PROVIDER,    "MULTI_UNC_PROVIDER"},
        {FILE_DEVICE_NAMED_PIPE,            "NAMED_PIPE"},
        {FILE_DEVICE_NETWORK,               "NETWORK"},
        {FILE_DEVICE_NETWORK_BROWSER,       "NETWORK_BROWSER"},
        {FILE_DEVICE_NETWORK_FILE_SYSTEM,   "NETWORK_FILE_SYSTEM"},
        {FILE_DEVICE_NETWORK_REDIRECTOR,    "NETWORK_REDIRECTOR"},
        {FILE_DEVICE_NULL,                  "NULL"},
        {FILE_DEVICE_PARALLEL_PORT,         "PARALLEL_PORT"},
        {FILE_DEVICE_PHYSICAL_NETCARD,      "PHYSICAL_NETCARD"},
        {FILE_DEVICE_PRINTER,               "PRINTER"},
        {FILE_DEVICE_SCANNER,               "SCANNER"},
        {FILE_DEVICE_SCREEN,                "SCREEN"},
        {FILE_DEVICE_SERENUM,               "SERENUM"},
        {FILE_DEVICE_SERIAL_MOUSE_PORT,     "SERIAL_MOUSE_PORT"},
        {FILE_DEVICE_SERIAL_PORT,           "SERIAL_PORT"},
        {FILE_DEVICE_SMARTCARD,             "SMARTCARD"},
        {FILE_DEVICE_SMB,                   "SMB"},
        {FILE_DEVICE_SOUND,                 "SOUND"},
        {FILE_DEVICE_STREAMS,               "STREAMS"},
        {FILE_DEVICE_TAPE,                  "TAPE"},
        {FILE_DEVICE_TAPE_FILE_SYSTEM,      "TAPE_FILE_SYSTEM"},
        {FILE_DEVICE_TERMSRV,               "TERMSRV"},
        {FILE_DEVICE_TRANSPORT,             "TRANSPORT"},
        {FILE_DEVICE_UNKNOWN,               "UNKNOWN"},
        {FILE_DEVICE_VDM,                   "VDM"},
        {FILE_DEVICE_VIDEO,                 "VIDEO"},
        {FILE_DEVICE_VIRTUAL_DISK,          "VIRTUAL_DISK"},
#ifdef FILE_DEVICE_VMBUS
        {FILE_DEVICE_VMBUS,                 "VMBUS"},
#endif
        {FILE_DEVICE_WAVE_IN,               "WAVE_IN"},
        {FILE_DEVICE_WAVE_OUT,              "WAVE_OUT"},
#ifdef FILE_DEVICE_WPD
        {FILE_DEVICE_WPD,                   "WPD"},
#endif
        {0,                                 NULL}};

static const char *_device_type_name (int DeviceType)
{
int i;

for (i=0; DeviceTypes[i].desc; i++)
    if (DeviceTypes[i].Type == DeviceType)
        return DeviceTypes[i].desc;
return "Unknown";
}

static t_stat sim_os_disk_implemented_raw (void)
{
return SCPE_OK;
}

static SMP_FILE* sim_os_disk_open_raw (const char *rawdevicename, const char *openmode)
{
HANDLE Handle;
DWORD DesiredAccess = 0;

if (strchr (openmode, 'r'))
    DesiredAccess |= GENERIC_READ;
if (strchr (openmode, 'w') || strchr (openmode, '+'))
    DesiredAccess |= GENERIC_WRITE;
Handle = CreateFileA (rawdevicename, DesiredAccess, FILE_SHARE_READ|FILE_SHARE_WRITE, NULL, OPEN_EXISTING, FILE_FLAG_RANDOM_ACCESS|FILE_FLAG_WRITE_THROUGH, NULL);
if (Handle == INVALID_HANDLE_VALUE) {
    _set_errno_from_status (GetLastError ());
    return NULL;
    }
return (SMP_FILE* )Handle;
}

static int sim_os_disk_close_raw (SMP_FILE* f)
{
if (!CloseHandle ((HANDLE)f)) {
    _set_errno_from_status (GetLastError ());
    return EOF;
    }
return 0;
}

static void sim_os_disk_flush_raw (SMP_FILE* f)
{
FlushFileBuffers ((HANDLE)f);
}

static t_addr sim_os_disk_size_raw (SMP_FILE* Disk)
{
DWORD IoctlReturnSize;
LARGE_INTEGER Size;
WINBASEAPI BOOL WINAPI GetFileSizeEx(HANDLE hFile, PLARGE_INTEGER lpFileSize);

if (GetFileSizeEx((HANDLE)Disk, &Size))
    return (t_addr)(Size.QuadPart);
#ifdef IOCTL_STORAGE_READ_CAPACITY
if (1) {
    STORAGE_READ_CAPACITY S;

    ZeroMemory (&S, sizeof (S));
    S.Version = sizeof (STORAGE_READ_CAPACITY);
    if (DeviceIoControl((HANDLE)Disk,                      /* handle to volume */
                         IOCTL_STORAGE_READ_CAPACITY,      /* dwIoControlCode */
                         NULL,                             /* lpInBuffer */
                         0,                                /* nInBufferSize */
                         (LPVOID) &S,                      /* output buffer */
                         (DWORD) sizeof(S),                /* size of output buffer */
                         (LPDWORD) &IoctlReturnSize,       /* number of bytes returned */
                         (LPOVERLAPPED) NULL))             /* OVERLAPPED structure */
        return (t_addr)(S.DiskLength.QuadPart);
    }
#endif
#ifdef IOCTL_DISK_GET_DRIVE_GEOMETRY_EX
if (1) {
    DISK_GEOMETRY_EX G;

    ZeroMemory (&G, sizeof (G));
    if (DeviceIoControl((HANDLE)Disk,                      /* handle to volume */
                         IOCTL_DISK_GET_DRIVE_GEOMETRY_EX, /* dwIoControlCode */
                         NULL,                             /* lpInBuffer */
                         0,                                /* nInBufferSize */
                         (LPVOID) &G,                      /* output buffer */
                         (DWORD) sizeof(G),                /* size of output buffer */
                         (LPDWORD) &IoctlReturnSize,       /* number of bytes returned */
                         (LPOVERLAPPED) NULL))             /* OVERLAPPED structure */
        return (t_addr)(G.DiskSize.QuadPart);
    }
#endif
#ifdef IOCTL_DISK_GET_DRIVE_GEOMETRY
if (1) {
    DISK_GEOMETRY G;

    if (DeviceIoControl((HANDLE)Disk,                      /* handle to volume */
                         IOCTL_DISK_GET_DRIVE_GEOMETRY,    /* dwIoControlCode */
                         NULL,                             /* lpInBuffer */
                         0,                                /* nInBufferSize */
                         (LPVOID) &G,                      /* output buffer */
                         (DWORD) sizeof(G),                /* size of output buffer */
                         (LPDWORD) &IoctlReturnSize,       /* number of bytes returned */
                         (LPOVERLAPPED) NULL))             /* OVERLAPPED structure */
        return (t_addr)(G.Cylinders.QuadPart*G.TracksPerCylinder*G.SectorsPerTrack*G.BytesPerSector);
    }
#endif
_set_errno_from_status (GetLastError ());
return (t_addr)-1;
}

static t_stat sim_os_disk_unload_raw (SMP_FILE* Disk)
{
#ifdef IOCTL_STORAGE_EJECT_MEDIA
DWORD BytesReturned;
uint32 Removable = FALSE;

sim_os_disk_info_raw (Disk, NULL, &Removable);
if (Removable) {
    if (!DeviceIoControl((HANDLE)Disk,                  /* handle to disk */
                         IOCTL_STORAGE_EJECT_MEDIA,     /* dwIoControlCode */
                         NULL,                          /* lpInBuffer */
                         0,                             /* nInBufferSize */
                         NULL,                          /* lpOutBuffer */
                         0,                             /* nOutBufferSize */
                         (LPDWORD) &BytesReturned,      /* number of bytes returned */
                         (LPOVERLAPPED) NULL)) {        /* OVERLAPPED structure */
        _set_errno_from_status (GetLastError ());
        return SCPE_IOERR;
        }
    }
return SCPE_OK;
#else
return SCPE_NOFNC;
#endif
}

static t_bool sim_os_disk_isavailable_raw (SMP_FILE* Disk)
{
#ifdef IOCTL_STORAGE_EJECT_MEDIA
DWORD BytesReturned;
uint32 Removable = FALSE;

sim_os_disk_info_raw (Disk, NULL, &Removable);
if (Removable) {
    if (!DeviceIoControl((HANDLE)Disk,                  /* handle to disk */
                         IOCTL_STORAGE_CHECK_VERIFY,    /* dwIoControlCode */
                         NULL,                          /* lpInBuffer */
                         0,                             /* nInBufferSize */
                         NULL,                          /* lpOutBuffer */
                         0,                             /* nOutBufferSize */
                         (LPDWORD) &BytesReturned,      /* number of bytes returned */
                         (LPOVERLAPPED) NULL)) {        /* OVERLAPPED structure */
        _set_errno_from_status (GetLastError ());
        return FALSE;
        }
    }
#endif
return TRUE;
}

static t_stat sim_os_disk_info_raw (SMP_FILE* Disk, uint32 *sector_size, uint32 *removable)
{
DWORD IoctlReturnSize;
#ifndef __GNUC__
STORAGE_DEVICE_NUMBER Device;

ZeroMemory (&Device, sizeof (Device));
if (DeviceIoControl((HANDLE)Disk,                      /* handle to volume */
                     IOCTL_STORAGE_GET_DEVICE_NUMBER,  /* dwIoControlCode */
                     NULL,                             /* lpInBuffer */
                     0,                                /* nInBufferSize */
                     (LPVOID) &Device,                 /* output buffer */
                     (DWORD) sizeof(Device),           /* size of output buffer */
                     (LPDWORD) &IoctlReturnSize,       /* number of bytes returned */
                     (LPOVERLAPPED) NULL))             /* OVERLAPPED structure */
     smp_printf ("Device OK - Type: %s, Number: %d\n", _device_type_name (Device.DeviceType), Device.DeviceNumber);
#endif

if (sector_size)
    *sector_size = 512;
if (removable)
    *removable = 0;
#ifdef IOCTL_STORAGE_READ_CAPACITY
if (1) {
    STORAGE_READ_CAPACITY S;

    ZeroMemory (&S, sizeof (S));
    S.Version = sizeof (STORAGE_READ_CAPACITY);
    if (DeviceIoControl((HANDLE)Disk,                      /* handle to volume */
                         IOCTL_STORAGE_READ_CAPACITY,      /* dwIoControlCode */
                         NULL,                             /* lpInBuffer */
                         0,                                /* nInBufferSize */
                         (LPVOID) &S,                      /* output buffer */
                         (DWORD) sizeof(S),                /* size of output buffer */
                         (LPDWORD) &IoctlReturnSize,       /* number of bytes returned */
                         (LPOVERLAPPED) NULL))             /* OVERLAPPED structure */
        if (sector_size)
            *sector_size = S.BlockLength;
    }
#endif
#ifdef IOCTL_DISK_GET_DRIVE_GEOMETRY_EX
if (1) {
    DISK_GEOMETRY_EX G;

    ZeroMemory (&G, sizeof (G));
    if (DeviceIoControl((HANDLE)Disk,                      /* handle to volume */
                         IOCTL_DISK_GET_DRIVE_GEOMETRY_EX, /* dwIoControlCode */
                         NULL,                             /* lpInBuffer */
                         0,                                /* nInBufferSize */
                         (LPVOID) &G,                      /* output buffer */
                         (DWORD) sizeof(G),                /* size of output buffer */
                         (LPDWORD) &IoctlReturnSize,       /* number of bytes returned */
                         (LPOVERLAPPED) NULL))             /* OVERLAPPED structure */
        if (sector_size)
            *sector_size = G.Geometry.BytesPerSector;
    }
#endif
#ifdef IOCTL_DISK_GET_DRIVE_GEOMETRY
if (1) {
    DISK_GEOMETRY G;

    if (DeviceIoControl((HANDLE)Disk,                      /* handle to volume */
                         IOCTL_DISK_GET_DRIVE_GEOMETRY,    /* dwIoControlCode */
                         NULL,                             /* lpInBuffer */
                         0,                                /* nInBufferSize */
                         (LPVOID) &G,                      /* output buffer */
                         (DWORD) sizeof(G),                /* size of output buffer */
                         (LPDWORD) &IoctlReturnSize,       /* number of bytes returned */
                         (LPOVERLAPPED) NULL))             /* OVERLAPPED structure */
        if (sector_size)
            *sector_size = G.BytesPerSector;
    }
#endif
#ifdef IOCTL_STORAGE_GET_HOTPLUG_INFO
if (1) {
    STORAGE_HOTPLUG_INFO H;

    ZeroMemory (&H, sizeof (H));
    if (DeviceIoControl((HANDLE)Disk,                      /* handle to volume */
                         IOCTL_STORAGE_GET_HOTPLUG_INFO,   /* dwIoControlCode */
                         NULL,                             /* lpInBuffer */
                         0,                                /* nInBufferSize */
                         (LPVOID) &H,                      /* output buffer */
                         (DWORD) sizeof(H),                /* size of output buffer */
                         (LPDWORD) &IoctlReturnSize,       /* number of bytes returned */
                         (LPOVERLAPPED) NULL))             /* OVERLAPPED structure */
        if (removable)
            *removable = H.MediaRemovable;
    }
#endif
if (removable && *removable)
    smp_printf ("Removable Device\n");
return SCPE_OK;
}

static t_stat sim_os_disk_rdsect (UNIT *uptr, t_lba lba, uint8 *buf, t_seccnt *sectsread, t_seccnt sects)
{
OVERLAPPED pos;
disk_context* ctx = (disk_context*) uptr->disk_ctx;
long long addr;

sim_debug (ctx->dbit, ctx->dptr, "sim_os_disk_rdsect(unit=%d, lba=0x%X, sects=%d)\n", sim_unit_index(uptr), lba, sects);

addr = ((long long)lba) * ctx->sector_size;
memset (&pos, 0, sizeof (pos));
pos.Offset = (DWORD)addr;
pos.OffsetHigh = (DWORD)(addr >> 32);
if (ReadFile ((HANDLE)(uptr->fileref), buf, sects * ctx->sector_size, (LPDWORD)sectsread, &pos)) {
    if (sectsread)
        *sectsread /= ctx->sector_size;
    return SCPE_OK;
    }
_set_errno_from_status (GetLastError ());
return SCPE_IOERR;
}

static t_stat sim_os_disk_wrsect (UNIT *uptr, t_lba lba, uint8 *buf, t_seccnt *sectswritten, t_seccnt sects)
{
OVERLAPPED pos;
disk_context* ctx = (disk_context*) uptr->disk_ctx;
long long addr;

sim_debug (ctx->dbit, ctx->dptr, "sim_os_disk_wrsect(unit=%d, lba=0x%X, sects=%d)\n", sim_unit_index(uptr), lba, sects);

addr = ((long long)lba) * ctx->sector_size;
memset (&pos, 0, sizeof (pos));
pos.Offset = (DWORD)addr;
pos.OffsetHigh = (DWORD)(addr >> 32);
if (WriteFile ((HANDLE)(uptr->fileref), buf, sects * ctx->sector_size, (LPDWORD)sectswritten, &pos)) {
    if (sectswritten)
        *sectswritten /= ctx->sector_size;
    return SCPE_OK;
    }
_set_errno_from_status (GetLastError ());
return SCPE_IOERR;
}

#elif defined (__linux) || defined (__sun__) || defined(__APPLE__)

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

static t_stat sim_os_disk_implemented_raw (void)
{
return SCPE_OK;
}

static SMP_FILE* sim_os_disk_open_raw (const char *rawdevicename, const char *openmode)
{
int fd;
int mode = 0;

if (strchr (openmode, 'r') && (strchr (openmode, '+') || strchr (openmode, 'w')))
    mode = O_RDWR;
else
    if (strchr (openmode, 'r'))
        mode = O_RDONLY;
#ifdef O_LARGEFILE
mode |= O_LARGEFILE;
#endif
#ifdef O_DSYNC
mode |= O_DSYNC;
#endif 
return (SMP_FILE* )((long)open (rawdevicename, mode, 0));
}

static int sim_os_disk_close_raw (SMP_FILE* f)
{
return close ((int)((long)f));
}

static void sim_os_disk_flush_raw (SMP_FILE* f)
{
fsync ((int)((long)f));
}

static t_addr sim_os_disk_size_raw (SMP_FILE* f)
{
#if defined(__APPLE__)
struct stat statb;

if (sizeof(statb.st_size) != sizeof(t_addr) || fstat ((int)((long)f), &statb))
    return (t_addr)-1;
return (t_addr)statb.st_size;
#else
struct stat64 statb;

if (fstat64 ((int)((long)f), &statb))
    return (t_addr)-1;
return (t_addr)statb.st_size;
#endif
}

static t_stat sim_os_disk_unload_raw (SMP_FILE* f)
{
return SCPE_IOERR;
}

static t_bool sim_os_disk_isavailable_raw (SMP_FILE* Disk)
{
return TRUE;
}

static t_stat sim_os_disk_rdsect (UNIT *uptr, t_lba lba, uint8 *buf, t_seccnt *sectsread, t_seccnt sects)
{
disk_context* ctx = (disk_context*) uptr->disk_ctx;
off_t addr;
ssize_t bytesread;

sim_debug (ctx->dbit, ctx->dptr, "sim_os_disk_rdsect(unit=%d, lba=0x%X, sects=%d)\n", sim_unit_index(uptr), lba, sects);

addr = ((off_t)lba) * ctx->sector_size;
bytesread = pread((int)((long)uptr->fileref), buf, sects * ctx->sector_size, addr); 
if (bytesread < 0) {
    if (sectsread)
        *sectsread = 0;
    return SCPE_IOERR;
    }
if (sectsread)
    *sectsread = bytesread / ctx->sector_size;
return SCPE_OK;
}

static t_stat sim_os_disk_wrsect (UNIT *uptr, t_lba lba, uint8 *buf, t_seccnt *sectswritten, t_seccnt sects)
{
disk_context* ctx = (disk_context*) uptr->disk_ctx;
off_t addr;
ssize_t byteswritten;

sim_debug (ctx->dbit, ctx->dptr, "sim_os_disk_wrsect(unit=%d, lba=0x%X, sects=%d)\n", sim_unit_index(uptr), lba, sects);

addr = ((off_t)lba) * ctx->sector_size;
byteswritten = pwrite((int)((long)uptr->fileref), buf, sects * ctx->sector_size, addr); 
if (byteswritten < 0) {
    if (sectswritten)
        *sectswritten = 0;
    return SCPE_IOERR;
    }
if (sectswritten)
    *sectswritten = byteswritten / ctx->sector_size;
return SCPE_OK;
}

static t_stat sim_os_disk_info_raw (SMP_FILE* f, uint32 *sector_size, uint32 *removable)
{
if (sector_size)
    *sector_size = 512;
if (removable)
    *removable = 0;
return SCPE_OK;
}

#else
/*============================================================================*/
/*                        Non-implemented versions                            */
/*============================================================================*/

static t_stat sim_os_disk_implemented_raw (void)
{
return SCPE_NOFNC;
}

static SMP_FILE* sim_os_disk_open_raw (const char *rawdevicename, const char *openmode)
{
return NULL;
}

static int sim_os_disk_close_raw (SMP_FILE* f)
{
return EOF;
}

static void sim_os_disk_flush_raw (SMP_FILE* f)
{
}

static t_addr sim_os_disk_size_raw (SMP_FILE* f)
{
return (t_addr)-1;
}

static t_stat sim_os_disk_unload_raw (SMP_FILE* f)
{
return SCPE_NOFNC;
}

static t_bool sim_os_disk_isavailable_raw (SMP_FILE* Disk)
{
return FALSE;
}

static t_stat sim_os_disk_rdsect (UNIT *uptr, t_lba lba, uint8 *buf, t_seccnt *sectsread, t_seccnt sects)
{
return SCPE_NOFNC;
}

static t_stat sim_os_disk_wrsect (UNIT *uptr, t_lba lba, uint8 *buf, t_seccnt *sectswritten, t_seccnt sects)
{
return SCPE_NOFNC;
}

static t_stat sim_os_disk_info_raw (SMP_FILE* f, uint32 *sector_size, uint32 *removable)
{
return SCPE_NOFNC;
}

#endif

/* OS Independent Disk Virtual Disk (VHD) I/O support */

#if (defined (VMS) && !(defined (__ALPHA) || defined (__ia64)))
#define DONT_DO_VHD_SUPPORT  /* VAX/VMS compilers don't have 64 bit integers */
#endif

#if defined (DONT_DO_VHD_SUPPORT)

/*============================================================================*/
/*                        Non-implemented version                             */
/*   This is only for hody systems which don't have 64 bit integer types      */
/*============================================================================*/

static t_stat sim_vhd_disk_implemented (void)
{
return SCPE_NOFNC;
}

static SMP_FILE* sim_vhd_disk_open (const char *rawdevicename, const char *openmode)
{
return NULL;
}

static SMP_FILE* sim_vhd_disk_create (const char *szVHDPath, t_addr desiredsize)
{
return NULL;
}

static SMP_FILE* sim_vhd_disk_create_diff (const char *szVHDPath, const char *szParentVHDPath)
{
return NULL;
}

static int sim_vhd_disk_close (SMP_FILE* f)
{
return -1;
}

static void sim_vhd_disk_flush (SMP_FILE* f)
{
}

static t_addr sim_vhd_disk_size (SMP_FILE* f)
{
return (t_addr)-1;
}

static t_stat sim_vhd_disk_rdsect (UNIT *uptr, t_lba lba, uint8 *buf, t_seccnt *sectsread, t_seccnt sects)
{
return SCPE_IOERR;
}

static t_stat sim_vhd_disk_wrsect (UNIT *uptr, t_lba lba, uint8 *buf, t_seccnt *sectswritten, t_seccnt sects)
{
return SCPE_IOERR;
}

static t_stat sim_vhd_disk_set_dtype (SMP_FILE* f, const char *dtype)
{
return SCPE_NOFNC;
}

static const char *sim_vhd_disk_get_dtype (SMP_FILE* f)
{
return NULL;
}

#else

/*++
    This code follows the details specified in the "Virtual Hard Disk Image 
    Format Specification", Version 1.0 October 11, 2006.
--*/

typedef t_uint64    uint64;
typedef t_int64     int64;

typedef struct _VHD_Footer {
    /*
    Cookies are used to uniquely identify the original creator of the hard disk 
    image. The values are case-sensitive.  Microsoft uses the �conectix� string 
    to identify this file as a hard disk image created by Microsoft Virtual 
    Server, Virtual PC, and predecessor products. The cookie is stored as an 
    eight-character ASCII string with the �c� in the first byte, the �o� in 
    the second byte, and so on.
    */
    char Cookie[8];
    /*
    This is a bit field used to indicate specific feature support. The following 
    table displays the list of features. 
    Any fields not listed are reserved. 

    Feature Value:
       No features enabled     0x00000000
       Temporary               0x00000001
       Reserved                0x00000002

       No features enabled. 
              The hard disk image has no special features enabled in it.
       Temporary.
              This bit is set if the current disk is a temporary disk. A 
              temporary disk designation indicates to an application that 
              this disk is a candidate for deletion on shutdown. 
       Reserved.
              This bit must always be set to 1.
       All other bits are also reserved and should be set to 0.
    */
    uint32 Features;
    /*
    This field is divided into a major/minor version and matches the version of 
    the specification used in creating the file. The most-significant two bytes 
    are for the major version. The least-significant two bytes are the minor 
    version.  This must match the file format specification. For the current 
    specification, this field must be initialized to 0x00010000.
    The major version will be incremented only when the file format is modified 
    in such a way that it is no longer compatible with older versions of the 
    file format.
    */
    uint32 FileFormatVersion;
    /*
    This field holds the absolute byte offset, from the beginning of the file, 
    to the next structure. This field is used for dynamic disks and differencing 
    disks, but not fixed disks. For fixed disks, this field should be set to 
    0xFFFFFFFF. 
    */
    uint64 DataOffset;
    /*
    This field stores the creation time of a hard disk image. This is the number 
    of seconds since January 1, 2000 12:00:00 AM in UTC/GMT.
    */
    uint32 TimeStamp;
    /*
    This field is used to document which application created the hard disk. The 
    field is a left-justified text field. It uses a single-byte character set. 
    If the hard disk is created by Microsoft Virtual PC, "vpc " is written in 
    this field. If the hard disk image is created by Microsoft Virtual Server, 
    then "vs  " is written in this field.
    Other applications should use their own unique identifiers. 
    */
    char CreatorApplication[4];
    /*
    This field holds the major/minor version of the application that created 
    the hard disk image.  Virtual Server 2004 sets this value to 0x00010000 and
    Virtual PC 2004 sets this to 0x00050000.
    */
    uint32 CreatorVersion;
    /*
    This field stores the type of host operating system this disk image is 
    created on.
       Host OS type    Value
       Windows         0x5769326B (Wi2k)
       Macintosh       0x4D616320 (Mac )
    */
    uint8 CreatorHostOS[4];
    /*
    This field stores the size of the hard disk in bytes, from the perspective 
    of the virtual machine, at creation time. This field is for informational 
    purposes. 
    */
    uint64 OriginalSize;
    /*
    This field stores the current size of the hard disk, in bytes, from the 
    perspective of the virtual machine.
    This value is same as the original size when the hard disk is created. 
    This value can change depending on whether the hard disk is expanded. 
    */
    uint64 CurrentSize;
    /*
    This field stores the cylinder, heads, and sectors per track value for the 
    hard disk. 
       Disk Geometry field          Size (bytes)
       Cylinder                     2
       Heads                        1
       Sectors per track/cylinder   1

    When a hard disk is configured as an ATA hard disk, the CHS values (that is, 
    Cylinder, Heads, Sectors per track) are used by the ATA controller to 
    determine the size of the disk. When the user creates a hard disk of a 
    certain size, the size of the hard disk image in the virtual machine is 
    smaller than that created by the user. This is because CHS value calculated 
    from the hard disk size is rounded down. The pseudo-code for the algorithm 
    used to determine the CHS values can be found in the appendix of this 
    document. 
    */
    uint32 DiskGeometry;
    /*
       Disk Type field              Value
       None                         0
       Reserved (deprecated)        1
       Fixed hard disk              2
       Dynamic hard disk            3
       Differencing hard disk       4
       Reserved (deprecated)        5
       Reserved (deprecated)        6
    */
    uint32 DiskType;
    /*
    This field holds a basic checksum of the hard disk footer. It is just a 
    one�s complement of the sum of all the bytes in the footer without the 
    checksum field.
    If the checksum verification fails, the Virtual PC and Virtual Server 
    products will instead use the header. If the checksum in the header also 
    fails, the file should be assumed to be corrupt. The pseudo-code for the 
    algorithm used to determine the checksum can be found in the appendix of 
    this document. 
    */
    uint32 Checksum;
    /*
    Every hard disk has a unique ID stored in the hard disk. This is used to 
    identify the hard disk. This is a 128-bit universally unique identifier 
    (UUID). This field is used to associate a parent hard disk image with its 
    differencing hard disk image(s).
    */
    uint8 UniqueID[16];
    /*
    This field holds a one-byte flag that describes whether the system is in 
    saved state. If the hard disk is in the saved state the value is set to 1.
    Operations such as compaction and expansion cannot be performed on a hard 
    disk in a saved state.
    */
    uint8 SavedState;
    /*
    This field contains zeroes. It is 427 bytes in size. 
    */
    uint8 Reserved1[11];
    /*
    This field is an extension to the VHD spec and includes a simh drive type 
    name as a nul terminated string.
    */
    uint8 DriveType[16];
    /*
    This field contains zeroes. It is 400 bytes in size. 
    */
    uint8 Reserved[400];
    } VHD_Footer;

/*
For dynamic and differencing disk images, the �Data Offset� field within 
the image footer points to a secondary structure that provides additional 
information about the disk image. The dynamic disk header should appear on 
a sector (512-byte) boundary.
*/
typedef struct _VHD_DynamicDiskHeader {
    /*
    This field holds the value "cxsparse". This field identifies the header.
    */
    char Cookie[8];
    /*
    This field contains the absolute byte offset to the next structure in the 
    hard disk image. It is currently unused by existing formats and should be 
    set to 0xFFFFFFFF.
    */
    uint64 DataOffset;
    /*
    This field stores the absolute byte offset of the Block Allocation Table 
    (BAT) in the file. 
    */
    uint64 TableOffset;
    /*
    This field stores the version of the dynamic disk header. The field is 
    divided into Major/Minor version. The least-significant two bytes represent
    the minor version, and the most-significant two bytes represent the major 
    version. This must match with the file format specification. For this 
    specification, this field must be initialized to 0x00010000. 
    The major version will be incremented only when the header format is 
    modified in such a way that it is no longer compatible with older versions 
    of the product.
    */
    uint32 HeaderVersion;
    /*
    This field holds the maximum entries present in the BAT. This should be 
    equal to the number of blocks in the disk (that is, the disk size divided 
    by the block size). 
    */
    uint32 MaxTableEntries;
    /*
    A block is a unit of expansion for dynamic and differencing hard disks. It 
    is stored in bytes. This size does not include the size of the block bitmap. 
    It is only the size of the data section of the block. The sectors per block 
    must always be a power of two. The default value is 0x00200000 (indicating a 
    block size of 2 MB).
    */
    uint32 BlockSize;
    /*
    This field holds a basic checksum of the dynamic header. It is a one�s 
    complement of the sum of all the bytes in the header without the checksum 
    field.
    If the checksum verification fails the file should be assumed to be corrupt.
    */
    uint32 Checksum;
    /*
    This field is used for differencing hard disks. A differencing hard disk 
    stores a 128-bit UUID of the parent hard disk. For more information, see 
    �Creating Differencing Hard Disk Images� later in this paper.
    */
    uint8 ParentUniqueID[16];
    /*
    This field stores the modification time stamp of the parent hard disk. This 
    is the number of seconds since January 1, 2000 12:00:00 AM in UTC/GMT.
    */
    uint32 ParentTimeStamp;
    /*
    This field should be set to zero. 
    */
    uint32 Reserved0;
    /*
    This field contains a Unicode string (UTF-16) of the parent hard disk 
    filename. 
    */
    char ParentUnicodeName[512];
    /*
    These entries store an absolute byte offset in the file where the parent 
    locator for a differencing hard disk is stored. This field is used only for 
    differencing disks and should be set to zero for dynamic disks. 
    */
    struct VHD_ParentLocator {
        /*
        The platform code describes which platform-specific format is used for the 
        file locator. For Windows, a file locator is stored as a path (for example. 
        �c:\disksimages\ParentDisk.vhd�). On a Macintosh system, the file locator 
        is a binary large object (blob) that contains an �alias.� The parent locator 
        table is used to support moving hard disk images across platforms.
        Some current platform codes include the following:
           Platform Code        Description
           None (0x0)
           Wi2r (0x57693272)    [deprecated]
           Wi2k (0x5769326B)    [deprecated]
           W2ru (0x57327275)    Unicode pathname (UTF-16) on Windows relative to the differencing disk pathname.
           W2ku (0x57326B75)    Absolute Unicode (UTF-16) pathname on Windows.
           Mac (0x4D616320)     (Mac OS alias stored as a blob)
           MacX(0x4D616358)     A file URL with UTF-8 encoding conforming to RFC 2396.
        */
        uint8 PlatformCode[4];
        /*
        This field stores the number of 512-byte sectors needed to store the parent 
        hard disk locator.
        */
        uint32 PlatformDataSpace;
        /*
        This field stores the actual length of the parent hard disk locator in bytes.
        */
        uint32 PlatformDataLength;
        /*
        This field must be set to zero.
        */
        uint32 Reserved;
        /*
        This field stores the absolute file offset in bytes where the platform 
        specific file locator data is stored.
        */
        uint64 PlatformDataOffset;
        /*
        This field stores the absolute file offset in bytes where the platform 
        specific file locator data is stored.
        */
        } ParentLocatorEntries[8];
    /*
    This must be initialized to zeroes.
    */
    char Reserved[256];
    } VHD_DynamicDiskHeader;

#define VHD_BAT_FREE_ENTRY (0xFFFFFFFF)
#define VHD_DATA_BLOCK_ALIGNMENT ((uint64)4096)    /* Optimum when underlying storage has 4k sectors */

static char *VHD_DiskTypes[] =
    {
    "None",                       /* 0 */
    "Reserved (deprecated)",      /* 1 */
    "Fixed hard disk",            /* 2 */
#define VHD_DT_Fixed                 2
    "Dynamic hard disk",          /* 3 */
#define VHD_DT_Dynamic               3
    "Differencing hard disk",     /* 4 */
#define VHD_DT_Differencing          4
    "Reserved (deprecated)",      /* 5 */
    "Reserved (deprecated)",      /* 6 */
    };

static uint32 NtoHl(uint32 value);

static uint64 NtoHll(uint64 value);

typedef struct VHD_IOData *VHDHANDLE;

static t_stat ReadFilePosition(SMP_FILE* File, void *buf, size_t bufsize, size_t *bytesread, uint64 position)
{
uint32 err = sim_fseek (File, (t_addr)position, SEEK_SET);
size_t i;

if (!err) {
    i = fread (buf, 1, bufsize, File);
    err = ferror (File);
    if ((!err) && bytesread)
        *bytesread = i;
    }
return (err ? SCPE_IOERR : SCPE_OK);
}

static t_stat WriteFilePosition(SMP_FILE* File, void *buf, size_t bufsize, size_t *byteswritten, uint64 position)
{
uint32 err = sim_fseek (File, (t_addr)position, SEEK_SET);
size_t i;

if (!err) {
    i = fwrite (buf, 1, bufsize, File);
    err = ferror (File);
    if ((!err) && byteswritten)
        *byteswritten = i;
    }
return (err ? SCPE_IOERR : SCPE_OK);
}

static uint32
CalculateVhdFooterChecksum(void *data, 
                           size_t size)
{
uint32 sum = 0;
uint8 *c = (uint8 *)data;

while (size--)
    sum += *c++;
return ~sum;
}

#if defined (__ALPHA) || defined (__ia64) || defined (VMS) || defined(__x86_32__) || defined(__x86_64__)
#ifndef __BYTE_ORDER__
#define __BYTE_ORDER__ __ORDER_LITTLE_ENDIAN__
#endif
#endif
#ifndef __BYTE_ORDER__
#define __BYTE_ORDER__ UNKNOWN
#endif
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
static uint32
NtoHl(uint32 value)
{
uint8 *l = (uint8 *)&value;
return l[3] | (l[2]<<8) | (l[1]<<16) | (l[0]<<24);
}

static uint64
NtoHll(uint64 value)
{
uint8 *l = (uint8 *)&value;
uint64 highresult = l[3] | (l[2]<<8) | (l[1]<<16) | (l[0]<<24);
uint32 lowresult = l[7] | (l[6]<<8) | (l[5]<<16) | (l[4]<<24);
return (highresult << 32) | lowresult;
}
#elif  __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
static uint32
NtoHl(uint32 value)
{
return value;
}

static uint64
NtoHll(uint64 value)
{
return value;
}
#else
static uint32
NtoHl(uint32 value)
{
uint8 *l = (uint8 *)&value;

if (sim_end)
    return l[3] | (l[2]<<8) | (l[1]<<16) | (l[0]<<24);
return value;
}

static uint64
NtoHll(uint64 value)
{
uint8 *l = (uint8 *)&value;

if (sim_end) {
    uint64 highresult = l[3] | (l[2]<<8) | (l[1]<<16) | (l[0]<<24);
    uint32 lowresult = l[7] | (l[6]<<8) | (l[5]<<16) | (l[4]<<24);
    return (highresult << 32) | lowresult;
    }
return value;
}
#endif

static
int
GetVHDFooter(const char *szVHDPath, 
             VHD_Footer *sFooter, 
             VHD_DynamicDiskHeader *sDynamic,
             uint32 **aBAT,
             uint32 *ModifiedTimeStamp,
             char *szParentVHDPath,
             size_t ParentVHDPathSize)
{
SMP_FILE* File = NULL;
uint64 position;
uint32 sum, saved_sum;
int Return = 0;
VHD_Footer sHeader;
struct stat statb;

if (sFooter)
    memset(sFooter, '\0', sizeof(*sFooter));
if (sDynamic)
    memset(sDynamic, '\0', sizeof(*sDynamic));
if (aBAT)
    *aBAT = NULL;
File = sim_fopen (szVHDPath, "rb");
if (!File) {
    Return = errno;
    goto Return_Cleanup;
    }
if (ModifiedTimeStamp)
    if (stat (szVHDPath, &statb)) {
        Return = errno;
        goto Return_Cleanup;
        }
    else
        *ModifiedTimeStamp = NtoHl ((uint32)(statb.st_mtime-946684800));
position = sim_fsize_ex (File);
if (((int64)position) == -1) {
    Return = errno;
    goto Return_Cleanup;
    }
position -= sizeof(*sFooter);
if (ReadFilePosition(File, 
                     sFooter, 
                     sizeof(*sFooter), 
                     NULL, 
                     position)) {
    Return = errno;
    goto Return_Cleanup;
    }
saved_sum = NtoHl(sFooter->Checksum);
sFooter->Checksum = 0;
sum = CalculateVhdFooterChecksum(sFooter, sizeof(*sFooter));
sFooter->Checksum = NtoHl(saved_sum);
if ((sum != saved_sum) || (memcmp("conectix", sFooter->Cookie, sizeof(sFooter->Cookie)))) {
    Return = EINVAL;                                    /* File Corrupt */
    goto Return_Cleanup;
    }
if (ReadFilePosition(File, 
                     &sHeader, 
                     sizeof(sHeader), 
                     NULL, 
                     (uint64)0)) {
    Return = errno;
    goto Return_Cleanup;
    }
if ((NtoHl(sFooter->DiskType) != VHD_DT_Dynamic) &&
    (NtoHl(sFooter->DiskType) != VHD_DT_Differencing) &&
    (NtoHl(sFooter->DiskType) != VHD_DT_Fixed)) {
    Return = EINVAL;                                    /* File Corrupt */
    goto Return_Cleanup;
    }
if (((NtoHl(sFooter->DiskType) == VHD_DT_Dynamic) ||
     (NtoHl(sFooter->DiskType) == VHD_DT_Differencing)) &&
     memcmp(sFooter, &sHeader, sizeof(sHeader))) {
    Return = EINVAL;                                    /* File Corrupt */
    goto Return_Cleanup;
    }
if ((sDynamic) &&
    ((NtoHl(sFooter->DiskType) == VHD_DT_Dynamic) ||
     (NtoHl(sFooter->DiskType) == VHD_DT_Differencing))) {
    if (ReadFilePosition(File, 
                         sDynamic, 
                         sizeof (*sDynamic), 
                         NULL, 
                         NtoHll (sFooter->DataOffset))) {
        Return = errno;
        goto Return_Cleanup;
        }
    saved_sum = NtoHl (sDynamic->Checksum);
    sDynamic->Checksum = 0;
    sum = CalculateVhdFooterChecksum (sDynamic, sizeof(*sDynamic));
    sDynamic->Checksum = NtoHl (saved_sum);
    if ((sum != saved_sum) || (memcmp ("cxsparse", sDynamic->Cookie, sizeof (sDynamic->Cookie)))) {
        Return = errno;
        goto Return_Cleanup;
        }
    if (aBAT)
        {
        *aBAT = (uint32*) malloc(512*((sizeof(**aBAT)*NtoHl(sDynamic->MaxTableEntries)+511)/512));
        if (ReadFilePosition(File, 
                             *aBAT, 
                             sizeof (**aBAT)*NtoHl(sDynamic->MaxTableEntries), 
                             NULL, 
                             NtoHll (sDynamic->TableOffset))) {
            Return = EINVAL;                            /* File Corrupt */
            goto Return_Cleanup;
            }
        }
    if (szParentVHDPath && ParentVHDPathSize) {
        VHD_Footer sParentFooter;

        memset (szParentVHDPath, '\0', ParentVHDPathSize);
        if (NtoHl (sFooter->DiskType) == VHD_DT_Differencing)
            {
            size_t i, j;

            for (j=0; j<8; ++j)
            {
                uint8 *Pdata;
                char ParentName[256];
                char CheckPath[256];
                uint32 ParentModificationTime;

                if ('\0' == sDynamic->ParentLocatorEntries[j].PlatformCode[0])
                    continue;
                memset (ParentName, '\0', sizeof(ParentName));
                memset (CheckPath, '\0', sizeof(CheckPath));
                Pdata = (uint8*) calloc (1, NtoHl(sDynamic->ParentLocatorEntries[j].PlatformDataSpace)+1);
                if (!Pdata)
                    continue;
                if (ReadFilePosition(File, 
                                     Pdata, 
                                     NtoHl (sDynamic->ParentLocatorEntries[j].PlatformDataSpace), 
                                     NULL, 
                                     NtoHll (sDynamic->ParentLocatorEntries[j].PlatformDataOffset))) {
                    free (Pdata);
                    continue;
                    }
                for (i=0; i<NtoHl(sDynamic->ParentLocatorEntries[j].PlatformDataLength); i+=2)
                    if ((Pdata[i] == '\0') && (Pdata[i+1] == '\0')) {
                        ParentName[i/2] = '\0';
                        break;
                        }
                    else
                        ParentName[i/2] = Pdata[i] ? Pdata[i] : Pdata[i+1];
                free (Pdata);
                if (0 == memcmp (sDynamic->ParentLocatorEntries[j].PlatformCode, "W2ku", 4))
                    strncpy (CheckPath, ParentName, sizeof (CheckPath)-1);
            else
                if (0 == memcmp (sDynamic->ParentLocatorEntries[j].PlatformCode, "W2ru", 4)) {
                const char *c;

                if ((c = strrchr (szVHDPath, '/')) || (c = strrchr (szVHDPath, '\\')))
                    memcpy (CheckPath, szVHDPath, c-szVHDPath+1);
                    strncpy (CheckPath+strlen(CheckPath), ParentName, sizeof (CheckPath)-(strlen (CheckPath)+1));
                }
                if ((0 == GetVHDFooter(CheckPath, 
                                       &sParentFooter, 
                                       NULL,
                                       NULL,
                                       &ParentModificationTime,
                                       NULL, 
                                       0)) &&
                    (0 == memcmp (sDynamic->ParentUniqueID, sParentFooter.UniqueID, sizeof (sParentFooter.UniqueID))) &&
                    (sDynamic->ParentTimeStamp == ParentModificationTime))
                    {
                    strncpy (szParentVHDPath, CheckPath, ParentVHDPathSize);
                    break;
                    }
            }
            if (!szParentVHDPath)
                Return = EINVAL;                        /* File Corrupt */
            }
        }
    }
Return_Cleanup:
if (File)
    fclose(File);
if (aBAT && (0 != Return)) {
    free (*aBAT);
    *aBAT = NULL;
    }
return errno = Return;
}

struct VHD_IOData {
    VHD_Footer Footer;
    VHD_DynamicDiskHeader Dynamic;
    uint32 *BAT;
    SMP_FILE* File;
    char ParentVHDPath[512];
    struct VHD_IOData *Parent;
    };

static t_stat sim_vhd_disk_implemented (void)
{
return SCPE_OK;
}

static t_stat sim_vhd_disk_set_dtype (SMP_FILE* f, const char *dtype)
{
VHDHANDLE hVHD  = (VHDHANDLE)f;
int Status = 0;

memset (hVHD->Footer.DriveType, '\0', sizeof hVHD->Footer.DriveType);
memcpy (hVHD->Footer.DriveType, dtype, ((1+strlen (dtype)) < sizeof (hVHD->Footer.DriveType)) ? (1+strlen (dtype)) : sizeof (hVHD->Footer.DriveType));
hVHD->Footer.Checksum = 0;
hVHD->Footer.Checksum = NtoHl (CalculateVhdFooterChecksum (&hVHD->Footer, sizeof(hVHD->Footer)));

if (NtoHl (hVHD->Footer.DiskType) == VHD_DT_Fixed) {
    if (WriteFilePosition(hVHD->File,
                          &hVHD->Footer,
                          sizeof(hVHD->Footer),
                          NULL,
                          NtoHll (hVHD->Footer.CurrentSize)))
        Status = errno;
    goto Cleanup_Return;
    }
else {
    uint64 position;

    position = sim_fsize_ex (hVHD->File);
    if (((int64)position) == -1) {
        Status = errno;
        goto Cleanup_Return;
        }
    position -= sizeof(hVHD->Footer);
    /* Update both copies on a dynamic disk */
    if (WriteFilePosition(hVHD->File,
                          &hVHD->Footer,
                          sizeof(hVHD->Footer),
                          NULL,
                          (uint64)0)) {
        Status = errno;
        goto Cleanup_Return;
        }
    if (WriteFilePosition(hVHD->File,
                          &hVHD->Footer,
                          sizeof(hVHD->Footer),
                          NULL,
                          position)) {
        Status = errno;
        goto Cleanup_Return;
        }
    }
Cleanup_Return:
if (Status)
    return SCPE_IOERR;
return SCPE_OK;
}

static const char *sim_vhd_disk_get_dtype (SMP_FILE* f)
{
VHDHANDLE hVHD  = (VHDHANDLE)f;

return (char *)(&hVHD->Footer.DriveType[0]);
}

static SMP_FILE* sim_vhd_disk_open (const char *szVHDPath, const char *DesiredAccess)
    {
    VHDHANDLE hVHD = (VHDHANDLE) calloc (1, sizeof(*hVHD));
    int Status;

    if (!hVHD)
        return (SMP_FILE* )hVHD;
    if (Status = GetVHDFooter (szVHDPath, 
                               &hVHD->Footer, 
                               &hVHD->Dynamic, 
                               &hVHD->BAT,
                               NULL,
                               hVHD->ParentVHDPath,
                               sizeof (hVHD->ParentVHDPath)))
        goto Cleanup_Return;
    if (NtoHl (hVHD->Footer.DiskType) == VHD_DT_Differencing) {
        hVHD->Parent = (VHDHANDLE)sim_vhd_disk_open (hVHD->ParentVHDPath, "rb");
        if (!hVHD->Parent) {
            Status = errno;
            goto Cleanup_Return;
            }
        }
    if (hVHD->Footer.SavedState) {
        Status = EAGAIN;                                /* Busy */
        goto Cleanup_Return;
        }
    hVHD->File = sim_fopen (szVHDPath, DesiredAccess);
    if (!hVHD->File) {
        Status = errno;
        goto Cleanup_Return;
        }
Cleanup_Return:
    if (Status) {
        free (hVHD->BAT);
        free (hVHD);
        hVHD = NULL;
        }
    errno = Status;
    return (SMP_FILE* )hVHD;
    }

static int sim_vhd_disk_close (SMP_FILE* f)
{
VHDHANDLE hVHD = (VHDHANDLE)f;

if (NULL != hVHD) {
    if (hVHD->Parent)
        sim_vhd_disk_close ((SMP_FILE* )hVHD->Parent);
    free (hVHD->BAT);
    if (hVHD->File) {
        fflush (hVHD->File);
        fclose (hVHD->File);
        }
    free (hVHD);
    return 0;
    }
return -1;
}

static void sim_vhd_disk_flush (SMP_FILE* f)
{
VHDHANDLE hVHD = (VHDHANDLE)f;

if ((NULL != hVHD) && (hVHD->File))
    fflush (hVHD->File);
}

static t_addr sim_vhd_disk_size (SMP_FILE* f)
{
VHDHANDLE hVHD = (VHDHANDLE)f;

return (t_addr)(NtoHll (hVHD->Footer.CurrentSize));
}

#include <stdlib.h>
#include <time.h>
static void
_rand_uuid_gen (void *uuidaddr)
{
int i;
uint8 *b = (uint8 *)uuidaddr;
uint32 timenow = (uint32)time (NULL);

memcpy (uuidaddr, &timenow, sizeof (timenow));
srand ((unsigned)timenow);
for (i=4; i<16; i++) {
    b[i] = (uint8)rand();
    }
}

#if defined (_WIN32)
static void
uuid_gen (void *uuidaddr)
{
static
RPC_STATUS
(RPC_ENTRY *UuidCreate_c) (void *);

if (!UuidCreate_c) {
    HINSTANCE hDll;
    hDll = LoadLibraryA("rpcrt4.dll");
    UuidCreate_c = (RPC_STATUS (RPC_ENTRY *) (void*)) GetProcAddress(hDll, "UuidCreate");
    }
if (UuidCreate_c)
    UuidCreate_c(uuidaddr);
else
    _rand_uuid_gen (uuidaddr);
}
#elif defined (HAVE_DLOPEN)
#include <dlfcn.h>

static void
uuid_gen (void *uuidaddr)
{
void (*uuid_generate_c) (void *) = NULL;
void *handle;

#define __STR_QUOTE(tok) #tok
#define __STR(tok) __STR_QUOTE(tok)
    handle = dlopen("libuuid." __STR(HAVE_DLOPEN), RTLD_NOW|RTLD_GLOBAL);
    if (handle)
        uuid_generate_c = (void (*)(void *)) dlsym(handle, "uuid_generate");
if (uuid_generate_c)
    uuid_generate_c(uuidaddr);
else
    _rand_uuid_gen (uuidaddr);
    if (handle)
        dlclose(handle);
}
#else
static void
uuid_gen (void *uuidaddr)
{
_rand_uuid_gen (uuidaddr);
}
#endif

static VHDHANDLE
CreateVirtualDisk(const char *szVHDPath,
                  uint32 SizeInSectors,
                  uint32 BlockSize,
                  t_bool bFixedVHD)
{
VHD_Footer Footer;
VHD_DynamicDiskHeader Dynamic;
uint32 *BAT = NULL;
time_t now;
uint32 i;
SMP_FILE* File = NULL;
uint32 Status = 0;
uint32 BytesPerSector = 512;
uint64 SizeInBytes = ((uint64)SizeInSectors)*BytesPerSector;
uint32 MaxTableEntries;
VHDHANDLE hVHD = NULL;

if (SizeInBytes > ((uint64)(1024*1024*1024))*2040) {
    Status = EFBIG;
    goto Cleanup_Return;
    }
if (File = sim_fopen (szVHDPath, "rb")) {
    fclose (File);
    File = NULL;
    Status = EEXIST;
    goto Cleanup_Return;
    }
File = sim_fopen (szVHDPath, "wb");
if (!File) {
    Status = errno;
    goto Cleanup_Return;
    }

memset (&Footer, 0, sizeof(Footer));
memcpy (Footer.Cookie, "conectix", 8);
Footer.Features = NtoHl (0x00000002);;
Footer.FileFormatVersion = NtoHl (0x00010000);;
Footer.DataOffset = NtoHll (bFixedVHD ? ((long long)-1) : (long long)(sizeof(Footer)));
time (&now);
Footer.TimeStamp = NtoHl ((uint32)(now-946684800));
memcpy (Footer.CreatorApplication, "simh", 4);
Footer.CreatorVersion = NtoHl (0x00010000);
memcpy (Footer.CreatorHostOS, "Wi2k", 4);
Footer.OriginalSize = NtoHll (SizeInBytes);
Footer.CurrentSize = NtoHll (SizeInBytes);
uuid_gen (Footer.UniqueID);
Footer.DiskType = NtoHl (bFixedVHD ? VHD_DT_Fixed : VHD_DT_Dynamic);
Footer.DiskGeometry = NtoHl (0xFFFF10FF);
if (1) { /* CHS Calculation */
    uint32 totalSectors = (uint32)(SizeInBytes/BytesPerSector);/* Total data sectors present in the disk image */
    uint32 cylinders;                                          /* Number of cylinders present on the disk */
    uint32 heads;                                              /* Number of heads present on the disk */
    uint32 sectorsPerTrack;                                    /* Sectors per track on the disk */
    uint32 cylinderTimesHeads;                                 /* Cylinders x heads */

    if (totalSectors > 65535 * 16 * 255)
        totalSectors = 65535 * 16 * 255;

    if (totalSectors >= 65535 * 16 * 63) {
        sectorsPerTrack = 255;
        heads = 16;
        cylinderTimesHeads = totalSectors / sectorsPerTrack;
        }
    else {
        sectorsPerTrack = 17; 
        cylinderTimesHeads = totalSectors / sectorsPerTrack;

        heads = (cylinderTimesHeads + 1023) / 1024;

        if (heads < 4)
            heads = 4;
        if (cylinderTimesHeads >= (heads * 1024) || heads > 16)
            {
            sectorsPerTrack = 31;
            heads = 16;
            cylinderTimesHeads = totalSectors / sectorsPerTrack;    
            }
        if (cylinderTimesHeads >= (heads * 1024))
            {
            sectorsPerTrack = 63;
            heads = 16;
            cylinderTimesHeads = totalSectors / sectorsPerTrack;
            }
        }
    cylinders = cylinderTimesHeads / heads;
    Footer.DiskGeometry = NtoHl ((cylinders<<16)|(heads<<8)|sectorsPerTrack);
    }
Footer.Checksum = NtoHl (CalculateVhdFooterChecksum(&Footer, sizeof(Footer)));

if (bFixedVHD) {
    if (WriteFilePosition(File,
                          &Footer,
                          sizeof(Footer),
                          NULL,
                          SizeInBytes))
        Status = errno;
    goto Cleanup_Return;
    }

/* Dynamic Disk */
memset (&Dynamic, 0, sizeof(Dynamic));
memcpy (Dynamic.Cookie, "cxsparse", 8);
Dynamic.DataOffset = NtoHll (0xFFFFFFFFFFFFFFFFLL);
Dynamic.TableOffset = NtoHll ((uint64)(BytesPerSector*((sizeof(Dynamic)+sizeof(Footer)+BytesPerSector-1)/BytesPerSector)));
Dynamic.HeaderVersion = NtoHl (0x00010000);
if (0 == BlockSize)
    BlockSize = 2*1024*1024;
Dynamic.BlockSize = NtoHl (BlockSize);
MaxTableEntries = (uint32)((SizeInBytes+BlockSize-1)/BlockSize);
Dynamic.MaxTableEntries = NtoHl (MaxTableEntries);
Dynamic.Checksum = NtoHl (CalculateVhdFooterChecksum(&Dynamic, sizeof(Dynamic)));
BAT = (uint32*) malloc (BytesPerSector*((MaxTableEntries*sizeof(*BAT)+BytesPerSector-1)/BytesPerSector));
memset (BAT, 0, BytesPerSector*((MaxTableEntries*sizeof(*BAT)+BytesPerSector-1)/BytesPerSector));
for (i=0; i<MaxTableEntries; ++i)
    BAT[i] = VHD_BAT_FREE_ENTRY;

if (WriteFilePosition(File,
                      &Footer,
                      sizeof(Footer),
                      NULL,
                      0)) {
    Status = errno;
    goto Cleanup_Return;
    }
if (WriteFilePosition(File,
                      &Dynamic,
                      sizeof(Dynamic),
                      NULL,
                      NtoHll(Footer.DataOffset))) {
    Status = errno;
    goto Cleanup_Return;
    }
if (WriteFilePosition(File,
                      BAT,
                      BytesPerSector*((MaxTableEntries*sizeof(*BAT)+BytesPerSector-1)/BytesPerSector),
                      NULL,
                      NtoHll(Dynamic.TableOffset))) {
    Status = errno;
    goto Cleanup_Return;
    }
if (WriteFilePosition(File,
                      &Footer,
                      sizeof(Footer),
                      NULL,
                      NtoHll(Dynamic.TableOffset)+BytesPerSector*((MaxTableEntries*sizeof(*BAT)+BytesPerSector-1)/BytesPerSector))) {
    Status = errno;
    goto Cleanup_Return;
    }

Cleanup_Return:
free (BAT);
if (File)
    fclose (File);
if (Status) {
    if (Status != EEXIST)
        remove (szVHDPath);
    }
else {
    hVHD = (VHDHANDLE)sim_vhd_disk_open (szVHDPath, "rb+");
    if (!hVHD)
        Status = errno;
    }
errno = Status;
return hVHD;
}

static void
ExpandToFullPath (const char *szFileSpec,
                  char *szFullFileSpecBuffer,
                  size_t BufferSize)
{
#ifdef _WIN32
GetFullPathNameA (szFileSpec, (DWORD)BufferSize, szFullFileSpecBuffer, NULL);
#else
strncpy (szFullFileSpecBuffer, szFileSpec, BufferSize);
#endif
}

static VHDHANDLE
CreateDifferencingVirtualDisk(const char *szVHDPath, 
                              const char *szParentVHDPath)
{
uint32 BytesPerSector = 512;
VHDHANDLE hVHD = NULL;
VHD_Footer ParentFooter;
VHD_DynamicDiskHeader ParentDynamic;
uint32 ParentTimeStamp;
uint32 Status = 0;
char *RelativeParentVHDPath = NULL;
char *FullParentVHDPath = NULL;
char *RelativeParentVHDPathBuffer = NULL;
char *FullParentVHDPathBuffer = NULL;
char *FullVHDPath = NULL;
size_t i, RelativeMatch, UpDirectories, LocatorsWritten = 0;
int64 LocatorPosition;

if (Status = GetVHDFooter (szParentVHDPath, 
                           &ParentFooter, 
                           &ParentDynamic, 
                           NULL, 
                           &ParentTimeStamp,
                           NULL, 
                           0))
    goto Cleanup_Return;
hVHD = CreateVirtualDisk (szVHDPath,
                          (uint32)(NtoHll(ParentFooter.CurrentSize)/BytesPerSector),
                          NtoHl(ParentDynamic.BlockSize),
                          FALSE);
if (!hVHD) {
    Status = errno;
    goto Cleanup_Return;
    }
LocatorPosition = NtoHll (hVHD->Dynamic.TableOffset)+BytesPerSector*((NtoHl (hVHD->Dynamic.MaxTableEntries)*sizeof(*hVHD->BAT)+BytesPerSector-1)/BytesPerSector);
hVHD->Dynamic.Checksum = 0;
RelativeParentVHDPath = (char*) calloc (1, BytesPerSector+1);
FullParentVHDPath = (char*) calloc (1, BytesPerSector+1);
RelativeParentVHDPathBuffer = (char*) calloc (1, BytesPerSector);
FullParentVHDPathBuffer = (char*) calloc (1, BytesPerSector);
FullVHDPath = (char*) calloc (1, BytesPerSector+1);
ExpandToFullPath (szParentVHDPath, FullParentVHDPath, BytesPerSector);
for (i=0; i < strlen (FullParentVHDPath); i++)
    hVHD->Dynamic.ParentUnicodeName[i*2+1] = FullParentVHDPath[i];
for (i=0; i < strlen (FullParentVHDPath); i++)
    FullParentVHDPathBuffer[i*2] = FullParentVHDPath[i];
ExpandToFullPath (szVHDPath, FullVHDPath, BytesPerSector);
for (i=0, RelativeMatch=UpDirectories=0; i<strlen(FullVHDPath); i++)
    if ((FullVHDPath[i] == '\\') || (FullVHDPath[i] == '/'))
        if (memcmp (FullVHDPath, FullParentVHDPath, i+1))
            ++UpDirectories;
        else
            RelativeMatch = i;
if (RelativeMatch) {
    char UpDir[4] = "../";

    UpDir[2] = FullParentVHDPath[RelativeMatch];
    if (UpDirectories)
        for (i=0; i<UpDirectories; i++)
            strcpy (RelativeParentVHDPath+strlen (RelativeParentVHDPath), UpDir);
    else
        strcpy (RelativeParentVHDPath+strlen (RelativeParentVHDPath), UpDir+1);
    strcpy (RelativeParentVHDPath+strlen (RelativeParentVHDPath), &FullParentVHDPath[RelativeMatch+1]);
    }
for (i=0; i < strlen(RelativeParentVHDPath); i++)
    RelativeParentVHDPathBuffer[i*2] = RelativeParentVHDPath[i];
hVHD->Dynamic.ParentTimeStamp = ParentTimeStamp;
memcpy (hVHD->Dynamic.ParentUniqueID, ParentFooter.UniqueID, sizeof (hVHD->Dynamic.ParentUniqueID));
hVHD->Dynamic.ParentLocatorEntries[7].PlatformDataSpace = NtoHl (BytesPerSector);
hVHD->Dynamic.ParentLocatorEntries[7].PlatformDataOffset = NtoHll (LocatorPosition+LocatorsWritten*BytesPerSector);
++LocatorsWritten;
hVHD->Dynamic.ParentLocatorEntries[6].PlatformDataSpace = NtoHl (BytesPerSector);
hVHD->Dynamic.ParentLocatorEntries[6].PlatformDataOffset = NtoHll (LocatorPosition+LocatorsWritten*BytesPerSector);
++LocatorsWritten;
if (RelativeMatch) {
    memcpy (hVHD->Dynamic.ParentLocatorEntries[5].PlatformCode, "W2ru", 4);
    hVHD->Dynamic.ParentLocatorEntries[5].PlatformDataSpace = NtoHl (BytesPerSector);
    hVHD->Dynamic.ParentLocatorEntries[5].PlatformDataLength = NtoHl ((uint32)(2*strlen(RelativeParentVHDPath)));
    hVHD->Dynamic.ParentLocatorEntries[5].Reserved = 0;
    hVHD->Dynamic.ParentLocatorEntries[5].PlatformDataOffset = NtoHll (LocatorPosition+LocatorsWritten*BytesPerSector);
    ++LocatorsWritten;
    }
memcpy (hVHD->Dynamic.ParentLocatorEntries[4].PlatformCode, "W2ku", 4);
hVHD->Dynamic.ParentLocatorEntries[4].PlatformDataSpace = NtoHl (BytesPerSector);
hVHD->Dynamic.ParentLocatorEntries[4].PlatformDataLength = NtoHl ((uint32)(2*strlen(FullParentVHDPath)));
hVHD->Dynamic.ParentLocatorEntries[4].Reserved = 0;
hVHD->Dynamic.ParentLocatorEntries[4].PlatformDataOffset = NtoHll (LocatorPosition+LocatorsWritten*BytesPerSector);
++LocatorsWritten;
hVHD->Dynamic.Checksum = NtoHl (CalculateVhdFooterChecksum (&hVHD->Dynamic, sizeof(hVHD->Dynamic)));
hVHD->Footer.Checksum = 0;
hVHD->Footer.DiskType = NtoHl (VHD_DT_Differencing);
memcpy (hVHD->Footer.DriveType, ParentFooter.DriveType, sizeof (hVHD->Footer.DriveType));
hVHD->Footer.Checksum = NtoHl (CalculateVhdFooterChecksum (&hVHD->Footer, sizeof(hVHD->Footer)));

if (WriteFilePosition (hVHD->File,
                       &hVHD->Footer,
                       sizeof (hVHD->Footer),
                       NULL,
                       0)) {
    Status = errno;
    goto Cleanup_Return;
    }
if (WriteFilePosition (hVHD->File,
                       &hVHD->Dynamic,
                       sizeof (hVHD->Dynamic),
                       NULL,
                       NtoHll (hVHD->Footer.DataOffset))) {
    Status = errno;
    goto Cleanup_Return;
    }
LocatorsWritten = 0;
if (RelativeMatch) {
    if (WriteFilePosition (hVHD->File,
                           RelativeParentVHDPath,
                           BytesPerSector,
                           NULL,
                           LocatorPosition+LocatorsWritten*BytesPerSector)) {
        Status = errno;
        goto Cleanup_Return;
        }
    ++LocatorsWritten;
    }
if (WriteFilePosition (hVHD->File,
                       FullParentVHDPath,
                       BytesPerSector,
                       NULL,
                       LocatorPosition+LocatorsWritten*BytesPerSector)) {
    Status = errno;
    goto Cleanup_Return;
    }
++LocatorsWritten;
if (RelativeMatch) {
    if (WriteFilePosition (hVHD->File,
                           RelativeParentVHDPathBuffer,
                           BytesPerSector,
                           NULL,
                           LocatorPosition+LocatorsWritten*BytesPerSector)) {
        Status = errno;
        goto Cleanup_Return;
        }
    ++LocatorsWritten;
    }
if (WriteFilePosition (hVHD->File,
                       FullParentVHDPathBuffer,
                       BytesPerSector,
                       NULL,
                       LocatorPosition+LocatorsWritten*BytesPerSector)) {
    Status = errno;
    goto Cleanup_Return;
    }
++LocatorsWritten;
if (WriteFilePosition (hVHD->File,
                       &hVHD->Footer,
                       sizeof(hVHD->Footer),
                       NULL,
                       LocatorPosition+LocatorsWritten*BytesPerSector)) {
    Status = errno;
    goto Cleanup_Return;
    }

Cleanup_Return:
free (RelativeParentVHDPath);
free (FullParentVHDPath);
free (RelativeParentVHDPathBuffer);
free (FullParentVHDPathBuffer);
free (FullVHDPath);
sim_vhd_disk_close ((SMP_FILE* )hVHD);
hVHD = NULL;
if (Status) {
    if (EEXIST != Status)
        remove (szVHDPath);
    }
else {
    hVHD = (VHDHANDLE)sim_vhd_disk_open (szVHDPath, "rb+");
    if (!hVHD)
        Status = errno;
    }
errno = Status;
return hVHD;
}

static SMP_FILE* sim_vhd_disk_create (const char *szVHDPath, t_addr desiredsize)
{
return (SMP_FILE* )CreateVirtualDisk (szVHDPath, (uint32)(desiredsize/512), 0, (sim_switches & SWMASK ('X')));
}

static SMP_FILE* sim_vhd_disk_create_diff (const char *szVHDPath, const char *szParentVHDPath)
{
return (SMP_FILE* )CreateDifferencingVirtualDisk (szVHDPath, szParentVHDPath);
}

static t_stat
ReadVirtualDiskSectors(VHDHANDLE hVHD,
                       uint8 *buf,
                       t_seccnt sects,
                       t_seccnt *sectsread,
                       uint32 SectorSize,
                       t_lba lba)
{
uint64 BlockOffset = ((uint64)lba)*SectorSize;
uint32 BlocksRead = 0;
uint32 SectorsInRead;
size_t BytesRead = 0;

if (!hVHD || (hVHD->File == NULL)) {
    errno = EBADF;
    return SCPE_IOERR;
    }
if ((BlockOffset + sects*SectorSize) > (uint64)NtoHll (hVHD->Footer.CurrentSize)) {
    errno = ERANGE;
    return SCPE_IOERR;
    }
if (NtoHl (hVHD->Footer.DiskType) == VHD_DT_Fixed) {
    if (ReadFilePosition(hVHD->File,
                         buf,
                         sects*SectorSize,
                         &BytesRead,
                         BlockOffset)) {
        if (sectsread)
            *sectsread = (t_seccnt)(BytesRead/SectorSize);
        return SCPE_IOERR;
        }
    if (sectsread)
        *sectsread /= SectorSize;
    return SCPE_OK;
    }
/* We are now dealing with a Dynamically expanding or differencing disk */
while (sects) {
    uint32 SectorsPerBlock = NtoHl (hVHD->Dynamic.BlockSize)/SectorSize;
    uint64 BlockNumber = lba/SectorsPerBlock;
    uint32 BitMapBytes = (7+(NtoHl (hVHD->Dynamic.BlockSize)/SectorSize))/8;
    uint32 BitMapSectors = (BitMapBytes+SectorSize-1)/SectorSize;

    SectorsInRead = 1;
    if (hVHD->BAT[BlockNumber] == VHD_BAT_FREE_ENTRY) {
        if (!hVHD->Parent)
            memset (buf, 0, SectorSize);
        else {
            SectorsInRead = SectorsPerBlock - lba%SectorsPerBlock;
            if (SectorsInRead > sects)
                SectorsInRead = sects;
            if (ReadVirtualDiskSectors(hVHD->Parent,
                                       buf,
                                       SectorsInRead,
                                       NULL,
                                       SectorSize,
                                       lba)) {
                if (sectsread)
                    *sectsread = BlocksRead;
                return FALSE;
                }
            }
        }
    else {
        BlockOffset = SectorSize*((uint64)(NtoHl (hVHD->BAT[BlockNumber]) + lba%SectorsPerBlock + BitMapSectors));
        SectorsInRead = SectorsPerBlock - lba%SectorsPerBlock;
        if (SectorsInRead > sects)
            SectorsInRead = sects;
        if (ReadFilePosition(hVHD->File,
                             buf,
                             SectorsInRead*SectorSize,
                             NULL,
                             BlockOffset)) {
            if (sectsread)
                *sectsread = BlocksRead;
            return SCPE_IOERR;
            }
        }
    sects -= SectorsInRead;
    buf = (uint8 *)(((char *)buf) + SectorSize*SectorsInRead);
    lba += SectorsInRead;
    BlocksRead += SectorsInRead;
    }
if (sectsread)
    *sectsread = BlocksRead;
return SCPE_OK;
}

static t_stat sim_vhd_disk_rdsect (UNIT *uptr, t_lba lba, uint8 *buf, t_seccnt *sectsread, t_seccnt sects)
{
VHDHANDLE hVHD = (VHDHANDLE)uptr->fileref;
disk_context* ctx = (disk_context*) uptr->disk_ctx;

return ReadVirtualDiskSectors(hVHD, buf, sects, sectsread, ctx->sector_size, lba);
}

static t_bool
BufferIsZeros(void *Buffer, size_t BufferSize)
{
size_t i;
char *c = (char *)Buffer;

for (i=0; i<BufferSize; ++i)
    if (c[i])
        return FALSE;
return TRUE;
}

static t_bool
VhdBlockHasAllZeroSectors(void *Block, size_t BlockSize)
{
uint8 *BitMap = (uint8 *)Block;
size_t SectorSize = 512;
size_t BitMapSize = (BlockSize/SectorSize+7)/8;
uint8 *Buffer = BitMap + BitMapSize;
size_t Sector;

for (Sector=0; Sector<BlockSize/SectorSize; ++Sector) {
    /* We need Endian rules for BitMap interpretation
       These are not documented in the Version 1.0 specification, AND 
       observations of Virtual PC's and Hyper-V's use of VHD's suggests
       that they really don't manage this detail at the sector level.
       What they appear to do is that whenever a Block is instantiated
       All potential bitmap bits are set to 1 which means that the 
       current block has fully populated data for the current VHD.  
       The same is true in the differencing disk case (i.e. a copy of
       the whole block is made from the parent to the current 
       differencing disk whenever any data is written to a new block). */
    if (!BufferIsZeros(Buffer, SectorSize))
        return FALSE;
    Buffer += SectorSize;
    }
return TRUE;
}

static t_stat
WriteVirtualDiskSectors(VHDHANDLE hVHD,
                        uint8 *buf,
                        t_seccnt sects,
                        t_seccnt *sectswritten,
                        uint32 SectorSize,
                        t_lba lba)
{
uint64 BlockOffset = ((uint64)lba)*SectorSize;
uint32 BlocksWritten = 0;
uint32 SectorsInWrite;
size_t BytesWritten = 0;

if (!hVHD || !hVHD->File) {
    errno = EBADF;
    return SCPE_IOERR;
    }
if ((BlockOffset + sects*SectorSize) > (uint64)NtoHll(hVHD->Footer.CurrentSize)) {
    errno = ERANGE;
    return SCPE_IOERR;
    }
if (NtoHl(hVHD->Footer.DiskType) == VHD_DT_Fixed) {
    if (WriteFilePosition(hVHD->File,
                          buf,
                          sects*SectorSize,
                          &BytesWritten,
                          BlockOffset)) {
        if (sectswritten)
            *sectswritten = (t_seccnt)(BytesWritten/SectorSize);
        return SCPE_IOERR;
        }
    if (sectswritten)
        *sectswritten /= SectorSize;
    return SCPE_OK;
    }
/* We are now dealing with a Dynamically expanding or differencing disk */
while (sects) {
    uint32 SectorsPerBlock = NtoHl(hVHD->Dynamic.BlockSize)/SectorSize;
    uint64 BlockNumber = lba/SectorsPerBlock;
    uint32 BitMapBytes = (7+(NtoHl(hVHD->Dynamic.BlockSize)/SectorSize))/8;
    uint32 BitMapSectors = (BitMapBytes+SectorSize-1)/SectorSize;

    if (BlockNumber >= NtoHl(hVHD->Dynamic.MaxTableEntries)) {
        if (sectswritten)
            *sectswritten = BlocksWritten;
        return SCPE_EOF;
        }
    SectorsInWrite = 1;
    if (hVHD->BAT[BlockNumber] == VHD_BAT_FREE_ENTRY) {
        void *BitMap = NULL;
        void *BlockData = NULL;

        if (!hVHD->Parent && BufferIsZeros(buf, SectorSize))
            goto IO_Done;
        /* Need to allocate a new Data Block. */
        BlockOffset = sim_fsize_ex (hVHD->File);
        if (((int64)BlockOffset) == -1)
            return SCPE_IOERR;
        BitMap = malloc(BitMapSectors*SectorSize);
        memset(BitMap, 0xFF, BitMapBytes);
        BlockOffset -= sizeof(hVHD->Footer);

        /* align the data portion of the block to the desired alignment */
        /* compute the address of the data portion of the block */
        BlockOffset += BitMapSectors*SectorSize;
        /* round up this address to the desired alignment */
        BlockOffset += VHD_DATA_BLOCK_ALIGNMENT-1;
        BlockOffset &= ~(VHD_DATA_BLOCK_ALIGNMENT-1);
        /* the actual block address is the beginning of the block bitmap */
        BlockOffset -= BitMapSectors*SectorSize;

        hVHD->BAT[BlockNumber] = NtoHl((uint32)(BlockOffset/SectorSize));
        if (WriteFilePosition(hVHD->File,
                              BitMap,
                              BitMapSectors*SectorSize,
                              NULL,
                              BlockOffset)) {
            free (BitMap);
            return SCPE_IOERR;
            }
        free(BitMap);
        BitMap = NULL;
        BlockOffset += SectorSize * (SectorsPerBlock + BitMapSectors);
        if (WriteFilePosition(hVHD->File,
                              &hVHD->Footer,
                              sizeof(hVHD->Footer),
                              NULL,
                              BlockOffset))
            goto Fatal_IO_Error;
        if (WriteFilePosition(hVHD->File,
                              hVHD->BAT,
                              SectorSize*((sizeof(*hVHD->BAT)*NtoHl(hVHD->Dynamic.MaxTableEntries)+511)/512),
                              NULL,
                              NtoHll(hVHD->Dynamic.TableOffset)))
            goto Fatal_IO_Error;
        if (hVHD->Parent)
            { /* Need to populate data block contents from parent VHD */
            BlockData = malloc(SectorsPerBlock*SectorSize);
            if (ReadVirtualDiskSectors(hVHD->Parent,
                                       (uint8*) BlockData, 
                                       SectorsPerBlock,
                                       NULL,
                                       SectorSize,
                                       (lba/SectorsPerBlock)*SectorsPerBlock))
                goto Fatal_IO_Error;
            if (WriteVirtualDiskSectors(hVHD,
                                        (uint8*) BlockData, 
                                        SectorsPerBlock,
                                        NULL,
                                        SectorSize,
                                        (lba/SectorsPerBlock)*SectorsPerBlock))
                goto Fatal_IO_Error;
            free(BlockData);
            }
        continue;
Fatal_IO_Error:
        free (BitMap);
        free (BlockData);
        fclose (hVHD->File);
        hVHD->File = NULL;
        return SCPE_IOERR;
        }
    else {
        BlockOffset = 512*((uint64)(NtoHl(hVHD->BAT[BlockNumber]) + lba%SectorsPerBlock + BitMapSectors));
        SectorsInWrite = SectorsPerBlock - lba%SectorsPerBlock;
        if (SectorsInWrite > sects)
            SectorsInWrite = sects;
        if (WriteFilePosition(hVHD->File,
                              buf,
                              SectorsInWrite*SectorSize,
                              NULL,
                              BlockOffset)) {
            if (sectswritten)
                *sectswritten = BlocksWritten;
            return SCPE_IOERR;
            }
        }
IO_Done:
    sects -= SectorsInWrite;
    buf = (uint8 *)(((char *)buf) + SectorsInWrite*SectorSize);
    lba += SectorsInWrite;
    BlocksWritten += SectorsInWrite;
    }
if (sectswritten)
    *sectswritten = BlocksWritten;
return SCPE_OK;
}

static t_stat sim_vhd_disk_wrsect (UNIT *uptr, t_lba lba, uint8 *buf, t_seccnt *sectswritten, t_seccnt sects)
{
VHDHANDLE hVHD = (VHDHANDLE)uptr->fileref;
disk_context* ctx = (disk_context*) uptr->disk_ctx;

return WriteVirtualDiskSectors(hVHD, buf, sects, sectswritten, ctx->sector_size, lba);
}
#endif