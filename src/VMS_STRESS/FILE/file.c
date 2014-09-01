/*
 * FILE.C - program for ST-FILE stress test
 *
 * Executes a loop filling file with generated content, then reading it back and checking
 * if content is valid.
 *
 * Usage: FILE {RMS|QIO} proc# [passes]
 *
 *     RMS mode uses RMS for IO thus passing through RMS and FCP/volume caches
 *     QIO uses directed IO bypassing cache (IO$_READVBLK|IO$M_NOVCACHE and IO$_WRITEVBLK|IO$M_NOVCACHE)
 *     CRT mode does not work: fseek seems not to position properly and somehow garbles file content
 *
 *     proc# is a process number
 *
 *     passes is a number of passes to execute before terminating, if 0 or omitted then infinite
 *
 */

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <varargs.h>
#include <ssdef.h>
#include <stsdef.h>
#include <descrip.h>
#include <starlet.h>
#include <math.h>
#include <libwaitdef.h>
#include <lib$routines.h>
#include <unistd.h>
#include <rms.h>
#include <iodef.h>

/***************************************************************************************
*  Helper macros, type definitions etc.                                                *
***************************************************************************************/

#ifndef FALSE
#  define FALSE 0
#endif

#ifndef TRUE
#  define TRUE 1
#endif

#ifndef MAX_PATH
#  define MAX_PATH 512
#endif

#define CHECK(cond)  do { if (! (cond))  goto cleanup; } while (0)

#define check(cond)  do { if (! (cond)) { status = SS$_ABORT; goto cleanup; } } while (0)
#define check_vms(st)  do { if (! $VMS_STATUS_SUCCESS(st))  goto cleanup; } while (0)
#define check_vms_status(st)  do { status = (st); if (! $VMS_STATUS_SUCCESS(status))  goto cleanup; } while (0)
#define fail(st)  do { status = (st); goto cleanup; } while (0)
#define vms_success(st)  $VMS_STATUS_SUCCESS(st)

#define streqi(s1, s2)  (0 == strcasecmp((s1), (s2)))
#define countof(a) (sizeof(a) / sizeof((a)[0]))

typedef unsigned int uint32;
typedef unsigned short uint16;
typedef unsigned char byte_t;
typedef unsigned int bool_t;

typedef struct __iosb
{
    uint16 iosb$w_status;
    uint16 iosb$w_count;
    uint32 iosb$l_extended;
}
IOSB;

typedef struct __VA_RANGE
{
    void* start;
    void* end;
}
VA_RANGE;

typedef struct dsc$descriptor DESC;

static void mkdesc(DESC* dsc, const char* s)
{
    dsc->dsc$w_length = (uint16) strlen((char*) s);
    dsc->dsc$a_pointer = (char*) s;
    dsc->dsc$b_dtype = DSC$K_DTYPE_T;
    dsc->dsc$b_class = DSC$K_CLASS_S;
}

static inline uint32 hash32(uint32 key)
{
    key += key << 12;
    key ^= key >> 22;
    key += key << 4;
    key ^= key >> 9;
    key += key << 10;
    key ^= key >> 2;
    key += key << 7;
    key ^= key >> 12;
    return key;
}

/***************************************************************************************
*  Configuration parameters                                                            *
***************************************************************************************/

/* blocks to write and read in a single iteration */
#define NBLOCKS  128

/***************************************************************************************
*  Module-global data                                                                  *
***************************************************************************************/

#define IO_METHOD_RMS   0
#define IO_METHOD_QIO   1
#define IO_METHOD_CRT   2

static uint32 io_method = IO_METHOD_RMS;
static uint32 process_number;
static uint32 random_id;
static uint32 block_seq = 1;
byte_t* pblocks = NULL;     /* buffer of NBLOCKS pagelets */
byte_t* ppage = NULL;       /* buffer of one pagelet */
static uint32 tm_start[2];
static uint32 pid;
static FILE* fp = NULL;
static char work_file_name[MAX_PATH];
uint32 passes = 0;

static struct FAB fab;
static struct RAB rab;
static bool_t rms_open = FALSE;
static bool_t qio_open = FALSE;
static uint32 efn = 0;

/***************************************************************************************
*  Local function prototypes                                                           *
***************************************************************************************/

static void usage();
static void fill_next_block(byte_t* pb, uint32 blkindex);
void create_file_conditional();
static void close_reopen_file();
static void close_file();
static void delete_file();
static void write_block(long offset, byte_t* pb);
static void read_block(long offset, byte_t* pb);
static void bad(const char* msg);
static void bad_st(const char* msg, uint32 status);
static void baderr(const char* msg);
FILE* do_fopen(const char* mode);

/***************************************************************************************
*  Main routine                                                                        *
***************************************************************************************/

int main(int argc, char** argv)
{
    uint32 status;
    uint32 nblocks;
    uint32 pass;
    int k;
    char c;

    /*
     * Parse arguments
     */

    if (argc != 3 && argc != 4)
        usage();

    if (streqi(argv[1], "rms"))
        io_method = IO_METHOD_RMS;
    else if (streqi(argv[1], "qio"))
        io_method = IO_METHOD_QIO;
    else if (streqi(argv[1], "crt"))
        io_method = IO_METHOD_CRT;
    else
        usage();

    if (1 != sscanf(argv[2], "%ud%c", &process_number, & c))
        usage();

    if (argc == 4)
    {
        if (1 != sscanf(argv[3], "%ud%c", &passes, & c))
            usage();
    }

    /*
     * Initialize aux data
     */
    status = lib$get_ef(& efn);
    if (! vms_success(status))
        bad_st("unable to allocate event flag", status);

    /*
     * Initialize randomness
     */
    check_vms_status(sys$gettim(& tm_start));
    pid = getpid();
    random_id = hash32(tm_start[0]) ^ hash32(pid);
    random_id = hash32(random_id) ^ hash32(process_number);

    /*
     * Allocate read-write buffers
     */
    nblocks = NBLOCKS + 1;
    check_vms_status(lib$get_vm_page(&nblocks, &ppage));
    pblocks = ppage + 512;

    /*
     * Perform main loop
     */
    sprintf(work_file_name, "temp%d.tmp;1", process_number);
    delete_file();
    for (pass = 0; ; pass++)
    {
        /* create or open file if necessary */
        create_file_conditional();

        /* generate data blocks */
        for (k = 0;  k < NBLOCKS;  k++)
            fill_next_block(pblocks + 512 * k, k);

        /* write sequence of NBLOCKS blocks */
        for (k = 0;  k < NBLOCKS;  k++)
            write_block(k * 512, pblocks + 512 * k);

        /* every second time: close/reopen file */
        if ((pass % 2) == 0)
            close_reopen_file();

        /* read blocks in reverse order and compare */
        for (k = NBLOCKS - 1;  k >= 0;  k--)
        {
            read_block(k * 512, ppage);
            if (0 != memcmp(ppage, pblocks + 512 * k, 512))
            {
                uint32* pr = (uint32*) ppage;
                uint32* pw = (uint32*) (pblocks + 512 * k);
                fprintf(stderr, "Read/write data mismatch: process %d, pass %d, blk %d\n    wr: (%X, %X, %X)\n    rd: (%X, %X, %X)\n",
                        process_number, pass, k, pw[0], pw[1], pw[2], pr[0], pr[1], pr[2]);
                exit(SS$_ABORT | STS$M_INHIB_MSG);
            }
        }

        /* after each 10th time close and delete file and re-create it later */
        if ((pass % 10) == 0)
        {
            close_file();
            delete_file();
        }

        /* after specified number of iterations exit and delete file */
        if (passes && pass + 1 == passes)
        {
            close_file();
            delete_file();
            break;
        }
    }

    return SS$_NORMAL;

cleanup:
    exit(status);
}

static void fill_next_block(byte_t* pb, uint32 blkindex)
{
    uint32* px = (uint32*) pb;
    int k;

    *px++ = (process_number << 16) | blkindex;
    *px++ = block_seq++;
    for (k = 0;  k < 512/sizeof(uint32) - 2;  k++)
    {
        *px++ = random_id;
        random_id = hash32((random_id + tm_start[0] + process_number) ^ pid);
    }
}

static void usage()
{
    fprintf(stderr, "usage: FILE {RMS|QIO} proc# [passes]\n");
    exit(SS$_INVARG | STS$M_INHIB_MSG);
}

void create_file_conditional()
{
    uint32 status;

    if (io_method == IO_METHOD_CRT)
    {
        if (fp == NULL)
        {
            fp = do_fopen("rb+");
            if (fp == NULL)
            {
                if (fp  = do_fopen("wb"))
                {
                    fclose(fp);
                    fp = do_fopen("rb+");
                }
            }
            if (fp == NULL)
                baderr("unable to create or open file");
        }
        if (fseek(fp, 0, SEEK_SET))
            baderr("unable to rewind file");
    }
    else if (io_method == IO_METHOD_RMS)
    {
        if (! rms_open)
        {
            fab = cc$rms_fab;
            fab.fab$l_fna = work_file_name;
            fab.fab$b_fns = strlen(work_file_name);
            fab.fab$l_alq = NBLOCKS;
            fab.fab$b_fac = FAB$M_BIO|FAB$M_GET|FAB$M_PUT;
            fab.fab$v_cif = 1;
            fab.fab$w_gbc = 2;
            fab.fab$w_mrs = 512;
            fab.fab$b_rfm = FAB$C_FIX;
            fab.fab$v_nil = 1;

            status = sys$create(&fab);
            if (! vms_success(status))
                bad_st("unable to create or open work file", status);

            rab = cc$rms_rab;
            rab.rab$l_fab = & fab;
            rab.rab$w_rsz = 512;
            rab.rab$l_rbf = (char*) ppage;
            rab.rab$w_usz = 512;
            rab.rab$l_ubf = (char*) pblocks;
            rab.rab$b_mbf = 1;
            rab.rab$b_rac = RAB$C_RFA;
            rab.rab$v_bio = 1;
            rab.rab$v_wbh = 1; 

            status = sys$connect(&rab);
            if (! vms_success(status))
                bad_st("unable to create or open work file (sys$connect failed)", status);

            rms_open = TRUE;
        }
        else
        {
            status = sys$rewind(&rab);
            if (! vms_success(status))
                bad_st("unable to position in work file (sys$rewind failed)", status);
        }
    }
    else
    {
        if (! qio_open)
        {
            fab = cc$rms_fab;
            fab.fab$l_fna = work_file_name;
            fab.fab$b_fns = strlen(work_file_name);
            fab.fab$l_alq = NBLOCKS;
            fab.fab$b_fac = FAB$M_BIO|FAB$M_GET|FAB$M_PUT;
            fab.fab$v_cif = 1;
            fab.fab$w_gbc = 2;
            fab.fab$w_mrs = 512;
            fab.fab$b_rfm = FAB$C_FIX;
            fab.fab$v_nil = 1;
            fab.fab$v_ufo = 1;

            status = sys$create(&fab);
            if (! vms_success(status))
                bad_st("unable to create or open work file", status);

            qio_open = TRUE;
        }
    }
}

FILE* do_fopen(const char* mode)
{
    char alq[32];
    sprintf(alq, "alq=%d", NBLOCKS);
    return fopen(work_file_name, mode, alq, "rfm=fix", "mrs=512", "ctx=bin", "fop=sup", "rop=asy,syncsts,wbh", "gbc=2", "mbf=1", "shr=nil");
    // return fopen(work_file_name, mode, "rfm=fix", "mrs=512");
}

static void close_reopen_file()
{
    close_file();
    create_file_conditional();
}

static void close_file()
{
    uint32 status;

    if (io_method == IO_METHOD_CRT)
    {
        if (fp)
        {
            if (fclose(fp))
                baderr("unable to close file");
            fp = NULL;
        }
    }
    else if (io_method == IO_METHOD_RMS)
    {
        if (rms_open)
        {
            status = sys$disconnect(&rab);
            if (! vms_success(status))
                bad_st("unable close work file (sys$disconnect failed)", status);
            status = sys$close(&fab);
            if (! vms_success(status))
                bad_st("unable to close work file (sys$close failed)", status);
            rms_open = FALSE;
        }
    }
    else
    {
        if (qio_open)
        {
            sys$dassgn(fab.fab$l_stv);
            qio_open = FALSE;
        }
    }
}

static void delete_file()
{
    close_file();
    remove(work_file_name);
    if (0 == access(work_file_name, F_OK))
        bad("was unable to delete work file");
}

static void write_block(long offset, byte_t* pb)
{
    uint32 status;

    if (io_method == IO_METHOD_CRT)
    {
        if (fwrite(pb, 1, 512, fp) != 512 || fflush(fp) || fsync(fileno(fp)))
            baderr("write to file failed");
    }
    else if (io_method == IO_METHOD_RMS)
    {
        rab.rab$l_rbf = (char*) pb;
        rab.rab$w_rsz = 512;
        rab.rab$l_bkt = offset / 512 + 1;
        status = sys$write(& rab);
        if (! vms_success(status))
            bad_st("unable to write work file (sys$write failed)", status);
        status = sys$flush(& rab);
        if (! vms_success(status))
            bad_st("unable to write work file (sys$flush failed)", status);
    }
    else
    {
        IOSB iosb;
        uint32 vbn = offset / 512 + 1;

        status = sys$qiow(efn, fab.fab$l_stv, IO$_WRITEVBLK|IO$M_NOVCACHE, &iosb, NULL, 0, pb, 512, vbn, 0, 0, 0);
        if (! vms_success(status))
            bad_st("unable to write work file (sys$qio failed)", status);
        if (! vms_success(iosb.iosb$w_status))
            bad_st("unable to write work file (sys$qio failed)", iosb.iosb$w_status);
        if (iosb.iosb$w_count != 512)
            baderr("incomplete block write");
    }
}

static void read_block(long offset, byte_t* pb)
{
    uint32 status;

    if (io_method == IO_METHOD_CRT)
    {
        int count;
        if (fseek(fp, offset, SEEK_SET))
            baderr("unable to position in file");
        count = fread(pb, 1, 512, fp);
        if (count != 512)
        {
            if (count < 0)
            {
                baderr("reading from file failed");
            }
            else
            {
                fprintf(stderr, "Error: reading from file failed (read %d instead of 512)\n", count);
                exit(SS$_ABORT | STS$M_INHIB_MSG);
            }
        }
    }
    else if (io_method == IO_METHOD_RMS)
    {
        rab.rab$l_ubf = (char*) pb;
        rab.rab$w_usz = 512;
        rab.rab$l_bkt = offset / 512 + 1;
        status = sys$read(&rab);
        if (! vms_success(status))
            bad_st("unable to read work file (sys$read failed)", status);
        if (rab.rab$w_rsz != 512)
            baderr("incomplete block read");
        if (rab.rab$l_rbf != (char*) pb)
            memcpy(pb, (void*) rab.rab$l_rbf, 512);
    }
    else
    {
        IOSB iosb;
        uint32 vbn = offset / 512 + 1;

        status = sys$qiow(efn, fab.fab$l_stv, IO$_READVBLK|IO$M_NOVCACHE, &iosb, NULL, 0, pb, 512, vbn, 0, 0, 0);
        if (! vms_success(status))
            bad_st("unable to read work file (sys$qio failed)", status);
        if (! vms_success(iosb.iosb$w_status))
            bad_st("unable to read work file (sys$qio failed)", iosb.iosb$w_status);
        if (iosb.iosb$w_count != 512)
            baderr("incomplete block read");
    }
}

static void bad(const char* msg)
{
    fprintf(stderr, "Error: %s\n", msg);
    exit(SS$_ABORT | STS$M_INHIB_MSG);
}

static void bad_st(const char* msg, uint32 status)
{
    fprintf(stderr, "Error: %s\n", msg);
    exit(status);
}

static void baderr(const char* msg)
{
    fprintf(stderr, "Error: ");
    perror(msg);
    exit(SS$_ABORT | STS$M_INHIB_MSG);
}
