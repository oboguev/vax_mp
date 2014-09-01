/*
 * PAGE.C - program for ST-PAGE stress test
 *
 * Executes a loop filling virtual memory pages at random with data, then verifying content some time later
 * and overwriting the content again. Causes high paging rate and stress-tests paging system to verify its
 * integrity.
 *
 * Usage: PAGE proc# npages
 *
 *     proc# is a process number
 *
 *     npages is the number of virtual pages to use in stress buffer
 *
 * ToDo:
 *
 *     1) Implement test to page global memory section, in addition to private pages.
 *        50% of paging can go to global section, 50% can go to private pages.
 *
 *     2) Implement test to page global memory section actually shared by process instances.
 *        Each instance should write to and read back its data from a slot in each of the section's pages.
 *        The slot is defined by process index. Thus there can be up to 512 / 4 = 128 process instances,
 *        or even more if word or byte reads/writes are used instead of longword reads/writes.
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

#define PAGESIZE 512

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
*  Module-global data                                                                  *
***************************************************************************************/

static uint32 process_number;
static uint32 npages;
static uint32 random_id;
static uint32 tm_start[2];
static uint32 pid;
static byte_t* ppages = NULL;
static uint32* ptrack = NULL;

/***************************************************************************************
*  Local function prototypes                                                           *
***************************************************************************************/

static void usage();
static void bad(const char* msg);
static void bad_st(const char* msg, uint32 status);
static void baderr(const char* msg);
static uint32 mkrand();

/***************************************************************************************
*  Main routine                                                                        *
***************************************************************************************/

int main(int argc, char** argv)
{
    uint32 status;
    int k;
    char c;
    uint32* pp;
    uint32 val;

    /*
     * Parse arguments
     */
    if (argc != 3)
        usage();

    if (1 != sscanf(argv[1], "%ud%c", &process_number, & c))
        usage();

    if (1 != sscanf(argv[2], "%ud%c", &npages, & c))
        usage();

    /*
     * Initialize randomness
     */
    check_vms_status(sys$gettim(& tm_start));
    pid = getpid();
    random_id = hash32(tm_start[0]) ^ hash32(pid);
    random_id = hash32(random_id) ^ hash32(process_number);

    /*
     * Allocate pages
     */
    check_vms_status(lib$get_vm_page(&npages, &ppages));

    /*
     * Allocate tracking buffer
     */
    k = (npages * sizeof(uint32)) / PAGESIZE + 1;
    check_vms_status(lib$get_vm_page(&k, &ptrack));
    memset(ptrack, 0, npages * sizeof(uint32));

    /*
     * Perform main loop
     */
    for (;;)
    {
        /* get page index at random */
        uint32 index = mkrand() % npages;

        uint32* page = (uint32*) (ppages + PAGESIZE * index);

        /* check if page was filled and if so verify its content */
        if (val = ptrack[index])
        {
            pp = page;
#define CHKVAL  if (val != *pp++)  goto badvalue
            for (k = 0;  k < PAGESIZE / (32 * sizeof(uint32));  k++)
            {
                CHKVAL;  CHKVAL;  CHKVAL;  CHKVAL;
                CHKVAL;  CHKVAL;  CHKVAL;  CHKVAL;
                CHKVAL;  CHKVAL;  CHKVAL;  CHKVAL;
                CHKVAL;  CHKVAL;  CHKVAL;  CHKVAL;
                CHKVAL;  CHKVAL;  CHKVAL;  CHKVAL;
                CHKVAL;  CHKVAL;  CHKVAL;  CHKVAL;
                CHKVAL;  CHKVAL;  CHKVAL;  CHKVAL;
                CHKVAL;  CHKVAL;  CHKVAL;  CHKVAL;
            }
        }

        /* generate new filler value and fill the page */
        val = ptrack[index] = mkrand();
        pp = page;
        for (k = 0;  k < PAGESIZE / (32 * sizeof(uint32));  k++)
        {
            *pp++ = val;  *pp++ = val;  *pp++ = val;  *pp++ = val;
            *pp++ = val;  *pp++ = val;  *pp++ = val;  *pp++ = val;
            *pp++ = val;  *pp++ = val;  *pp++ = val;  *pp++ = val;
            *pp++ = val;  *pp++ = val;  *pp++ = val;  *pp++ = val;
            *pp++ = val;  *pp++ = val;  *pp++ = val;  *pp++ = val;
            *pp++ = val;  *pp++ = val;  *pp++ = val;  *pp++ = val;
            *pp++ = val;  *pp++ = val;  *pp++ = val;  *pp++ = val;
            *pp++ = val;  *pp++ = val;  *pp++ = val;  *pp++ = val;
        }
    }

    return SS$_NORMAL;

cleanup:
    exit(status);

badvalue:
    fprintf(stderr, "Memory consistency verification failed, expected %08X, found %08X\n", val, pp[-1]);
}

static uint32 mkrand()
{
    uint32 status;

    for (;;)
    {
        random_id = hash32((random_id + tm_start[0] + process_number) ^ pid);
        if (random_id)  return random_id;
        check_vms_status(sys$gettim(& tm_start));
    }

cleanup:
    exit(status);
}

static void usage()
{
    fprintf(stderr, "usage: PAGE proc# npages\n");
    exit(SS$_INVARG | STS$M_INHIB_MSG);
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
