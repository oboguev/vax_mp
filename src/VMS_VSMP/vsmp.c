/*
 * VSMP.C 
 *
 *    Activate and manage virtual SMP for OpenVMS when executing on SIMH VAX MP simulator
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
#include <chfdef.h>
#include <descrip.h>
#include <libwaitdef.h>
#include <lib$routines.h>
#include <starlet.h>
#include <lckdef.h>
#include <syidef.h>
#include <psldef.h>
#include <lnmdef.h>
#include <dcdef.h>
#include <math.h>

#ifndef SYI$_NODE_SYSTEMID
#  define SYI$_NODE_SYSTEMID 4307
#endif

/***************************************************************************************
*  Status codes                                                                        *
***************************************************************************************/

globalvalue unsigned int VSMP_MSG_SYNTAX;
globalvalue unsigned int VSMP_MSG_MISLINKED;
globalvalue unsigned int VSMP_MSG_MISBUILT;
globalvalue unsigned int VSMP_MSG_SYS_NOT_VAXMP;
globalvalue unsigned int VSMP_MSG_ALREADY_SMP;
globalvalue unsigned int VSMP_MSG_ALREADY_LOADED;
globalvalue unsigned int VSMP_MSG_NOT_LOADED;
globalvalue unsigned int VSMP_MSG_VERSION_MISMATCH;
globalvalue unsigned int VSMP_MSG_VMS_NOT_MULTI;
globalvalue unsigned int VSMP_MSG_CALIBR_UNSTABLE;
globalvalue unsigned int VSMP_MSG_LDR_VERIFY;
globalvalue unsigned int VSMP_MSG_LDR_SCH_CUR_TO_COM;
globalvalue unsigned int VSMP_MSG_LDR_EXE_PROC_IDLE;
globalvalue unsigned int VSMP_MSG_VM_REFUSED;
globalvalue unsigned int VSMP_MSG_IDLE_NEVER;
globalvalue unsigned int VSMP_MSG_UNMOD_DRV;
globalvalue unsigned int VSMP_MSG_INVOPTVAL;
globalvalue unsigned int VSMP_MSG_IVPATCHID;
globalvalue unsigned int VSMP_MSG_XDTINVALID;
globalvalue unsigned int VSMP_MSG_MSGRTNSLDR;
globalvalue unsigned int VSMP_MSG_XQTIMXMT_P;
globalvalue unsigned int VSMP_MSG_UNSUPPXQ;
globalvalue unsigned int VSMP_MSG_XQTIMXMT_C;
globalvalue unsigned int VSMP_MSG_XQTX1_P;
globalvalue unsigned int VSMP_MSG_XQTX2_P;
globalvalue unsigned int VSMP_MSG_XQTX3_P;
globalvalue unsigned int VSMP_MSG_XQTX4_P;
globalvalue unsigned int VSMP_MSG_XQTX5_P;
globalvalue unsigned int VSMP_MSG_XQTX6_P;
globalvalue unsigned int VSMP_MSG_XQTX7_P;
globalvalue unsigned int VSMP_MSG_XQTX8_P;
globalvalue unsigned int VSMP_MSG_XQTX9_P;
globalvalue unsigned int VSMP_MSG_XQTX10_P;
globalvalue unsigned int VSMP_MSG_XQRX1_P;
globalvalue unsigned int VSMP_MSG_XQRX2_P;
globalvalue unsigned int VSMP_MSG_XQRX3_P;
globalvalue unsigned int VSMP_MSG_XQRX4_P;
globalvalue unsigned int VSMP_MSG_PU1_P;
globalvalue unsigned int VSMP_MSG_PU2_P;
globalvalue unsigned int VSMP_MSG_PU3_P;
globalvalue unsigned int VSMP_MSG_PU4_P;
globalvalue unsigned int VSMP_MSG_PU5_P;
globalvalue unsigned int VSMP_MSG_PU6_P;
globalvalue unsigned int VSMP_MSG_PU7_P;
globalvalue unsigned int VSMP_MSG_NONATIVE;
globalvalue unsigned int VSMP_MSG_SYNCWIDLEOFF;
globalvalue unsigned int VSMP_MSG_IVPARSET;
globalvalue unsigned int VSMP_MSG_IVLOCKRTRY;
globalvalue unsigned int VSMP_MSG_VLOW_SW_SYS;
globalvalue unsigned int VSMP_MSG_VLOW_SW_ILK;
globalvalue unsigned int VSMP_MSG_LOW_SW_SYS;
globalvalue unsigned int VSMP_MSG_LOW_SW_ILK;
globalvalue unsigned int VSMP_MSG_ADV_SW_SYS;
globalvalue unsigned int VSMP_MSG_ADV_SW_ILK;
globalvalue unsigned int VSMP_MSG_SYS_LESS_ILK;
globalvalue unsigned int VSMP_MSG_ADV_ISW_SYS;
globalvalue unsigned int VSMP_MSG_UNSTART;
globalvalue unsigned int VSMP_MSG_SPWHIGH;
globalvalue unsigned int VSMP_MSG_ADV_SPW_HIGH;
globalvalue unsigned int VSMP_MSG_LSPWHIGH;
globalvalue unsigned int VSMP_MSG_ADV_LSPW_HIGH;

globalvalue unsigned int VSMP_MSG_CALIBRATING;
globalvalue unsigned int VSMP_MSG_CALIBRATED;
globalvalue unsigned int VSMP_MSG_CALIBRETRY;
globalvalue unsigned int VSMP_MSG_LOADED;

/***************************************************************************************
*  Helper macros, type definitions etc.                                                *
***************************************************************************************/

#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

#define EXTERN globalref
#define INLINE __inline

#define CHECK(cond)  do { if (! (cond))  goto cleanup; } while (0)

#define check(cond)  do { if (! (cond)) { status = SS$_ABORT; goto cleanup; } } while (0)
#define check_vms(st)  do { if (! $VMS_STATUS_SUCCESS(st))  goto cleanup; } while (0)
#define check_vms_status(st)  do { status = (st); if (! $VMS_STATUS_SUCCESS(status))  goto cleanup; } while (0)
#define fail(st)  do { status = (st); goto cleanup; } while (0)

#define streqi(s1, s2)  (0 == strcasecmp((s1), (s2)))

#define memzero(m)  memset(&m, 0, sizeof(m))
#define memzero_ar(m)  memset(m, 0, sizeof(m))

#define offset_ptr(type, base, offset)  ((type*) ((byte_t*)(base) + (offset)))
#define offset_ref(type, base, offset)  (* offset_ptr(type, (base), (offset)))

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

typedef struct __lksb
{
    uint16 lksb$w_status;
    uint16 lksb$w_reserved;
    uint32 lksb$l_lock_id;
    byte_t lksb$b_value_blk[64];
}
LKSB;

typedef struct __ILE3
{
    uint16 ile3$w_length;
    uint16 ile3$w_code;
    void* ile3$ps_bufaddr;
    uint16* ile3$ps_retlen_addr ;
}
ILE3;

typedef struct __VA_RANGE
{
    void* start;
    void* end;
}
VA_RANGE;

typedef struct dsc$descriptor DESC;

static int kstrlen(const char* p);

static void mkdesc(DESC* dsc, const char* s)
{
    dsc->dsc$w_length = (uint16) kstrlen((char*) s);
    dsc->dsc$a_pointer = (char*) s;
    dsc->dsc$b_dtype = DSC$K_DTYPE_T;
    dsc->dsc$b_class = DSC$K_CLASS_S;
}

static void ile3_make(ILE3* ile, uint16 code, uint16 length, void* bufaddr, uint16* retlen)
{
    ile->ile3$w_code = code;
    ile->ile3$w_length = length;
    ile->ile3$ps_bufaddr = bufaddr;
    ile->ile3$ps_retlen_addr  = retlen;
}

static void ile3_term(ILE3* ile)
{
    ile->ile3$w_code = 0;
    ile->ile3$w_length = 0;
    ile->ile3$ps_bufaddr = NULL;
    ile->ile3$ps_retlen_addr  = NULL;
}

INLINE static int imin(int a, int b)
{
    return (a < b) ? a : b;
}

INLINE static int imax(int a, int b)
{
    return (a > b) ? a : b;
}

INLINE static int uimin(uint32 a, uint32 b)
{
    return (a < b) ? a : b;
}

INLINE static int uimax(uint32 a, uint32 b)
{
    return (a > b) ? a : b;
}

/* call routine at relocated address inside the kernel-resident copy of the loadable code */
#define kcall_reloc(rtype, routine) ((rtype) ((char*) kaddress + ((char*) (rtype) (routine) - (char*) &kload_start)))
#define RCALL(rtype, routine) (*kcall_reloc(rtype, routine))

/* access data at relocated address inside the kernel-resident copy of the loadable code */
#define kdata_reloc(dtype, dname) (* (dtype*) ((char*) kaddress + ((char*) (dtype*) (&(dname)) - (char*) &kload_start)))
#define kdata_reloc_ptr(ptype, ptr) ((ptype) ((char*) kaddress + ((char*) (ptr) - (char*) &kload_start)))

typedef uint32 (*kcall_onload_t)(uint32 cpu_mask, uint32 idle, uint32 timesync, uint32 nopatch);
typedef uint32 (*kcall_get_idle_t)(uint32* idle);
typedef uint32 (*kcall_set_idle_t)(uint32 idle);
typedef uint32 (*kcall_get_timesync_t)(uint32* timesync);
typedef uint32 (*kcall_set_timesync_t)(uint32 timesync);
typedef void   (*patch_xqdrv_instr_t)(void* addr_mov_xmt_tmo, uint32 xqtimeout);
typedef bool_t (*lookup_code_checker_t)(const byte_t* xaddr, void* arg);
typedef bool_t (*set_patchdesc_lookup_found_t)(uint32 patch_id, const void* addr);
typedef void   (*setup_patch_t)(const void* addr);

#define SIM_K_IDLE_OFF      0
#define SIM_K_IDLE_ON       1
#define SIM_K_IDLE_NEVER    2

#define SIM_K_TIMESYNC_OFF      0
#define SIM_K_TIMESYNC_ON       1

#define MAXARGSIZE 64

#define XQ_MIN_TIMEOUT 5
#define XQ_MAX_TIMEOUT 255
#define XQ_DEF_TIMEOUT 255

/***************************************************************************************
*  SIMH VAX MP API definitions                                                         *
***************************************************************************************/

#define VAXMP_API_SIGNATURE    0x484D4953

#define VAXMP_API_OP_QUERY     1
#define VAXMP_API_OP_IDLE      2

#define VAXMP_VM_ID            0x504D5856  /* 'VXMP' */

#define VAXMP_SMP_OPTION_PORTABLE_INTERLOCK  (1 << 0)
#define VAXMP_SMP_OPTION_NATIVE_INTERLOCK    (1 << 1)

#define SYNCW_SYS  (1 << 0)
#define SYNCW_ILK  (1 << 1)

/***************************************************************************************
*  Module-global data                                                                  *
***************************************************************************************/

/* SIMH QUERY data */
static volatile bool_t simh_present = FALSE;
static uint32 simh_api_version;
static uint32 simh_ncpus;
static uint32 simh_cpu_mask;
static uint32 simh_vm_id;
static uint32 simh_vm_version;
static uint32 simh_smp_revision;
static uint32 simh_smp_options;
static uint32 simh_smp_multiplier;
static uint32 simh_smp_divider;
static uint32 simh_host_turbo_factor;
static uint32 simh_syncw;

static byte_t* kaddress = 0;           /* address of resident image */

static uint32 exe_tenusec;             /* recalculated loop calibration counters */
static uint32 exe_ubdelay;             /* ... */

static uint32 kerror_cause_usr = 0;    /* cause of error (user-mode copy of kerror_cause, 
                                          copied from kerror_cause in resident image in S0) */

static uint32 kargs_none[1] = { 0 };

static uint32 smp_idle = SIM_K_IDLE_ON;
static uint32 smp_timesync = SIM_K_TIMESYNC_ON;

static int pfork_pool_pages = 1;

static uint32 nopatch = 0;                    /* NOPATCH flags */
static uint32 xqtimeout = XQ_DEF_TIMEOUT;

static void* xqdrv_dpt = NULL;                /* XQDRIVER DPT location */
static uint32 xqdrv_size = 0;                 /* XQDRIVER size */
static byte_t xqdrv_orig_xmt_tmo = 0;
static void* xqdrv_addr_xmt_tmo = NULL;
static uint32 xqdrv_lsb_xmt_tmo = 0;

static void* pudrv_dpt = NULL;                /* PUDRIVER DPT location */
static uint32 pudrv_size = 0;                 /* PUDRIVER size */

static uint32 syncw_sys_pct = 66;             /* SYS synchronization window size (percent of max) */
static uint32 syncw_ilk_pct = 66;             /* ILK synchronization window size (percent of max) */

static bool_t use_native_interlock = FALSE;

/***************************************************************************************
*  External references                                                                 *
***************************************************************************************/

EXTERN byte_t kload_start;
EXTERN byte_t kload_end;
EXTERN byte_t kload_blksize;
EXTERN byte_t kload_wslock_start;
EXTERN byte_t kload_wslock_end;
EXTERN uint32 kerror_cause;
EXTERN uint32 xdelta_range[2];
EXTERN uint32 msg_rtns_range[2];
EXTERN const byte_t* lookup_code_addr;

EXTERN uint32 syncw_on;                       /* SYNCW_SYS, SYNCW_ILK */
EXTERN uint32 syncw_winsize_sys;              /* SYS synchronization window size (cycles) */
EXTERN uint32 syncw_winsize_ilk;              /* ILK synchronization window size (cycles) */

EXTERN uint32 smp_must_options;
EXTERN uint32 smp_want_options;

/***************************************************************************************
*  External references to system data                                                  *
***************************************************************************************/

EXTERN uint32 exe$gl_lockrtry;
EXTERN uint32 sgn$gl_smp_lngspinwait;
EXTERN uint32 sgn$gl_smp_spinwait;
EXTERN uint32 exe$gl_time_control;

/***************************************************************************************
*  Dynamic patch IDs                                                                   *
***************************************************************************************/

globalvalue unsigned int PATCH_ID_XDELTA;
globalvalue unsigned int PATCH_ID_CHSEP;
globalvalue unsigned int PATCH_ID_RESCHED;
globalvalue unsigned int PATCH_ID_NUMTIM;
globalvalue unsigned int PATCH_ID_MFYCAP;
globalvalue unsigned int PATCH_ID_LOCKRTRY;
globalvalue unsigned int PATCH_ID_XQTIMXMT;
globalvalue unsigned int PATCH_ID_UCBTMO;
globalvalue unsigned int PATCH_ID_CRBTMO;
globalvalue unsigned int PATCH_ID_XQTX1;
globalvalue unsigned int PATCH_ID_XQTX2;
globalvalue unsigned int PATCH_ID_XQTX3;
globalvalue unsigned int PATCH_ID_XQTX4;
globalvalue unsigned int PATCH_ID_XQTX5;
globalvalue unsigned int PATCH_ID_XQTX6;
globalvalue unsigned int PATCH_ID_XQTX7;
globalvalue unsigned int PATCH_ID_XQTX8;
globalvalue unsigned int PATCH_ID_XQTX9;
globalvalue unsigned int PATCH_ID_XQTX10;
globalvalue unsigned int PATCH_ID_XQRX1;
globalvalue unsigned int PATCH_ID_XQRX2;
globalvalue unsigned int PATCH_ID_XQRX3;
globalvalue unsigned int PATCH_ID_XQRX4;
globalvalue unsigned int PATCH_ID_PU1;
globalvalue unsigned int PATCH_ID_PU2;
globalvalue unsigned int PATCH_ID_PU3;
globalvalue unsigned int PATCH_ID_PU4;
globalvalue unsigned int PATCH_ID_PU5;
globalvalue unsigned int PATCH_ID_PU6;
globalvalue unsigned int PATCH_ID_PU7;

static uint32 xq_all_patches_mask;
static uint32 pu_all_patches_mask;

/***************************************************************************************
*  Local function prototypes                                                           *
***************************************************************************************/

static void verify_linking_integrity();
static void usage();
static uint32 cmd_query(int argc, char** argv);
static uint32 cmd_load(int argc, char** argv);
static uint32 cmd_show(int argc, char** argv);
static uint32 cmd_set_affinity(int argc, char** argv);
static uint32 cmd_show_affinity(int argc, char** argv);
static uint32 cmd_set(int argc, char** argv);
static void query_simh();
void mtpr_simh(uint32* args);
static uint32 query_simh_exc_handler(struct chf$signal_array* sig, struct chf$mech_array* mech);
static bool_t is_keyword(const char* s, const char* kwd, int len);
static bool_t split_key_value(const char* arg, char* key, size_t keysize, char* value, size_t valuesize);
static bool_t is_qualifier(const char* s, const char* kwd, int len, const char** pqvalue);
static uint32 parse_nopatch(const char* value);
static void check_smp_enabled(bool_t fatal);
static void perform_calibration();
static void verify_calibration();
static void apply_calibration();
static bool_t is_calibration_stable(const uint32* m, int nsamples, int* pmax);
static uint32 k_load_resident_image();
static uint32 locate_xqdriver();
static uint32 prepare_xqdrv_patches();
static uint32 prepare_xqdrv_timxmt_patch();
static uint32 apply_xqdrv_timxmt_patch(int stage);
static uint32 locate_pudriver();
static uint32 prepare_pudrv_patches();
static void inv_opt_val(const char* optname);
static void validate_syncw_parameters();
static void print_msg(uint32 status);
static void print_msg_1(uint32 status, uint32 arg);
static void print_2msgs(uint32 st1, uint32 st2);
static void print_3msgs_val(uint32 st1, uint32 st2, uint32 st3, uint32 val);
static void check_resident_image_loaded();
static void check_resident_image_not_loaded();
static uint32 k_check_resident_image_loaded();
static void lock_nonpaged();
static void acquire_node_lock();
static void release_node_lock();
static void exit_handler(uint32 status);
static char* fmt_x8(char* buf, uint32 value);
static char* fmt_x2(char* buf, uint32 value);
static char* fmt_s(char* buf, const char* value);
static void kprint_prefix_x8_crlf(const char* prefix, uint32 value);
static bool_t parse_address(const char* sa, byte_t** paddr);
static uint32 k_gather_device_affinity(uint32* buffer, uint32 bufsize, uint32* overflow);
static uint32 k_set_device_affinity(const char* pattern, uint32 affinity);
static void k_strupr(char *s);
static void k_itoa(char* s, int v);
static bool_t k_match_pattern(const char* pattern, const char* name, int namelen, uint32 unit, bool_t endstar);
static bool_t k_streq(const char* s1, const char* s2);
static uint32 parse_udec(const char* sv, const char* optname);
static uint32 lookup_code_match(const byte_t* xaddr, const byte_t* pstart, const byte_t* pend);
static uint32 accvio_handler(struct chf$signal_array* sig, struct chf$mech_array* mech);
static uint32 safe_read(const byte_t* addr, byte_t* pvalue, uint32 count);
uint32 lookup_code_ex(const byte_t* xaddr, int max_before, int max_after, const byte_t* pstart, const byte_t* pend, 
              const byte_t* mask, const byte_t* mask_e, lookup_code_checker_t checker, void* checker_arg);
static uint32 lookup_code_match_ex(const byte_t* xaddr, const byte_t* pstart, const byte_t* pend, 
              const byte_t* mask, lookup_code_checker_t checker, void* checker_arg);
static void* locate_driver(const char* drivername);
uint32 lookup_in_xqdrv(uint32 errstatus, int initial, const byte_t* pstart, const byte_t* pend);
uint32 lookup_in_xqdrv_ex(uint32 errstatus, int initial, const byte_t* pstart, const byte_t* pend,
              const byte_t* mask, const byte_t* mask_e, lookup_code_checker_t checker, void* checker_arg);
uint32 lookup_in_pudrv(uint32 errstatus, int initial, const byte_t* pstart, const byte_t* pend);
uint32 lookup_in_pudrv_ex(uint32 errstatus, int initial, const byte_t* pstart, const byte_t* pend,
              const byte_t* mask, const byte_t* mask_e, lookup_code_checker_t checker, void* checker_arg);
uint32 lookup_in_drv(void* dpt, uint32 drv_size, uint32 errstatus, int initial, const byte_t* pstart, const byte_t* pend);
uint32 lookup_in_drv_ex(void* dpt, uint32 drv_size, uint32 errstatus, int initial, const byte_t* pstart, const byte_t* pend,
              const byte_t* mask, const byte_t* mask_e, lookup_code_checker_t checker, void* checker_arg);

/***************************************************************************************
*  Exported functions prototypes                                                       *
***************************************************************************************/

uint32 lookup_code(const byte_t* xaddr, int max_before, int max_after, const byte_t* pstart, const byte_t* pend);
bool_t k_streqi_cnt(const char* s1, const char* s2, int len);

/***************************************************************************************
*  Functions in KLOAD and DYNPATCH                                                     *
***************************************************************************************/

// uint32 kmalloc_kb64(uint32 size, void** p);
// void kfree_kb64(void* p);
uint32 kmalloc_anysize(uint32 reqsize, void** paddr, uint32* pblksize);
void kfree_anysize(void* p, uint32 blksize);
uint32 kload_verify_compatible(void* addr);
uint32 kload_calibrate_sysloops(uint32* tenusec, uint32* ubdelay);
uint32 kload_apply_calibration(uint32 tenusec, uint32 ubdelay);
uint32 k_check_smp_enabled(uint32* enabled, uint32* unmod_driver);
void kload_set_process_deletable(bool_t deletable);
void kload_flush_instruction_stream();
uint32 kcall_onload(uint32 idle, uint32 timesync);
uint32 kcall_get_idle(uint32* idle);
uint32 kcall_get_timesync(uint32* timesync);
uint32 kcall_set_idle(uint32 idle);
uint32 kcall_set_timesync(uint32 timesync);
void kprint(const char* s);
void kprint_crlf(const char* s);
void kmemcpy(void* dst, const void* src, uint32 size);
void iodb_lock_rd();
void iodb_lock_wr();
void iodb_unlock(uint32 ipl);
void* get_localsb_addr();
void k_set_ucb_affinity(void* ucb, uint32 affinity);
uint32 k_extend_pfork_pool(int npages);
uint32 k_locate_ldr_img(int namelen, char* name, uint32* range, uint32* is_valid);
uint32 lookup_patches(uint32 nopatch);
void k_lock_system_pages(uint32 nopatch);
void k_unlock_system_pages(uint32 nopatch);
void pin_driver(void* dpt);
void patch_xqdrv_instr(void* addr_mov_xmt_tmo, uint32 xqtimeout);
void set_xq_xmt_timeout(void* ucb, uint32 lsb_xmt_tmo, uint32 timeout);
bool_t set_patchdesc_lookup_found(uint32 patch_id, const void* addr);
void setup_patch_pu1(const void* addr);
void setup_patch_pu2(const void* addr);
void setup_patch_pu3(const void* addr);
void setup_patch_pu4(const void* addr);
void setup_patch_pu5(const void* addr);
void setup_patch_pu6(const void* addr);
void setup_patch_pu7(const void* addr);
void dbg_brk();

/***************************************************************************************
*  Main routine                                                                        *
***************************************************************************************/

int main(int argc, char** argv)
{
    xq_all_patches_mask = (1 << PATCH_ID_XQTIMXMT) |
                          (1 << PATCH_ID_XQTX1) |
                          (1 << PATCH_ID_XQTX2) |
                          (1 << PATCH_ID_XQTX3) |
                          (1 << PATCH_ID_XQTX4) |
                          (1 << PATCH_ID_XQTX5) |
                          (1 << PATCH_ID_XQTX6) |
                          (1 << PATCH_ID_XQTX7) |
                          (1 << PATCH_ID_XQTX8) |
                          (1 << PATCH_ID_XQTX9) |
                          (1 << PATCH_ID_XQTX10) |
                          (1 << PATCH_ID_XQRX1) |
                          (1 << PATCH_ID_XQRX2) |
                          (1 << PATCH_ID_XQRX3) |
                          (1 << PATCH_ID_XQRX4);

    pu_all_patches_mask = (1 << PATCH_ID_PU1) |
                          (1 << PATCH_ID_PU2) |
                          (1 << PATCH_ID_PU3) |
                          (1 << PATCH_ID_PU4) |
                          (1 << PATCH_ID_PU5) |
                          (1 << PATCH_ID_PU6) |
                          (1 << PATCH_ID_PU7);

    verify_linking_integrity();

    /* Check if running on SIMH VAX MP */
    query_simh();
    if (! simh_present)
        return VSMP_MSG_SYS_NOT_VAXMP;

    if (argc == 1)  usage();

    if (is_keyword(argv[1], "QUERY", 1))
    {
        return cmd_query(argc - 2, argv + 2);
    }
    else if (is_keyword(argv[1], "LOAD", 2))
    {
        return cmd_load(argc - 2, argv + 2);
    }
    else if (is_keyword(argv[1], "SET", 2))
    {
        return cmd_set(argc - 2, argv + 2);
    }
    else if (is_keyword(argv[1], "SHOW", 2))
    {
        return cmd_show(argc - 2, argv + 2);
    }
    else
    {
        usage();
    }
}

/***************************************************************************************
*  Display usage help                                                                  *
***************************************************************************************/

static void usage()
{
    print_msg(VSMP_MSG_SYNTAX);
    fprintf(stderr, "\n");
    fprintf(stderr, "VSMP usage:\n");
    fprintf(stderr, "\n");
    fprintf(stderr, "    VSMP QUERY\n");
    fprintf(stderr, "\n");
    fprintf(stderr, "    VSMP LOAD [IDLE=ON|OFF|NEVER] [TIMESYNC=ON|OFF] [NOPATCH=(list)]\n");
    fprintf(stderr, "              [XQTIMEOUT=<nsec>] [INTERLOCK=PORTABLE|NATIVE]\n");
    fprintf(stderr, "              [SYNCW=(SYS|ILK|SYS,ILK|ALL|NONE)]\n");
    fprintf(stderr, "              [SYNCW_SYS=pct] [SYNCW_ILK=pct]\n");
    fprintf(stderr, "\n");
    fprintf(stderr, "    VSMP SET [IDLE=ON|OFF] [TIMESYNC=ON|OFF]\n");
    fprintf(stderr, "\n");
    fprintf(stderr, "    VSMP SHOW [IDLE] [TIMESYNC]\n");
    fprintf(stderr, "\n");
    fprintf(stderr, "    VSMP SET AFFINITY { device | device-pattern }\n");
    fprintf(stderr, "                      /CPU={PRIMARY|ALL}\n");
    fprintf(stderr, "\n");
    fprintf(stderr, "    VSMP SHOW AFFINITY { device | device-pattern | /ALL }\n");
    fprintf(stderr, "\n");
    exit(VSMP_MSG_SYNTAX | STS$M_INHIB_MSG);
}

/***************************************************************************************
*  Display QUERY data                                                                  *
***************************************************************************************/

static uint32 cmd_query(int argc, char** argv)
{
    uint32 smp_options;

    if (argc != 0)  usage();

    printf("VSMP tool version: 1.0\n");
    printf("SIMH VAX MP simulator API version: %d\n", simh_api_version);
    printf("Number of virtual processors configured: %d\n", simh_ncpus);
    printf("Processor IDs mask: %08X\n", simh_cpu_mask);
    printf("Advised host SMT slow-down factor: %g\n", (double) simh_smp_multiplier / (double) simh_smp_divider);
    printf("Advised host turbo factor: %d%%\n", simh_host_turbo_factor);
    printf("vSMP revision level: %d\n", simh_smp_revision);
    printf("vSMP options: %X\n", simh_smp_options);

    smp_options = simh_smp_options;
    if (smp_options & VAXMP_SMP_OPTION_PORTABLE_INTERLOCK)
    {
        printf("     portable interlock\n");
        smp_options &= ~VAXMP_SMP_OPTION_PORTABLE_INTERLOCK;
    }
    if (smp_options & VAXMP_SMP_OPTION_NATIVE_INTERLOCK)
    {
        printf("     native interlock\n");
        smp_options &= ~VAXMP_SMP_OPTION_NATIVE_INTERLOCK;
    }
    if (smp_options)
    {
        printf("     unknown %X\n", smp_options);
    }

    printf("VAX virtual machine provider: ");
    if (simh_vm_id == VAXMP_VM_ID)
    {
        printf("SIMH VAX MP");
    }
    else
    {
        printf("%c%c%c%c", 
            simh_vm_id & 0xFF, 
            (simh_vm_id >> 8) & 0xFF, 
            (simh_vm_id >> 16) & 0xFF, 
            (simh_vm_id >> 24) & 0xFF);
    }

    printf(" %d.%d", (simh_vm_version >> 24) & 0xFF, (simh_vm_version >> 16) & 0xFF);
    if (simh_vm_version & 0xFFFF);
    {
        printf(".%d", (simh_vm_version >> 8) & 0xFF);
        if (simh_vm_version & 0xFF)
            printf("-%d", simh_vm_version & 0xFF);
    }
    printf("\n");
    printf("Synchronization window(s) active: ");
    switch (simh_syncw & (SYNCW_SYS | SYNCW_ILK))
    {
    case (SYNCW_SYS | SYNCW_ILK):
        printf("SYS,ILK\n");
        break;
    case SYNCW_SYS:
        printf("SYS\n");
        break;
    case SYNCW_ILK:
        printf("ILK\n");
        break;
    case 0:
        printf("none\n");
        break;
    }

    return SS$_NORMAL;
}

/***************************************************************************************
*  Query for SIMH VAX MP presence and configuration                                    *
***************************************************************************************/

static void query_simh()
{
    simh_present = FALSE;
    uint32 args[15];
    args[0] = VAXMP_API_SIGNATURE;
    args[1] = VAXMP_API_OP_QUERY;
    args[2] = 1;                     /* guest API version */
    args[3] = 0;                     /* response status, none yet */
    lib$establish(query_simh_exc_handler);
    mtpr_simh(args);
    lib$revert();
    simh_present = (args[3] == 1);
    simh_api_version = args[4];
    simh_ncpus = args[5];
    simh_cpu_mask = args[6];
    simh_vm_id = args[7];
    simh_vm_version = args[8];
    simh_smp_revision = args[9];
    simh_smp_options = args[10];
    simh_smp_multiplier = args[11];
    simh_smp_divider = args[12];
    simh_host_turbo_factor = args[13];
    simh_syncw = args[14];
}

static uint32 query_simh_exc_handler(struct chf$signal_array* sig, struct chf$mech_array* mech)
{
    if (sig->chf$l_sig_name == SS$_UNWIND ||
        sig->chf$l_sig_name == SS$_OPCDEC)
    {
        return sys$unwind(NULL, NULL);
    }
    else
    {
        return SS$_RESIGNAL;
    }
}

/***************************************************************************************
*  Load resident module                                                                *
***************************************************************************************/

static uint32 cmd_load(int argc, char** argv)
{
    uint32 status;
    int ak;
    char key[MAXARGSIZE];
    char value[MAXARGSIZE];
    double f;
    uint32 kargs[5];
    static const char* xdelta_name = "SYSTEM_DEBUG.EXE";
    static const char* msg_rtns_name = "MESSAGE_ROUTINES.EXE";

    /* parse arguments */
    for (ak = 0;  ak < argc;  ak++)
    {
        if (! split_key_value(argv[ak], key, countof(key), value, countof(value)))
            usage();

        if (is_keyword(key, "IDLE", 2))
        {
            if (is_keyword(value, "ON", 2))
                smp_idle = SIM_K_IDLE_ON;
            else if (is_keyword(value, "OFF", 3))
                smp_idle = SIM_K_IDLE_OFF;
            else if (is_keyword(value, "NEVER", 3))
                smp_idle = SIM_K_IDLE_NEVER;
            else
                usage();
        }
        else if (is_keyword(key, "TIMESYNC", 2))
        {
            if (is_keyword(value, "ON", 2))
                smp_timesync = SIM_K_TIMESYNC_ON;
            else if (is_keyword(value, "OFF", 3))
                smp_timesync = SIM_K_TIMESYNC_OFF;
            else
                usage();
        }
        else if (is_keyword(key, "NOPATCH", 3))
        {
            nopatch |= parse_nopatch(value);
        }
        else if (is_keyword(key, "XQTIMEOUT", 2))
        {   
            xqtimeout = parse_udec(value, "XQTIMEOUT");
            if (! (xqtimeout >= XQ_MIN_TIMEOUT && xqtimeout <= XQ_MAX_TIMEOUT))
                inv_opt_val("XQTIMEOUT");
        }
        else if (is_keyword(key, "INTERLOCK", 3))
        {
            if (is_keyword(value, "PORTABLE", 1))
                use_native_interlock = FALSE;
            else if (is_keyword(value, "NATIVE", 1))
                use_native_interlock = TRUE;
            else
                inv_opt_val("INTERLOCK");
        }
        else if (is_keyword(key, "SYNCW", -1))
        {
            /*
             * SYNCW=(SYS,ILK) or SYS or ILK or ALL or NONE
             */
            char* list;
            char* tok;
            int k;

            syncw_on = 0;

            list = strdup(value);
            if (list == NULL)  exit(SS$_INSFMEM);
            k_strupr(list);

            if (*list == '(')
            {
                list++;
                k = strlen(list);
                if (k == 0 || list[k - 1] != ')')
                    usage();
                list[k - 1] = '\0';
            }

            tok = strtok(list, ",");
            while (tok)
            {
                if (is_keyword(tok, "SYS", 1))
                {
                    syncw_on |= SYNCW_SYS;
                }
                else if (is_keyword(tok, "ILK", 1))
                {
                    syncw_on |= SYNCW_ILK;
                }
                else if (is_keyword(tok, "ALL", 1))
                {
                    syncw_on |= SYNCW_SYS | SYNCW_ILK;
                }
                else if (is_keyword(tok, "NONE", 1))
                {
                    syncw_on = 0;
                }
                else
                {
                    inv_opt_val("SYNCW");
                }
                tok = strtok(NULL, ",");
            }
        }
        else if (is_keyword(key, "SYNCW_SYS", -1))
        {
            /*
             * percentage of maximum window width to use as safe window size
             * default = 66%
             * range = 10-90%
             */
            syncw_sys_pct = parse_udec(value, "SYNCW_SYS");
            if (! (syncw_sys_pct >= 10 && syncw_sys_pct <= 90))
                inv_opt_val("SYNCW_SYS");
        }
        else if (is_keyword(key, "SYNCW_ILK", -1))
        {
            /*
             * percentage of maximum window width to use as safe window size
             * default = 66%
             * range = 10-90%
             */
            syncw_ilk_pct = parse_udec(value, "SYNCW_ILK");
            if (! (syncw_ilk_pct >= 10 && syncw_ilk_pct <= 90))
                inv_opt_val("SYNCW_ILK");
        }
        else
        {
            usage();
        }
    }

    /* validate parameter combination */
    validate_syncw_parameters();

    /* begin pre-load preparations */
    check_smp_enabled(TRUE);
    lock_nonpaged();

    /* check if XDelta is loaded */
    if (0 == (nopatch & (1 << PATCH_ID_XDELTA)))
    {
        uint32 xdelta_is_valid = 0;
        kargs[0] = 4;
        kargs[1] = strlen(xdelta_name);
        kargs[2] = (uint32) (char*) xdelta_name;
        kargs[3] = (uint32) (uint32*) xdelta_range;
        kargs[4] = (uint32) & xdelta_is_valid;
        check_vms_status(sys$cmkrnl(k_locate_ldr_img, kargs));
        if (xdelta_range[0] && !xdelta_is_valid)  exit(VSMP_MSG_XDTINVALID);
    }

    /* adjust nopatch arguments */
    if (smp_idle == SIM_K_IDLE_NEVER)
    {
        nopatch |= 1 << PATCH_ID_CHSEP;
        nopatch |= 1 << PATCH_ID_RESCHED;
    }
    if (xdelta_range[0] == 0)
    {
        nopatch |= 1 << PATCH_ID_XDELTA;
    }

    /* locate message_routines.exe */
    if (0 == (nopatch & (1 << PATCH_ID_NUMTIM)))
    {
        uint32 msg_rtns_is_valid = 0;
        kargs[0] = 4;
        kargs[1] = strlen(msg_rtns_name);
        kargs[2] = (uint32) (char*) msg_rtns_name;
        kargs[3] = (uint32) (uint32*) msg_rtns_range;
        kargs[4] = (uint32) & msg_rtns_is_valid;
        check_vms_status(sys$cmkrnl(k_locate_ldr_img, kargs));
        if (msg_rtns_range[0] == 0 || !msg_rtns_is_valid)  exit(VSMP_MSG_MSGRTNSLDR);
    }

    /* continue pre-load preparations */
    acquire_node_lock();
    check_resident_image_not_loaded();
    check_vms_status(lookup_patches(nopatch));

    /* continue pre-load preparations */
    perform_calibration();
    verify_calibration();
    apply_calibration();

    /* 
     * Calculate number of PFORK (fork-to-primary) pool pages required. These pages are used to store messages
     * for printing to console emitted by high-IPL code during SMP state transitons (used by routines
     * SMP$WRITE_OPA0 and SMP$FORK_TO_PRIMARY). Guestimate of 5.5 pages per processor makes it ample.
     */
    f = simh_ncpus * 5.5;
    pfork_pool_pages = (int) ceil(f);
    if (f - pfork_pool_pages >= 0.5)
        pfork_pool_pages++;
    if (pfork_pool_pages > 255)
        pfork_pool_pages = 255;

    /*
     * calculate SYNCW SYS window size
     */
    if (syncw_on & SYNCW_SYS)
    {
        uint32 spinwait = uimin(sgn$gl_smp_lngspinwait, sgn$gl_smp_spinwait);
        /* translating loop count to instruction count is complicated, but the tightest loop
           is just sobgtr instruction, so map as 1:1 */
        double fws = (double) spinwait * (double) exe_tenusec * (double) exe_ubdelay * (double) syncw_sys_pct / 100.0;
        /* avoid overflows */
        if (fws > (double) 0x7F000000) fws = (double) 0x7F000000;
        syncw_winsize_sys = floor(fws);

        /* check that window size is reasonable */
        if (syncw_winsize_sys < 40000)
        {
            print_2msgs(VSMP_MSG_VLOW_SW_SYS, VSMP_MSG_ADV_SW_SYS);
            exit(VSMP_MSG_VLOW_SW_SYS | STS$M_INHIB_MSG);
        }
        else if (syncw_winsize_sys < 200000)
        {
            print_2msgs(VSMP_MSG_LOW_SW_SYS, VSMP_MSG_ADV_SW_SYS);
        }
    }
    else
    {
        syncw_winsize_sys = 0;
    }

    /*
     * calculate SYNCW ILK window size (see VAX MP Techinical Overview for details)
     */
    if (syncw_on & SYNCW_ILK)
    {
        /* number of loops hardwired in $INSQHI etc. macros */
        const uint32 hardwired_nloops = 900000;
        if (exe$gl_lockrtry >= 0x80000000)  exit(VSMP_MSG_IVLOCKRTRY);
        uint32 nloops = (nopatch & (1 << PATCH_ID_LOCKRTRY)) ? uimin(exe$gl_lockrtry, hardwired_nloops) 
                                                             : hardwired_nloops;
        /* there are at minimum three instructions in $INSQHI etc. loops */
        double fws = (double) nloops * 3 * (double) syncw_ilk_pct / 100.0;
        /* avoid overflows */
        if (fws > (double) 0x7F000000) fws = (double) 0x7F000000;
        syncw_winsize_ilk = floor(fws);

        /* check that window size is reasonable */
        if (syncw_winsize_ilk < 40000)
        {
            print_2msgs(VSMP_MSG_VLOW_SW_ILK, VSMP_MSG_ADV_SW_ILK);
            exit(VSMP_MSG_VLOW_SW_ILK | STS$M_INHIB_MSG);
        }
        else if (syncw_winsize_ilk < 200000)
        {
            print_2msgs(VSMP_MSG_LOW_SW_ILK, VSMP_MSG_ADV_SW_ILK);
        }
    }
    else
    {
        syncw_winsize_ilk = 0;
    }

    if ((syncw_on & (SYNCW_SYS|SYNCW_ILK)) == (SYNCW_SYS|SYNCW_ILK) && syncw_winsize_sys < syncw_winsize_ilk)
        print_2msgs(VSMP_MSG_SYS_LESS_ILK, VSMP_MSG_ADV_ISW_SYS);

    /* set requested SMP options */
    smp_must_options = use_native_interlock ? VAXMP_SMP_OPTION_NATIVE_INTERLOCK
                                            : VAXMP_SMP_OPTION_PORTABLE_INTERLOCK;
    smp_want_options = 0;

    /* perform loading */
    kerror_cause_usr = 0;
    status = sys$cmkrnl(k_load_resident_image, kargs_none);
    if (status == VSMP_MSG_LDR_VERIFY && kerror_cause_usr)
    {
        print_2msgs(status, kerror_cause_usr);
        exit(status | STS$M_INHIB_MSG);
    }
    else
    {
        check_vms(status);
        print_msg(VSMP_MSG_LOADED);
        return SS$_NORMAL;
    }

cleanup:
    exit(status);
}

static uint32 parse_nopatch(const char* value)
{
    char* list;
    char* tok;
    int k;
    uint32 nopatch = 0;

    list = strdup(value);
    if (list == NULL)  exit(SS$_INSFMEM);
    k_strupr(list);

    if (*list == '(')
    {
        list++;
        k = strlen(list);
        if (k == 0 || list[k - 1] != ')')
            usage();
        list[k - 1] = '\0';
    }

    tok = strtok(list, ",");
    while (tok)
    {
        if (is_keyword(tok, "XDELTA", 2))
            nopatch |= (1 << PATCH_ID_XDELTA);
        else if (is_keyword(tok, "CHSEP", 3))
            nopatch |= (1 << PATCH_ID_CHSEP);
        else if (is_keyword(tok, "RESCHED", 3))
            nopatch |= (1 << PATCH_ID_RESCHED);
        else if (is_keyword(tok, "NUMTIM", 3))
            nopatch |= (1 << PATCH_ID_NUMTIM);
        else if (is_keyword(tok, "MFYCAP", 4))
            nopatch |= (1 << PATCH_ID_MFYCAP);
        else if (is_keyword(tok, "LOCKRTRY", 3))
            nopatch |= (1 << PATCH_ID_LOCKRTRY);
        else if (is_keyword(tok, "XQTIMXMT", 4))
            nopatch |= (1 << PATCH_ID_XQTIMXMT);
        else if (is_keyword(tok, "UCBTMO", 3))
            nopatch |= (1 << PATCH_ID_UCBTMO);
        else if (is_keyword(tok, "CRBTMO", 3))
            nopatch |= (1 << PATCH_ID_CRBTMO);
        else if (is_keyword(tok, "XQTX1", -1))
            nopatch |= (1 << PATCH_ID_XQTX1);
        else if (is_keyword(tok, "XQTX2", 0))
            nopatch |= (1 << PATCH_ID_XQTX2);
        else if (is_keyword(tok, "XQTX3", 0))
            nopatch |= (1 << PATCH_ID_XQTX3);
        else if (is_keyword(tok, "XQTX4", 0))
            nopatch |= (1 << PATCH_ID_XQTX4);
        else if (is_keyword(tok, "XQTX5", 0))
            nopatch |= (1 << PATCH_ID_XQTX5);
        else if (is_keyword(tok, "XQTX6", 0))
            nopatch |= (1 << PATCH_ID_XQTX6);
        else if (is_keyword(tok, "XQTX7", 0))
            nopatch |= (1 << PATCH_ID_XQTX7);
        else if (is_keyword(tok, "XQTX8", 0))
            nopatch |= (1 << PATCH_ID_XQTX8);
        else if (is_keyword(tok, "XQTX9", 0))
            nopatch |= (1 << PATCH_ID_XQTX9);
        else if (is_keyword(tok, "XQTX10", -1))
            nopatch |= (1 << PATCH_ID_XQTX10);
        else if (is_keyword(tok, "XQRX1", 0))
            nopatch |= (1 << PATCH_ID_XQRX1);
        else if (is_keyword(tok, "XQRX2", 0))
            nopatch |= (1 << PATCH_ID_XQRX2);
        else if (is_keyword(tok, "XQRX3", 0))
            nopatch |= (1 << PATCH_ID_XQRX3);
        else if (is_keyword(tok, "XQRX4", 0))
            nopatch |= (1 << PATCH_ID_XQRX4);
        else if (is_keyword(tok, "PU1", 0))
            nopatch |= (1 << PATCH_ID_PU1);
        else if (is_keyword(tok, "PU2", 0))
            nopatch |= (1 << PATCH_ID_PU2);
        else if (is_keyword(tok, "PU3", 0))
            nopatch |= (1 << PATCH_ID_PU3);
        else if (is_keyword(tok, "PU4", 0))
            nopatch |= (1 << PATCH_ID_PU4);
        else if (is_keyword(tok, "PU5", 0))
            nopatch |= (1 << PATCH_ID_PU5);
        else if (is_keyword(tok, "PU6", 0))
            nopatch |= (1 << PATCH_ID_PU6);
        else if (is_keyword(tok, "PU7", 0))
            nopatch |= (1 << PATCH_ID_PU7);
        else
        {
            print_msg_1(VSMP_MSG_IVPATCHID, (uint32) tok);
            exit(VSMP_MSG_IVPATCHID | STS$M_INHIB_MSG);
        }
        tok = strtok(NULL, ",");
    }

    /* 
     * CRBTMO and UCBTMO are not currently utilized, so disable them
     */
    nopatch |= (1 << PATCH_ID_UCBTMO);
    nopatch |= (1 << PATCH_ID_CRBTMO);

    return nopatch;
}

static void validate_syncw_parameters()
{
    uint32 time_control = exe$gl_time_control & 0x6;
    bool_t valid = FALSE;

    if (use_native_interlock && 0 == (simh_smp_options & VAXMP_SMP_OPTION_NATIVE_INTERLOCK))
    {
        print_msg(VSMP_MSG_NONATIVE);
        use_native_interlock = FALSE;
    }

    if (smp_idle == SIM_K_IDLE_NEVER && (syncw_on & SYNCW_ILK))
    {
        print_msg(VSMP_MSG_SYNCWIDLEOFF);
        smp_idle = SIM_K_IDLE_OFF;
    }

    if (use_native_interlock)
    {
        /* native */
        if (syncw_on == (SYNCW_SYS|SYNCW_ILK))
        {
            valid = (time_control == 2 || time_control == 6);
        }
        else if (syncw_on == SYNCW_ILK)
        {
            valid = (time_control == 6);
        }
    }
    else 
    {
        /* portable */
        if (syncw_on == (SYNCW_SYS|SYNCW_ILK))
        {
            valid = (time_control == 2 || time_control == 6);
        }
        else if (syncw_on == SYNCW_SYS)
        {
            valid = (time_control == 2 || time_control == 6);
        }
        else if (syncw_on == SYNCW_ILK)
        {
            valid = (time_control == 6);
        }
        else if (syncw_on == 0)
        {
            valid = (time_control == 6);
        }
    }

    if (! valid)
    {
        char msg[256];
        char* pmsg;

        sprintf(msg, "INTERLOCK=%s, SYNCW=", use_native_interlock ? "NATIVE" : "PORTABLE");
        pmsg = msg + strlen(msg);
        switch (syncw_on)
        {
        case (SYNCW_SYS | SYNCW_ILK):
            sprintf(pmsg, "(SYS,ILK)");
            break;
        case SYNCW_SYS:
            sprintf(pmsg, "SYS");
            break;
        case SYNCW_ILK:
            sprintf(pmsg, "ILK");
            break;
        case 0:
            sprintf(pmsg, "NONE");
            break;
        }
        sprintf(msg + strlen(msg), ", SYSGEN TIME_CONTROL=%d", time_control);

        print_msg_1(VSMP_MSG_IVPARSET, (uint32) msg);
        exit(VSMP_MSG_IVPARSET | STS$M_INHIB_MSG);
    }
}

/***************************************************************************************
*  Set control parameters                                                              *
***************************************************************************************/

static uint32 cmd_set(int argc, char** argv)
{
    uint32 status;
    uint32 kargs[2];
    int ak;
    char key[MAXARGSIZE];
    char value[MAXARGSIZE];
    bool_t set_idle = FALSE;
    bool_t set_timesync = FALSE;
    uint32 new_idle;
    uint32 new_timesync;
    uint32 old_idle;

    /* parse arguments */
    if (argc == 0)  usage();

    if (is_keyword(argv[0], "AFFINITY", 2))
    {
        return cmd_set_affinity(argc - 1, argv + 1);
    }

    for (ak = 0;  ak < argc;  ak++)
    {
        if (! split_key_value(argv[ak], key, countof(key), value, countof(value)))
            usage();

        if (is_keyword(key, "IDLE", 2))
        {
            set_idle = TRUE;

            if (is_keyword(value, "ON", 2))
                new_idle = SIM_K_IDLE_ON;
            else if (is_keyword(value, "OFF", 3))
                new_idle = SIM_K_IDLE_OFF;
            else if (is_keyword(value, "NEVER", 3))
                new_idle = SIM_K_IDLE_NEVER;
            else
                usage();
        }
        else if (is_keyword(key, "TIMESYNC", 2))
        {
            set_timesync = TRUE;

            if (is_keyword(value, "ON", 2))
                new_timesync = SIM_K_TIMESYNC_ON;
            else if (is_keyword(value, "OFF", 3))
                new_timesync = SIM_K_TIMESYNC_OFF;
            else
                usage();
        }
        else
        {
            usage();
        }
    }

    /* prepare */
    lock_nonpaged();
    acquire_node_lock();
    check_resident_image_loaded();

    /* verify IDLE value */
    if (set_idle)
    {
        kargs[0] = 1;
        kargs[1] = (uint32) & old_idle;
        check_vms_status(sys$cmkrnl(kcall_reloc(kcall_get_idle_t, kcall_get_idle), kargs));
        if (old_idle == new_idle)  set_idle = FALSE;
        if (set_idle && old_idle == SIM_K_IDLE_NEVER)
            exit(VSMP_MSG_IDLE_NEVER);
    }

    /* set IDLE */
    if (set_idle)
    {
        kargs[0] = 1;
        kargs[1] = new_idle;
        check_vms_status(sys$cmkrnl(kcall_reloc(kcall_set_idle_t, kcall_set_idle), kargs));
    }

    /* set TIMESYNC */
    if (set_timesync)
    {
        kargs[0] = 1;
        kargs[1] = new_timesync;
        check_vms_status(sys$cmkrnl(kcall_reloc(kcall_set_timesync_t, kcall_set_timesync), kargs));
    }

    return SS$_NORMAL;

cleanup:
    exit(status);
}

/***************************************************************************************
*  Show parameters and status                                                          *
***************************************************************************************/

static uint32 cmd_show(int argc, char** argv)
{
    uint32 status;
    uint32 kargs[2];
    int ak;
    char key[MAXARGSIZE];
    char value[MAXARGSIZE];
    bool_t show_idle = FALSE;
    bool_t show_timesync = FALSE;

    /* parse arguments */
    if (argc == 0)
    {
        show_idle = TRUE;
        show_timesync = TRUE;
    }
    else if (is_keyword(argv[0], "AFFINITY", 2))
    {
        return cmd_show_affinity(argc - 1, argv + 1);
    }
    else for (ak = 0;  ak < argc;  ak++)
    {
        if (is_keyword(argv[ak], "IDLE", 2))
            show_idle = TRUE;
        else if (is_keyword(argv[ak], "TIMESYNC", 2))
            show_timesync = TRUE;
        else
            usage();
    }

    /* prepare */
    lock_nonpaged();
    acquire_node_lock();
    check_resident_image_loaded();

    if (show_idle)
    {
        kargs[0] = 1;
        kargs[1] = (uint32) & smp_idle;
        check_vms_status(sys$cmkrnl(kcall_reloc(kcall_get_idle_t, kcall_get_idle), kargs));
        switch (smp_idle)
        {
        case SIM_K_IDLE_OFF:
            printf("IDLE is OFF\n");
            break;
        case SIM_K_IDLE_ON:
            printf("IDLE is ON\n");
            break;
        case SIM_K_IDLE_NEVER:
            printf("IDLE is NEVER\n");
            break;
        }
    }

    if (show_timesync)
    {
        kargs[0] = 1;
        kargs[1] = (uint32) & smp_timesync;
        check_vms_status(sys$cmkrnl(kcall_reloc(kcall_get_timesync_t, kcall_get_timesync), kargs));
        switch (smp_timesync)
        {
        case SIM_K_TIMESYNC_OFF:
            printf("TIMESYNC is OFF\n");
            break;
        case SIM_K_TIMESYNC_ON:
            printf("TIMESYNC is ON\n");
            break;
        }
    }

    return SS$_NORMAL;

cleanup:
    exit(status);
}

/***************************************************************************************
*  Set device affinity                                                                 *
***************************************************************************************/

#define DDB_NAME_STR_SIZE 31

static uint32 cmd_set_affinity(int argc, char** argv)
{
    char* pattern = NULL;
    uint32 status;
    uint32 kargs[3];
    int ak;
    uint32 cmd_set_affinity_value;
    bool_t cmd_set_affinity_value_set = FALSE;

    globalvalue uint32 ddb$s_name_str;

    /* check size is in range */
    if (DDB_NAME_STR_SIZE < ddb$s_name_str)
        exit(VSMP_MSG_MISBUILT);

    /* parse the arguments */
    for (ak = 0;  ak < argc;  ak++)
    {
        char* arg = argv[ak];
        if (arg[0] == '/')
        {
            const char* qvalue;
            if (is_qualifier(arg, "CPUS", 1, &qvalue))
            {
                if (qvalue && is_keyword(qvalue, "PRIMARY", 1))
                {
                    cmd_set_affinity_value = 0;
                    cmd_set_affinity_value_set = TRUE;
                }
                else if (qvalue && is_keyword(qvalue, "ALL", 1))
                {
                    cmd_set_affinity_value = 0xFFFFFFFF;
                    cmd_set_affinity_value_set = TRUE;
                }
                else
                {
                    usage();
                }
            }
            else
            {
                usage();
            }
        }
        else
        {
            if (pattern)
                usage();
            else
                pattern = arg;
        }
    }

    /* validate the arguments */
    if (! cmd_set_affinity_value_set)
        usage();

    if (pattern == NULL || *pattern == '\0')
        usage();

    pattern = strdup(pattern);
    if (pattern == NULL)  exit(SS$_INSFMEM);
    k_strupr(pattern);

    check_smp_enabled(FALSE);
    lock_nonpaged();

    kargs[0] = 2;
    kargs[1] = (uint32) pattern;
    kargs[2] = cmd_set_affinity_value;
    check_vms_status(sys$cmkrnl(k_set_device_affinity, kargs));

    return SS$_NORMAL;

cleanup:
    exit(status);
}

static uint32 k_set_device_affinity(const char* pattern, uint32 affinity)
{
    globalvalue uint32 sb$l_ddb;
    globalvalue uint32 ddb$l_link;
    globalvalue uint32 ddb$b_name_len;
    globalvalue uint32 ddb$t_name_str;
    globalvalue uint32 ddb$l_ucb;
    globalvalue uint32 ucb$l_link;
    globalvalue uint32 ucb$w_unit;
    globalvalue uint32 ucb$l_affinity;

    void* sb;
    void* ddb;
    void* ucb;
    int match_count = 0;
    char namebuf[DDB_NAME_STR_SIZE + 10];
    uint32 status;

    /* lock IO database, returns here at IPL ASTDEL */
    iodb_lock_wr();

    /* get SB for local system */
    sb = get_localsb_addr();

    /* enumerate DDBs */
    for (ddb = offset_ref(void*, sb, sb$l_ddb);  ddb != NULL;  ddb = offset_ref(void*, ddb, ddb$l_link))
    {
        uint32 dnamelen = offset_ref(byte_t, ddb, ddb$b_name_len);
        char* dname = offset_ptr(char, ddb, ddb$t_name_str);
        kmemcpy(namebuf, dname, dnamelen);
        namebuf[dnamelen] = 0;
        if (k_streq(namebuf, "MBA"))  continue;
        if (k_streq(namebuf, "NLA"))  continue;

        /* enumerate UCBs */
        for (ucb = offset_ref(void*, ddb, ddb$l_ucb);  ucb != NULL;  ucb = offset_ref(void*, ucb, ucb$l_link))
        {
            if (k_match_pattern(pattern, dname, dnamelen, offset_ref(uint16, ucb, ucb$w_unit), FALSE))
            {
                k_set_ucb_affinity(ucb, affinity);
                match_count++;
            }
        }
    }

    /* unlock IO database, reset IPL to 0 */
    iodb_unlock(0);

    status = SS$_NORMAL;

    if (match_count == 0)
        status = SS$_NOSUCHDEV;

    return status;
}

/***************************************************************************************
*  Display device affinity                                                             *
***************************************************************************************/

typedef struct __ucb_info
{
    uint32 unit;
    uint32 affinity;
}
ucb_info;

typedef struct __ddb_info
{
    uint32 namelen;
    char name[DDB_NAME_STR_SIZE + 1];
}
ddb_info;

static uint32 cmd_show_affinity(int argc, char** argv)
{
    char* pattern = NULL;
    uint32 status;
    uint32 bufsize;
    uint32* buffer = NULL;
    uint32* bp;
    uint32 kargs[4];
    uint32 overflow;
    uint32 mark;
    ddb_info* p_ddb_info;
    ucb_info* p_ucb_info;
    bool_t header = TRUE;

    globalvalue uint32 ddb$s_name_str;

    /* check size is in range */
    if (DDB_NAME_STR_SIZE < ddb$s_name_str)
        exit(VSMP_MSG_MISBUILT);

    if (argc == 0)
    {
        pattern = "*";
    }
    else if (argc != 1)
    {
        usage();
    }
    else if (is_keyword(argv[0], "/ALL", 2))
    {
        pattern = "*";
    }
    else
    {
        pattern = argv[0];
    }

    pattern = strdup(pattern);
    if (pattern == NULL)  exit(SS$_INSFMEM);
    k_strupr(pattern);

    check_smp_enabled(FALSE);
    lock_nonpaged();

    for (bufsize = 16 * 1024; ; bufsize = bufsize * 2)
    {
        buffer = (uint32*) malloc(bufsize);
        if (buffer == NULL)  exit(SS$_INSFMEM);
        kargs[0] = 3;
        kargs[1] = (uint32) buffer;
        kargs[2] = bufsize;
        kargs[3] = (uint32) & overflow;
        overflow = FALSE;
        check_vms_status(sys$cmkrnl(k_gather_device_affinity, kargs));
        if (! overflow)  break;
        free (buffer);
    }

    for (bp = buffer; (mark = *bp++) != 'E'; )
    {
        if (mark == 'D')
        {
            p_ddb_info = (ddb_info*) bp;
            bp = offset_ptr(uint32, bp, sizeof(ddb_info));
        }
        else // mark == 'U'
        {
            char namebuf[DDB_NAME_STR_SIZE + 10];

            p_ucb_info = (ucb_info*) bp;
            bp = offset_ptr(uint32, bp, sizeof(ucb_info));
            if (streqi(p_ddb_info->name, "MBA"))  continue;
            if (streqi(p_ddb_info->name, "NLA"))  continue;
            if (k_match_pattern(pattern, p_ddb_info->name, p_ddb_info->namelen, p_ucb_info->unit, TRUE))
            {
                if (header)
                {
                    printf("Local device affinity to CPUs:\n\n");
                    header = FALSE;
                }

                sprintf(namebuf, "%s%d", p_ddb_info->name, p_ucb_info->unit);
                printf("  %-8s \t", namebuf);
                if (p_ucb_info->affinity == 0xFFFFFFFF)
                {
                    printf("ALL\n");
                }
                else if (p_ucb_info->affinity == 0)
                {
                    printf("PRIMARY\n");
                }
                else
                {
                    bool_t first = TRUE;
                    for (int k = 0;  k <= 31;  k++)
                    {
                        printf(first ? "%02d" : " %02d", k);
                        first = FALSE;
                    }
                    printf("\n");
                }
            }
        }
    }

    return SS$_NORMAL;

cleanup:
    exit(status);
}

/*
 * Gather device affinity information from IO database into the output buffer,
 * as a sequence of D-records (representing data from DDBs) and U-records (data from UCBs)
 * terminated by E-record (The End).
 *
 * If buffer size is not sufficient, *p_overflow is set to TRUE, and caller should call
 * again with a larger buffer.
 */
static uint32 k_gather_device_affinity(uint32* buffer, uint32 bufsize, uint32* p_overflow)
{
    globalvalue uint32 sb$l_ddb;
    globalvalue uint32 ddb$l_link;
    globalvalue uint32 ddb$b_name_len;
    globalvalue uint32 ddb$t_name_str;
    globalvalue uint32 ddb$l_ucb;
    globalvalue uint32 ucb$l_link;
    globalvalue uint32 ucb$w_unit;
    globalvalue uint32 ucb$l_affinity;

    void* sb;
    void* ddb;
    void* ucb;
    ddb_info* p_ddb_info;
    ucb_info* p_ucb_info;

    *p_overflow = FALSE;

    /* lock IO database, returns here at IPL ASTDEL */
    iodb_lock_rd();

    /* get SB for local system */
    sb = get_localsb_addr();

    /* enumerate DDBs */
    for (ddb = offset_ref(void*, sb, sb$l_ddb);  ddb != NULL;  ddb = offset_ref(void*, ddb, ddb$l_link))
    {
        /* check enough space for D-mark, DDB info and E-mark */
        if (bufsize < sizeof(uint32) + sizeof(ddb_info) + sizeof(uint32))
            goto overflow;

        /* emit D-mark and DDB info */
        *buffer++ = 'D';
        p_ddb_info = (ddb_info*) buffer;
        p_ddb_info->namelen = offset_ref(byte_t, ddb, ddb$b_name_len);
        kmemcpy(p_ddb_info->name, offset_ptr(byte_t, ddb, ddb$t_name_str), p_ddb_info->namelen);
        p_ddb_info->name[p_ddb_info->namelen] = 0;

        /* advance buffer */
        buffer = offset_ptr(uint32, buffer, sizeof(ddb_info));
        bufsize -= sizeof(uint32) + sizeof(ddb_info);

        /* enumerate UCBs */
        for (ucb = offset_ref(void*, ddb, ddb$l_ucb);  ucb != NULL;  ucb = offset_ref(void*, ucb, ucb$l_link))
        {
            /* check enough space for U-mark, UCB info and E-mark */
            if (bufsize < sizeof(uint32) + sizeof(ucb_info) + sizeof(uint32))
                goto overflow;

            /* emit U-mark and UCB info */
            *buffer++ = 'U';
            p_ucb_info = (ucb_info*) buffer;
            p_ucb_info->unit = offset_ref(uint16, ucb, ucb$w_unit);
            p_ucb_info->affinity = offset_ref(uint32, ucb, ucb$l_affinity);

            /* advance buffer */
            buffer = offset_ptr(uint32, buffer, sizeof(ucb_info));
            bufsize -= sizeof(uint32) + sizeof(ucb_info);
        }
    }

    /* emit E-mark */
    *buffer = 'E';

    /* unlock IO database, reset IPL to 0 */
    iodb_unlock(0);

    return SS$_NORMAL;

    /* return in case of buffer overflow */
overflow:
    *p_overflow = TRUE;
    iodb_unlock(0);
    return SS$_NORMAL;
}

/*
 * Check if device unit (described by "name/namelen" and "unit" number) matches the supplied pattern.
 * If "endstar" is true, there is an implied "*" at the end of the pattern argument unless the pattern contains a digit,
 * i.e. pattern "XQA" matches device "XQA2", but pattern "XQA0" does not match it.
 */
static bool_t k_match_pattern(const char* pattern, const char* name, int namelen, uint32 unit, bool_t endstar)
{
    char uname[DDB_NAME_STR_SIZE + 10];
    const char* unp = uname;
    const char* pp = pattern;
    char c;

    kmemcpy(uname, name, namelen);
    k_itoa(uname + namelen, unit);

    while (c = *pp++)
    {
        if (c >= '0' && c <= '9')
            endstar = FALSE;

        if (c == '*')
        {
            while (*pp == '*')  pp++;
            if (*pp)
                return FALSE;
            return TRUE;
        }

        if (c != *unp++)
            return FALSE;
    }

    if (*unp == '\0' || endstar)
        return TRUE;

    return FALSE;
}

/***************************************************************************************
*  Perform processor calibration before statring SMP until the reading is stable       *
***************************************************************************************/

#define CALIBR_MIN_SAMPLES  25
#define CALIBR_MAX_SAMPLES  200
#define CALIBR_TOLERANCE_RANGE 0.15
#define CALIBR_MIN_WITHIN_TR 5

/*
 * Perform CPU busy-wait loops calibration multiple times to eliminate the effect of possible
 * concurrent load on host system that may result in the under-calibration.
 */
static void perform_calibration()
{
    int status;
    int nsamples;
    int ns;
    int pass = 1;
    uint32 tenusec[CALIBR_MAX_SAMPLES];
    uint32 ubdelay[CALIBR_MAX_SAMPLES];
    uint32 mul[CALIBR_MAX_SAMPLES];

    print_msg(VSMP_MSG_CALIBRATING);

again:

    for (nsamples = 0;  nsamples < CALIBR_MAX_SAMPLES;  )
    {
        uint32 kargs[3];
        kargs[0] = 2;
        kargs[1] = (uint32) & tenusec[nsamples];
        kargs[2] = (uint32) & ubdelay[nsamples];
        check_vms_status(sys$cmkrnl(kload_calibrate_sysloops, kargs));
        mul[nsamples] = tenusec[nsamples] * ubdelay[nsamples];
        nsamples++;
        if (nsamples >= CALIBR_MIN_SAMPLES && is_calibration_stable(mul, nsamples, & ns))
        {
            exe_ubdelay = ubdelay[ns];
            /*
             * Busy-wait loops are two-layered:
             *
             *     TENUSEC controls outer loop.
             *     UBDEALY control inner loop.
             *
             * On SMT/Hyperthreaded host processors measured refults may be affected by load running on the same
             * core during calibration and slowing down the calibration loop, and may therefore result in the
             * undercalibration of the loop.
             *
             * Adjust TENUSEC for SMT slow-down factor and host turbo factor advised by VMM.
             */
            double f = (double) tenusec[ns] * (double) simh_smp_multiplier / (double) simh_smp_divider;
            f *= (double) simh_host_turbo_factor / 100.0;
            exe_tenusec = (int) floor(f);
            if (f - exe_tenusec > 0.5)  exe_tenusec++;
            print_msg_1(VSMP_MSG_CALIBRATED, nsamples);
            return;
        }
    }

    /*
     * Sometimes calibraton fails due to spurious reasons,
     * such as external load or perhaps cache flush/cold/warm effects.
     * Try one more time before giving up.
     */
    if (pass++ <= 1)
    {
        print_msg(VSMP_MSG_CALIBRETRY);
        goto again;
    }

    /* was unable to attain stable calibration */
    status = VSMP_MSG_CALIBR_UNSTABLE;

cleanup:
    exit(status);
}

static bool_t is_calibration_stable(const uint32* m, int nsamples, int* pmax)
{
    int maxm = 0;
    int in_tolerance_range = 0;
    int k;

    /* find maximum */
    for (k = 0;  k < nsamples;  k++)
    {
        if (k == 0 || m[k] > maxm)
        {
            maxm = m[k];
            *pmax = k;
        }
    }

    /* count samples close to maximum */
    for (k = 0;  k < nsamples;  k++)
    {
        if (k != *pmax && m[k] >= maxm * (1.0 - CALIBR_TOLERANCE_RANGE))
        {
            if (++in_tolerance_range >= CALIBR_MIN_WITHIN_TR)
                return TRUE;
        }
    }

    return FALSE;
}

/*
 * Verify that calibration would not cause overflow:
 *
 *     TENUSEC * UBDELAY * SPINWAIT <= 0x7FFFFFFF
 *     TENUSEC * UBDELAY * LNGSPINWAIT <= 0x7FFFFFFF
 *
 * Note that we cannot just knock down SPINWAIT/LNGSPINWAIT value in case overflow is detected,
 * because by now they had been copied in numerous spinlocks, including spinlocks outside of
 * well-known spinlocks and IO devices spinlocks. Thus, in base VMS alone there is internal
 * CRDERR spinlock, log manager spinlocks and there are may be various internal private spinlocks
 * in various laryered privileged components and drivers that we cannot know about.
 *
 * Therefore we cannot adjust SPINWAIT/LNGSPINWAIT values dynamically as we cannot be aware of
 * all locations where the values had been copied by now. Therefore the only available recourse
 * is to issue user an advisory message to reduce SPINWAIT/LNGSPINWAIT and to reboot.
 */
static void verify_calibration()
{
    uint32 maxwait = 0x7FFFFFFF / (exe_tenusec * exe_ubdelay);
    uint32 status = SS$_NORMAL;

    uint32 twait = (uint32) floor(maxwait * 0.95);
    twait -= twait % 1000;

    if (sgn$gl_smp_spinwait > maxwait)
    {
        print_3msgs_val(VSMP_MSG_UNSTART, VSMP_MSG_SPWHIGH, VSMP_MSG_ADV_SPW_HIGH, twait);
        status = VSMP_MSG_SPWHIGH;
    }

    if (sgn$gl_smp_lngspinwait > maxwait)
    {
        print_3msgs_val(VSMP_MSG_UNSTART, VSMP_MSG_LSPWHIGH, VSMP_MSG_ADV_LSPW_HIGH, twait);
        status = VSMP_MSG_LSPWHIGH;
    }

    if (status != SS$_NORMAL)
        exit(status | STS$M_INHIB_MSG);
}

static void apply_calibration()
{
    int status;
    uint32 kargs[3];
    kargs[0] = 2;
    kargs[1] = exe_tenusec;
    kargs[2] = exe_ubdelay;
    check_vms_status(sys$cmkrnl(kload_apply_calibration, kargs));
    return;

cleanup:
    exit(status);
}

/***************************************************************************************
*  Check whether SMP is enabled and VMS multiprocesing system image is loaded          *
***************************************************************************************/

static void check_smp_enabled(bool_t fatal)
{
    uint32 smp_enabled = FALSE;
    uint32 smp_unmod_drv = FALSE;
    uint32 kargs[3];
    kargs[0] = 2;
    kargs[1] = (uint32) &smp_enabled;
    kargs[2] = (uint32) &smp_unmod_drv;
    int status = sys$cmkrnl(k_check_smp_enabled, kargs);
    if (! $VMS_STATUS_SUCCESS(status))  exit(status);

    if (smp_enabled)
    {
        if (smp_unmod_drv)
        {
            if (fatal)
            {
                exit(VSMP_MSG_UNMOD_DRV);
            }
            else
            {
                uint32 status = (VSMP_MSG_UNMOD_DRV & ~STS$M_SEVERITY) | (STS$K_WARNING << STS$V_SEVERITY);
                print_msg(status);
            }
        }
    }
    else if (fatal)
    {
        print_msg(VSMP_MSG_VMS_NOT_MULTI);
        fprintf(stderr, "\n");
        fprintf(stderr, "VSMP: OpenVMS multiprocessing system image is not loaded.\n");
        fprintf(stderr, "      Please change the value of SYSGEN (SYSBOOT) parameter\n");
        fprintf(stderr, "      MULTIPROCESSING to 2 and reboot.\n");
        exit(VSMP_MSG_VMS_NOT_MULTI | STS$M_INHIB_MSG);
    }
    else
    {
        uint32 status = (VSMP_MSG_VMS_NOT_MULTI & ~STS$M_SEVERITY) | (STS$K_INFO << STS$V_SEVERITY);
        print_msg(status);
        exit(status | STS$M_INHIB_MSG);
    }
}

/**********************************************************************************************
*  Check whether VSMP resident image is loaded and loaded version matches current executable  *
**********************************************************************************************/

static $DESCRIPTOR(lnm_table, "LNM$SYSTEM_TABLE");
static $DESCRIPTOR(lnm_name, "VSMP$KERNEL_LOAD_ADDRESS");

static void check_resident_image_loaded()
{
    uint32 kargs[1];
    kargs[0] = 0;
    int status = sys$cmkrnl(k_check_resident_image_loaded, kargs);
    if (! $VMS_STATUS_SUCCESS(status))  exit(status);
    if (status != SS$_WASSET)  exit(VSMP_MSG_NOT_LOADED);
}

static void check_resident_image_not_loaded()
{
    uint32 kargs[1];
    kargs[0] = 0;
    int status = sys$cmkrnl(k_check_resident_image_loaded, kargs);
    if (! $VMS_STATUS_SUCCESS(status))  exit(status);
    if (status != SS$_WASCLR)  exit(VSMP_MSG_ALREADY_LOADED);
}

static uint32 k_check_resident_image_loaded()
{
    uint32 status;
    ILE3 item_list[3];
    char lnm_value[256];
    uint16 lnm_length = 0;
    int lnm_max_index = -1;

    /*
     * Address of the loaded kernel-resident code is stored in system-wide logical name.
     * Read logical name value.
     */

    ile3_make(&item_list[0], LNM$_STRING, sizeof(lnm_value) - 1, lnm_value, & lnm_length);
    ile3_make(&item_list[1], LNM$_MAX_INDEX, 4, &lnm_max_index, NULL);
    ile3_term(&item_list[2]);

    status = sys$trnlnm(NULL, &lnm_table, &lnm_name, & PSL$C_KERNEL, &item_list);
    if (status == SS$_NOLOGNAM)
        return SS$_WASCLR;

    check_vms(status);

    /* logical name does exists, check and parse it */
    if (lnm_max_index != 0 || lnm_length >= sizeof(lnm_value))
        fail(SS$_IVLOGNAM);
    lnm_value[lnm_length] = 0;
    if (! parse_address(lnm_value, & kaddress))
        fail(SS$_IVLOGNAM);
    if (! ((uint32) kaddress >= 0x80000000 && (uint32) kaddress < 0xC0000000))
        fail(SS$_IVLOGNAM);

    /* verify if resident image is compatible with current executable */
    check_vms_status(kload_verify_compatible(kaddress));
    return SS$_WASSET;

cleanup:

    return status;
}

/***************************************************************************************
*  Load resident image into the kernel                                                 *
***************************************************************************************/

/*
 * Load resident part into the kernel.
 * Returns VMS-structured status.
 * On success, sets 'kaddress' variable to the address of the loaded resident part.
 * On failure, sets kaddress to NULL.
 */
static uint32 k_load_resident_image()
{
    uint32 status;
    ILE3 item_list[3];
    char lnm_value[256];
    uint32 lnm_attributes;
    uint32 kload_size; 
    bool_t did_kmalloc = FALSE;
    bool_t did_crelnm = FALSE;
    bool_t did_lock_system_pages = FALSE;
    bool_t did_disable_delete_process = FALSE;
    bool_t did_lock_iodb = FALSE;
    uint32 blksize;
    char* xp;
    int k;

    kaddress = 0;

    /* disable process deletion while we are holding resources */
    kload_set_process_deletable(FALSE);
    did_disable_delete_process = TRUE;

    /* lock select system pages in memory (system working set), as required for patches */
    k_lock_system_pages(nopatch);
    did_lock_system_pages = TRUE;

    /* extend fork-to-primary pool if required */
    check_vms_status(k_extend_pfork_pool(pfork_pool_pages));

    /* allocate block from non-paged pool and copy the resident code into it */
    kload_size = &kload_end - &kload_start + 1;
    check_vms_status(kmalloc_anysize(kload_size, (void**) & kaddress, & blksize));
    did_kmalloc = TRUE;

    /* copy content into the allocated block, preserve block size in the block header */
    kmemcpy(kaddress, &kload_start, kload_size);
    kdata_reloc(uint32, kload_blksize) = blksize;

    /* flush instruction prefetch (obviously excessive, especially for SIMH) */
    kload_flush_instruction_stream();

    /* store the address of allocated block in system-wide logical name */
    xp = fmt_x8(lnm_value, (uint32) kaddress);
    *xp = '\0';
    lnm_attributes = LNM$M_TERMINAL;
    ile3_make(&item_list[0], LNM$_ATTRIBUTES, 4, &lnm_attributes, NULL);
    ile3_make(&item_list[1], LNM$_STRING, kstrlen(lnm_value), lnm_value, NULL);
    ile3_term(&item_list[2]);
    check_vms_status(sys$crelnm(&LNM$M_NO_ALIAS, &lnm_table, &lnm_name, &PSL$C_KERNEL, &item_list));
    did_crelnm = TRUE;

    /* any XQDRIVER patches to apply? */
    if ((nopatch & xq_all_patches_mask) != xq_all_patches_mask)
    {
        /* lock IO database, returns here at IPL ASTDEL */
        if (! did_lock_iodb)
        {
            iodb_lock_wr();
            did_lock_iodb = TRUE;
        }

        /* locate XQDRIVER */
        check_vms_status(locate_xqdriver());

        /* if XQDRIVER is not loaded, disable all patches for it */
        if (xqdrv_dpt == 0)
        {
            nopatch |= xq_all_patches_mask;
        }
        else
        {
            check_vms_status(prepare_xqdrv_patches());
        }
    }

    /* any PUDRIVER patches to apply? */
    if ((nopatch & pu_all_patches_mask) != pu_all_patches_mask)
    {
        /* lock IO database, returns here at IPL ASTDEL */
        if (! did_lock_iodb)
        {
            iodb_lock_wr();
            did_lock_iodb = TRUE;
        }

        /* locate PUDRIVER */
        check_vms_status(locate_pudriver());

        /* if PUDRIVER is not loaded, disable all patches for it */
        if (pudrv_dpt == 0)
        {
            nopatch |= pu_all_patches_mask;
        }
        else
        {
            check_vms_status(prepare_pudrv_patches());
        }
    }

    /* invoke "onload" handler in the loaded image */
    // kprint_prefix_x8_crlf("** kaddress: ", (uint32) kaddress);
    // kprint_prefix_x8_crlf("** kcall_reloc(kcall_onload): ", (uint32) kcall_reloc(kcall_onload_t, kcall_onload));
    status = RCALL(kcall_onload_t, kcall_onload)(simh_cpu_mask, smp_idle, smp_timesync, nopatch);
    kerror_cause_usr = kdata_reloc(uint32, kerror_cause);
    // kprint_prefix_x8_crlf("** onload status: ", status);
    check_vms_status(status);

cleanup:

    if ($VMS_STATUS_SUCCESS(status) || status == VSMP_MSG_VM_REFUSED)
    {
        if ((nopatch & xq_all_patches_mask) != xq_all_patches_mask && xqdrv_dpt)
            pin_driver(xqdrv_dpt);

        if (0 == (nopatch & (1 << PATCH_ID_XQTIMXMT)))
            apply_xqdrv_timxmt_patch(1);

        if ((nopatch & pu_all_patches_mask) != pu_all_patches_mask && pudrv_dpt)
            pin_driver(pudrv_dpt);
    }

    if (did_lock_iodb)
        iodb_unlock(0);

    if (! $VMS_STATUS_SUCCESS(status) && status != VSMP_MSG_VM_REFUSED)
    {
        if (did_kmalloc)
            kfree_anysize(kaddress, blksize);
        if (did_crelnm)
            sys$dellnm(&lnm_table, &lnm_name, &PSL$C_KERNEL);
        if (did_lock_system_pages)
            k_unlock_system_pages(nopatch);
        kaddress = NULL;
    }

    if (did_disable_delete_process)
        kload_set_process_deletable(TRUE);

    return status;
}

/***************************************************************************************
*  Locate XQDRIVER and PUDRIVER if loaded                                              *
***************************************************************************************/

static uint32 locate_xqdriver()
{
    void* dpt = locate_driver("XQDRIVER");

    if (dpt != NULL)
    {
        globalvalue uint32 dpt$w_size;

        /* store DPT address */
        xqdrv_dpt = dpt;
        xqdrv_size = offset_ref(uint16, dpt, dpt$w_size);
    }

    return SS$_NORMAL;
}

static uint32 locate_pudriver()
{
    void* dpt = locate_driver("PUDRIVER");

    if (dpt != NULL)
    {
        globalvalue uint32 dpt$w_size;

        /* store DPT address */
        pudrv_dpt = dpt;
        pudrv_size = offset_ref(uint16, dpt, dpt$w_size);
    }

    return SS$_NORMAL;
}

static void* locate_driver(const char* drivername)
{
    byte_t namelen = (byte_t) kstrlen(drivername);
    void* dpt;

    globalvalue uint32 dpt$l_flink;
    globalvalue uint32 dpt$t_name;
    globalref uint32 ioc$gl_dptlist;

    /*
     * Enumerate DPTs to find the driver
     */
    for (dpt = (void*) ioc$gl_dptlist;  dpt != &ioc$gl_dptlist;  dpt = offset_ref(void*, dpt, dpt$l_flink))
    {
        byte_t* p = offset_ptr(byte_t, dpt, dpt$t_name);
        if (*p == namelen && k_streqi_cnt((const char*) p + 1, drivername, namelen))
        {
            return dpt;
        }
    }

    return NULL;
}

/***************************************************************************************
*  Prepare patches for XQDRIVER                                                        *
***************************************************************************************/

static bool_t xqrxtx_checker(const byte_t* xa, void* arg);

static uint32 prepare_xqdrv_patches()
{
    uint32 status;
    uint32 patch_id;

    if (0 == (nopatch & (1 << PATCH_ID_XQTX1)))
    {
        EXTERN byte_t xqtx1_look, xqtx1_look_e;
        check_vms_status(lookup_in_xqdrv(VSMP_MSG_XQTX1_P, 0x1556, &xqtx1_look, &xqtx1_look_e));
        status = VSMP_MSG_XQTX1_P;
        CHECK(RCALL(set_patchdesc_lookup_found_t, set_patchdesc_lookup_found)(PATCH_ID_XQTX1, lookup_code_addr));
    }

    if (0 == (nopatch & (1 << PATCH_ID_XQTX2)))
    {
        EXTERN byte_t xqtx2_look, xqtx2_look_e;
        check_vms_status(lookup_in_xqdrv(VSMP_MSG_XQTX2_P, 0x1574, &xqtx2_look, &xqtx2_look_e));
        status = VSMP_MSG_XQTX2_P;
        CHECK(RCALL(set_patchdesc_lookup_found_t, set_patchdesc_lookup_found)(PATCH_ID_XQTX2, lookup_code_addr));
    }

    if (0 == (nopatch & (1 << PATCH_ID_XQTX3)))
    {
        EXTERN byte_t xqtx3_look, xqtx3_look_e, xqtx3_look_mask, xqtx3_look_mask_e;
        check_vms_status(lookup_in_xqdrv_ex(VSMP_MSG_XQTX3_P, 0x159A, &xqtx3_look, &xqtx3_look_e, &xqtx3_look_mask, &xqtx3_look_mask_e, NULL, NULL));
        status = VSMP_MSG_XQTX3_P;
        CHECK(RCALL(set_patchdesc_lookup_found_t, set_patchdesc_lookup_found)(PATCH_ID_XQTX3, lookup_code_addr));
    }

    if (0 == (nopatch & (1 << PATCH_ID_XQTX4)))
    {
        EXTERN byte_t xqtx4_look, xqtx4_look_e, xqtx4_look_mask, xqtx4_look_mask_e;
        EXTERN byte_t xqtx4_handler_jmp;
        const byte_t* p;
        int offset;

        patch_id = PATCH_ID_XQTX4;
        /* if lookup fails, one possible cause is that DEVICELOCK size might have changed, so mask size mismatches lookup pattern size */
        check_vms_status(lookup_in_xqdrv_ex(VSMP_MSG_XQTX4_P, 0x15E6, &xqtx4_look, &xqtx4_look_e, &xqtx4_look_mask, &xqtx4_look_mask_e, xqrxtx_checker, &patch_id));
        status = VSMP_MSG_XQTX4_P;
        CHECK(RCALL(set_patchdesc_lookup_found_t, set_patchdesc_lookup_found)(PATCH_ID_XQTX4, lookup_code_addr));

        /* calculate target of BBC instruction at lookup_code_addr + 37 */
        p = lookup_code_addr;
        offset = 0xFF & (uint32) p[41];
        if (offset & 0x80)  offset |= 0xFFFFFF00;

        /* store as jump address in replacement patch data */
        *kdata_reloc_ptr(const void**, &xqtx4_handler_jmp + 2) = p + 42 + offset;
    }

    if (0 == (nopatch & (1 << PATCH_ID_XQTX5)))
    {
        EXTERN byte_t xqtx5_look, xqtx5_look_e, xqtx5_look_mask, xqtx5_look_mask_e;
        check_vms_status(lookup_in_xqdrv_ex(VSMP_MSG_XQTX5_P, 0xADA, &xqtx5_look, &xqtx5_look_e, &xqtx5_look_mask, &xqtx5_look_mask_e, NULL, NULL));
        status = VSMP_MSG_XQTX5_P;
        CHECK(RCALL(set_patchdesc_lookup_found_t, set_patchdesc_lookup_found)(PATCH_ID_XQTX5, lookup_code_addr));
    }

    if (0 == (nopatch & (1 << PATCH_ID_XQTX6)))
    {
        EXTERN byte_t xqtx6_look, xqtx6_look_e, xqtx6_look_mask, xqtx6_look_mask_e;
        check_vms_status(lookup_in_xqdrv_ex(VSMP_MSG_XQTX6_P, 0xE89, &xqtx6_look, &xqtx6_look_e, &xqtx6_look_mask, &xqtx6_look_mask_e, NULL, NULL));
        status = VSMP_MSG_XQTX6_P;
        CHECK(RCALL(set_patchdesc_lookup_found_t, set_patchdesc_lookup_found)(PATCH_ID_XQTX6, lookup_code_addr));
    }

    if (0 == (nopatch & (1 << PATCH_ID_XQTX7)))
    {
        EXTERN byte_t xqtx7_look, xqtx7_look_e, xqtx7_look_mask, xqtx7_look_mask_e;
        check_vms_status(lookup_in_xqdrv_ex(VSMP_MSG_XQTX7_P, 0x16A2, &xqtx7_look, &xqtx7_look_e, &xqtx7_look_mask, &xqtx7_look_mask_e, NULL, NULL));
        status = VSMP_MSG_XQTX7_P;
        CHECK(RCALL(set_patchdesc_lookup_found_t, set_patchdesc_lookup_found)(PATCH_ID_XQTX7, lookup_code_addr));
    }

    if (0 == (nopatch & (1 << PATCH_ID_XQTX8)))
    {
        EXTERN byte_t xqtx8_look, xqtx8_look_e, xqtx8_look_mask, xqtx8_look_mask_e;
        check_vms_status(lookup_in_xqdrv_ex(VSMP_MSG_XQTX8_P, 0x16A2, &xqtx8_look, &xqtx8_look_e, &xqtx8_look_mask, &xqtx8_look_mask_e, NULL, NULL));
        status = VSMP_MSG_XQTX8_P;
        CHECK(RCALL(set_patchdesc_lookup_found_t, set_patchdesc_lookup_found)(PATCH_ID_XQTX8, lookup_code_addr));
    }

    if (0 == (nopatch & (1 << PATCH_ID_XQTX9)))
    {
        EXTERN byte_t xqtx9_look, xqtx9_look_e, xqtx9_look_mask, xqtx9_look_mask_e, xqtx9_handler_movl, xqtx9_handler_jmp;
        const byte_t* p;
        int offset;

        check_vms_status(lookup_in_xqdrv_ex(VSMP_MSG_XQTX9_P, 0x1485, &xqtx9_look, &xqtx9_look_e, &xqtx9_look_mask, &xqtx9_look_mask_e, NULL, NULL));
        status = VSMP_MSG_XQTX9_P;
        CHECK(RCALL(set_patchdesc_lookup_found_t, set_patchdesc_lookup_found)(PATCH_ID_XQTX9, lookup_code_addr));
        kmemcpy(kdata_reloc_ptr(byte_t*, &xqtx9_handler_movl), lookup_code_addr + 10, 9);

        /* calculate target of BBC instruction at lookup_code_addr + 19 */
        p = lookup_code_addr;
        offset = 0xFF & (uint32) p[23];
        if (offset & 0x80)  offset |= 0xFFFFFF00;

        /* store as jump address in replacement patch data */
        *kdata_reloc_ptr(const void**, &xqtx9_handler_jmp + 2) = p + 24 + offset;
    }

    if (0 == (nopatch & (1 << PATCH_ID_XQTX10)))
    {
        EXTERN byte_t xqtx10_look, xqtx10_look_e, xqtx10_look_mask, xqtx10_look_mask_e;
        /* if lookup fails, one possible cause is that DEVICELOCK size might have changed, so mask size mismatches lookup pattern size */
        check_vms_status(lookup_in_xqdrv_ex(VSMP_MSG_XQTX10_P, 0xEF3, &xqtx10_look, &xqtx10_look_e, &xqtx10_look_mask, &xqtx10_look_mask_e, NULL, NULL));
        status = VSMP_MSG_XQTX10_P;
        CHECK(RCALL(set_patchdesc_lookup_found_t, set_patchdesc_lookup_found)(PATCH_ID_XQTX10, lookup_code_addr));
    }


    if (0 == (nopatch & (1 << PATCH_ID_XQRX1)))
    {
        EXTERN byte_t xqrx1_look, xqrx1_look_e, xqrx1_look_mask, xqrx1_look_mask_e;
        patch_id = PATCH_ID_XQRX1;
        check_vms_status(lookup_in_xqdrv_ex(VSMP_MSG_XQRX1_P, 0x12EB, &xqrx1_look, &xqrx1_look_e, &xqrx1_look_mask, &xqrx1_look_mask_e, xqrxtx_checker, &patch_id));
        status = VSMP_MSG_XQRX1_P;
        CHECK(RCALL(set_patchdesc_lookup_found_t, set_patchdesc_lookup_found)(PATCH_ID_XQRX1, lookup_code_addr));
    }

    if (0 == (nopatch & (1 << PATCH_ID_XQRX2)))
    {
        EXTERN byte_t xqrx2_look, xqrx2_look_e, xqrx2_look_mask, xqrx2_look_mask_e;
        check_vms_status(lookup_in_xqdrv_ex(VSMP_MSG_XQRX2_P, 0xADA, &xqrx2_look, &xqrx2_look_e, &xqrx2_look_mask, &xqrx2_look_mask_e, NULL, NULL));
        status = VSMP_MSG_XQRX2_P;
        CHECK(RCALL(set_patchdesc_lookup_found_t, set_patchdesc_lookup_found)(PATCH_ID_XQRX2, lookup_code_addr));
    }

    if (0 == (nopatch & (1 << PATCH_ID_XQRX3)))
    {
        EXTERN byte_t xqrx3_look, xqrx3_look_e, xqrx3_look_mask, xqrx3_look_mask_e;
        check_vms_status(lookup_in_xqdrv_ex(VSMP_MSG_XQRX3_P, 0xCDB, &xqrx3_look, &xqrx3_look_e, &xqrx3_look_mask, &xqrx3_look_mask_e, NULL, NULL));
        status = VSMP_MSG_XQRX3_P;
        CHECK(RCALL(set_patchdesc_lookup_found_t, set_patchdesc_lookup_found)(PATCH_ID_XQRX3, lookup_code_addr));
    }

    if (0 == (nopatch & (1 << PATCH_ID_XQRX4)))
    {
        EXTERN byte_t xqrx4_look, xqrx4_look_e, xqrx4_look_mask, xqrx4_look_mask_e, xqrx4_handler_bicb;
        check_vms_status(lookup_in_xqdrv_ex(VSMP_MSG_XQRX4_P, 0x1307, &xqrx4_look, &xqrx4_look_e, &xqrx4_look_mask, &xqrx4_look_mask_e, NULL, NULL));
        status = VSMP_MSG_XQRX4_P;
        CHECK(RCALL(set_patchdesc_lookup_found_t, set_patchdesc_lookup_found)(PATCH_ID_XQRX4, lookup_code_addr));
        kmemcpy(kdata_reloc_ptr(byte_t*, &xqrx4_handler_bicb), lookup_code_addr + 24, 7);
    }

    if (0 == (nopatch & (1 << PATCH_ID_XQTIMXMT)))
    {
        check_vms_status(prepare_xqdrv_timxmt_patch());
        check_vms_status(apply_xqdrv_timxmt_patch(0));
    }

    return SS$_NORMAL;

cleanup:

    return status;
}

static bool_t xqrxtx_checker(const byte_t* xa, void* arg)
{
    uint32 patch_id = * (uint32*) arg;

    if (patch_id == PATCH_ID_XQTX4)
    {
        /* check BBC and BBS point to the same target address 
           and BNEQ also points to the same address */
        return ((byte_t)(xa[41] - xa[46]) == 5 &&
                (byte_t)(xa[46] - xa[52]) == 6);
    }
    else if (patch_id == PATCH_ID_XQRX1)
    {
        /* check second BBC and BNEQ point to the same target address */
        return ((byte_t)(xa[27] - xa[20]) == 0xF9);
    }
    else
    {
        return FALSE;
    }
}

/***************************************************************************************
*  Prepare patch for XQDRIVER XMT timeout                                              *
***************************************************************************************/

/*
 * Entered and left at IPL ASTDEL with IODB mutex held.
 */
static uint32 prepare_xqdrv_timxmt_patch()
{
    const uint32 pat_2_startoff = 0x1B02;
    int max_before;
    int max_after;
    const byte_t* pat1_addr;
    byte_t pat1[7];
    uint32 status;

    EXTERN byte_t xqdrv_pat_1;
    EXTERN byte_t xqdrv_pat_2;
    EXTERN byte_t xqdrv_pat_2e;
    const byte_t* pxqdrv_pat_1 = &xqdrv_pat_1;

    /*
     * Locate instruction that sets timeout in XQ LSB.
     * Begin by searching code pattern located in XQDRIVER's routine SUB_START_CTRL_TIMER (module [PHV_LAN.SRC]DEQNA.MAR).
     */
    CHECK(pat_2_startoff + (&xqdrv_pat_2e - &xqdrv_pat_2) + 4 < xqdrv_size);

    max_before = 0x800;
    max_after = xqdrv_size - (&xqdrv_pat_2e - &xqdrv_pat_2) - 4;
    if (max_after > 0x800)
        max_after = 0x800;
    check_vms_status(lookup_code(offset_ptr(byte_t, xqdrv_dpt, pat_2_startoff), max_before, max_after, &xqdrv_pat_2, &xqdrv_pat_2e));

    /*
     * Back off from XQDRV_PAT_2 to XQDRV_PAT_1 and see if active XQDRIVER code
     * there contains expected instruction MOVB S^#VAL, W^OFFSET(R4)
     */
    pat1_addr = lookup_code_addr - (&xqdrv_pat_2 - &xqdrv_pat_1);
    check_vms_status(safe_read(pat1_addr, pat1, 5));
    CHECK(pat1[0] == pxqdrv_pat_1[0]);
    CHECK(pat1[2] == pxqdrv_pat_1[2]);

    /* extract and store value of original timeout (5 seconds in VMS 7.3) */
    xqdrv_orig_xmt_tmo = pat1[1];

    /* store address of XQDRIVER MOVB instruction that sets timeout */
    xqdrv_addr_xmt_tmo = (void*) pat1_addr;

    /* store the value of LSB$G_QNA_TIMXMT (offset into LSB) */
    xqdrv_lsb_xmt_tmo = (((uint32) pat1[4]) << 8) | (pat1[3]);
    CHECK(xqdrv_lsb_xmt_tmo);
    xqdrv_lsb_xmt_tmo--;
    CHECK(xqdrv_lsb_xmt_tmo >= 0x20 && xqdrv_lsb_xmt_tmo <= 0xFFF0);

    /*
     * Next instruction should be MOVW #VAL, W^OFFSET2(R4)
     */
    check_vms_status(safe_read(pat1_addr + 5, pat1, 7));
    CHECK(pat1[0] == pxqdrv_pat_1[5 + 0]);
    CHECK(pat1[1] == pxqdrv_pat_1[5 + 1]);
    CHECK(pat1[4] == pxqdrv_pat_1[5 + 4]);

    return SS$_NORMAL;

cleanup:
    xqdrv_orig_xmt_tmo = 0;
    xqdrv_addr_xmt_tmo = NULL;
    xqdrv_lsb_xmt_tmo = 0;
    return VSMP_MSG_XQTIMXMT_P;
}

/***************************************************************************************
*  Apply patch for XQDRIVER XMT timeout                                                *
***************************************************************************************/

/*
 * Entered and left at IPL ASTDEL with IODB mutex held.
 * Called two times. Sequence:
 *
 *     prepare_xqdrv_timxmt_patch();
 *     apply_xqdrv_timxmt_patch(stage = 0);
 *     kcall_onload
 *     apply_xqdrv_timxmt_patch(stage = 1);
 *
 * At stage 0 perform pre-patch validation.
 *
 * At stage 1 perform data modification.
 * When called for stage 1, returned status value is ignored. Must not fail.
 *
 */
static uint32 apply_xqdrv_timxmt_patch(int stage)
{
    void* sb;
    void* ddb;
    void* ucb;
    void* crb;
    void* lsb;
    uint32 status;
    int pass;
    byte_t curr_xmt_tmo;

    globalvalue uint32 sb$l_ddb;
    globalvalue uint32 ddb$l_link;
    globalvalue uint32 ddb$l_ucb;
    globalvalue uint32 ucb$b_type;
    globalvalue uint32 ucb$w_size;
    globalvalue uint32 ucb$b_devtype;
    globalvalue uint32 ucb$l_crb;
    globalvalue uint32 ddb$b_drvnam_len;
    globalvalue uint32 ddb$t_drvnam_str;
    globalvalue uint32 crb$l_auxstruc;
    globalvalue uint32 DYN$C_CDB;

    /* if XQDRIVER is not loaded, nothing to do */
    if (xqdrv_dpt == NULL)
        return SS$_NORMAL;

    if (xqtimeout < xqdrv_orig_xmt_tmo)
        xqtimeout = xqdrv_orig_xmt_tmo;
    if (xqtimeout > 255)
        xqtimeout = 255;

    /* get SB for local system */
    sb = get_localsb_addr();

    if (stage == 1)
    {
        /* disable driver unloading -- already done by the caller */
        // pin_driver(xqdrv_dpt);

        /* 
         * Patch XQDRIVER instructions.
         *
         * MOVB instruction that sets timeout value uses immed short operand (S^#5) and is too short
         * to be patched in place, therefore we need to replace two insructions by JSB to a resident
         * location, for this we need to elevate IPL to POWER, but this code is pageable, so we need to
         * call non-pageable code to do this.
         */
        RCALL(patch_xqdrv_instr_t, patch_xqdrv_instr)(xqdrv_addr_xmt_tmo, xqtimeout);
    }

    /* scan all local devices in the system that are connected to XQDRIVER */
    for (ddb = offset_ref(void*, sb, sb$l_ddb);  ddb != NULL;  ddb = offset_ref(void*, ddb, ddb$l_link))
    {
        byte_t* pname = offset_ptr(byte_t, ddb, ddb$b_drvnam_len);
        if (*pname == 8 && k_streqi_cnt((const char*) pname + 1, "XQDRIVER", 8))
        {
            CHECK(ucb = offset_ref(void*, ddb, ddb$l_ucb));
            if (stage == 0)
            {
                /* check device is DEQNA or DELQA, not DEQTA */
                if (offset_ref(byte_t, ucb, ucb$b_devtype) != DT$_DEQNA &&
                    offset_ref(byte_t, ucb, ucb$b_devtype) != DT$_XQ_DELQA)
                {
                    return VSMP_MSG_UNSUPPXQ;
                }
                /* LSB address is stored in CRB$_AUXSTRUC */
                CHECK(crb = offset_ref(void*, ucb, ucb$l_crb));
                if (lsb = offset_ref(void*, crb, crb$l_auxstruc))
                {
                    CHECK(offset_ref(byte_t, lsb, ucb$b_type) == DYN$C_CDB);
                    CHECK(offset_ref(uint16, lsb, ucb$w_size) >= xqdrv_lsb_xmt_tmo + 2);
                    curr_xmt_tmo = offset_ref(byte_t, lsb, xqdrv_lsb_xmt_tmo + 1);
                    if (curr_xmt_tmo)
                    {
                        CHECK(curr_xmt_tmo == xqdrv_orig_xmt_tmo);
                        if (xqtimeout < curr_xmt_tmo)
                            xqtimeout = curr_xmt_tmo;
                    }
                    else
                    {
                        /* This LSB has not been initialized yet, skip it. */
                    }
                }
            }
            else
            {
                set_xq_xmt_timeout(ucb, xqdrv_lsb_xmt_tmo, xqtimeout);
            }
        }
    }

    return SS$_NORMAL;

cleanup:

    return VSMP_MSG_XQTIMXMT_C;
}

/***************************************************************************************
*  Prepare patches for PUDRIVER                                                        *
***************************************************************************************/

static uint32 prepare_pudrv_patches()
{
    uint32 status;
    uint32 patch_id;

    if (0 == (nopatch & (1 << PATCH_ID_PU1)))
    {
        EXTERN byte_t pu1_look, pu1_look_e, pu1_look_mask, pu1_look_mask_e;
        /* if lookup fails, one possible cause is that READ_CSR size might have changed, so mask size mismatches lookup pattern size */
        check_vms_status(lookup_in_pudrv_ex(VSMP_MSG_PU1_P, 0x2D40, &pu1_look, &pu1_look_e, &pu1_look_mask, &pu1_look_mask_e, NULL, NULL));
        status = VSMP_MSG_PU1_P;
        CHECK(RCALL(set_patchdesc_lookup_found_t, set_patchdesc_lookup_found)(PATCH_ID_PU1, lookup_code_addr));
        RCALL(setup_patch_t, setup_patch_pu1)(lookup_code_addr);
    }

    if (0 == (nopatch & (1 << PATCH_ID_PU2)))
    {
        EXTERN byte_t pu2_look, pu2_look_e, pu2_look_mask, pu2_look_mask_e;
        check_vms_status(lookup_in_pudrv_ex(VSMP_MSG_PU2_P, 0x2842, &pu2_look, &pu2_look_e, &pu2_look_mask, &pu2_look_mask_e, NULL, NULL));
        status = VSMP_MSG_PU2_P;
        CHECK(RCALL(set_patchdesc_lookup_found_t, set_patchdesc_lookup_found)(PATCH_ID_PU2, lookup_code_addr));
        RCALL(setup_patch_t, setup_patch_pu2)(lookup_code_addr);
    }

    if (0 == (nopatch & (1 << PATCH_ID_PU3)))
    {
        EXTERN byte_t pu3_look, pu3_look_e, pu3_look_mask, pu3_look_mask_e;
        check_vms_status(lookup_in_pudrv_ex(VSMP_MSG_PU3_P, 0x3066, &pu3_look, &pu3_look_e, &pu3_look_mask, &pu3_look_mask_e, NULL, NULL));
        status = VSMP_MSG_PU3_P;
        CHECK(RCALL(set_patchdesc_lookup_found_t, set_patchdesc_lookup_found)(PATCH_ID_PU3, lookup_code_addr));
        RCALL(setup_patch_t, setup_patch_pu3)(lookup_code_addr);
    }

    if (0 == (nopatch & (1 << PATCH_ID_PU4)))
    {
        EXTERN byte_t pu4_look, pu4_look_e, pu4_look_mask, pu4_look_mask_e;
        check_vms_status(lookup_in_pudrv_ex(VSMP_MSG_PU4_P, 0x3176, &pu4_look, &pu4_look_e, &pu4_look_mask, &pu4_look_mask_e, NULL, NULL));
        status = VSMP_MSG_PU4_P;
        CHECK(RCALL(set_patchdesc_lookup_found_t, set_patchdesc_lookup_found)(PATCH_ID_PU4, lookup_code_addr));
        RCALL(setup_patch_t, setup_patch_pu4)(lookup_code_addr);
    }

    if (0 == (nopatch & (1 << PATCH_ID_PU5)))
    {
        EXTERN byte_t pu5_look, pu5_look_e, pu5_look_mask, pu5_look_mask_e;
        check_vms_status(lookup_in_pudrv_ex(VSMP_MSG_PU5_P, 0x2CFC, &pu5_look, &pu5_look_e, &pu5_look_mask, &pu5_look_mask_e, NULL, NULL));
        status = VSMP_MSG_PU5_P;
        CHECK(RCALL(set_patchdesc_lookup_found_t, set_patchdesc_lookup_found)(PATCH_ID_PU5, lookup_code_addr));
        RCALL(setup_patch_t, setup_patch_pu5)(lookup_code_addr);
    }

    if (0 == (nopatch & (1 << PATCH_ID_PU6)))
    {
        EXTERN byte_t pu6_look, pu6_look_e, pu6_look_mask, pu6_look_mask_e;
        check_vms_status(lookup_in_pudrv_ex(VSMP_MSG_PU6_P, 0x294B, &pu6_look, &pu6_look_e, &pu6_look_mask, &pu6_look_mask_e, NULL, NULL));
        status = VSMP_MSG_PU6_P;
        CHECK(RCALL(set_patchdesc_lookup_found_t, set_patchdesc_lookup_found)(PATCH_ID_PU6, lookup_code_addr));
        RCALL(setup_patch_t, setup_patch_pu6)(lookup_code_addr);
    }

    if (0 == (nopatch & (1 << PATCH_ID_PU7)))
    {
        EXTERN byte_t pu7_look, pu7_look_e, pu7_look_mask, pu7_look_mask_e;
        check_vms_status(lookup_in_pudrv_ex(VSMP_MSG_PU7_P, 0x29DA, &pu7_look, &pu7_look_e, &pu7_look_mask, &pu7_look_mask_e, NULL, NULL));
        status = VSMP_MSG_PU7_P;
        CHECK(RCALL(set_patchdesc_lookup_found_t, set_patchdesc_lookup_found)(PATCH_ID_PU7, lookup_code_addr));
        RCALL(setup_patch_t, setup_patch_pu7)(lookup_code_addr);
    }

    return SS$_NORMAL;

cleanup:

    return status;
}

/***************************************************************************************
*  Lock non-paged code into working set                                                *
***************************************************************************************/

static void lock_nonpaged()
{
    VA_RANGE va_range;
    uint32 status;

    /* lock page(s) of loader code that is invoked at elevated IPL */
    va_range.start = &kload_wslock_start;
    va_range.end = &kload_wslock_end;
    check_vms_status(sys$lkwset(&va_range, NULL, NULL));
    return;

cleanup:
    exit(status);
}

/***************************************************************************************
*  Verify image linking integrity                                                      *
***************************************************************************************/

#define check_kloadable(x) CHECK((void*) (&(x)) >= (void*) &kload_start && (void*) (&(x)) <= (void*) &kload_end)
#define check_kloadable_routine(x) CHECK((void*) (x) >= (void*) &kload_start && (void*) (x) <= (void*) &kload_end)

static void verify_linking_integrity()
{
    EXTERN byte_t timesync_ctrl;
    EXTERN uint32 activate_timesync_tqe;

    EXTERN uint32 vcon$old_putchar;
    EXTERN uint32 vcon$old_getchar;
    EXTERN uint32 vc_sv_txcs;
    EXTERN uint32 vc_sv_rxcs;
    EXTERN uint32 vc_sv_ipl;
    EXTERN uint32 vcreg_txdb;
    EXTERN uint16 vcreg_txie;
    EXTERN uint16 vcreg_txdone;
    EXTERN uint32 vcreg_rxdb;
    EXTERN uint16 vcreg_rxie;
    EXTERN uint16 vcreg_rxdone;

    /* declaration of routine addresses only */
    void vsmp$intproc();
    void vsmp$intall();
    void vsmp$intall_bit();
    void vsmp$intall_acq();
    void vsmp$intall_bit_acq();
    void vsmp$stop_cpu();
    void vsmp$show_cpu();
    void vsmp$halt_cpu();
    void vsmp$controlp_cpus();
    void vsmp$invalid_single();
    void vsmp$select_primary();
    void vsmp$setup_smp();
    void vsmp$read_todr();
    void vsmp$write_todr();
    void vsmp$intsr();
    void vsmp$spec_ipint();
    void vsmp$iniprocreg();
    void exe_clear_errors();
    void vsmp$start_cpu();
    void vsmp$setup_cpu();
    void vcon$putchar();
    void vcon$getchar();
    void vcon$owncty();
    void vcon$releasecty();
    void vsmp$virtcons_server();

    check_kloadable(kerror_cause);
    check_kloadable(timesync_ctrl);
    check_kloadable(activate_timesync_tqe);
    check_kloadable(vcon$old_putchar);
    check_kloadable(vcon$old_getchar);

    check_kloadable(vc_sv_txcs);
    check_kloadable(vc_sv_rxcs);
    check_kloadable(vc_sv_ipl);
    check_kloadable(vcreg_txdb);
    check_kloadable(vcreg_txie);
    check_kloadable(vcreg_txdone);
    check_kloadable(vcreg_rxdb);
    check_kloadable(vcreg_rxie);
    check_kloadable(vcreg_rxdone);

    check_kloadable_routine(kcall_onload);
    check_kloadable_routine(kcall_get_idle);
    check_kloadable_routine(kcall_set_idle);
    check_kloadable_routine(kprint);
    check_kloadable_routine(kcall_get_timesync);
    check_kloadable_routine(kcall_set_timesync);
    check_kloadable_routine(set_patchdesc_lookup_found);
    check_kloadable_routine(patch_xqdrv_instr);
    check_kloadable_routine(setup_patch_pu1);
    check_kloadable_routine(setup_patch_pu2);
    check_kloadable_routine(setup_patch_pu3);
    check_kloadable_routine(setup_patch_pu4);
    check_kloadable_routine(setup_patch_pu5);
    check_kloadable_routine(setup_patch_pu6);
    check_kloadable_routine(setup_patch_pu7);

    check_kloadable_routine(vsmp$intproc);
    check_kloadable_routine(vsmp$intall);
    check_kloadable_routine(vsmp$intall_bit);
    check_kloadable_routine(vsmp$intall_acq);
    check_kloadable_routine(vsmp$intall_bit_acq);
    check_kloadable_routine(vsmp$stop_cpu);
    check_kloadable_routine(vsmp$show_cpu);
    check_kloadable_routine(vsmp$halt_cpu);
    check_kloadable_routine(vsmp$controlp_cpus);
    check_kloadable_routine(vsmp$invalid_single);
    check_kloadable_routine(vsmp$select_primary);
    check_kloadable_routine(vsmp$setup_smp);
    check_kloadable_routine(vsmp$read_todr);
    check_kloadable_routine(vsmp$write_todr);
    check_kloadable_routine(vsmp$intsr);
    check_kloadable_routine(vsmp$spec_ipint);
    check_kloadable_routine(vsmp$iniprocreg);
    check_kloadable_routine(exe_clear_errors);
    check_kloadable_routine(vsmp$start_cpu);
    check_kloadable_routine(vsmp$setup_cpu);
    check_kloadable_routine(vcon$putchar);
    check_kloadable_routine(vcon$getchar);
    check_kloadable_routine(vcon$owncty);
    check_kloadable_routine(vcon$releasecty);
    check_kloadable_routine(vsmp$virtcons_server);

    return;

cleanup:
    exit(VSMP_MSG_MISLINKED);
}

#undef check_kloadable
#undef check_kloadable_routine

/***************************************************************************************
*  Acquire/Release lock preventing concurrent execution of other VSMP instances        *
 * on this node                                                                        *
***************************************************************************************/

static bool_t enq_held = FALSE;       /* set if holding ENQ lock */
static LKSB lksb;
static uint32 exit_handler_blk[4];
uint32 exit_status;

static void acquire_node_lock()
{
    ILE3 item_list[3];
    IOSB iosb;
    DESC desc;
    byte_t sysid[6];
    uint16 retlen;
    int status = SS$_ABORT;
    char* xp;
    char lockname[sizeof("VSMP$VAXMP_KLOAD_112233445566") + 4];

    /*
     * declare exit handler to release lock: this is redundant in user mode, 
     * as user-mode locks will be released on image rundown
     */
    exit_handler_blk[0] = 0;
    exit_handler_blk[1] = (uint32) exit_handler;
    exit_handler_blk[2] = 1;
    exit_handler_blk[3] = (uint32) & exit_status;
    sys$dclexh(exit_handler_blk);

    /* get node ID */
    ile3_make(&item_list[0], SYI$_NODE_SYSTEMID, 6, sysid, & retlen);
    ile3_term(&item_list[1]);

    check_vms_status(sys$getsyiw(0, NULL, NULL, &item_list, &iosb, NULL, 0));
    check_vms_status(iosb.iosb$w_status);
    check(retlen == 6);

    /* build lock name and acquire the lock */
    xp = fmt_s(lockname, "VSMP$VAXMP_KLOAD_");
    xp = fmt_x2(xp, sysid[5]);
    xp = fmt_x2(xp, sysid[4]);
    xp = fmt_x2(xp, sysid[3]);
    xp = fmt_x2(xp, sysid[2]);
    xp = fmt_x2(xp, sysid[1]);
    xp = fmt_x2(xp, sysid[0]);
    *xp = '\0';
    mkdesc(&desc, lockname);
    check_vms_status(sys$enqw(0, LCK$K_EXMODE, &lksb, LCK$M_SYSTEM, &desc, 0, NULL, 0, NULL, PSL$C_KERNEL, 0, NULL));
    check_vms_status(lksb.lksb$w_status);

    enq_held = TRUE;

    return;

cleanup:
    exit(status);
}

static void exit_handler(uint32 status)
{
    release_node_lock();
}

static void release_node_lock()
{
    if (enq_held)
    {
        sys$deq(lksb.lksb$l_lock_id, NULL, PSL$C_KERNEL, 0);
        enq_held = FALSE;
    }
}

/***************************************************************************************
* Utility routines                                                                     *
***************************************************************************************/

/* try to match command line argument to a known keyword */
static bool_t is_keyword(const char* s, const char* kwd, int len)
{
    int k;

    if (len == 0)
    {
        len = strlen(kwd);
    }
    else if (len < 0)
    {
        len = strlen(kwd);
        if (len != strlen(s))
            return FALSE;
    }

    for (k = 0; ;  k++)
    {
        if (s[k] == '\0' && kwd[k] == '\0')  break;
        if (s[k] == '\0' && k >= len)  break;
        if (toupper(s[k]) != toupper(kwd[k]))
            return FALSE;
    }

    return TRUE;
}

static bool_t split_key_value(const char* arg, char* key, size_t keysize, char* value, size_t valuesize)
{
    char* xp = key;
    const char* p = arg;

    while (*p && *p != '=')
    {
        if (xp - key == keysize - 1)
            return FALSE;
        *xp++ = *p++;
    }
    *xp = '\0';

    if (*p++ != '=')
        return FALSE;

    xp = value;
    while (*p)
    {
        if (xp - value == valuesize - 1)
            return FALSE;
        *xp++ = *p++;
    }
    *xp = '\0';

    return TRUE;
}

/* try to match command line argument to a known qualifier and get its value (if any) */
static bool_t is_qualifier(const char* s, const char* kwd, int len, const char** pqvalue)
{
    int k;

    *pqvalue = NULL;

    if (*s++ != '/')
        return FALSE;

    for (k = 0; ;  k++)
    {
        if (s[k] == '\0' || s[k] == '=')
        {
            if (kwd[k] == '\0' || k >= len)  break;
        }
        if (toupper(s[k]) != toupper(kwd[k]))
            return FALSE;
    }

    if (s[k] == '\0')
        return TRUE;

    if (s[k] != '=')
        return FALSE;

    if (s[k + 1] == '\0')
        return FALSE;

    *pqvalue = & s[k + 1];
    return TRUE;
}

static void inv_opt_val(const char* optname)
{
    print_msg_1(VSMP_MSG_INVOPTVAL, (uint32) optname);
    exit(VSMP_MSG_INVOPTVAL | STS$M_INHIB_MSG);
}

static void print_msg(uint32 status)
{
    uint32 msgvec[] = { 0x000F0002, status, 0x000F0000 };
    sys$putmsg(msgvec);
}

static void print_msg_1(uint32 status, uint32 arg)
{
    uint32 nargs = 1;
    uint32 msgvec[] = { 0x000F0000 | (nargs + 2), status, 0x000F0000 | nargs, arg };
    sys$putmsg(msgvec);
}

static void print_2msgs(uint32 st1, uint32 st2)
{
    uint32 msgvec[] = { 0x000F0004, st1, 0x000F0000, st2, 0x000F0000 };
    sys$putmsg(msgvec);
}

static void print_3msgs_val(uint32 st1, uint32 st2, uint32 st3, uint32 val)
{
    uint32 nargs = 1;
    uint32 msgvec[] = { 0x000F0000 | (nargs + 6), st1, 0x000F0000, st2, 0x000F0000, st3, 0x000F0000 | nargs, val };
    sys$putmsg(msgvec);
}

static bool_t parse_address(const char* sa, byte_t** paddr)
{
    int k, status;
    uint32 res = 0;
    check(kstrlen(sa) == 8);
    unsigned char c;
    while (c = (unsigned char) *sa++)
    {
        if (c >= '0' && c <= '9')
            res = (res << 4) + (c - '0');
        else if (c >= 'A' && c <= 'F')
            res = (res << 4) + (c - 'A') + 10;
        else if (c >= 'a' && c <= 'f')
            res = (res << 4) + (c - 'a') + 10;
        else
            check(FALSE);
    }

    *paddr = (byte_t*) res;
    return TRUE;

cleanup:
    *paddr = NULL;
    return FALSE;
}

static int kstrlen(const char* p)
{
    const char* ep = p;
    while (*ep++)  ;
    return ep - p - 1;
}

static char* fmt_s(char* bp, const char* value)
{
    while (*bp++ = *value++)
        ;
    return bp;
}

const static char* xdigits = "0123456789ABCDEF";

static char* fmt_x2(char* bp, uint32 value)
{
    *bp++ = xdigits[(value >> 4) & 0xF];
    *bp++ = xdigits[value & 0xF];
    return bp;
}

static char* fmt_x8(char* bp, uint32 value)
{
    bp = fmt_x2(bp, (value >> 24) & 0xFF);
    bp = fmt_x2(bp, (value >> 16) & 0xFF);
    bp = fmt_x2(bp, (value >> 8) & 0xFF);
    bp = fmt_x2(bp, value & 0xFF);
    return bp;
}

static void kprint_prefix_x8_crlf(const char* prefix, uint32 value)
{
    char buf[9];
    fmt_x8(buf, value);
    buf[8] = '\0';
    kprint(prefix);
    kprint_crlf(buf);
}

/* uppercase the string */
static void k_strupr(char *s)
{
    char c;
    while (c = *s)
    {
        if (c >= 'a' && c <= 'z')
            c += 'A' - 'a';
        *s++ = c;
    }
}

/* convert "v" into output buffer as nul-terminated decimal string */
static void k_itoa(char* s, int v)
{
    if (v == 0)
    {
        *s++ = '0';
    }
    else
    {
        char buf[20];
        char* bp = buf;
        while (v)
        {
            *bp++ = '0' + (v % 10);
            v /= 10;
        }
        bp--;
        while (bp >= buf)
        {
            *s++ = *bp--;
        }

    }
    *s++ = '\0';
}

static bool_t k_streq(const char* s1, const char* s2)
{
    for (;;)
    {
        if (*s1 == '\0' && *s2 == '\0')
            return TRUE;
        if (*s1++ != *s2++)
            return FALSE;
    }
}

/* 
 * This function is called by MACRO code
 *
 * Compare string "s1" and "s2" upto "len" characters.
 */
bool_t k_streqi_cnt(const char* s1, const char* s2, int len)
{
    int k;

    for (k = 0;  k < len;  k++)
    {
        char c1 = *s1++;
        char c2 = *s2++;
        if (c1 >= 'a' && c1 <= 'z')  c1 = c1 - 'a' + 'A';
        if (c2 >= 'a' && c2 <= 'z')  c2 = c2 - 'a' + 'A';
        if (c1 != c2)  return FALSE;
        if (c1 == 0)  return TRUE;
    }

    return TRUE;
}

/* parse unsigned decimal number */
static uint32 parse_udec(const char* sv, const char* optname)
{
    uint32 res = 0;
    uint32 ores = 0;
    char c;

    CHECK(*sv);
    while (c = *sv++)
    {
        CHECK(c >= '0' && c <= '9');
        CHECK(res <= 0xFFFFFFFF / 10);
        res *= 10;
        ores = res;
        res += c - '0';
        CHECK(res >= ores);
    }

    return res;

cleanup:
    inv_opt_val(optname);
}

/*
 * This function is called by MACRO code
 *
 * Look up code identified by pattern in range "pstart ... pend" around "xaddr",
 * limited to offsets upto "max_before" and "max_after" bytes around "xaddr".
 */
uint32 lookup_code(const byte_t* xaddr, int max_before, int max_after, const byte_t* pstart, const byte_t* pend)
{
    int maxoff;
    int off;

    maxoff = max_before;
    if (max_after > maxoff)  maxoff = max_after;

    if ($VMS_STATUS_SUCCESS(lookup_code_match(xaddr, pstart, pend)))
    {
        lookup_code_addr = xaddr;
        return SS$_NORMAL;
    }

    for (off = 1;  off <= maxoff;  off++)
    {
        if (off <= max_before && $VMS_STATUS_SUCCESS(lookup_code_match(xaddr - off, pstart, pend)))
        {
            lookup_code_addr = xaddr - off;
            return SS$_NORMAL;
        }

        if (off <= max_after && $VMS_STATUS_SUCCESS(lookup_code_match(xaddr + off, pstart, pend)))
        {
            lookup_code_addr = xaddr + off;
            return SS$_NORMAL;
        }
    }

    return SS$_IDMISMATCH;
}

static uint32 lookup_code_match(const byte_t* xaddr, const byte_t* pstart, const byte_t* pend)
{
    lib$establish(accvio_handler);
    while (pstart != pend)
    {
        if (*pstart++ != *xaddr++)
            return SS$_IDMISMATCH;
    }
    lib$revert();
    return SS$_NORMAL;
}

/*
 * lookup_code_ex is similar to lookup_code, however it will ignore difference in pattern bytes indicated
 * by corresponding mask byte != 0, and will also require "checker" routine to confirm potential match.
 */
uint32 lookup_code_ex(const byte_t* xaddr, int max_before, int max_after, const byte_t* pstart, const byte_t* pend, 
                      const byte_t* mask, const byte_t* mask_e, lookup_code_checker_t checker, void* checker_arg)
{
    int maxoff;
    int off;

    /* check mask size validity */
    if (mask && mask_e && mask_e - mask != pend - pstart)
        return SS$_IDMISMATCH;

    maxoff = max_before;
    if (max_after > maxoff)  maxoff = max_after;

    if ($VMS_STATUS_SUCCESS(lookup_code_match_ex(xaddr, pstart, pend, mask, checker, checker_arg)))
    {
        lookup_code_addr = xaddr;
        return SS$_NORMAL;
    }

    for (off = 1;  off <= maxoff;  off++)
    {
        if (off <= max_before && $VMS_STATUS_SUCCESS(lookup_code_match_ex(xaddr - off, pstart, pend, mask, checker, checker_arg)))
        {
            lookup_code_addr = xaddr - off;
            return SS$_NORMAL;
        }

        if (off <= max_after && $VMS_STATUS_SUCCESS(lookup_code_match_ex(xaddr + off, pstart, pend, mask, checker, checker_arg)))
        {
            lookup_code_addr = xaddr + off;
            return SS$_NORMAL;
        }
    }

    return SS$_IDMISMATCH;
}

static uint32 lookup_code_match_ex(const byte_t* xaddr, const byte_t* pstart, const byte_t* pend, 
                                   const byte_t* mask, lookup_code_checker_t checker, void* checker_arg)
{
    const byte_t* xa = xaddr;
    lib$establish(accvio_handler);
    while (pstart != pend)
    {
        if ((mask == NULL || *mask == 0) && *pstart != *xaddr)
            return SS$_IDMISMATCH;
        pstart++;
        xaddr++;
        if (mask)  mask++;
    }
    if (checker && !(*checker)(xa, checker_arg))
        return SS$_IDMISMATCH;
    lib$revert();
    return SS$_NORMAL;
}

static uint32 safe_read(const byte_t* addr, byte_t* pvalue, uint32 count)
{
    lib$establish(accvio_handler);
    while (count--)
        *pvalue++ = *addr++;
    lib$revert();
    return SS$_NORMAL;
}


static uint32 accvio_handler(struct chf$signal_array* sig, struct chf$mech_array* mech)
{
    if (sig->chf$l_sig_name == SS$_ACCVIO)
    {
        return lib$sig_to_ret(sig, mech);
    }
    else
    {
        return SS$_RESIGNAL;
    }
}

/*
 * Lookup helpers for the code patterns in XQDRIVER, PUDRIVER and any driver
 */
uint32 lookup_in_xqdrv(uint32 errstatus, int initial, const byte_t* pstart, const byte_t* pend)
{
    return lookup_in_drv(xqdrv_dpt, xqdrv_size, errstatus, initial, pstart, pend);
}

uint32 lookup_in_xqdrv_ex(uint32 errstatus, int initial, const byte_t* pstart, const byte_t* pend,
                          const byte_t* mask, const byte_t* mask_e, lookup_code_checker_t checker, void* checker_arg)
{
    return lookup_in_drv_ex(xqdrv_dpt, xqdrv_size, errstatus, initial, pstart, pend, mask, mask_e, checker, checker_arg);
}

uint32 lookup_in_pudrv(uint32 errstatus, int initial, const byte_t* pstart, const byte_t* pend)
{
    return lookup_in_drv(pudrv_dpt, pudrv_size, errstatus, initial, pstart, pend);
}

uint32 lookup_in_pudrv_ex(uint32 errstatus, int initial, const byte_t* pstart, const byte_t* pend,
                          const byte_t* mask, const byte_t* mask_e, lookup_code_checker_t checker, void* checker_arg)
{
    return lookup_in_drv_ex(pudrv_dpt, pudrv_size, errstatus, initial, pstart, pend, mask, mask_e, checker, checker_arg);
}

uint32 lookup_in_drv(void* dpt, uint32 drv_size, uint32 errstatus, int initial, const byte_t* pstart, const byte_t* pend)
{
    return lookup_in_drv_ex(dpt, drv_size, errstatus, initial, pstart, pend, NULL, NULL, NULL, NULL);
}

uint32 lookup_in_drv_ex(void* dpt, uint32 drv_size, uint32 errstatus, int initial, const byte_t* pstart, const byte_t* pend,
                        const byte_t* mask, const byte_t* mask_e, lookup_code_checker_t checker, void* checker_arg)
{
    int max_before = 0x800;
    int max_after = 0x800;
    int patlen = pend - pstart;
    uint32 status;

    if (mask && mask_e && mask_e - mask != patlen)
        return errstatus;

    CHECK(dpt && drv_size && drv_size >= patlen);
    if (initial + patlen > drv_size)
    {
        initial = drv_size - patlen;
        CHECK(initial >= 0);
    }

    max_after = imin(max_after, drv_size - (initial + patlen));
    max_before = imin(max_before, initial);
    if (mask == NULL && checker == NULL)
        check_vms_status(lookup_code(offset_ptr(byte_t, dpt, initial), max_before, max_after, pstart, pend));
    else
        check_vms_status(lookup_code_ex(offset_ptr(byte_t, dpt, initial), max_before, max_after, pstart, pend, mask, mask_e, checker, checker_arg));
    return SS$_NORMAL;

cleanup:
    return errstatus;
}
