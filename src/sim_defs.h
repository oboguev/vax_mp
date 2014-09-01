/* sim_defs.h: simulator definitions

   Copyright (c) 1993-2008, Robert M Supnik

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

   05-Jan-11    MP      Added Asynch I/O support (SPO: modified for VAX MP)
   18-Jan-11    MP      Added log file reference count support
   21-Jul-08    RMS     Removed inlining support
   28-May-08    RMS     Added inlining support
   28-Jun-07    RMS     Added IA64 VMS support (from Norm Lastovica)
   18-Jun-07    RMS     Added UNIT_IDLE flag
   18-Mar-07    RMS     Added UNIT_TEXT flag
   07-Mar-07    JDB     Added DEBUG_PRJ macro
   18-Oct-06    RMS     Added limit check for clock synchronized keyboard waits
   13-Jul-06    RMS     Guarantee CBUFSIZE is at least 256
   07-Jan-06    RMS     Added support for breakpoint spaces
                        Added REG_FIT flag
   16-Aug-05    RMS     Fixed C++ declaration and cast problems
   11-Mar-05    RMS     Moved 64b data type definitions outside USE_INT64
   07-Feb-05    RMS     Added assertion fail stop
   05-Nov-04    RMS     Added support for SHOW opt=val
   20-Oct-04    RMS     Converted all base types to typedefs
   21-Sep-04    RMS     Added switch to flag stop message printout
   06-Feb-04    RMS     Moved device and unit user flags fields (V3.2)
                RMS     Added REG_VMAD
   29-Dec-03    RMS     Added output stall status
   15-Jun-03    RMS     Added register flag REG_VMIO
   23-Apr-03    RMS     Revised for 32b/64b t_addr
   14-Mar-03    RMS     Lengthened default serial output wait
   31-Mar-03    RMS     Added u5, u6 fields
   18-Mar-03    RMS     Added logical name support
                        Moved magtape definitions to sim_tape.h
                        Moved breakpoint definitions from scp.c
   03-Mar-03    RMS     Added sim_fsize
   08-Feb-03    RMS     Changed sim_os_sleep to void, added match_ext
   05-Jan-03    RMS     Added hidden switch definitions, device dyn memory support,
                        parameters for function pointers, case sensitive SET support
   22-Dec-02    RMS     Added break flag
   08-Oct-02    RMS     Increased simulator error code space
                        Added Telnet errors
                        Added end of medium support
                        Added help messages to CTAB
                        Added flag and context fields to DEVICE
                        Added restore flag masks
                        Revised 64b definitions
   02-May-02    RMS     Removed log status codes
   22-Apr-02    RMS     Added magtape record length error
   30-Dec-01    RMS     Generalized timer package, added circular arrays
   07-Dec-01    RMS     Added breakpoint package
   01-Dec-01    RMS     Added read-only unit support, extended SET/SHOW features,
                        improved error messages
   24-Nov-01    RMS     Added unit-based registers
   27-Sep-01    RMS     Added queue count prototype
   17-Sep-01    RMS     Removed multiple console support
   07-Sep-01    RMS     Removed conditional externs on function prototypes
   31-Aug-01    RMS     Changed int64 to t_int64 for Windoze
   17-Jul-01    RMS     Added additional function prototypes
   27-May-01    RMS     Added multiple console support
   15-May-01    RMS     Increased string buffer size
   25-Feb-01    RMS     Revisions for V2.6
   15-Oct-00    RMS     Editorial revisions for V2.5
   11-Jul-99    RMS     Added unsigned int data types
   14-Apr-99    RMS     Converted t_addr to unsigned
   04-Oct-98    RMS     Additional definitions for V2.4

   The interface between the simulator control package (SCP) and the
   simulator consists of the following routines and data structures

        sim_name                simulator name string
        sim_devices[]           array of pointers to simulated devices
        sim_PC                  pointer to saved PC register descriptor
        sim_interval            simulator interval to next event
        sim_stop_messages[]     array of pointers to stop messages
        sim_instr()             instruction execution routine
        sim_load()              binary loader routine
        sim_emax                maximum number of words in an instruction

   In addition, the simulator must supply routines to print and parse
   architecture specific formats

        print_sym               print symbolic output
        parse_sym               parse symbolic input
*/

// #pragma message ("Loading sim_defs.h")

#ifndef _SIM_DEFS_H_
#define _SIM_DEFS_H_    0

#include <stddef.h>
#include <limits.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <errno.h>
#include <limits.h>

#if defined(___llvm__)
#  message "LLVM compiler is suboptimal for SIMH. Use regular GCC if possible."
#endif

#if defined(__linux)
#  define __STDC_LIMIT_MACROS
#  define __STDC_FORMAT_MACROS
#  include <stdint.h>
#  include <inttypes.h>
#elif defined(__APPLE__)
#  include <stdint.h>
#  include <inttypes.h>
#endif

#if defined(_WIN32) && !defined(PRIu64)
#  define PRIu64 "I64u"
#endif

#ifdef _WIN32
#  pragma warning(disable:4996)
#endif

#ifndef TRUE
#  define TRUE     1
#  define FALSE    0
#endif

#ifndef __ORDER_LITTLE_ENDIAN__
#  define __ORDER_LITTLE_ENDIAN__ 1234
#endif

#ifndef __ORDER_BIG_ENDIAN__
#  define __ORDER_BIG_ENDIAN__ 4321
#endif

#if !defined(UINT64_MAX) && defined(_UI64_MAX)
#  define UINT64_MAX _UI64_MAX
#endif

#if !defined(UINT32_MAX) && defined(UINT_MAX)
#  define UINT32_MAX UINT_MAX
#endif

#if !defined(INT32_MAX) && defined(INT_MAX)
#  define INT32_MAX INT_MAX
#endif

/* suppress compiler warning message when code does not have return statement
   after invoking a function that never returns */
#define never_returns_bool_t  return FALSE
#define never_returns_int32   return 0

#ifndef __SIZEOF_POINTER__
#  if defined(_WIN64)
#    define __SIZEOF_POINTER__  8
#  elif defined(_WIN32)
#    define __SIZEOF_POINTER__  4
#  else
#     error Unimplemented
#  endif
#endif

#ifdef _WIN32
# define SIM_INLINE __forceinline
#elif defined(__GNUC__)
# define SIM_INLINE inline __attribute__ ((__always_inline__))
#else
# define SIM_INLINE inline
#endif

#if defined(__GNUC__)
#  define likely(x)       __builtin_expect((x), TRUE)
#  define unlikely(x)     __builtin_expect((x), FALSE)
#else
#  define likely(x)       (x)
#  define unlikely(x)     (x)
#endif

#if !defined(USE_CLOCK_THREAD)
#  if defined(_WIN32) || defined (__linux) || defined(__APPLE__)
#    define USE_CLOCK_THREAD TRUE
#  else
#    define USE_CLOCK_THREAD FALSE
#  endif
#endif

#if defined(_WIN32)
#  define streqi(a, b) (0 == stricmp((a), (b)))
#elif defined(__linux) || defined(__APPLE__)
#  include <strings.h>
#  define streqi(a, b) (0 == strcasecmp((a), (b)))
#endif

#if (PATH_MAX >= 128)
#  define CBUFSIZE        (128 + PATH_MAX)                /* string buf size */
#else
#  define CBUFSIZE        256
#endif

/* thread-interlocked file access */
class smp_file_critical_section;

class SMP_FILE
{
public:
    FILE* stream;
    smp_file_critical_section* lock_cs;
};

extern SMP_FILE* smp_stdin;
extern SMP_FILE* smp_stdout;
extern SMP_FILE* smp_stderr;
extern SMP_FILE* sim_log;

struct sim_smp_fileref
{
    char                name[CBUFSIZE];                 /* file name */
    SMP_FILE            *file;                          /* file handle */
    int                 refcount;                       /* reference count */
};
typedef struct sim_smp_fileref SMP_FILEREF;

/* forward declarations */
class sim_unit;
typedef sim_unit UNIT;
class CPU_UNIT;
class run_scope_context;
#define RUN_DECL       CPU_UNIT* cpu_unit
#define RUN_PASS       cpu_unit 
#define RUN_PASS_NULL  NULL
#define RUN_RSCX_DECL  CPU_UNIT* cpu_unit, run_scope_context* rscx
#define RUN_RSCX_PASS  cpu_unit, rscx 

int smp_printf(const char* fmt, ...);
int fprintf(SMP_FILE* stream, const char* fmt, ...);
int fflush(SMP_FILE* stream);
int fscanf(SMP_FILE* stream, const char* fmt, ...);
int fgetc(SMP_FILE *stream);
char *fgets(char *string, int n, SMP_FILE *stream);
size_t fread(void *buffer, size_t size, size_t count, SMP_FILE *stream);
int fputc(int c, SMP_FILE *stream);
int fputs(const char *string, SMP_FILE *stream);
size_t fwrite(const void *buffer, size_t size, size_t count, SMP_FILE *stream);
int fseek(SMP_FILE* stream, long offset, int origin);
int fsetpos(SMP_FILE *stream, const fpos_t *pos);
void rewind(SMP_FILE *stream);
int vfprintf(SMP_FILE *stream, const char *format, va_list argptr);
long ftell(SMP_FILE* stream);
int ferror(SMP_FILE* stream);
int feof(SMP_FILE* stream);
int getc(SMP_FILE *stream);
int putc(int c, SMP_FILE *stream);
int setvbuf(SMP_FILE *stream, char *buffer, int mode, size_t size);
void clearerr(SMP_FILE *stream);
int fclose(SMP_FILE *stream);
int fgetpos(SMP_FILE *stream, fpos_t *pos);
void smp_perror(const char *string);
int _fileno(SMP_FILE* stream);
int smp_putchar(int c);
SMP_FILE *smp_fopen(const char* filename, const char* mode);
SMP_FILE *smp_fopen64(const char* filename, const char* mode);
SMP_FILE *smp_file_wrap(FILE* fp);
#if defined (__linux)
int fseeko64(SMP_FILE *sfd, off64_t offset, int whence);
off64_t ftello64(SMP_FILE *sfd);
#elif defined (__APPLE__) || defined (__FreeBSD__)
int fseeko(SMP_FILE *sfd, off_t offset, int whence);
off_t ftello(SMP_FILE *sfd);
#endif
void panic(const char* cause);
void* operator_new_aligned(size_t size, size_t alignment);
void operator_delete_aligned(void* p);

/* alignment and padding for structures and their fields */
#if defined(_WIN32)
# define SIM_ALIGN_8   __declspec(align(1))
# define SIM_ALIGN_16  __declspec(align(2))
# define SIM_ALIGN_32  __declspec(align(4))
# define SIM_ALIGN_64  __declspec(align(8))
#elif defined(__GNUC__)
# define SIM_ALIGN_8   __attribute__ ((__aligned__ (1)))  __attribute__ ((packed))
# define SIM_ALIGN_16  __attribute__ ((__aligned__ (2)))
# define SIM_ALIGN_32  __attribute__ ((__aligned__ (4)))
# define SIM_ALIGN_64  __attribute__ ((__aligned__ (8)))
#else
# error Unsupported compiler
#endif

/* Length specific integer declarations */

#if defined (VMS)
#  include <ints.h>
#else
typedef signed char     int8;
typedef signed short    int16;
typedef signed int      int32;
typedef unsigned char   uint8;
typedef unsigned short  uint16;
typedef unsigned int    uint32;
#endif

typedef int             t_stat;                         /* status */
typedef int             t_bool;                         /* boolean */
typedef unsigned char   t_byte;

/* 64b integers */

#if defined (__GNUC__)                                  /* GCC */
typedef signed long long        t_int64;
typedef unsigned long long      t_uint64;
#elif defined (_WIN32)                                  /* Windows */
typedef signed __int64          t_int64;
typedef unsigned __int64        t_uint64;
#elif (defined (__ALPHA) || defined (__ia64)) && defined (VMS) /* 64b VMS */
typedef signed __int64          t_int64;
typedef unsigned __int64        t_uint64;
#elif defined (__ALPHA) && defined (__unix__)           /* Alpha UNIX */
typedef signed long             t_int64;
typedef unsigned long           t_uint64;
#else                                                   /* default */
#define t_int64                 signed long long
#define t_uint64                unsigned long long
#endif                                                  /* end 64b */

#if defined (USE_INT64)                                 /* 64b data */
typedef t_int64         t_svalue;                       /* signed value */
typedef t_uint64        t_value;                        /* value */
#else                                                   /* 32b data */
typedef int32           t_svalue;
typedef uint32          t_value;
#endif                                                  /* end 64b data */

#if __SIZEOF_POINTER__ == 8
# define SIM_ALIGN_PTR  SIM_ALIGN_64
  typedef t_int64 t_addr_off;
  typedef t_uint64 t_addr_val;
#else
# define SIM_ALIGN_PTR  SIM_ALIGN_32
  typedef int32 t_addr_off;
  typedef uint32 t_addr_val;
#endif

#if defined (USE_INT64) && defined (USE_ADDR64)         /* 64b address */
  typedef t_uint64         t_addr;
# define T_ADDR_W          64
# define SIM_ALIGN_T_ADDR  SIM_ALIGN_64
#else                                                   /* 32b address */
  typedef uint32           t_addr;
# define T_ADDR_W          32
# define SIM_ALIGN_T_ADDR  SIM_ALIGN_32
#endif                                                  /* end 64b address */

#if defined (USE_INT64)
   typedef t_uint64 UINT64;
#  define UINT64_EQ(a, b) ((a) == (b))
#  define UINT64_LT(a, b) ((a) < (b))
#  define UINT64_LE(a, b) ((a) <= (b))
#  define UINT64_GT(a, b) ((a) > (b))
#  define UINT64_GE(a, b) ((a) >= (b))
#  define UINT64_SET_ZERO(a)  do {(a) = 0; } while (0)
#  define UINT64_IS_ZERO(a)  ((a) == 0)
#  define UINT64_INC(a)  (++(a))
#  define UINT64_ZERO_VALUE 0
#  define UINT64_ISMAX(a) ((a) == UINT64_MAX)
#  define UINT64_TO_DOUBLE(a) ((double) (a))
#  define UINT64_FROM_UINT32(a64, b32)  do { (a64) = (UINT64) (uint32) (b32); } while (0)
#  define UINT64_LT_UINT32(a64, b32)  ((a64) < (b32))
#  define UINT64_MUL_UINT32(a64, b32)   do { (a64) *= (b32); } while (0)
#  define UINT64_DIV_UINT32(a64, b32)   do { (a64) /= (b32); } while (0)
#  define UINT64_TO_UINT32(a64, b32, v)  do { if ((a64) <= UINT32_MAX) { (b32) = (uint32) (a64);  (v) = TRUE; }  else { (v) = FALSE; } } while (0)
#else
   typedef struct { uint32 lo; uint32 hi; } UINT64;
#  define UINT64_EQ(a, b) ((a).hi == (b).hi && (a).lo == (b).lo)
#  define UINT64_GT(a, b) ((a).hi > (b).hi || (a).hi == (b).hi && (a).lo > (b).lo)
#  define UINT64_GE(a, b) ((a).hi > (b).hi || (a).hi == (b).hi && (a).lo >= (b).lo)
#  define UINT64_LT(a, b) ((a).hi < (b).hi || (a).hi == (b).hi && (a).lo < (b).lo)
#  define UINT64_LE(a, b) ((a).hi < (b).hi || (a).hi == (b).hi && (a).lo <= (b).lo)
#  define UINT64_SET_ZERO(a)  do {(a).hi = 0; (a).lo = 0; } while (0)
#  define UINT64_INC(a)  do { if ((a).lo++ == 0) (a).hi++; } while (0)
#  define UINT64_ZERO_VALUE { 0, 0 }
#  define UINT64_IS_ZERO(a)  ((a).lo == 0 && (a).hi == 0)
#  define UINT64_ISMAX(a) ((a).lo == UINT32_MAX && (a).hi == UINT32_MAX)
#  define UINT64_TO_DOUBLE(a) (4294967296.0 * (double) a.hi + (double) a.lo)
#  define UINT64_FROM_UINT32(a64, b32)  do { (a64).hi = 0; (a64).lo = (b32); } while (0)
#  define UINT64_LT_UINT32(a64, b32)  ((a64).hi == 0 && (a64).lo < (b32).lo)
#  error Implement UINT64_MUL_UINT32 and UINT64_DIV_UINT32
#  define UINT64_TO_UINT32(a64, b32, v)  do { if ((a64).hi == 0) { (b32) = (a64).lo;  (v) = TRUE; }  else { (v) = FALSE; } } while (0)
#endif

/* min/max helpers */

static uint32 SIM_INLINE imax(uint32 a, uint32 b)
{
    return (a > b) ? a : b;
}

static int32 SIM_INLINE imax(int32 a, int32 b)
{
    return (a > b) ? a : b;
}

static uint32 SIM_INLINE imin(uint32 a, uint32 b)
{
    return (a < b) ? a : b;
}

static int32 SIM_INLINE imin(int32 a, int32 b)
{
    return (a < b) ? a : b;
}


/* System independent definitions */

#define FLIP_SIZE       (1 << 16)                       /* flip buf size */
#if !defined (PATH_MAX)                                 /* usually in limits */
#define PATH_MAX        512
#endif

/* Breakpoint spaces definitions */

#define SIM_BKPT_N_SPC  64                              /* max number spaces */
#define SIM_BKPT_V_SPC  26                              /* location in arg */

/* Extended switch definitions (bits >= 26) */

#define SIM_SW_HIDE     (1u << 26)                      /* enable hiding */
#define SIM_SW_REST     (1u << 27)                      /* attach/restore */
#define SIM_SW_REG      (1u << 28)                      /* register value */
#define SIM_SW_STOP     (1u << 29)                      /* stop message */

/* Simulator status codes

   0                    ok
   1 - (SCPE_BASE - 1)  simulator specific
   SCPE_BASE - n        general
*/

#define SCPE_OK         0                               /* normal return */
#define SCPE_BASE       64                              /* base for messages */
#define SCPE_NXM        (SCPE_BASE + 0)                 /* nxm */
#define SCPE_UNATT      (SCPE_BASE + 1)                 /* no file */
#define SCPE_IOERR      (SCPE_BASE + 2)                 /* I/O error */
#define SCPE_CSUM       (SCPE_BASE + 3)                 /* loader cksum */
#define SCPE_FMT        (SCPE_BASE + 4)                 /* loader format */
#define SCPE_NOATT      (SCPE_BASE + 5)                 /* not attachable */
#define SCPE_OPENERR    (SCPE_BASE + 6)                 /* open error */
#define SCPE_MEM        (SCPE_BASE + 7)                 /* alloc error */
#define SCPE_ARG        (SCPE_BASE + 8)                 /* argument error */
#define SCPE_STEP       (SCPE_BASE + 9)                 /* step expired */
#define SCPE_UNK        (SCPE_BASE + 10)                /* unknown command */
#define SCPE_RO         (SCPE_BASE + 11)                /* read only */
#define SCPE_INCOMP     (SCPE_BASE + 12)                /* incomplete */
#define SCPE_STOP       (SCPE_BASE + 13)                /* sim stopped */
#define SCPE_EXIT       (SCPE_BASE + 14)                /* sim exit */
#define SCPE_TTIERR     (SCPE_BASE + 15)                /* console tti err */
#define SCPE_TTOERR     (SCPE_BASE + 16)                /* console tto err */
#define SCPE_EOF        (SCPE_BASE + 17)                /* end of file */
#define SCPE_REL        (SCPE_BASE + 18)                /* relocation error */
#define SCPE_NOPARAM    (SCPE_BASE + 19)                /* no parameters */
#define SCPE_ALATT      (SCPE_BASE + 20)                /* already attached */
#define SCPE_TIMER      (SCPE_BASE + 21)                /* hwre timer err */
#define SCPE_SIGERR     (SCPE_BASE + 22)                /* signal err */
#define SCPE_TTYERR     (SCPE_BASE + 23)                /* tty setup err */
#define SCPE_SUB        (SCPE_BASE + 24)                /* subscript err */
#define SCPE_NOFNC      (SCPE_BASE + 25)                /* func not imp */
#define SCPE_UDIS       (SCPE_BASE + 26)                /* unit disabled */
#define SCPE_NORO       (SCPE_BASE + 27)                /* rd only not ok */
#define SCPE_INVSW      (SCPE_BASE + 28)                /* invalid switch */
#define SCPE_MISVAL     (SCPE_BASE + 29)                /* missing value */
#define SCPE_2FARG      (SCPE_BASE + 30)                /* too few arguments */
#define SCPE_2MARG      (SCPE_BASE + 31)                /* too many arguments */
#define SCPE_NXDEV      (SCPE_BASE + 32)                /* nx device */
#define SCPE_NXUN       (SCPE_BASE + 33)                /* nx unit */
#define SCPE_NXREG      (SCPE_BASE + 34)                /* nx register */
#define SCPE_NXPAR      (SCPE_BASE + 35)                /* nx parameter */
#define SCPE_NEST       (SCPE_BASE + 36)                /* nested DO */
#define SCPE_IERR       (SCPE_BASE + 37)                /* internal error */
#define SCPE_MTRLNT     (SCPE_BASE + 38)                /* tape rec lnt error */
#define SCPE_LOST       (SCPE_BASE + 39)                /* Telnet conn lost */
#define SCPE_TTMO       (SCPE_BASE + 40)                /* Telnet conn timeout */
#define SCPE_STALL      (SCPE_BASE + 41)                /* Telnet conn stall */
#define SCPE_AFAIL      (SCPE_BASE + 42)                /* assert failed */
#define SCPE_SWSTP      (SCPE_BASE + 43)                /* cannot step because syncw limit reached */

#define SCPE_MAX_ERR    (SCPE_BASE + 43)                /* Maximum SCPE Error Value */

#define SCPE_KFLAG      0010000                         /* tti data flag */
#define SCPE_BREAK      0020000                         /* tti break flag */

/* Print value format codes */

#define PV_RZRO         0                               /* right, zero fill */
#define PV_RSPC         1                               /* right, space fill */
#define PV_LEFT         2                               /* left justify */

/* Default timing parameters */

#define KBD_POLL_WAIT   5000                            /* keyboard poll */
#define KBD_MAX_WAIT    500000
#define SERIAL_IN_WAIT  100                             /* serial in time */
#define SERIAL_OUT_WAIT 100                             /* serial output */
#define NOQUEUE_WAIT    10000                           /* min check time */
#define KBD_LIM_WAIT(x) (((x) > KBD_MAX_WAIT)? KBD_MAX_WAIT: (x))
#define KBD_WAIT(w,s)   ((w)? w: KBD_LIM_WAIT (s))

/* Convert switch letter to bit mask */

#define SWMASK(x) (1u << (((int) (x)) - ((int) 'A')))

/* String match */

#define MATCH_CMD(ptr,cmd) strncmp ((ptr), (cmd), strlen (ptr))

/* Utility classes and functions */
#include "sim_util.h"

/* Threading primitives */
#include "sim_threads.h"

/* 
 * try/catch: define USE_C_TRY_CATCH to use try/catch constructs based on setjmp/longjmp
 * rather than native C/C++ try-catch. On some systems this may yield better performance,
 * however we do not observe this with MSVC/Windows and Linux/GCC/GLIBC.
 */
// #define USE_C_TRY_CATCH
#include "sim_try.h"

/* Threading primitives, part 2 */
#include "sim_threads2.h"

/* Break-to-debugger helper */

#if defined(_WIN32) && defined(__x86_32__)
#  define sim_DebugBreak() do { __asm {int 3} ; } while (0)
#elif defined(__GNUC__) && (defined(__x86_32__) || defined(__x86_64__))
#  define sim_DebugBreak() do { __asm__("int3"); } while (0)
#else
extern void sim_DebugBreak();
#endif

/* Helpers for execution/CPU context */

extern CPU_UNIT cpu_unit_0;

class run_scope_context
{
public:
    CPU_UNIT* cpu_unit;              /* currently selected VCPU context */
    sim_thread_type_t thread_type;   /* thread type: CPU, IOP, CONSOLE */
    smp_thread_t thread_handle;      /* thread handle or SMP_THREAD_NULL */
    t_bool thread_handle_valid;      /* true if thread_handle is valid */
    uint32 thread_cpu_id;            /* for CPU threads only: thread's original context CPU ID */
    int vm_critical_locks;           /* count of VM critical locks held by this thread */
    int os_hi_critical_locks;        /* count of OS_HI critical locks held by this thread */

    /* the following cells are used only for CONSOLE and IOP threads */
    sim_thread_priority_t bprio;     /* thread base priority */
    sim_thread_priority_t cprio;     /* thread current priority */
    t_bool reevaluating_prio;        /* inside sim_reevaluate_noncpu_thread_priority */

    run_scope_context()
    {
        cpu_unit = NULL;
        thread_type = SIM_THREAD_TYPE_CONSOLE;
        thread_handle = SMP_THREAD_NULL;
        thread_handle_valid = FALSE;
        thread_cpu_id = (uint32) -1;
        vm_critical_locks = 0;
        os_hi_critical_locks = 0;
        bprio = SIMH_THREAD_PRIORITY_INVALID;
        cprio = SIMH_THREAD_PRIORITY_INVALID;
        reevaluating_prio = FALSE;
    }

    run_scope_context(CPU_UNIT* cpu_unit)
    {
        this->cpu_unit = cpu_unit;
        this->thread_type = SIM_THREAD_TYPE_CONSOLE;
        thread_handle = SMP_THREAD_NULL;
        thread_handle_valid = FALSE;
        thread_cpu_id = (uint32) -1;
        vm_critical_locks = 0;
        os_hi_critical_locks = 0;
        bprio = SIMH_THREAD_PRIORITY_INVALID;
        cprio = SIMH_THREAD_PRIORITY_INVALID;
        reevaluating_prio = FALSE;
    }

    run_scope_context(CPU_UNIT* cpu_unit, sim_thread_type_t thread_type)
    {
        this->cpu_unit = cpu_unit;
        this->thread_type = thread_type;
        thread_handle = SMP_THREAD_NULL;
        thread_handle_valid = FALSE;
        thread_cpu_id = (uint32) -1;
        vm_critical_locks = 0;
        os_hi_critical_locks = 0;
        bprio = SIMH_THREAD_PRIORITY_INVALID;
        cprio = SIMH_THREAD_PRIORITY_INVALID;
        reevaluating_prio = FALSE;
    }

    run_scope_context(CPU_UNIT* cpu_unit, sim_thread_type_t thread_type, smp_thread_t thread_handle)
    {
        this->cpu_unit = cpu_unit;
        this->thread_type = thread_type;
        this->thread_handle = thread_handle;
        thread_handle_valid = TRUE;
        thread_cpu_id = (uint32) -1;
        vm_critical_locks = 0;
        os_hi_critical_locks = 0;
        bprio = SIMH_THREAD_PRIORITY_INVALID;
        cprio = SIMH_THREAD_PRIORITY_INVALID;
        reevaluating_prio = FALSE;
    }

    void set_current();
    void set_thread_type(sim_thread_type_t thread_type) { this->thread_type = thread_type; }
    static void set_current(run_scope_context* rscx);
    static run_scope_context* get_current();

    void setting_priority(sim_thread_priority_t prio)
    {
        if (this != NULL && 
            (thread_type == SIM_THREAD_TYPE_CONSOLE || thread_type == SIM_THREAD_TYPE_IOP) &&
            !reevaluating_prio)
        {
            cprio = bprio = prio;
        }
    }
};

#define RUN_SCOPE                                                      \
    CPU_UNIT* cpu_unit = NULL;                                         \
    do                                                                 \
    {                                                                  \
        run_scope_context* rscx = run_scope_context::get_current();    \
        cpu_unit = rscx ? rscx->cpu_unit : NULL;                       \
    }                                                                  \
    while (0);
#define RUN_SCOPE_RSCX                                                 \
    run_scope_context* rscx = run_scope_context::get_current();        \
    CPU_UNIT* cpu_unit = rscx ? rscx->cpu_unit : NULL;
#define RUN_SCOPE_RSCX_ONLY                                            \
    run_scope_context* rscx = run_scope_context::get_current();

#define RUN_SVC_DECL   RUN_DECL

#define sim_ncpus (cpu_dev.numunits)     /* number of processors configured in the system */

class sim_reg;

/* Device data structure */

class sim_device {
public:
    const char          *name;                          /* name */
    sim_unit            **units;                        /* units (array of pointers to UNIT, size numunits) */
    sim_reg             *registers;                     /* registers */
    struct sim_mtab     *modifiers;                     /* modifiers */
    SIM_ALIGN_32 uint32 numunits;                       /* #units */
    uint32              aradix;                         /* address radix */
    uint32              awidth;                         /* address width */
    uint32              aincr;                          /* addr increment */
    uint32              dradix;                         /* data radix */
    uint32              dwidth;                         /* data width */
    t_stat              (*examine)(t_value *v, t_addr a, sim_unit *up,
                            int32 sw);                  /* examine routine */
    t_stat              (*deposit)(t_value v, t_addr a, sim_unit *up,
                            int32 sw);                  /* deposit routine */
    t_stat              (*reset)(sim_device *dp);/* reset routine */
    t_stat              (*boot)(int32 u, sim_device *dp);
                                                        /* boot routine */
    t_stat              (*attach)(sim_unit *up, char *cp);
                                                        /* attach routine */
    t_stat              (*detach)(sim_unit *up); /* detach routine */
    void                *ctxt;                          /* context */
    uint32              flags;                          /* flags */
    uint32              dctrl;                          /* debug control */
    struct sim_debtab   *debflags;                      /* debug flags */
    t_stat              (*msize)(sim_unit *up, int32 v, char *cp, void *dp);
                                                        /* mem size routine */
    char                *lname;                         /* logical name */
    uint32              a_reset_count;                  /* number of resets on this device (used by ASYNCH_IO) */
};
typedef sim_device DEVICE;

extern DEVICE *sim_devices[];

/* Device flags */

#define DEV_V_DIS       0                               /* dev disabled */
#define DEV_V_DISABLE   1                               /* dev disable-able */
#define DEV_V_DYNM      2                               /* mem size dynamic */
#define DEV_V_NET       3                               /* network attach */
#define DEV_V_DEBUG     4                               /* debug capability */
#define DEV_V_RAW       5                               /* raw supported */
#define DEV_V_RAWONLY   6                               /* only raw supported */
#define DEV_V_PERCPU    7                               /* device and its units are per-CPU */
#define DEV_V_UF_31     12                              /* user flags, V3.1 */
#define DEV_V_UF        16                              /* user flags */
#define DEV_V_RSV       31                              /* reserved */

#define DEV_DIS         (1 << DEV_V_DIS)
#define DEV_DISABLE     (1 << DEV_V_DISABLE)
#define DEV_DYNM        (1 << DEV_V_DYNM)
#define DEV_NET         (1 << DEV_V_NET)
#define DEV_DEBUG       (1 << DEV_V_DEBUG)
#define DEV_RAW         (1 << DEV_V_RAW)
#define DEV_RAWONLY     (1 << DEV_V_RAWONLY)
#define DEV_PERCPU      (1 << DEV_V_PERCPU)

#define DEV_UFMASK_31   (((1u << DEV_V_RSV) - 1) & ~((1u << DEV_V_UF_31) - 1))
#define DEV_UFMASK      (((1u << DEV_V_RSV) - 1) & ~((1u << DEV_V_UF) - 1))
#define DEV_RFLAGS      (DEV_UFMASK|DEV_DIS)            /* restored flags */

/* Unit data structure

   Parts of the unit structure are device specific, that is, they are
   not referenced by the simulator control package and can be freely
   used by device simulators.  Fields starting with 'buf', and flags
   starting with 'UF', are device specific.  The definitions given here
   are for a typical sequential device.
*/

/* Unit flags */ 
#define UNIT_V_UF_31    12                              /* dev spec, V3.1 */
#define UNIT_V_UF       16                              /* device specific */
#define UNIT_V_RSV      31                              /* reserved!! */

#define UNIT_ATTABLE    000001                          /* attachable */
#define UNIT_RO         000002                          /* read only */
#define UNIT_FIX        000004                          /* fixed capacity */
#define UNIT_SEQ        000010                          /* sequential */
#define UNIT_ATT        000020                          /* attached */
#define UNIT_BINK       000040                          /* K = power of 2 */
#define UNIT_BUFABLE    000100                          /* bufferable */
#define UNIT_MUSTBUF    000200                          /* must buffer */
#define UNIT_BUF        000400                          /* buffered */
#define UNIT_ROABLE     001000                          /* read only ok */
#define UNIT_DISABLE    002000                          /* disable-able */
#define UNIT_DIS        004000                          /* disabled */
#define UNIT_RAW        010000                          /* raw mode */
#define UNIT_TEXT       020000                          /* text mode */
#define UNIT_IDLE       040000                          /* idle eligible */
#define UNIT_ISCPU     0100000                          /* is CPU */

#define UNIT_UFMASK_31  (((1u << UNIT_V_RSV) - 1) & ~((1u << UNIT_V_UF_31) - 1))
#define UNIT_UFMASK     (((1u << UNIT_V_RSV) - 1) & ~((1u << UNIT_V_UF) - 1))
#define UNIT_RFLAGS     (UNIT_UFMASK|UNIT_DIS)          /* restored flags */

/*
 * You can subclass sim_unit (as we do for CPU_UNIT), however keep in mind that C++ is not Java.
 * In C++ a pointer to a derived class and a pointer to a base class, for the same object, are not guaranteed
 * to hold the same address in memory. Often they do, but sometimes they don't, for example when using 
 * multiple inheritance or virtual inheritance or when Base class does not have virtual functions and vtable,
 * but Derived does. In these cases
 *
 *      (void*) (Base*) p != (void*) (Derived*) p
 *
 * and this can land you in trouble with SIMH code (which is originally a C code and does not expect C++ tricks),
 * so you want to avoid it.
 *
 * Avoid using multiple inheritance or virtual inheritance. Also if you declare a virtual function in a
 * subclass of sim_unit, make sure sim_unit declares some virtual function too, so both sim_unit and all its
 * subclasses do have vtable and thus for all classes involved fields are uniformly offset from the start of the object
 * by the size of vtable pointer.
 *
 * Finally, you can include consistency check in the initialization code similar to
 *
 *      if ((void*) (CPU_UNIT*) &cpu_unit_0 != (void*) (UNIT*) &cpu_unit_0)
 *          panic("Broken assumption: CPU_UNIT casts to UNIT with address change");
 *
 * You have been warned.
 *
 */
class sim_unit
{
public:
    t_stat              (*action)(RUN_SVC_DECL, sim_unit *up);  /* action routine */
    char                *filename;                              /* open file name */
    SMP_FILE            *fileref;                               /* file reference */
    void                *filebuf;                               /* memory buffer */
    uint32              hwmark;                                 /* high water mark */
    uint32              flags;                                  /* flags */
    t_addr              capac;                                  /* capacity */
    t_addr              pos;                                    /* file position */
    int32               buf;                                    /* buffer */
    int32               wait;                                   /* wait */
    int32               u3;                                     /* device specific */
    int32               u4;                                     /* device specific */
    int32               u5;                                     /* device specific */
    int32               u6;                                     /* device specific */
    void                (*io_flush)(sim_unit* up);              /* io flush routine */
    uint32              io_starttime;                           /* async I/O start time */
    uint32              io_startcpu;                            /* async I/O start cpu id */
    void                *up7;                                   /* device specific */
    void                *up8;                                   /* device specific */
    int32               unitno;                                 /* unit number */
    DEVICE*             device;                                 /* backpointer to DEVICE */
    CPU_UNIT*           clock_queue_cpu;                        /* pointer to CPU that has this unit in its clock queue,
                                                                   can be accessed for read or write only when holding device lock,
                                                                   clock_queue_cpu is not used per-CPU devices */
    smp_lock*           lock;                                   /* lock used to lock the unit (incl. for clock queue operations) or NULL */

    sim_unit*           a_next;                                 /* next asynch active */
    void                (*a_check_completion)(sim_unit*);
    t_stat              (*a_activate_call)(sim_unit*, int32);
    int32               a_sim_interval;

    sim_unit(t_stat (*action)(RUN_SVC_DECL, sim_unit *up), uint32 flags, t_addr capacity)
    {
        init_unit_object(action, flags, capacity);
    }

    sim_unit(t_stat (*action)(RUN_SVC_DECL, sim_unit *up), uint32 flags, t_addr capacity, int32 wait)
    {
        init_unit_object(action, flags, capacity);
        this->wait = wait;
    }

    t_bool is_cpu()
    {
        return (flags & UNIT_ISCPU) != 0;
    }

    void init_unit_object(t_stat (*action)(RUN_SVC_DECL, sim_unit *up), uint32 flags, t_addr capacity)
    {
        this->action = action;
        this->filename = NULL;
        this->fileref = NULL;
        this->filebuf = NULL;
        this->hwmark = 0;
        this->flags = flags;
        this->capac = capacity;
        this->pos = 0;
        this->buf = 0;
        this->wait = 0;
        this->u3 = 0;
        this->u4 = 0;
        this->u5 = 0;
        this->u6 = 0;
        this->io_flush = NULL;
        this->io_starttime = 0;
        this->up7 = 0;
        this->up8 = 0;
        this->unitno = 0;
        this->device = NULL;
        this->clock_queue_cpu = NULL;
        this->lock = NULL;
        this->a_check_completion = NULL;
        this->a_activate_call = NULL;
    }
};

#define sim_unit_index(uptr) ((uptr)->unitno)

// old: #define UDATA(act,fl,cap) UNIT(NULL,act,NULL,NULL,NULL,0,0,(fl),(cap),0,0)
#define UDATA(act,fl,cap) new UNIT((act), (fl), (cap))
#define UDATA_SINGLE(act,fl,cap) ((act), (fl), (cap))
#define UDATA_SINGLE_WAIT(act,fl,cap,wait) ((act), (fl), (cap), (wait))
#define UNIT_TABLE_SINGLE(udesc) UNIT* udesc##_table [] = {& udesc};

#define IS_PERCPU_UNIT(uptr)  ((uptr) == &sim_throt_unit || ((uptr)->device->flags & DEV_PERCPU))
extern UNIT sim_throt_unit;

/*
 * check if unit clock queue entry had been cancelled,
 * should be called by xx_svc after acquiring device lock
 */
#define RUN_SVC_CHECK_CANCELLED(uptr)                         \
    do {                                                      \
        if (! IS_PERCPU_UNIT(uptr))                           \
        {                                                     \
            if ((uptr)->clock_queue_cpu != cpu_unit)          \
                return SCPE_OK;                               \
            (uptr)->clock_queue_cpu = NULL;                   \
        }                                                     \
    } while (0)

/* Register data structure */

class sim_reg;
typedef sim_reg REG;

class sim_reg
{
public:
    char                *name;                          /* name */
    struct
    {
        char            loctype;                        /* location type */
        SIM_ALIGN_32 
        int32           loc_ix1;                        /* for REG_LOCTYPE_GBL_UNIT, loc_unit_index */
        t_addr          loc_a1;                         /* for REG_LOCTYPE_GBL_UNIT,  loc_unit_fielf_offset */
        t_value         (*get_rvalue)(REG* r, uint32 idx);                      /* routines for REG_LOCTYPE_DYN */
        void            (*set_rvalue)(REG* r, uint32 idx, t_value value);       /* ... */
        int             (*get_vcpu)(REG* r, uint32 idx);                        /* ... */
    } locinfo;
    SIM_ALIGN_32 volatile void*  loc;                   /* location */
    uint32              radix;                          /* radix */
    uint32              width;                          /* width */
    uint32              offset;                         /* starting bit */
    uint32              depth;                          /* save depth */
    uint32              flags;                          /* flags */
    uint32              qptr;                           /* circ q ptr (for loc == REG_LOCTYPE_GBL) */
    t_addr              qptr_offset;                    /* circ q ptr (for loc == REG_LOCTYPE_CPU) */

    void* getloc(RUN_DECL);
    void* getloc_unit_idx(RUN_DECL, uint32 idx);
    void setqptr(RUN_DECL, uint32 qptr);
    uint32 getqptr(RUN_DECL);
};

#define REG_FMT         00003                           /* see PV_x */
#define REG_RO          00004                           /* read only */
#define REG_HIDDEN      00010                           /* hidden */
#define REG_NZ          00020                           /* must be non-zero */
#define REG_UNIT        00040                           /* in unit struct */
#define REG_CIRC        00100                           /* circular array */
#define REG_VMIO        00200                           /* use VM data print/parse */
#define REG_VMAD        00400                           /* use VM addr print/parse */
#define REG_FIT         01000                           /* fit access to size */
#define REG_HRO         (REG_RO | REG_HIDDEN)           /* hidden, read only */

#define REG_LOCTYPE_DYN       'D'                       /* register is read and writtem via locinfo.get_rvalue and locinfo.set_rvalue */
#define REG_LOCTYPE_GBL       'G'                       /* register location: global, loc is direct address */
#define REG_LOCTYPE_CPU       'C'                       /* register location: per-cpu, loc is byte offset in CPU_UNIT.cpu_context */
#define REG_LOCTYPE_GBL_UNIT  'U'                       /* register location: located in UNIT descriptor itself,
                                                           loc is the address of UNIT* pointers array, i.e. of type UNIT**,
                                                           loc_unit_index is the index into the array,
                                                           loc_unit_fielf_offset is byte offset into UNIT structure */

/* Command tables, base and alternate formats */

struct sim_ctab
{
    const char          *name;                          /* name */
    t_stat              (*action)(int32 flag, char *cptr);
                                                        /* action routine */
    int32               arg;                            /* argument */
    const char          *help;                          /* help string */
};

struct sim_c1tab
{
    const char          *name;                          /* name */
    t_stat              (*action)(sim_device *dptr, sim_unit *uptr,
                            int32 flag, char *cptr);    /* action routine */
    int32               arg;                            /* argument */
    const char          *help;                          /* help string */
};

struct sim_shtab
{
    const char          *name;                          /* name */
    t_stat              (*action)(SMP_FILE *st, sim_device *dptr,
                            sim_unit *uptr, int32 flag, char *cptr);
    int32               arg;                            /* argument */
    const char          *help;                          /* help string */
};

/* Modifier table - only extended entries have disp, reg, or flags */

struct sim_mtab
{
    uint32              mask;                           /* mask */
    uint32              match;                          /* match */
    const char          *pstring;                       /* print string */
    const char          *mstring;                       /* match string */
    t_stat              (*valid)(sim_unit *up, int32 v, char *cp, void *dp);
                                                        /* validation routine */
    t_stat              (*disp)(SMP_FILE *st, sim_unit *up, int32 v, void *dp);
                                                        /* display routine */
    void                *desc;                          /* value descriptor */
                                                        /* REG * if MTAB_VAL */
                                                        /* int * if not */
};

#define MTAB_XTD        (1u << UNIT_V_RSV)              /* ext entry flag */
#define MTAB_VDV        001                             /* valid for dev */
#define MTAB_VUN        002                             /* valid for unit */
#define MTAB_VAL        004                             /* takes a value */
#define MTAB_NMO        010                             /* only if named */
#define MTAB_NC         020                             /* no UC conversion */
#define MTAB_SHP        040                             /* show takes parameter */

/* Search table */

struct sim_schtab
{
    int32               logic;                          /* logical operator */
    int32               boolop;                         /* boolean operator */
    t_value             mask;                           /* mask for logical */
    t_value             comp;                           /* comparison for boolean */
};

/* Breakpoint table */

struct sim_brktab
{
    t_addr              addr;                           /* address */
    int32               typ;                            /* mask of types */
    int32               cnt;                            /* proceed count */     
    char                *act;                           /* action string */
};

/* Debug table */

struct sim_debtab
{
    char                *name;                          /* control name */
    uint32              mask;                           /* control bit */
};

#define DEBUG_PRS(d)    (sim_deb && d.dctrl)
#define DEBUG_PRD(d)    (sim_deb && d->dctrl)
#define DEBUG_PRI(d,m)  (sim_deb && (d.dctrl & (m)))
#define DEBUG_PRJ(d,m)  (sim_deb && (d->dctrl & (m)))

t_value reg_irdata_dev_rd(REG* r, uint32 idx);
void reg_irdata_dev_wr(REG* r, uint32 idx, t_value value);
int reg_irdata_dev_vcpu(REG* r, uint32 idx);

t_value reg_irdata_lvl_rd(REG* r, uint32 idx);
void reg_irdata_lvl_wr(REG* r, uint32 idx, t_value value);
int reg_irdata_lvl_vcpu(REG* r, uint32 idx);
t_value ws_min_rd(REG* r, uint32 idx);
void ws_min_wr(REG* r, uint32 idx, t_value value);
t_value ws_max_rd(REG* r, uint32 idx);
void ws_max_wr(REG* r, uint32 idx, t_value value);
t_value ws_lock_rd(REG* r, uint32 idx);
void ws_lock_wr(REG* r, uint32 idx, t_value value);

/* The following macros define structure contents */

#define xxDATA_CPU_OFF(field)    ((void*) & ((CPU_UNIT*)0)->cpu_context.field)
#define xxDATA_CPU_ARROFF(field) ((void*) ((CPU_UNIT*)0)->cpu_context.field)
#define xxDATA_UNIT_OFF(field)   ((void*) & ((UNIT*)0)->field)
#define HRDATA_CPU(nm,loc,wd) nm, {REG_LOCTYPE_CPU}, xxDATA_CPU_OFF(loc), 16, (wd), 0, 1
// #define ORDATA_CPU(nm,loc,wd) nm, {REG_LOCTYPE_CPU}, xxDATA_CPU_OFF(loc), 8, (wd), 0, 1
#define DRDATA_CPU(nm,loc,wd) nm, {REG_LOCTYPE_CPU}, xxDATA_CPU_OFF(loc), 10, (wd), 0, 1
#define FLDATA_CPU(nm,loc,pos) nm, {REG_LOCTYPE_CPU}, xxDATA_CPU_OFF(loc), 2, 1, (pos), 1
// #define GRDATA_CPU(nm,loc,rdx,wd,pos) nm, {REG_LOCTYPE_CPU}, xxDATA_CPU_OFF(loc), (rdx), (wd), (pos), 1
#define BRDATA_CPU(nm,loc,rdx,wd,dep) nm, {REG_LOCTYPE_CPU}, xxDATA_CPU_ARROFF(loc), (rdx), (wd), 0, (dep)
#define BRDATA_CPU_QPTR(qptr_loc) 0, (t_addr) xxDATA_CPU_OFF(qptr_loc)
#define HRDATA_DYN(nm,wd,rd,wr,vcpu) nm, {REG_LOCTYPE_DYN, 0, 0, rd, wr, vcpu}, 0, 16, (wd), 0, 1

#if defined (__STDC__) || defined (_WIN32)
#define HRDATA_GBL(nm,loc,wd) #nm, {REG_LOCTYPE_GBL}, &(loc), 16, (wd), 0, 1
#define HRDATA_GBL_RDX(nm,loc,wd,rdx) #nm, {REG_LOCTYPE_GBL}, &(loc), (rdx), (wd), 0, 1
#define ORDATA_GBL(nm,loc,wd) #nm, {REG_LOCTYPE_GBL}, &(loc), 8, (wd), 0, 1
#define DRDATA_GBL(nm,loc,wd) #nm, {REG_LOCTYPE_GBL}, &(loc), 10, (wd), 0, 1
#define FLDATA_GBL(nm,loc,pos) #nm, {REG_LOCTYPE_GBL}, &(loc), 2, 1, (pos), 1
#define GRDATA_GBL(nm,loc,rdx,wd,pos) #nm, {REG_LOCTYPE_GBL}, &(loc), (rdx), (wd), (pos), 1
#define BRDATA_GBL(nm,loc,rdx,wd,dep) #nm, {REG_LOCTYPE_GBL}, (loc), (rdx), (wd), 0, (dep)
#define URDATA_GBL(nm,uarrloc,udx,ufn,rdx,wd,off,dep,fl) \
    #nm, {REG_LOCTYPE_GBL_UNIT, (udx), (t_addr) xxDATA_UNIT_OFF(ufn)}, (uarrloc), (rdx), (wd), (off), (dep), ((fl) | REG_UNIT)
#define IRDATA_DEV(nm, ivcl) #nm, {REG_LOCTYPE_DYN, 0, 0, reg_irdata_dev_rd, reg_irdata_dev_wr, reg_irdata_dev_vcpu}, (void*) (ivcl), 2, 1, 0, 1
#define IRDATA_LVL(nm, ipl, curr_cpu) #nm, {REG_LOCTYPE_DYN, 0, 0, reg_irdata_lvl_rd, reg_irdata_lvl_wr, reg_irdata_lvl_vcpu}, (void*) (((curr_cpu) << 5) | ipl), 16, 32, 0, 1
#else
#define HRDATA_GBL(nm,loc,wd) "nm", {REG_LOCTYPE_GBL}, &(loc), 16, (wd), 0, 1
#define HRDATA_GBL_RDX(nm,loc,wd,rdx) "nm", {REG_LOCTYPE_GBL}, &(loc), (rdx), (wd), 0, 1
#define ORDATA_GBL(nm,loc,wd) "nm", {REG_LOCTYPE_GBL}, &(loc), 8, (wd), 0, 1
#define DRDATA_GBL(nm,loc,wd) "nm", {REG_LOCTYPE_GBL}, &(loc), 10, (wd), 0, 1
#define FLDATA_GBL(nm,loc,pos) "nm", {REG_LOCTYPE_GBL}, &(loc), 2, 1, (pos), 1
#define GRDATA_GBL(nm,loc,rdx,wd,pos) "nm", {REG_LOCTYPE_GBL}, &(loc), (rdx), (wd), (pos), 1
#define BRDATA_GBL(nm,loc,rdx,wd,dep) "nm", {REG_LOCTYPE_GBL}, (loc), (rdx), (wd), 0, (dep)
#define URDATA_GBL(nm,uarrloc,udx,ufn,rdx,wd,off,dep,fl) \
    "nm", {REG_LOCTYPE_GBL_UNIT, (udx), (t_addr) xxDATA_UNIT_OFF(ufn)}, (uarrloc), (rdx), (wd), (off), (dep), ((fl) | REG_UNIT)
#define IRDATA_DEV(nm, ivcl) "nm", {REG_LOCTYPE_DYN, 0, 0, reg_irdata_dev_rd, reg_irdata_dev_wr, reg_irdata_dev_vcpu}, (void*) (ivcl), 2, 1, 0, 1
#define IRDATA_LVL(nm, ipl, curr_cpu) "nm", {REG_LOCTYPE_DYN, 0, 0, reg_irdata_lvl_rd, reg_irdata_lvl_wr, reg_irdata_lvl_vcpu}, (void*) (((curr_cpu) << 5) | ipl), 16, 32, 0, 1
#endif

/* Typedefs for principal structures */

typedef struct sim_ctab CTAB;
typedef struct sim_c1tab C1TAB;
typedef struct sim_shtab SHTAB;
typedef struct sim_mtab MTAB;
typedef struct sim_schtab SCHTAB;
typedef struct sim_brktab BRKTAB;
typedef struct sim_debtab DEBTAB;

/* Forward declaration */

class InstHistory;

/* Exception declarations */

enum
{
    sim_exception_typeid = 0,
    sim_exception_SimError_typeid = 1,
    sim_exception_ABORT_typeid = 2
};

class sim_exception
{
private: 
    t_bool autoDelete;
    uint32 typemask;

public:
    sim_exception()
    {
        autoDelete = FALSE;
        typemask = (1 << sim_exception_typeid);
    }

    sim_exception(t_bool autoDelete)
    {
        this->autoDelete = autoDelete;
        typemask = (1 << sim_exception_typeid);
    }

    virtual ~sim_exception() 
    {
    }

    void checkAutoDelete()
    {
        if (autoDelete)  delete this;
    }

    t_bool isAutoDelete()
    {
        return autoDelete;
    }

    void setAutoDelete(t_bool autoDelete)
    {
        this->autoDelete = autoDelete;
    }

    void setType(int etypeid)
    {
        typemask |= (1 << etypeid);
    }

    t_bool isType(int etypeid)
    {
        return (typemask & (1 << etypeid)) != 0;
    }
};

class sim_exception_ABORT : public sim_exception
{
public:
    t_stat code;
    sim_exception_ABORT(t_stat code)
    {
        setType(sim_exception_ABORT_typeid);
        this->code = code;
    }

    sim_exception_ABORT(t_stat code, t_bool autoDelete) : sim_exception(autoDelete)
    {
        setType(sim_exception_ABORT_typeid);
        this->code = code;
    }
};

class sim_exception_SimError : public sim_exception
{
private:
    char* msg;
    static char* nomem_msg;
    sim_exception_SimError();
public:
    sim_exception_SimError(const char* msg) : sim_exception(TRUE)
    {
        setType(sim_exception_SimError_typeid);
        if ((this->msg = dupstr(msg)) == NULL)
            this->msg = nomem_msg;
    }

    ~sim_exception_SimError()
    {
        if (msg && msg != nomem_msg)  free(msg);
    }

    const char* get_message()
    {
        return msg ? msg : "Unknown error";
    }
};

/* clock queue entry */

class clock_queue_entry
{
public:
    clock_queue_entry*  next;          /* link to next entry in the active event queue or in free list */
    UNIT*               uptr;          /* unit waiting for time event */
    int32               time;          /* time out */
    int32               clk_cosched;   /* if 0, scheduled at 'time'
                                          if 1, coscheduled with next clock tick
                                          if 2, with clock tick after it, and so on */
};

typedef struct __tag_clock_queue_entry_info
{
    UNIT*               uptr;          /* unit waiting for time event */
    int32               time;          /* time out */
    int32               clk_cosched;   /* if 0, scheduled at 'time'
                                          if 1, coscheduled with next clock tick
                                          if 2, with clock tick after it, and so on */
}
clock_queue_entry_info;


/* maximum number of CPUs supported */
#define SIM_MAX_CPUS 32

#if defined (VM_VAX)
#include "vax_cpuctx.h"
#else
class CPU_CONTEXT
{
public:
};
#endif

/* timer declarations */

#include "sim_timer.h"

/* CPU states */
enum
{
    CPU_STATE_STANDBY = 0,
    CPU_STATE_RUNNABLE = 1,
    CPU_STATE_RUNNING = 2
};

typedef enum __SynclkPending
{
    SynclkNotPending = 0,
    SynclkPendingIE0 = 1,
    SynclkPendingIE1 = 2
}
SynclkPending;

/* CPU UNIT definition */

#define NO_CPU_ID ((uint32) -1)

class SIM_ALIGN_CACHELINE CPU_UNIT : public UNIT
{
public:
    uint8 cpu_id;         /* this CPU ID, ranges from 0 to (SIM_MAX_CPUS - 1) */
    uint8 cpu_state;      /* one of CPU_STATE_xxx states */

    /* current priority of the thread for this VCPU */
    sim_thread_priority_t              cpu_thread_priority;

    /* clock queue control */
    SIM_ALIGN_PTR  clock_queue_entry*  clock_queue;               /* active clock queue */
    SIM_ALIGN_PTR  clock_queue_entry*  clock_queue_freelist;      /* lookaside allocation list for clock queue entries */

    /* time bookkeeping */
    SIM_ALIGN_64 double                sim_time;                  /* per-CPU "global" time */
    SIM_ALIGN_32 uint32                sim_rtime;                 /* per-CPU "global" time with rollover */
    SIM_ALIGN_32 int32                 noqueue_time;              /* interval till next clock queue check if there is no entries in the queue */

    /* step control */
    SIM_ALIGN_32 uint32                sim_step;

    /* instruction counter towards sim_step */
    SIM_ALIGN_32 uint32                sim_instrs;

    /* CPU cycles accrued */
    atomic_uint32_var                  cpu_adv_cycles;

    /* CPU context */
    SIM_ALIGN_32   CPU_CONTEXT         cpu_context;

    /* records pending device interrupts */
    /*SIM_ALIGN_CACHELINE*/
    InterruptRegister                  cpu_intreg;

    /* cached exception object entry, to avoid allocating each every time */
    SIM_ALIGN_PTR  sim_exception_ABORT* cpu_exception_ABORT;

    /* CPU stop code */
    t_stat                             cpu_stop_code;

    /* flag to cause CPU shutdown */
    t_bool                             cpu_dostop;

    /* instruction stream history */
    SIM_ALIGN_PTR  InstHistory*        cpu_hst;  
    SIM_ALIGN_64   UINT64              cpu_hst_stamp;
    uint32                             cpu_hst_index;

    /* breakpoint package */
    t_bool                             sim_brk_pend[SIM_BKPT_N_SPC];
    SIM_ALIGN_T_ADDR t_addr            sim_brk_ploc[SIM_BKPT_N_SPC];
    SIM_ALIGN_PTR char*                sim_brk_act;

    /* clk_unit is active, used only if use_clock_thread is TRUE */
    t_bool                             clk_active;

    /* value of cpu_adv_cycles at last SYNCLK event, used only if use_clock_thread is TRUE */
    uint32                             cpu_last_synclk_cycles;

    /* value of cpu_adv_cycles at last clock slice tick, used only if use_clock_thread is TRUE */
    uint32                             cpu_last_tslice_tick_cycles;

    /* value of cpu_adv_cycles at last second tick, used only if use_clock_thread is TRUE */
    uint32                             cpu_last_second_tick_cycles;

    /* clocks calibration */
    int32                              cpu_rtc_ticks[SIM_NTIMERS];          /* ticks */
    int32                              cpu_rtc_hz[SIM_NTIMERS];             /* tick rate */
    uint32                             cpu_rtc_rtime[SIM_NTIMERS];          /* real time */
    uint32                             cpu_rtc_vtime[SIM_NTIMERS];          /* virtual time */
    uint32                             cpu_rtc_nxintv[SIM_NTIMERS];         /* next interval */
    int32                              cpu_rtc_based[SIM_NTIMERS];          /* base delay */
    int32                              cpu_rtc_currd[SIM_NTIMERS];          /* current delay */
    int32                              cpu_rtc_initd[SIM_NTIMERS];          /* initial delay */
    uint32                             cpu_rtc_elapsed[SIM_NTIMERS];        /* sec since init */
    uint32                             cpu_idle_sleep_us;                   /* usecs in voluntary sleep */
    uint32                             cpu_idle_sleep_cycles;               /* cycles in idle sleep */

    /* SSC delta timers */
    sim_delta_timer*                   cpu_ssc_delta_timer[2];

    /* mask of active SSC clocks (accounts only for clocks sampled real time, not ROM tests) */
    uint32                             sysd_active_mask;

    /* clock interrupt is being processed or pending */
    t_bool                             cpu_active_clk_interrupt;

    /* inter-processor interrupt is being processed or pending */
    t_bool                             cpu_active_ipi_interrupt;

    /* countdown timers for number of cycles or instructions to allow before raising next CLK interrupt,
       used only if SYNCLK clock strobe thread is used */
    uint32                             cpu_synclk_protect_os;
    uint32                             cpu_synclk_protect_dev;
    /* union of cpu_synclk_protect_os and cpu_synclk_protect_dev */
    t_bool                             cpu_synclk_protect;

    /* when set to SynclkPendingIE0, SYNCLK interrupt had been received, and per-tick activities such as
       execution of clock queue entries coscheduled with CLK and clock recalibraton should be performed
       when cpu_synclk_protect expires;
       when set to SynclkPendingIE1, in addition CLK should be raised when cpu_synclk_protect expires */
    SynclkPending                      cpu_synclk_pending;

    /* recursion control for cpu_reevaluate_thread_priority */
    t_bool                             cpu_inside_reevaluate_thread_priority;
    t_bool                             cpu_redo_reevaluate_thread_priority;

    /* CPU thread and "run" gate */
    smp_semaphore*                     cpu_run_gate;
    smp_thread_t                       cpu_thread;
    t_bool                             cpu_thread_created;

    /* cpu idle sleep / cpu_wakeup handshake */
    smp_interlocked_uint32_var         cpu_sleeping;
    smp_event*                         cpu_wakeup_event;

    /* TRUE if this secondary CPU wants syswide device events pending in its event queue to be transferred to the primary 
      (raised by the secondary when it is shutting down, cleared after the primary transfers events to its own queue) */
    t_bool                             cpu_requeue_syswide_pending;

    /* slave copy of syncw.cpu[].active, moved here to avoid frequent references to master copy that would cause
       interprocessor cache interference, replacing them by references to a private copy as much as possible;

       access to syncw.cpu[].active is protected by cpu_database_lock;
       access to syncw_active is unprotected and limited to owning VCPU and console thread (the latter only when VCPUs are paused);

       note that syncw_active "lags" behind the master copy: if other VCPU detects this CPU has IPI or CLK interrupts
       pending, but is not in SYS synchronization window yet, it will enter this CPU into SYS synchronization window
       and set SYNCW_SYS in syncw.cpu[].active, but not in syncw_active; SYNCW_SYS will be set in syncw_active some
       time later and only by this VCPU, when it tries to change its state; thus if SYNCW_SYS is set in syncw_active,
       VCPU is always guaranteed to be active in SYS window; however if it is not set, then syncw.cpu[].active
       must be consulted */
    uint32                              syncw_active;

    /* countdown counters for syncw position checking; unlike cpu_adv_cycles, syncw countdown is not (counter)advanced
     * by idle sleep, and counts (down) instructions, not cycles, i.e. complex instructions may change cpu_adv_cycles
     * by multiple units, but will change syncw countdown only by one unit
     */
    uint32                              syncw_countdown;
    uint32                              syncw_countdown_start;
    uint32                              syncw_countdown_sys;
    uint32                              syncw_countdown_ilk;

    /* when this VCPU waits in syncw on other VCPU, it waits for this event to be signalled */
    smp_event*                          syncw_wait_event;

    /* when this VCPU waits in syncw on other VCPU, the latter's id is recorded here, otherwise NO_CPU_ID */
    uint32                              syncw_wait_cpu_id;

    /* hack for detecting console ROM's CONTINUE sequence (MAPEN-REI) */
    uint32                              cpu_con_rei;
    t_bool                              cpu_con_rei_on;

    /* pad to ensure that per-CPU data does not interfere with shared data caching */
    t_byte                             pad[SMP_MAXCACHELINESIZE];

public:
    CPU_UNIT();

    static void* operator new(size_t size)    { return operator_new_aligned(size, SMP_MAXCACHELINESIZE); }
    static void  operator delete(void* p)     { operator_delete_aligned(p); }

    t_bool is_primary_cpu()
    {
        return (this == &cpu_unit_0);
    }

    t_bool is_secondary_cpu()
    {
        return (this != &cpu_unit_0);
    }

    t_bool is_running()
    {
        return cpu_state == CPU_STATE_RUNNING;
    }

    void init(uint8 cpu_id, uint8 cpu_state);

    void init_clock_queue();

    static CPU_UNIT* getBy(CPU_CONTEXT* ctxt);
};

inline CPU_UNIT* CPU_UNIT::getBy(CPU_CONTEXT* ctxt)
{
    t_addr_off ctxt_offset = (t_addr_off) & ((CPU_UNIT*)0)->cpu_context;
    return (CPU_UNIT*) ((t_byte*) ctxt - ctxt_offset);
}

#define CPU_CURRENT_CYCLES atomic_var(cpu_unit->cpu_adv_cycles)
#define XCPU_CURRENT_CYCLES atomic_var(xcpu->cpu_adv_cycles)
#define cpu_cycle() sim_interval--, CPU_CURRENT_CYCLES++

/* control VCPU thread priority if more than one VCPU is currently active and host is not a dedicated machine */
#define must_control_prio()  (sim_mp_active && !sim_host_dedicated)

static SIM_INLINE t_bool tmr_is_active(RUN_DECL)
{
    return 0 != cpu_unit->sysd_active_mask;
}

extern CPU_UNIT cpu_unit_0;
extern CPU_UNIT* cpu_units[SIM_MAX_CPUS];
extern UNIT* cpu_units_as_units[SIM_MAX_CPUS];
extern DEVICE cpu_dev;
extern int32 sim_units_percpu;                             /* number of per-CPU units in the system */
extern int32 sim_units_global;                             /* number of global units in the system */

#if SIM_MAX_CPUS <= 32
class cpu_set
{
private:
    uint32 mask;
public:
    cpu_set() { mask = 0; }
    void set(uint32 ix) { mask |= 1 << (ix & 0x1F); }
    void clear(uint32 ix) { mask &= ~(1 << (ix & 0x1F)); }
    void clear_all() { mask = 0; }
    t_bool is_set(uint32 ix) const { return 0 != (mask & (1 << (ix & 0x1F))); }
    t_bool is_clear(uint32 ix) const { return 0 == (mask & (1 << (ix & 0x1F))); }
    t_bool is_any_set() const { return mask != 0; }
    void op_or(const cpu_set* x) { mask |= x->mask; }
    uint32 count_set(uint32 ix1, uint32 ix2) const
    {
        uint32 count = 0;
        for (uint32 ix = ix1;  ix <= ix2;  ix++)
            if (is_set(ix))  count++;
        return count;
    }
};
#elif SIM_MAX_CPUS <= 64
/* could do via uint64 as well */
class cpu_set
{
private:
    uint32 mask[2];
public:
    cpu_set() { mask[0] = mask[1] = 0; }
    void set(uint32 ix) { mask[ix % 32] |= 1 << (ix & 0x1F); }
    void clear(uint32 ix) { mask[ix % 32] &= ~(1 << (ix & 0x1F)); }
    void clear_all() { mask[0] = mask[1] = 0; }
    t_bool is_set(uint32 ix) const { return 0 != (mask[ix % 32] & (1 << (ix & 0x1F))); }
    t_bool is_clear(uint32 ix) const { return 0 == (mask[ix % 32] & (1 << (ix & 0x1F))); }
    t_bool is_any_set() const { return (mask[0] | mask[1]) != 0; }
    void op_or(const cpu_set* x) { mask[0] |= x->mask[0];  mask[1] |= x->mask[1]; }
    uint32 count_set(uint32 ix1, uint32 ix2) const
    {
        uint32 count = 0;
        for (uint32 ix = ix1;  ix <= ix2;  ix++)
            if (is_set(ix))  count++;
        return count;
    }
};
#else
class cpu_set
{
private:
    uint32 mask[(SIM_MAX_CPUS + 31) / 32];
public:
    cpu_set() { memzero(mask); }
    void set(uint32 ix) { mask[ix >> 5] |= 1 << (ix & 0x1F); }
    void clear(uint32 ix); { mask[ix >> 5] &= ~(1 << (ix & 0x1F)); }
    void clear_all() { memzero(mask); }
    t_bool is_set(uint32 ix) const { 0 != (mask[ix >> 5] & (1 << (ix & 0x1F))); }
    t_bool is_clear(uint32 ix) const { 0 == (mask[ix >> 5] & (1 << (ix & 0x1F))); }
    t_bool is_any_set() const
    {
        for (uint32 k = 0; k < sizeof(mask) / sizeof(mask[0]); k++)
            if (mask[k])
                return TRUE;
        return FALSE;
    }
    void op_or(const cpu_set* x)
    {
        for (uint32 k = 0;  k < sizeof(mask) / sizeof(mask[0]);  k++)
            mask[k] |= x->mask[k];
    }
    uint32 count_set(uint32 ix1, uint32 ix2) const
    {
        uint32 count = 0;
        for (uint32 ix = ix1;  ix <= ix2;  ix++)
            if (is_set(ix))  count++;
        return count;
    }
};
#endif

inline void* sim_reg::getloc(RUN_DECL)
{
    switch (locinfo.loctype)
    {
    case REG_LOCTYPE_GBL:
        return (void*) loc;

    case REG_LOCTYPE_CPU:
        return ((char*) cpu_unit) + (t_addr) loc;

    case REG_LOCTYPE_GBL_UNIT:
        {
            UNIT** uap = (UNIT**) loc;
            UNIT* unit = uap[locinfo.loc_ix1];
            return ((char*) unit) + (t_addr) locinfo.loc_a1;
        }

    default:
        return NULL;
    }
}

inline void* sim_reg::getloc_unit_idx(RUN_DECL, uint32 idx)
{
    if (locinfo.loctype != REG_LOCTYPE_GBL_UNIT)
        panic("Invalid reference to register: register is not GBL_UNIT");

    UNIT** uap = (UNIT**) loc;
    UNIT* unit = uap[idx];
    return ((char*) unit) + (t_addr) locinfo.loc_a1;
}

inline void sim_reg::setqptr(RUN_DECL, uint32 qptr)
{
    switch (locinfo.loctype)
    {
    case REG_LOCTYPE_GBL:   this->qptr = qptr;
                            return;
    case REG_LOCTYPE_CPU:   * (uint32*) (((char*) cpu_unit) + qptr_offset) = qptr;
                            return;
    default:                return;
    }
}

inline uint32 sim_reg::getqptr(RUN_DECL)
{
    switch (locinfo.loctype)
    {
    case REG_LOCTYPE_GBL:   return qptr;
    case REG_LOCTYPE_CPU:   return * (uint32*) (((char*) cpu_unit) + qptr_offset);
    default:                return 0;
    }
}

#if defined(VM_VAX)
/* Function prototypes and inline code for physical memory interface */
#  include "vax_mmu.h"
#endif

class aio_context
{
public:
    DEVICE*               dptr;                /* device for unit (access to debug flags) */
    UNIT*                 uptr;                /* unit */
    uint32                dbit;                /* debugging bit */
    t_bool                asynch_io;           /* Asynchronous Interrupt scheduling enabled */
    smp_thread_t          io_thread;           /* I/O thread handle */
    t_bool                io_thread_created;   /* ... */
    smp_simple_semaphore* io_event;            /* sleep event for IOP worker thread */
    smp_event*            io_flush_ack;        /* signal "flushing completed" */
    t_bool                io_do_flush;         /* flag requesting IOP thread to perform flush */
    t_stat                io_status;           /* IO request comption status */
    uint32                io_reset_count;      /* copy of DEVICE.a_reset_count */

public:
    aio_context(UNIT* uptr);
    virtual ~aio_context();
    void asynch_init(smp_thread_routine_t start_routine, void* arg);
    void asynch_uninit();
    void thread_loop();
    virtual void perform_request() = 0;
    virtual t_bool has_request() = 0;
    void flush();
    virtual void perform_flush() = 0;
    void io_event_signal() { io_event->release(); }

private:
    aio_context();
};

/* Function prototypes */

#include "scp.h"
#include "sim_console.h"
#include "sim_fio.h"
void cpu_set_thread_priority(RUN_DECL, sim_thread_priority_t prio);
void cpu_set_thread_priority(RUN_RSCX_DECL, sim_thread_priority_t prio);
void* malloc_aligned(size_t size, size_t alignment);
void* calloc_aligned (size_t num, size_t elsize, size_t alignment);
void free_aligned(void* p);
const char* cpu_describe_state(CPU_UNIT* cpu_unit);
t_stat reset_cpu_and_its_devices(CPU_UNIT* cpu_unit);
t_stat reset_dev_thiscpu (DEVICE* dptr);
t_stat reset_dev_allcpus (DEVICE* dptr);
int sim_device_index (DEVICE* dptr);
t_bool cpu_may_sleep(RUN_DECL);
void wakeup_cpu(CPU_UNIT *xcpu);
void wakeup_cpus(RUN_DECL, uint32 old_idle_mask, uint32 new_idle_mask);
void cpu_begin_interlocked(RUN_DECL, volatile sim_thread_priority_t* sv_priority, volatile t_bool* sv_priority_stored);
void cpu_end_interlocked(RUN_DECL, sim_thread_priority_t sv_priority, t_bool sv_priority_stored);
void debug_out(const char* x);
void throw_sim_exception_ABORT(RUN_DECL, t_stat x);
t_bool cpu_stop_history ();
t_bool sim_brk_is_in_action ();
void perf_register_object(const char* name, smp_lock* object, t_bool copyname = FALSE);
void perf_unregister_object(smp_lock* object);
t_value reg_sirr_rd(REG* r, uint32 idx);
void reg_sirr_wr(REG* r, uint32 idx, t_value value);


/* initialization helper definitions */

class on_init_call
{
public:
    static on_init_call* head;

protected:
    on_init_call* next;
    void (*routine)();
    virtual void invoke()
    {
        (*routine)();
    }
    void insert()
    {
        this->next = head;
        head = this;
    }

public:
    on_init_call(void (*routine)())
    {
        this->routine = routine;
        insert();
    }

    static void invoke_all()
    {
        for (on_init_call* cp = head;  cp;  cp  = cp->next)
            cp->invoke();
    }
};

class on_init_smp_lock : public on_init_call
{
protected:
    smp_lock** ppcs;
    sim_lock_criticality_t criticality;
    uint32 cycles;
    const char* name;

    void invoke()
    {
        *ppcs = smp_lock::create(cycles);
        if (criticality != SIM_LOCK_CRITICALITY_NONE)
            (*ppcs)->set_criticality(criticality);
        perf_register_object(name, *ppcs);
    }

public:
    on_init_smp_lock(smp_lock** ppcs, sim_lock_criticality_t criticality, uint32 cycles, const char* name) : on_init_call(NULL)
    {
        this->ppcs = ppcs;
        this->criticality = criticality;
        this->cycles = cycles;
        this->name = name;
    }
};

class on_init_tls : public on_init_call
{
protected:
    smp_tls_key* pkey;

    void invoke()
    {
        if (! smp_tls_alloc(pkey))
            panic("Unable to initialize TLS variable");
    }

public:
    on_init_tls(smp_tls_key* pkey) : on_init_call(NULL)
    {
        this->pkey = pkey;
    }
};

#define AUTO_TLS(key)                                          \
    static smp_tls_key key;                                    \
    static on_init_tls __oninit_tls_##key(& key)

#define ON_INIT_INVOKE(routine)                                \
    static on_init_call __oninit_call_##routine(routine)

#define AUTO_INIT_LOCK(lockname, criticality, spinwait_cycles)              \
    static smp_lock* lockname = NULL;                                       \
    static on_init_smp_lock __oninit_smp_lock_##lockname(& lockname, criticality, spinwait_cycles, #lockname)

#define DEVLOCK_SPINWAIT_CYCLES  5000
#define AUTO_INIT_DEVLOCK(lockname)                            \
    AUTO_INIT_LOCK(lockname, SIM_LOCK_CRITICALITY_NONE, DEVLOCK_SPINWAIT_CYCLES)

/* auto-locking helpers */
class auto_lock : public sim_try_auto_destructable
{
    smp_lock* pcs;
    t_bool pcs_locked;

public:
    SIM_INLINE auto_lock(smp_lock* pcs, t_bool dolock = TRUE)
    {
        this->pcs = pcs;
        pcs_locked = FALSE;
        if (dolock)  lock();
        onConstructor();
    }
    SIM_INLINE ~auto_lock()
    {
        onDestroy(FALSE);
    }
    SIM_INLINE void onDestroy(t_bool unregistered)
    {
        if (onDestructor(unregistered))
            unlock();
    }
    SIM_INLINE void unlock()
    {
        if (pcs_locked)
        {
            pcs->unlock();
            pcs_locked = FALSE;
        }
    }
    SIM_INLINE void lock()
    {
        if (!pcs_locked)
        {
            pcs->lock();
            pcs_locked = TRUE;
        }
    }
};

#define AUTO_LOCK(lockname)  auto_lock auto_lock_##lockname(lockname)
#define AUTO_LOCK_NM(autoname, lockname)  auto_lock autoname(lockname)

#include "sim_syncw.h"

/* globals */
extern char sim_name[];
extern uint32 sim_idle_stable;
extern const uint32 sim_taddr_64;
extern atomic_int32 stop_cpus;
extern t_bool sim_asynch_enabled;
extern t_bool sim_brk_continue;
extern t_bool sim_vsmp_active;
extern t_bool sim_vsmp_idle_sleep;
extern t_bool sim_ws_prefaulted;
extern t_bool sim_ws_settings_changed;
extern t_bool sim_ws_lock;
extern uint32 sim_ws_min;
extern uint32 sim_ws_max;
extern uint32 sim_host_turbo;
extern t_bool sim_host_dedicated;
extern uint32 use_native_interlocked;

#endif
