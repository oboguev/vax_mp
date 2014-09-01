/* scp.c: simulator control program

   Copyright (c) 1993-2011, Robert M Supnik

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

   25-Sep-11    MP      Added the ability for a simulator built with 
                        SIM_ASYNCH_IO to change whether I/O is actually done
                        asynchronously by the new scp command SET ASYNCH and 
                        SET NOASYNCH
   22-Sep-11    MP      Added signal catching of SIGHUP and SIGTERM to cause 
                        simulator STOP.  This allows an externally signalled
                        event (i.e. system shutdown, or logoff) to signal a
                        running simulator of these events and to allow 
                        reasonable actions to be taken.  This will facilitate 
                        running a simulator as a 'service' on *nix platforms, 
                        given a sufficiently flexible simulator .ini file.  
   20-Apr-11    MP      Added expansion of %STATUS% and %TSTATUS% in do command
                        arguments.  STATUS is the numeric value of the last 
                        command error status and TSTATUS is the text message
                        relating to the last command error status
   17-Apr-11    MP      Changed sim_rest to defer attaching devices until after
                        device register contents have been restored since some
                        attach activities may reference register contained info.
   29-Jan-11    MP      Adjusted sim_debug to: 
                          - include the simulator timestamp (sim_gtime)
                            as part of the prefix for each line of output
                            SPO: not merged into VAX MP
                          - write complete lines at a time (avoid asynch I/O issues).
   05-Jan-11    MP      Added Asynch I/O support
   22-Jan-11    MP      Added SET ON, SET NOON, ON, GOTO and RETURN command support
                        (SPO: not merged from 3.8.2 to VAX MP)
   13-Jan-11    MP      Added "SHOW SHOW" and "SHOW <dev> SHOW" commands
   05-Jan-11    RMS     Fixed bug in deposit stride for numeric input (John Dundas)
   23-Dec-10    RMS     Clarified some help messages (Mark Pizzolato)
   08-Nov-10    RMS     Fixed handling of DO with no arguments (Dave Bryan)
   22-May-10    RMS     Added *nix READLINE support (Mark Pizzolato)
   08-Feb-09    RMS     Fixed warnings in help printouts
   29-Dec-08    RMS     Fixed implementation of MTAB_NC
   24-Nov-08    RMS     Revised RESTORE unit logic for consistency
   05-Sep-08    JDB     "detach_all" ignores error status returns if shutting down
   17-Aug-08    RMS     Revert RUN/BOOT to standard, rather than powerup, reset
   25-Jul-08    JDB     DO cmd missing params now default to null string
   29-Jun-08    JDB     DO cmd sub_args now allows "\\" to specify literal backslash
   31-Mar-08    RMS     Fixed bug in local/global register search (found by Mark Pizzolato)
                        Fixed bug in restore of RO units (from Mark Pizzolato)
   06-Feb-08    RMS     Added SET/SHO/NO BR with default argument
   18-Jul-07    RMS     Modified match_ext for VMS ext;version support
   28-Apr-07    RMS     Modified sim_instr invocation to call sim_rtcn_init_all
                        Fixed bug in get_sim_opt
                        Fixed bug in restoration with changed memory size
   08-Mar-07    JDB     Fixed breakpoint actions in DO command file processing
   30-Jan-07    RMS     Fixed bugs in get_ipaddr
   17-Oct-06    RMS     Added idle support
   04-Oct-06    JDB     DO cmd failure now echoes cmd unless -q
   14-Jul-06    RMS     Added sim_activate_abs
   02-Jun-06    JDB     Fixed do_cmd to exit nested files on assertion failure
                        Added -E switch to do_cmd to exit on any error
   14-Feb-06    RMS     Upgraded save file format to V3.5
   18-Jan-06    RMS     Added fprint_stopped_gen
                        Added breakpoint spaces
                        Fixed unaligned register access (found by Doug Carman)
   22-Sep-05    RMS     Fixed declarations (from Sterling Garwood)
   30-Aug-05    RMS     Revised to trim trailing spaces on file names
   25-Aug-05    RMS     Added variable default device support
   23-Aug-05    RMS     Added Linux line history support
   16-Aug-05    RMS     Fixed C++ declaration and cast problems
   01-May-05    RMS     Revised syntax for SET DEBUG (from Dave Bryan)
   22-Mar-05    JDB     Modified DO command to allow ten-level nesting
   18-Mar-05    RMS     Moved DETACH tests into detach_unit (from Dave Bryan)
                        Revised interface to fprint_sym, fparse_sym
   07-Feb-05    RMS     Added ASSERT command (from Dave Bryan)
   02-Feb-05    RMS     Fixed bug in global register search
   26-Dec-04    RMS     Qualified SAVE examine, RESTORE deposit with SIM_SW_REST
   10-Nov-04    JDB     Fixed logging of errors from cmds in "do" file
   05-Nov-04    RMS     Moved SET/SHOW DEBUG under CONSOLE hierarchy
                        Renamed unit OFFLINE/ONLINE to DISABLED/ENABLED (from Dave Bryan)
                        Revised to flush output files after simulation stop (from Dave Bryan)
   15-Oct-04    RMS     Fixed HELP to suppress duplicate descriptions
   27-Sep-04    RMS     Fixed comma-separation options in set (from David Bryan)
   09-Sep-04    RMS     Added -p option for RESET
   13-Aug-04    RMS     Qualified RESTORE detach with SIM_SW_REST
   17-Jul-04    RMS     Added ECHO command (from Dave Bryan)
   12-Jul-04    RMS     Fixed problem ATTACHing to read only files
                        (found by John Dundas)
   28-May-04    RMS     Added SET/SHOW CONSOLE
   14-Feb-04    RMS     Updated SAVE/RESTORE (V3.2)
                RMS     Added debug print routines (from Dave Hittner)
                RMS     Added sim_vm_parse_addr and sim_vm_fprint_addr
                RMS     Added REG_VMAD support
                RMS     Split out libraries
                RMS     Moved logging function to SCP
                RMS     Exposed step counter interface(s)
                RMS     Fixed double logging of SHOW BREAK (found by Mark Pizzolato)
                RMS     Fixed implementation of REG_VMIO
                RMS     Added SET/SHOW DEBUG, SET/SHOW <device> DEBUG,
                        SHOW <device> MODIFIERS, SHOW <device> RADIX
                RMS     Changed sim_fsize to take uptr argument
   29-Dec-03    RMS     Added Telnet console output stall support
   01-Nov-03    RMS     Cleaned up implicit detach on attach/restore
                        Fixed bug in command line read while logging (found by Mark Pizzolato)
   01-Sep-03    RMS     Fixed end-of-file problem in dep, idep
                        Fixed error on trailing spaces in dep, idep
   15-Jul-03    RMS     Removed unnecessary test in reset_all
   15-Jun-03    RMS     Added register flag REG_VMIO
   25-Apr-03    RMS     Added extended address support (V3.0)
                        Fixed bug in SAVE (found by Peter Schorn)
                        Added u5, u6 fields
                        Added logical name support
   03-Mar-03    RMS     Added sim_fsize
   27-Feb-03    RMS     Fixed bug in multiword deposits to files
   08-Feb-03    RMS     Changed sim_os_sleep to void, match_ext to char*
                        Added multiple actions, .ini file support
                        Added multiple switch evaluations per line
   07-Feb-03    RMS     Added VMS support for ! (from Mark Pizzolato)
   01-Feb-03    RMS     Added breakpoint table extension, actions
   14-Jan-03    RMS     Added missing function prototypes
   10-Jan-03    RMS     Added attach/restore flag, dynamic memory size support,
                        case sensitive SET options
   22-Dec-02    RMS     Added ! (OS command) feature (from Mark Pizzolato)
   17-Dec-02    RMS     Added get_ipaddr
   02-Dec-02    RMS     Added EValuate command
   16-Nov-02    RMS     Fixed bug in register name match algorithm
   13-Oct-02    RMS     Fixed Borland compiler warnings (found by Hans Pufal)
   05-Oct-02    RMS     Fixed bugs in set_logon, ssh_break (found by David Hittner)
                        Added support for fixed buffer devices
                        Added support for Telnet console, removed VT support
                        Added help <command>
                        Added VMS file optimizations (from Robert Alan Byer)
                        Added quiet mode, DO with parameters, GUI interface,
                           extensible commands (from Brian Knittel)
                        Added device enable/disable commands
   14-Jul-02    RMS     Fixed exit bug in do, added -v switch (from Brian Knittel)
   17-May-02    RMS     Fixed bug in fxread/fxwrite error usage (found by
                        Norm Lastovic)
   02-May-02    RMS     Added VT emulation interface, changed {NO}LOG to SET {NO}LOG
   22-Apr-02    RMS     Fixed laptop sleep problem in clock calibration, added
                        magtape record length error (found by Jonathan Engdahl)
   26-Feb-02    RMS     Fixed initialization bugs in do_cmd, get_aval
                        (found by Brian Knittel)
   10-Feb-02    RMS     Fixed problem in clock calibration
   06-Jan-02    RMS     Moved device enable/disable to simulators
   30-Dec-01    RMS     Generalized timer packaged, added circular arrays
   19-Dec-01    RMS     Fixed DO command bug (found by John Dundas)
   07-Dec-01    RMS     Implemented breakpoint package
   05-Dec-01    RMS     Fixed bug in universal register logic
   03-Dec-01    RMS     Added read-only units, extended SET/SHOW, universal registers
   24-Nov-01    RMS     Added unit-based registers
   16-Nov-01    RMS     Added DO command
   28-Oct-01    RMS     Added relative range addressing
   08-Oct-01    RMS     Added SHOW VERSION
   30-Sep-01    RMS     Relaxed attach test in BOOT
   27-Sep-01    RMS     Added queue count routine, fixed typo in ex/mod
   17-Sep-01    RMS     Removed multiple console support
   07-Sep-01    RMS     Removed conditional externs on function prototypes
                        Added special modifier print
   31-Aug-01    RMS     Changed int64 to t_int64 for Windoze (V2.7)
   18-Jul-01    RMS     Minor changes for Macintosh port
   12-Jun-01    RMS     Fixed bug in big-endian I/O (found by Dave Conroy)
   27-May-01    RMS     Added multiple console support
   16-May-01    RMS     Added logging
   15-May-01    RMS     Added features from Tim Litt
   12-May-01    RMS     Fixed missing return in disable_cmd
   25-Mar-01    RMS     Added ENABLE/DISABLE
   14-Mar-01    RMS     Revised LOAD/DUMP interface (again)
   05-Mar-01    RMS     Added clock calibration support
   05-Feb-01    RMS     Fixed bug, DETACH buffered unit with hwmark = 0
   04-Feb-01    RMS     Fixed bug, RESTORE not using device's attach routine
   21-Jan-01    RMS     Added relative time
   22-Dec-00    RMS     Fixed find_device for devices ending in numbers
   08-Dec-00    RMS     V2.5a changes
   30-Oct-00    RMS     Added output file option to examine
   11-Jul-99    RMS     V2.5 changes
   13-Apr-99    RMS     Fixed handling of 32b addresses
   04-Oct-98    RMS     V2.4 changes
   20-Aug-98    RMS     Added radix commands
   05-Jun-98    RMS     Fixed bug in ^D handling for UNIX
   10-Apr-98    RMS     Added switches to all commands
   26-Oct-97    RMS     Added search capability
   25-Jan-97    RMS     Revised data types
   23-Jan-97    RMS     Added bi-endian I/O
   06-Sep-96    RMS     Fixed bug in variable length IEXAMINE
   16-Jun-96    RMS     Changed interface to parse/print_sym
   06-Apr-96    RMS     Added error checking in reset all
   07-Jan-96    RMS     Added register buffers in save/restore
   11-Dec-95    RMS     Fixed ordering bug in save/restore
   22-May-95    RMS     Added symbolic input
   13-Apr-95    RMS     Added symbolic printouts
*/

/* Macros and data structures */

#include "sim_defs.h"
#include "sim_rev.h"
#include <signal.h>
#include <ctype.h>

#if defined(__linux) || defined(_WIN32)
#  include <malloc.h>
#endif

#if defined(__linux) || defined(__APPLE__)
#  include <unistd.h>
#endif

#include <time.h>
#include <sys/stat.h>

#if defined(HAVE_DLOPEN)                                /* Dynamic Readline support */
#include <dlfcn.h>
#endif

#define EX_D            0                               /* deposit */
#define EX_E            1                               /* examine */
#define EX_I            2                               /* interactive */
#define SCH_OR          0                               /* search logicals */
#define SCH_AND         1
#define SCH_XOR         2
#define SCH_E           0                               /* search booleans */
#define SCH_N           1
#define SCH_G           2
#define SCH_L           3
#define SCH_EE          4
#define SCH_NE          5
#define SCH_GE          6
#define SCH_LE          7
#define SSH_ST          0                               /* set */
#define SSH_SH          1                               /* show */
#define SSH_CL          2                               /* clear */

#define MAX_DO_NEST_LVL 10                              /* DO cmd nesting level */
#define SRBSIZ          1024                            /* save/restore buffer */
#define SIM_BRK_INILNT  4096                            /* bpt tbl length */
#define SIM_BRK_ALLTYP  0xFFFFFFFF

/*
 * CLOCKS & TIME KEEPING LEGEND
 * ============================
 *
 * Essential elements are as follows.
 *
 * sim_interval  - interval countdown counter till next clock queue check
 *
 *     -  per-CPU
 *     -  decremented by 1 by regular instructions
 *     -  decremented multiple times by complex instructions (MOVC3/5, SCANC etc.)
 *     -  decremented by idle sleep by (sleep_time * instructions_per_second)
 *
 * noqueue_time - time till next queue check if there are entries in the queue
 *
 *     -  per-CPU
 *     -  used when clock_queue == NULL,
 *     -  otherwise use clock_queue->time
 *     -  initialized to NOQUEUE_WAIT (10000), decreased by UPDATE_SIM_TIME
 *
 * sim_time, sim_rtime  - time on this virtual processor
 *
 *     -  per-CPU
 *     -  updated by UPDATE_SIM_TIME
 *     -  read via sim_gtime, sim_grtime
 *     -  sim_gtime is used for synchronicity window management (see below)
 *     -  sim_grtime used by SYSD clock
 *
 *     -  there is no notion of single "global time" shared by all processors, each processor has its own private time view,
 *        just like in a real hardware multiprocessor system CPU clocks can drift (because of hardware drift and because
 *        some clock tick interrupts can be blocked and skipped when handling higher-priority interrupts),
 *        however VAX MP processors keep within a certain synchronicity window with respect to each other's time;
 *        if some processor gets ahead of the synchronicity window, it stalls its own execution till other processors
 *        catch up and all processors fall back into allowed synchronicity window limits.
 *
 * SYSD clocks - per-CPU theoretically, but access by non-primary processors is disallowed
 *
 *     -  used only by firmware, SYSBOOT and INIT and final phase of shutdown
 *
 * UPDATE_SIM_TIME is called as
 *
 *      UPDATE_SIM_TIME (cpu_unit->clock_queue->time);
 *
 * or
 *      UPDATE_SIM_TIME (cpu_unit->noqueue_time);
 *
 * after some time passes and sim_interval decreases from its original value
 * of cpu_unit->clock_queue->time or cpu_unit->noqueue_time.
 *
 * This macro can *only* be executed either on a local processor or from the console thread
 * while the processor is paused.
 *
 */

#define UPDATE_SIM_TIME(x)                                      \
    do {                                                        \
        cpu_unit->sim_time += (x) - sim_interval;               \
        cpu_unit->sim_rtime += (uint32) ((x) - sim_interval);   \
        (x) = sim_interval;                                     \
    } while (0)

#define UPDATE_CPU_SIM_TIME()                                   \
    if (cpu_unit->clock_queue != NULL)                          \
    {                                                           \
        UPDATE_SIM_TIME (cpu_unit->clock_queue->time);          \
    }                                                           \
    else                                                        \
    {                                                           \
        UPDATE_SIM_TIME (cpu_unit->noqueue_time);               \
    }

#define SZ_D(dp) (size_map[((dp)->dwidth + CHAR_BIT - 1) / CHAR_BIT])
#define SZ_R(rp) \
    (size_map[((rp)->width + (rp)->offset + CHAR_BIT - 1) / CHAR_BIT])
#if defined (USE_INT64)
#define SZ_LOAD(sz,v,mb,j) \
    if (sz == sizeof (uint8)) v = *(((uint8 *) mb) + ((uint32) j)); \
    else if (sz == sizeof (uint16)) v = *(((uint16 *) mb) + ((uint32) j)); \
    else if (sz == sizeof (uint32)) v = *(((uint32 *) mb) + ((uint32) j)); \
    else v = *(((t_uint64 *) mb) + ((uint32) j));
#define SZ_STORE(sz,v,mb,j) \
    if (sz == sizeof (uint8)) *(((uint8 *) mb) + j) = (uint8) v; \
    else if (sz == sizeof (uint16)) *(((uint16 *) mb) + ((uint32) j)) = (uint16) v; \
    else if (sz == sizeof (uint32)) *(((uint32 *) mb) + ((uint32) j)) = (uint32) v; \
    else *(((t_uint64 *) mb) + ((uint32) j)) = v;
#else
#define SZ_LOAD(sz,v,mb,j) \
    if (sz == sizeof (uint8)) v = *(((uint8 *) mb) + ((uint32) j)); \
    else if (sz == sizeof (uint16)) v = *(((uint16 *) mb) + ((uint32) j)); \
    else v = *(((uint32 *) mb) + ((uint32) j));
#define SZ_STORE(sz,v,mb,j) \
    if (sz == sizeof (uint8)) *(((uint8 *) mb) + ((uint32) j)) = (uint8) v; \
    else if (sz == sizeof (uint16)) *(((uint16 *) mb) + ((uint32) j)) = (uint16) v; \
    else *(((uint32 *) mb) + ((uint32) j)) = v;
#endif
#define GET_SWITCHES(cp) \
    if ((cp = get_sim_sw (cp)) == NULL) return SCPE_INVSW
#define GET_RADIX(val,dft) \
    if (sim_switches & SWMASK ('O')) val = 8; \
    else if (sim_switches & SWMASK ('D')) val = 10; \
    else if (sim_switches & SWMASK ('H')) val = 16; \
    else val = dft;

/* VM interface */

extern REG *sim_PC;
extern const char *sim_stop_messages[];
extern t_stat sim_load (RUN_DECL, SMP_FILE *ptr, char *cptr, char *fnam, int32 flag);
extern int32 sim_emax;
extern t_stat fprint_sym (SMP_FILE *ofile, t_addr addr, t_value *val,
    UNIT *uptr, int32 sw);
extern t_stat parse_sym (char *cptr, t_addr addr, UNIT *uptr, t_value *val,
    int32 sw);

/* The per-simulator init routine is a weak global that defaults to NULL
   The other per-simulator pointers can be overrriden by the init routine */

void (*sim_vm_init) (void);
char* (*sim_vm_read) (char *ptr, int32 size, SMP_FILE *stream) = NULL;
void (*sim_vm_post) (t_bool from_scp) = NULL;
CTAB *sim_vm_cmd = NULL;
void (*sim_vm_fprint_addr) (SMP_FILE *st, DEVICE *dptr, t_addr addr) = NULL;
t_addr (*sim_vm_parse_addr) (DEVICE *dptr, char *cptr, char **tptr) = NULL;

/* Prototypes */

/* Set and show command processors */

t_stat set_dev_radix (DEVICE *dptr, UNIT *uptr, int32 flag, char *cptr);
t_stat set_dev_enbdis (DEVICE *dptr, UNIT *uptr, int32 flag, char *cptr);
t_stat set_dev_debug (DEVICE *dptr, UNIT *uptr, int32 flag, char *cptr);
t_stat set_unit_enbdis (DEVICE *dptr, UNIT *uptr, int32 flag, char *cptr);
t_stat ssh_break (SMP_FILE *st, char *cptr, int32 flg);
t_stat show_cmd_fi (SMP_FILE *ofile, int32 flag, char *cptr);
t_stat show_config (SMP_FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, char *cptr);
t_stat show_queue (SMP_FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, char *cptr);
t_stat show_time (SMP_FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, char *cptr);
t_stat show_mod_names (SMP_FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, char *cptr);
t_stat show_show_commands (SMP_FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, char *cptr);
t_stat show_log_names (SMP_FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, char *cptr);
t_stat show_dev_radix (SMP_FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, char *cptr);
t_stat show_dev_debug (SMP_FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, char *cptr);
t_stat show_dev_logicals (SMP_FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, char *cptr);
t_stat show_dev_modifiers (SMP_FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, char *cptr);
t_stat show_dev_show_commands (SMP_FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, char *cptr);
t_stat show_version (SMP_FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, char *cptr);
t_stat show_break (SMP_FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, char *cptr);
t_stat show_on (SMP_FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, char *cptr);
t_stat show_device (SMP_FILE *st, DEVICE *dptr, int32 flag);
t_stat show_unit (SMP_FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag);
t_stat show_all_mods (SMP_FILE *st, DEVICE *dptr, UNIT *uptr, int32 flg);
t_stat show_one_mod (SMP_FILE *st, DEVICE *dptr, UNIT *uptr, MTAB *mptr, char *cptr, int32 flag);
t_stat sim_check_console (int32 sec);
t_stat sim_save (SMP_FILE *sfile);
t_stat sim_rest (SMP_FILE *rfile);

/* cpu commands */

t_stat cpu_cmd (int32 flag, char *ptr);
t_stat cpu_cmd_multiprocessor (SMP_FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, char *cptr);
t_stat cpu_cmd_id (SMP_FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, char *cptr);
t_stat cpu_cmd_smt (SMP_FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, char *cptr);

/* Breakpoint package */

t_stat sim_brk_init (void);
t_stat sim_brk_set (t_addr loc, int32 sw, int32 ncnt, char *act);
t_stat sim_brk_clr (t_addr loc, int32 sw);
t_stat sim_brk_clrall (int32 sw);
t_stat sim_brk_show (SMP_FILE *st, t_addr loc, int32 sw);
t_stat sim_brk_showall (SMP_FILE *st, int32 sw);
void sim_brk_clract (void);
void sim_brk_npc (RUN_DECL, uint32 cnt);
BRKTAB *sim_brk_new (t_addr loc);
static t_bool sim_brk_is_action_pending ();
sim_cstream* sim_brk_get_action_script ();
const char* sim_brk_end_action_script (sim_cstream* script);

/* Commands support routines */

SCHTAB *get_search (char *cptr, int32 radix, SCHTAB *schptr);
int32 test_search (t_value val, SCHTAB *schptr);
char *get_glyph_gen (char *iptr, char *optr, char mchar, t_bool uc);
int32 get_switches (char *cptr);
char *get_sim_sw (char *cptr);
t_stat get_aval (t_addr addr, DEVICE *dptr, UNIT *uptr);
t_value get_rval (REG *rptr, uint32 idx);
void put_rval (REG *rptr, uint32 idx, t_value val);
t_value strtotv (char *inptr, char **endptr, uint32 radix);
void fprint_help (SMP_FILE *st);
void fprint_stopped (SMP_FILE *st);
void fprint_capac (SMP_FILE *st, DEVICE *dptr, UNIT *uptr);
char *read_line (char *ptr, int32 size, SMP_FILE *stream);
char *read_line_p (char *prompt, char *ptr, int32 size, SMP_FILE *stream);
REG *find_reg_glob (char *ptr, char **optr, DEVICE **gdptr);
char *sim_trim_endspc (char *cptr);

/* Forward references */

static void sim_wait_debugger(int* pargc, char* argv[]);
t_stat scp_attach_unit (DEVICE *dptr, UNIT *uptr, char *cptr);
t_stat scp_detach_unit (DEVICE *dptr, UNIT *uptr);
t_bool qdisable (DEVICE *dptr);
t_stat attach_err (UNIT *uptr, t_stat stat);
t_stat detach_all (int32 start_device, t_bool shutdown);
t_stat assign_device (DEVICE *dptr, char *cptr);
t_stat deassign_device (DEVICE *dptr);
t_stat ssh_break_one (SMP_FILE *st, int32 flg, t_addr lo, int32 cnt, char *aptr);
t_stat run_boot_prep (void);
t_stat exdep_reg_loop (SMP_FILE *ofile, SCHTAB *schptr, int32 flag, char *cptr,
    REG *lowr, REG *highr, uint32 lows, uint32 highs);
t_stat ex_reg (SMP_FILE *ofile, t_value val, int32 flag, REG *rptr, uint32 idx);
t_stat dep_reg (int32 flag, char *cptr, REG *rptr, uint32 idx);
t_stat exdep_addr_loop (SMP_FILE *ofile, SCHTAB *schptr, int32 flag, char *cptr,
    t_addr low, t_addr high, DEVICE *dptr, UNIT *uptr);
t_stat ex_addr (SMP_FILE *ofile, int32 flag, t_addr addr, DEVICE *dptr, UNIT *uptr);
t_stat dep_addr (int32 flag, char *cptr, t_addr addr, DEVICE *dptr,
    UNIT *uptr, int32 dfltinc);
void sub_args (char *instr, char *tmpbuf, int32 maxstr, char *do_arg[]);
static void print_prompt (SMP_FILE* fp, const char* component = NULL);
static void make_prompt (char* bp, const char* component = NULL);
void fprint_stopped_instr_or_state (RUN_DECL, SMP_FILE *st, const char* msg, REG *pc, DEVICE *dptr);
void fprint_stopped_instr (RUN_DECL, SMP_FILE *st, const char* msg, REG *pc, DEVICE *dptr);
static t_stat run_cmd_core (RUN_DECL, int32 runcmd);
void int_handler (int signal);
static clock_queue_entry* reverse_cqe_list(clock_queue_entry* list);
static void setup_cotimed_cqe_list(clock_queue_entry* list, int32 nticks);
static void insert_cotimed_cqe_list(RUN_DECL, clock_queue_entry* list, int32 newtime);
void sim_reevaluate_noncpu_thread_priority(run_scope_context* rscx);
t_stat set_on (int32 flag, char *cptr);
t_stat set_asynch (int32 flag, char *cptr);
t_stat sim_set_asynch (int32 flag, char *cptr);
t_stat sim_set_environment (int32 flag, char *cptr);
t_bool sim_cpu_has_syswide_events(RUN_DECL);

/* Global data */

DEVICE *sim_dflt_dev = NULL;
CPU_UNIT *sim_dflt_cpu = NULL;
run_scope_context* sim_dflt_rscx = NULL;
int32 sim_switches = 0;
SMP_FILE *sim_ofile = NULL;
SCHTAB *sim_schptr = FALSE;
DEVICE *sim_dfdev = NULL;
UNIT *sim_dfunit = NULL;
int32 sim_opt_out = 0;
uint32 sim_brk_summ = 0;
uint32 sim_brk_types = 0;
uint32 sim_brk_dflt = 0;
BRKTAB *sim_brk_tab = NULL;
int32 sim_brk_ent = 0;
int32 sim_brk_lnt = 0;
atomic_int32 sim_brk_ins = 0;
t_bool sim_brk_continue = FALSE;
typedef sim_cstream* sim_cstream_ptr_t;
static sim_stack<sim_cstream_ptr_t> sim_brk_action_stack;
int32 sim_quiet = 0;
int32 sim_step = 0;
atomic_int32 stop_cpus = 0;
t_value *sim_eval = NULL;
SMP_FILE *sim_log = NULL;                                  /* log file */
SMP_FILE *sim_deb = NULL;                                  /* debug file */
SMP_FILEREF *sim_log_ref = NULL;                           /* log file file reference */
SMP_FILEREF *sim_deb_ref = NULL;                           /* debug file file reference */
static int32 sim_do_echo = 0;                              /* the echo status of the currently open do file */
static t_stat sim_last_cmd_stat;                           /* command status */
static SCHTAB sim_stab;
t_bool sim_vcpu_per_core = FALSE;                          /* VCPU affinity control: if set true, try to assign at most 
                                                              one VCPU per single host PCPU core */
t_bool sim_vsmp_active = FALSE;                            /* virtual multiprocessing initialized (VSMP had called VAXMP_API_OP_INIT_SMP) */
uint32 sim_vsmp_os = 0;                                    /* guest OS ID (valid only if sim_vsmp_active is TRUE) */
t_bool sim_vsmp_idle_sleep = FALSE;                        /* TRUE if idle sleep is enabled (VSMP SET IDLE=ON), valid only if sim_vsmp_active is TRUE */
smp_affinity_kind_t sim_vcpu_affinity = SMP_AFFINITY_ALL;  /* VCPU affinity */
int32 sim_units_percpu = 0;                                /* number of per-CPU units in the system */
int32 sim_units_global = 0;                                /* number of global units in the system */
static clock_queue_entry_info* sim_requeue_info = NULL;    /* data buffer used by sim_requeue_syswide_events */
t_bool sim_asynch_enabled = FALSE;
extern UNIT sim_throt_unit;
t_bool sim_ttrun_mode = FALSE;
on_init_call* on_init_call::head = NULL;
smp_lock* cpu_database_lock = NULL;
smp_semaphore* cpu_attention = NULL;
smp_barrier* cpu_pause_sync_barrier = NULL;
smp_semaphore* cpu_clock_run_gate = NULL;
t_bool sim_clock_thread_created = FALSE;
t_bool use_clock_thread = USE_CLOCK_THREAD;
/*
 * If use_native_interlocked is true, use host native interlocked instructions to simulate target machine's
 * interlocked instructions. If false, use portable interlock 
 *
 * Do not enable this flag unless you understand well the ramification and, in particular, that synchronization
 * window may need to be extended to non-kernel processor modes and low IPLs in kernel mode.
 *
 * See "VAX MP Techincal Overview", chapters "Synchronization window" and "Implementation of VAX interlocked
 * instructions" for more details.
 */
uint32 use_native_interlocked = FALSE;
smp_thread_t sim_clock_thread = SMP_THREAD_NULL;
uint32 cpu_cycles_mark[SIM_MAX_CPUS];
uint32 cpu_cycles_sleepexit[SIM_MAX_CPUS];
cpu_set cpu_waitset[SIM_MAX_CPUS];
cpu_set cpu_running_set;
/*
 * sim_mp_active is FALSE when only one processor is active.
 *
 * Use is as follows:
 *
 *     Value is changed only either by VCPU thread holding cpu_database_lock or by console thread while VCPUs are paused.
 *
 *     weak_read(sim_mp_active) == FALSE means that only primary VCPU is active,
 *         it is guaranted that no other VCPUs are running
 *
 *     weak_read(sim_mp_active) == TRUE means _almost_ always that multiple VCPUs are running,
 *         however very occassionally it can also be TRUE if secondary just stopped but the change
 *         in sim_mp_active value performed by secondary shutdown had not propagated to the primary yet.
 *
 * In other words, weak_read(sim_mp_active) value of FALSE is reliable, but value of TRUE is not.
 * This is sufficient for the purposes of taking uniprocessor "fast paths" in the code that provides
 * optimized behavior for uniprocessor execution.
 *
 * Note that described rule applies only within VCPU threads, and also within console thread but only when
 * VCPU threads are paused. The rule is not valid within CLOCK or IOP threads or console thread when VCPU threads
 * are running.
 *
 */
atomic_bool sim_mp_active = FALSE;

t_bool sim_ws_prefaulted = FALSE;                          /* if TRUE, memory had been pre-faulted */
t_bool sim_ws_settings_changed = FALSE;                    /* if TRUE, working set settings below had been changed */
t_bool sim_ws_lock = FALSE;                                /* if TRUE, lock all pages in working set */
uint32 sim_ws_min = 0;                                     /* minimum working set size (MB) */
uint32 sim_ws_max = 0;                                     /* maximum working set size (MB) */
uint32 sim_host_turbo = 120;                               /* host CPU turbo factor (max cpu freq / min cpu freq) */
t_bool sim_host_dedicated = FALSE;                         /* if TRUE, host is wholly dedicated to running the simulator,
                                                              therefore do not perform VCPU thread priority managemenet */

#if defined USE_INT64
static const char *sim_si64 = "64b data";
#else
static const char *sim_si64 = "32b data";
#endif
#if defined USE_ADDR64
static const char *sim_sa64 = "64b addresses";
#else
static const char *sim_sa64 = "32b addresses";
#endif
#if defined (USE_NETWORK) || defined (USE_SHARED)
static const char *sim_snet = "Ethernet support";
#else
static const char *sim_snet = "no Ethernet";
#endif

/* Tables and strings */

const char save_vercur[] = "M3.5";
const char save_ver32[] = "V3.2";
const char save_ver30[] = "V3.0";
const struct scp_error
{
    char *code;
    char *message;
}
scp_errors[SCPE_MAX_ERR - SCPE_BASE + 1] =
{
    {"NXM",     "Address space exceeded"},
    {"UNATT",   "Unit not attached"},
    {"IOERR",   "I/O error"},
    {"CSUM",    "Checksum error"},
    {"FMT",     "Format error"},
    {"NOATT",   "Unit not attachable"},
    {"OPENERR", "File open error"},
    {"MEM",     "Memory exhausted"},
    {"ARG",     "Invalid argument"},
    {"STEP",    "Step expired"},
    {"UNK",     "Unknown command"},
    {"RO",      "Read only argument"},
    {"INCOMP",  "Command not completed"},
    {"STOP",    "Simulation stopped"},
    {"EXIT",    "Goodbye"},
    {"TTIERR",  "Console input I/O error"},
    {"TTOERR",  "Console output I/O error"},
    {"EOF",     "End of file"},
    {"REL",     "Relocation error"},
    {"NOPARAM", "No settable parameters"},
    {"ALATT",   "Unit already attached"},
    {"TIMER",   "Hardware timer error"},
    {"SIGERR",  "SIGINT handler setup error"},
    {"TTYERR",  "Console terminal setup error"},
    {"SUB",     "Subscript out of range"},
    {"NOFNC",   "Command not allowed"},
    {"UDIS",    "Unit disabled"},
    {"NORO",    "Read only operation not allowed"},
    {"INVSW",   "Invalid switch"},
    {"MISVAL",  "Missing value"},
    {"2FARG",   "Too few arguments"},
    {"2MARG",   "Too many arguments"},
    {"NXDEV",   "Non-existent device"},
    {"NXUN",    "Non-existent unit"},
    {"NXREG",   "Non-existent register"},
    {"NXPAR",   "Non-existent parameter"},
    {"NEST",    "Nested DO command limit exceeded"},
    {"IERR",    "Internal error"},
    {"MTRLNT",  "Invalid magtape record length"},
    {"LOST",    "Console Telnet connection lost"},
    {"TTMO",    "Console Telnet connection timed out"},
    {"STALL",   "Console Telnet output stall"},
    {"AFAIL",   "Assertion failed"},
    {"SWSTP",   "Unable to perform STEP since CPU is constrained by the synchronization window"},
};

const size_t size_map[] = { sizeof (int8),
    sizeof (int8), sizeof (int16), sizeof (int32), sizeof (int32)
#if defined (USE_INT64)
    , sizeof (t_int64), sizeof (t_int64), sizeof (t_int64), sizeof (t_int64)
#endif
};

const t_value width_mask[] = { 0,
    0x1, 0x3, 0x7, 0xF,
    0x1F, 0x3F, 0x7F, 0xFF,
    0x1FF, 0x3FF, 0x7FF, 0xFFF,
    0x1FFF, 0x3FFF, 0x7FFF, 0xFFFF,
    0x1FFFF, 0x3FFFF, 0x7FFFF, 0xFFFFF,
    0x1FFFFF, 0x3FFFFF, 0x7FFFFF, 0xFFFFFF,
    0x1FFFFFF, 0x3FFFFFF, 0x7FFFFFF, 0xFFFFFFF,
    0x1FFFFFFF, 0x3FFFFFFF, 0x7FFFFFFF, 0xFFFFFFFF
#if defined (USE_INT64)
    , 0x1FFFFFFFF, 0x3FFFFFFFF, 0x7FFFFFFFF, 0xFFFFFFFFF,
    0x1FFFFFFFFF, 0x3FFFFFFFFF, 0x7FFFFFFFFF, 0xFFFFFFFFFF,
    0x1FFFFFFFFFF, 0x3FFFFFFFFFF, 0x7FFFFFFFFFF, 0xFFFFFFFFFFF,
    0x1FFFFFFFFFFF, 0x3FFFFFFFFFFF, 0x7FFFFFFFFFFF, 0xFFFFFFFFFFFF,
    0x1FFFFFFFFFFFF, 0x3FFFFFFFFFFFF, 0x7FFFFFFFFFFFF, 0xFFFFFFFFFFFFF,
    0x1FFFFFFFFFFFFF, 0x3FFFFFFFFFFFFF, 0x7FFFFFFFFFFFFF, 0xFFFFFFFFFFFFFF,
    0x1FFFFFFFFFFFFFF, 0x3FFFFFFFFFFFFFF,
    0x7FFFFFFFFFFFFFF, 0xFFFFFFFFFFFFFFF,
    0x1FFFFFFFFFFFFFFF, 0x3FFFFFFFFFFFFFFF,
    0x7FFFFFFFFFFFFFFF, 0xFFFFFFFFFFFFFFFF
#endif
    };

static CTAB cmd_table[] = {
    { "RESET", &reset_cmd, 0,
      "r{eset} {ALL|<device>}     reset simulator\n" },
    { "EXAMINE", &exdep_cmd, EX_E,
      "e{xamine} <list>           examine memory or registers\n" },
    { "IEXAMINE", &exdep_cmd, EX_E+EX_I,
      "ie{xamine} <list>          interactive examine memory or registers\n" },
    { "DEPOSIT", &exdep_cmd, EX_D,
      "d{eposit} <list> <val>     deposit in memory or registers\n" },
    { "IDEPOSIT", &exdep_cmd, EX_D+EX_I,
      "id{eposit} <list>          interactive deposit in memory or registers\n" },
    { "EVALUATE", &eval_cmd, 0,
      "ev{aluate} <expr>          evaluate symbolic expression\n" },
    { "RUN", &run_cmd, RU_RUN,
      "ru{n} {new PC}             reset and start simulation\n" },
    { "GO", &run_cmd, RU_GO,
      "go {new PC}                start simulation\n" }, 
    { "STEP", &run_cmd, RU_STEP,
      "s{tep} {n}                 simulate n instructions\n" },
    { "CONT", &run_cmd, RU_CONT,
      "c{ont}                     continue simulation\n" },
    { "BOOT", &run_cmd, RU_BOOT,
      "b{oot} <unit>              bootstrap unit\n" },
    { "BREAK", &brk_cmd, SSH_ST,
      "br{eak} <list>             set breakpoints\n" },
    { "NOBREAK", &brk_cmd, SSH_CL,
      "nobr{eak} <list>           clear breakpoints\n" },
    { "ATTACH", &attach_cmd, 0,
      "at{tach} <unit> <file>     attach file to simulated unit\n" },
    { "DETACH", &detach_cmd, 0,
      "det{ach} <unit>            detach file from simulated unit\n" },
    { "ASSIGN", &assign_cmd, 0,
      "as{sign} <device> <name>   assign logical name for device\n" },
    { "DEASSIGN", &deassign_cmd, 0,
      "dea{ssign} <device>        deassign logical name for device\n" },
    { "SAVE", &save_cmd, 0,
      "sa{ve} <file>              save simulator to file\n" },
    { "RESTORE", &restore_cmd, 0,
      "rest{ore}|ge{t} <file>     restore simulator from file\n" },
    { "GET", &restore_cmd, 0, NULL },
    { "LOAD", &load_cmd, 0,
      "l{oad} <file> {<args>}     load binary file\n" },
    { "DUMP", &load_cmd, 1,
      "du(mp) <file> {<args>}     dump binary file\n" },
    { "EXIT", &exit_cmd, 0,
      "exi{t}|q{uit}|by{e}        exit from simulation\n" },
    { "QUIT", &exit_cmd, 0, NULL },
    { "BYE", &exit_cmd, 0, NULL },
    { "SET", &set_cmd, 0,
      "set console arg{,arg...}   set console options\n"
      "set console WRU            specify console drop to simh char\n"
      "set console BRK            specify console Break character\n"
      "set console DEL            specify console delete char\n"
      "set console PCHAR          specify console printable chars\n"
      "set console TELNET=port    specify console telnet port\n"
      "set console TELNET=LOG     specify console telnet logging\n"
      "set console TELNET=NOLOG   disables console telnet logging\n"
      "set console TELNET=BUFFERED[=bufsize]\n"
      "                           specify console telnet buffering\n"
      "set console TELNET=NOBUFFERED\n"
      "                           disables console telnet buffering\n"
      "set console TELNET=UNBUFFERED\n"
      "                           disables console telnet buffering\n"
      "set console NOTELNET       disable console telnet\n"
      "set console LOG            enable console logging\n"
      "set console NOLOG          disable console logging\n"
      "set console DEBUG          enable console debugging\n"
      "set console NODEBUG        disable console debugging\n"
      "set break <list>           set breakpoints\n"
      "set nobreak <list>         clear breakpoints\n"
      "set throttle x{M|K|%%}     set simulation rate\n"
      "set nothrottle             set simulation rate to maximum\n"
      "set asynch                 enable asynchronous I/O\n"
      "set noasynch               disable asynchronous I/O\n"
      "set environment name=val   set environment variable\n"
      "set <dev> OCT|DEC|HEX      set device display radix\n"
      "set <dev> ENABLED          enable device\n"
      "set <dev> DISABLED         disable device\n"
      "set <dev> DEBUG{=arg}      set device debug flags\n"
      "set <dev> NODEBUG={arg}    clear device debug flags\n"
      "set <dev> arg{,arg...}     set device parameters (see show modifiers)\n"
      "set <unit> ENABLED         enable unit\n"
      "set <unit> DISABLED        disable unit\n"
      "set <unit> arg{,arg...}    set unit parameters (see show modifiers)\n"
      // "set on                   enables error checking after command execution\n"
      // "set noon                 disables error checking after command execution\n"
      },
    { "SHOW", &show_cmd, 0,
      "sh{ow} br{eak} <list>      show breakpoints\n"
      "sh{ow} con{figuration}     show configuration\n"
      "sh{ow} cons{ole} {arg}     show console options\n"
      "sh{ow} dev{ices}           show devices\n"  
      "sh{ow} m{odifiers}         show modifiers for all devices\n" 
      "sh{ow} s{how}              show SHOW commands for all devices\n" 
      "sh{ow} n{ames}             show logical names\n" 
      "sh{ow} q{ueue}             show event queue\n"  
      "sh{ow} ti{me}              show simulated time\n"
      "sh{ow} th{rottle}          show simulation rate\n" 
      "sh{ow} a{synch}            show asynchronouse I/O state\n" 
      "sh{ow} ve{rsion}           show simulator version\n" 
      "sh{ow} <dev> RADIX         show device display radix\n"
      "sh{ow} <dev> DEBUG         show device debug flags\n"
      "sh{ow} <dev> MODIFIERS     show device modifiers\n"
      "sh{ow} <dev> NAMES         show device logical name\n"
      "sh{ow} <dev> SHOW          show device SHOW commands\n"
      "sh{ow} <dev> {arg,...}     show device parameters\n"
      "sh{ow} <unit> {arg,...}    show unit parameters\n"  },
    { "CPU", &cpu_cmd, 0,
      "cpu multi{processor} <n>   create virtual CPUs\n" 
      "cpu id <id>                select CPU for current context\n"
      "cpu id                     show currently selected CPU\n"
      "cpu smt <factor>           override SMT slow-down factor\n"  
      "cpu info                   detailed CPU state information\n" },
#if 1
    { "XDEV", &xdev_cmd, 0, NULL },   /* internal development/debugging tool */
#endif
    { "PERF", &perf_cmd, 0,
      "perf on [counter]          enable performance counter(s)\n" 
      "perf off [counter]         disable performance counter(s)\n" 
      "perf reset [counter]       reset performance counter(s)\n" 
      "perf show [counter]        display performance counter(s)\n" },
    { "DO", &do_cmd, 1,
      "do <file> {arg,arg...}     process command file\n" },
    { "ECHO", &echo_cmd, 0,
      "echo <string>              display <string>\n" },
    { "ASSERT", &assert_cmd, 0,
      "assert {<dev>} <cond>      test simulator state against condition\n" },
    { "HELP", &help_cmd, 0,
      "h{elp}                     type this message\n"
      "h{elp} <command>           type help for command\n" },
    { "!", &spawn_cmd, 0,         
      "!                          execute local command interpreter\n"
      "! <command>                execute local host command\n" },
    { NULL, NULL, 0 }
    };

#if defined(_WIN32)
static int setenv(const char *envname, const char *envval, int overwrite)
{
    char* envstr = (char*) malloc(strlen(envname) + strlen(envval) + 2);
    int r;
    if (envstr == NULL)
    {
        r = -1;
    }
    else
    {
        sprintf(envstr, "%s=%s", envname, envval);
        r = _putenv(envstr);
        free(envstr);
    }
    return r;
}
#endif

/* Main command loop */

static int main_exec (int argc, char *argv[]);

int main (int argc, char *argv[])
{
    sim_try_volatile int vol_argc = argc;   /* relax compiler warning */
    sim_wait_debugger(&argc, argv);

#if defined(USE_C_TRY_CATCH)
    /*
     * Note that sim_seh_frame::init() uses TLS, but we call it ahead of init_threads
     * in assumption that TLS is always ready to use and does not require initialization.
     */
    if (! sim_seh_frame::init())
    {
        fprintf(stderr, "Unable to initialize SEH framework\n");
        return 0;
    }
#endif

    sim_try
    {
        const char* ptname = "CONSOLE";

        smp_check_aligned(& sim_brk_ins);
        smp_check_aligned(& stop_cpus);
        smp_check_aligned(& hlt_pin);
        smp_check_aligned(& tmr_poll);
        smp_check_aligned(& tmxr_poll);

        init_threads_core();
        sim_timer_init();
        init_threads_ext();

        smp_stdin = smp_file_wrap(stdin);
        smp_stdout = smp_file_wrap(stdout);
        smp_stderr = smp_file_wrap(stderr);

        cpu_database_lock = smp_lock::create(smp_spinwait_min_us, 1000, 10000);
        cpu_database_lock->set_criticality(SIM_LOCK_CRITICALITY_VM);
        cpu_attention = smp_semaphore::create(0);
        cpu_clock_run_gate = smp_semaphore::create(0);
        cpu_pause_sync_barrier = smp_barrier::create(2);
        cpu_cycles_per_second_lock = smp_lock::create(smp_spinwait_min_us, 1000, 3000);
        cpu_cycles_per_second_lock->set_criticality(SIM_LOCK_CRITICALITY_VM);

        perf_register_object("cpu_database_lock", cpu_database_lock);
        perf_register_object("cpu_cycles_per_second_lock", cpu_cycles_per_second_lock);

        on_init_call::invoke_all();

#if defined(__linux)
        /* Linux System Monitor tool displays main thread's name as the name of the whole process */
        static char tname[128];
        sprintf(tname, "%s CONSOLE", sim_name);
        ptname = tname;
#endif

        smp_set_thread_name(ptname);
    }
    sim_catch (sim_exception_SimError, exc)
    {
        /* message here to stderr, not smp_stderr yet */
        fprintf (stderr, "Error: %s\n", exc->get_message());
        exc->checkAutoDelete();
        return 0;
    }
    sim_end_try

    sim_try
    {
        return main_exec (vol_argc, argv);
    }
    sim_catch (sim_exception_SimError, exc)
    {
        fprintf (smp_stderr, "Error: %s\n", exc->get_message());
        exc->checkAutoDelete();
        return 0;
    }
    sim_end_try
}

static int main_exec (int argc, char *argv[])
{
    char cbuf[CBUFSIZE], gbuf[CBUFSIZE], *cptr;
    int32 i, sw;
    t_bool lookswitch;
    t_stat stat;
    CTAB *cmdp;
    DEVICE *dptr;

    for (i = 0; (dptr = sim_devices[i]) != NULL; i++)
    {
        dptr->a_reset_count = 0;
        for (uint32 k = 0; k < dptr->numunits; k++)
        {
            dptr->units[k]->device = dptr;
            dptr->units[k]->unitno = k;
            if (dptr->flags & DEV_PERCPU)
                sim_units_percpu++;
            else
                sim_units_global++;
        }
    }

    sim_init_interrupt_info();

    sim_dflt_cpu = & cpu_unit_0;
    sim_dflt_rscx = new run_scope_context(&cpu_unit_0, SIM_THREAD_TYPE_CONSOLE);
    sim_dflt_rscx->set_current();
    syncw_init();
    init_cpu_unit_0();
    smp_set_thread_priority(SIMH_THREAD_PRIORITY_CONSOLE_PAUSED);
    memzero(cpu_cycles_mark);
    memzero(cpu_cycles_sleepexit);

#if defined (__MWERKS__) && defined (macintosh)
    argc = ccommand (&argv);
#endif

    *cbuf = 0;                                              /* init arg buffer */
    sim_switches = 0;                                       /* init switches */
    lookswitch = TRUE;
    for (i = 1; i < argc; i++)                              /* loop thru args */
    {
        if (argv[i] == NULL)                                /* paranoia */
            continue;
        if ((*argv[i] == '-') && lookswitch)                /* switch? */
        {
            if ((sw = get_switches (argv[i])) < 0)
            {
                fprintf (smp_stderr, "Invalid switch %s\n", argv[i]);
                return 0;
            }
            sim_switches = sim_switches | sw;
        }
        else
        {
            if ((strlen (argv[i]) + strlen (cbuf) + 1) >= CBUFSIZE)
            {
                fprintf (smp_stderr, "Argument string too long\n");
                return 0;
            }
            if (*cbuf)                                      /* concat args */
                strcat (cbuf, " "); 
            strcat (cbuf, argv[i]);
            lookswitch = FALSE;                             /* no more switches */
        }
    }
    sim_quiet = sim_switches & SWMASK ('Q');                /* -q means quiet */

    if (sim_vm_init != NULL)                                /* call once only */
        (*sim_vm_init)();
    sim_finit ();                                           /* init fio package */
    setenv ("SIM_NAME", sim_name, 1);                       /* publish simulator name */
    stop_cpus = 0;
    sim_log = NULL;
    if (sim_emax <= 0)
        sim_emax = 1;

    if ((stat = sim_ttinit ()) != SCPE_OK)
    {
        fprintf (smp_stderr, "Fatal terminal initialization error\n%s\n",
            sim_error_text (stat));
        return 0;
    }
    if ((sim_requeue_info = (clock_queue_entry_info*) calloc(sim_units_global + 1, sizeof(clock_queue_entry_info))) == NULL)
    {
        fprintf (smp_stderr, "Unable to allocate memory for internal data buffer\n");
        return 0;
    }
    if ((sim_eval = (t_value *) calloc (sim_emax, sizeof (t_value))) == NULL)
    {
        fprintf (smp_stderr, "Unable to allocate examine buffer\n");
        return 0;
    }
    if ((stat = reset_all_p (0)) != SCPE_OK)
    {
        fprintf (smp_stderr, "Fatal simulator initialization error\n%s\n",
            sim_error_text (stat));
        return 0;
    }
    if ((stat = sim_brk_init ()) != SCPE_OK)
    {
        fprintf (smp_stderr, "Fatal breakpoint table initialization error\n%s\n",
            sim_error_text (stat));
        return 0;
    }
    if (!sim_quiet)
    {
        smp_printf ("\n");
        show_version (smp_stdout, NULL, NULL, 0, NULL);
    }
    if (sim_dflt_dev == NULL)                               /* if no default */
        sim_dflt_dev = sim_devices[0];

    if (*cbuf)                                              /* cmd file arg? */
        stat = do_cmd (0, cbuf);                            /* proc cmd file */
    else if (*argv[0])                                      /* sim name arg? */
    {
        char nbuf[PATH_MAX + 7], *np;                       /* "path.ini" */
        nbuf[0] = '"';                                      /* starting " */
        strncpy (nbuf + 1, argv[0], PATH_MAX + 1);          /* copy sim name */
        if (np = match_ext (nbuf, "EXE"))                   /* remove .exe */
            *np = 0;
        strcat (nbuf, ".ini\"");                            /* add .ini" */
        stat = do_cmd (-1, nbuf);                           /* proc cmd file */
    }

    while (stat != SCPE_EXIT)                               /* in case exit */
    {
        sim_try
        {
            cptr = NULL;

            smp_set_thread_priority(SIMH_THREAD_PRIORITY_CONSOLE_PAUSED);

            stat = process_brk_actions (0, & cptr);
            if (stat == SCPE_EXIT)  break;

            if (cptr != NULL)
            {
                /* if sim_brk_end_action_script returned "cont" */
                print_prompt (smp_stdout, "brk");
                fprintf(smp_stdout, "%s\n", cptr);
            }
            else
            {
                if (sim_vm_read != NULL)                        /* sim routine? */
                {
                    print_prompt (smp_stdout);                  /* prompt */
                    cptr = (*sim_vm_read) (cbuf, CBUFSIZE, smp_stdin);
                }
                else
                {
                    char pmt[32];
                    make_prompt (pmt);
                    cptr = read_line_p (pmt, cbuf, CBUFSIZE, smp_stdin);   /* read command line */
                }
            }

            if (cptr == NULL)                                   /* EOF? */
            {
                if (sim_ttisatty())
                    continue;                                   /* ignore tty EOF */
                else
                    break;                                      /* otherwise exit */
            }

            if (*cptr == 0)                                     /* ignore blank */
                continue;
            sub_args (cbuf, gbuf, CBUFSIZE, argv);
            if (sim_log)                                        /* log cmd */
            {
                print_prompt (sim_log);
                fprintf (sim_log, "%s\n", cptr);
            }
            cptr = get_glyph (cptr, gbuf, 0);                   /* get command glyph */
            sim_switches = 0;                                   /* init switches */
            if (cmdp = find_cmd (gbuf))                         /* lookup command */
            {
                sim_dflt_rscx->cpu_unit = sim_dflt_cpu;
                stat = cmdp->action (cmdp->arg, cptr);          /* if found, exec */
                sim_dflt_rscx->cpu_unit = sim_dflt_cpu;
            }
            else
                stat = SCPE_UNK;
            if (stat >= SCPE_BASE)                              /* error? */
            {
                smp_printf ("%s\n", sim_error_text (stat));
                if (sim_log)
                    fprintf (sim_log, "%s\n", sim_error_text (stat));
            }
            if (sim_vm_post != NULL)
                (*sim_vm_post) (TRUE);

            if (stat != SCPE_OK || cmdp->action != &echo_cmd)
                sim_last_cmd_stat = stat;                       /* save command error status */
        }
        sim_catch (sim_exception_SimError, exc)
        {
            /* last chance handler for out-of-memory and similar critical conditions
               during console command execution, without losing the simulator's general state:
               reset pending actions, issue message and drop to console prompt */
            sim_brk_action_stack.reset();
            sim_brk_clract();
            sim_brk_continue = FALSE;
            fprintf (smp_stderr, "\nSevere error while handling commands: %s\n", exc->get_message());
            if (sim_log)
                fprintf (sim_log, "\nSevere error while handling commands: %s\n", exc->get_message());
            exc->checkAutoDelete();
        }
        sim_end_try
    }

    detach_all (0, TRUE);                                   /* close files */
    sim_set_deboff (0, NULL);                               /* close debug */
    sim_set_logoff (0, NULL);                               /* close log */
    sim_set_notelnet (0, NULL);                             /* close Telnet */
    sim_ttclose ();                                         /* close console */
    return 0;
}

t_stat process_new_brk_actions (int flag)
{
    if (sim_brk_is_action_pending ())
        return process_brk_actions (flag, NULL);
    else
        return SCPE_OK;
}

t_stat process_brk_actions (int flag, char** ppcmd)
{
    sim_cstream* brk_script = sim_brk_get_action_script ();
    t_stat stat = SCPE_OK;
    char* dummy;

    if (brk_script)
    {
        if (ppcmd == NULL)
            ppcmd = &dummy;

        int32 sv_sim_switches = sim_switches;
        sim_switches |= SWMASK ('V');
        sim_switches &= ~SWMASK ('E');

        stat = do_script (flag, brk_script, "brk");
        *ppcmd = (char*) sim_brk_end_action_script (brk_script);

        int32 rmask = SWMASK ('V') | SWMASK ('E');
        sv_sim_switches &= ~rmask;
        sv_sim_switches |= sv_sim_switches & rmask;
    }

    return stat;
}

static void print_prompt (SMP_FILE* fp, const char* component)
{
    char pmt[32];
    make_prompt (pmt, component);
    fprintf(fp, "%s", pmt);
}

static void make_prompt (char* bp, const char* component)
{
    if (component == NULL)
        component = "sim";

    if (sim_dflt_dev != NULL)
    {
        if (sim_dflt_dev == &cpu_dev && sim_ncpus == 1)
        {
            sprintf (bp, "%s> ", component);
        }
        else if (sim_dflt_dev == &cpu_dev && sim_ncpus != 1)
        {
            sprintf (bp, "%s-cpu%d> ", component, sim_dflt_cpu->cpu_id);
        }
        else
        {
            sprintf (bp, "%s-%s> ", component, sim_dflt_dev->name);
        }
    }
    else if (sim_ncpus != 1)
    {
        sprintf (bp, "%s-cpu%d> ", component, sim_dflt_cpu->cpu_id);
    }
    else
    {
        sprintf (bp, "%s> ", component);
    }
}

void check_select_cpu(UNIT* uptr)
{
    if (uptr && uptr->is_cpu())
    {
        sim_dflt_rscx->cpu_unit = (CPU_UNIT*) uptr;
    }
}

/* Find command routine */

CTAB *find_cmd (char *gbuf)
{
CTAB *cmdp = NULL;

if (sim_vm_cmd)                                         /* try ext commands */
    cmdp = find_ctab (sim_vm_cmd, gbuf);
if (cmdp == NULL)                                       /* try regular cmds */
    cmdp = find_ctab (cmd_table, gbuf);
return cmdp;
}

/* Exit command */

t_stat exit_cmd (int32 flag, char *cptr)
{
return SCPE_EXIT;
}

/* Help command */

void fprint_help (SMP_FILE *st)
{
CTAB *cmdp;

for (cmdp = sim_vm_cmd; cmdp && (cmdp->name != NULL); cmdp++) {
    if (cmdp->help)
        fputs (cmdp->help, st);
    }
for (cmdp = cmd_table; cmdp && (cmdp->name != NULL); cmdp++) {
    if (cmdp->help && (!sim_vm_cmd || !find_ctab (sim_vm_cmd, cmdp->name)))
        fputs (cmdp->help, st);
    }
return;
}

t_stat help_cmd (int32 flag, char *cptr)
{
char gbuf[CBUFSIZE];
CTAB *cmdp;

GET_SWITCHES (cptr);
if (*cptr) {
    cptr = get_glyph (cptr, gbuf, 0);
    if (*cptr)
        return SCPE_2MARG;
    if (cmdp = find_cmd (gbuf)) {
        fputs (cmdp->help, smp_stdout);
        if (sim_log)
            fputs (cmdp->help, sim_log);
        }
    else return SCPE_ARG;
    }
else {
    fprint_help (smp_stdout);
    if (sim_log)
        fprint_help (sim_log);
    }
return SCPE_OK;
}

/* Spawn command */

t_stat spawn_cmd (int32 flag, char *cptr)
{
t_stat status;
if ((cptr == NULL) || (strlen (cptr) == 0))
    cptr = getenv("SHELL");
if ((cptr == NULL) || (strlen (cptr) == 0))
    cptr = getenv("ComSpec");
#if defined (VMS)
if ((cptr == NULL) || (strlen (cptr) == 0))
    cptr = "SPAWN/INPUT=SYS$COMMAND:";
#endif
fflush(smp_stdout);                                         /* flush stdout */
if (sim_log)                                            /* flush log if enabled */
    fflush (sim_log);
status = system (cptr);
#if defined (VMS)
printf ("\n");
#endif

return status;
}

/* Echo command */

t_stat echo_cmd (int32 flag, char *cptr)
{
puts (cptr);
if (sim_log)
    fprintf (sim_log, "%s\n", cptr);
return SCPE_OK;
}

/* Do command

   Syntax: DO {-E} {-V} <filename> {<arguments>...}

   -E causes all command errors to be fatal; without it, only EXIT and ASSERT
   failure will stop a command file.

   -V causes commands to be echoed before execution.

   Note that SCPE_STEP ("Step expired") is considered a note and not an error
   and so does not abort command execution when using -E.
   
   Inputs:
        flag    =   caller and nesting level indicator
        fcptr   =   filename and optional arguments, space-separated
   Outputs:
        status  =   error status

   The "flag" input value indicates the source of the call, as follows:

        -1      =   initialization file (no error if not found)
         0      =   command line file
         1      =   "DO" command
        >1      =   nested "DO" command
*/

#define SCPE_DOFAILED   0040000                         /* fail in DO, not subproc */

t_stat do_cmd (int32 flag, char *fcptr)
{
    char *cptr, cbuf[CBUFSIZE], gbuf[CBUFSIZE], *c, quote, *do_arg[10];
    SMP_FILE *fpin;
    CTAB *cmdp;
    int32 echo = sim_do_echo, nargs, errabort;
    t_bool interactive, isdo, staying;
    t_stat stat;
    char *ocptr;

    stat = SCPE_OK;
    staying = TRUE;
    interactive = (flag > 0);                               /* issued interactively? */
    if (interactive)                                        /* get switches */
    {
        GET_SWITCHES (fcptr);
    }
    if (sim_switches & SWMASK ('V'))                        /* -v means echo */
        echo = 1;
    errabort = sim_switches & SWMASK ('E');                 /* -e means abort on error */

    c = fcptr;
    for (nargs = 0; nargs < 10; )                           /* extract arguments */
    {
        while (isspace (*c))                                /* skip blanks */
            c++;
        if (*c == 0)                                        /* all done? */
            do_arg [nargs++] = NULL;                        /* null argument */
        else
        {
            if (*c == '\'' || *c == '"')                    /* quoted string? */
                quote = *c++;
            else quote = 0;
            do_arg[nargs++] = c;                            /* save start */
            while (*c && (quote ? (*c != quote) : !isspace (*c)))
                c++;
            if (*c)                                         /* term at quote/spc */
                *c++ = 0;
        }
    }

    if (nargs <= 0 || do_arg[0] == NULL)                    /* need at least 1 */
        return SCPE_2FARG;
    if ((fpin = smp_fopen (do_arg[0], "r")) == NULL)        /* file failed to open? */
    {
        if (flag == 0)                                      /* cmd line file? */
             fprintf (smp_stderr, "Can't open file %s\n", do_arg[0]);
        if (flag > 1)
            return SCPE_OPENERR | SCPE_DOFAILED;            /* return failure with flag */
        else
            return SCPE_OPENERR;                            /* return failure */
    }
    if (flag < 1)                                           /* start at level 1 */
        flag = 1;

    do
    {
        stat = process_new_brk_actions (flag);              /* process break actions if pending in the CPUs */
        if (stat == SCPE_EXIT)
            return stat;

        ocptr = cptr = read_line (cbuf, CBUFSIZE, fpin);    /* get cmd line */
        sub_args (cbuf, gbuf, CBUFSIZE, do_arg);            /* substitute args */
        if (cptr == NULL)                                   /* EOF? */
        {
            stat = SCPE_OK;                                 /* set good return */
            break;
        }
        if (*cptr == 0)                                     /* ignore blank */
            continue;
        if (echo)                                           /* echo if -v */
            smp_printf("do> %s\n", cptr);
        if (echo && sim_log)
            fprintf (sim_log, "do> %s\n", cptr);
        cptr = get_glyph (cptr, gbuf, 0);                   /* get command glyph */
        sim_switches = 0;                                   /* init switches */
        isdo = FALSE;
        sim_do_echo = echo;
        if (cmdp = find_cmd (gbuf))                         /* lookup command */
        {
            isdo = (cmdp->action == &do_cmd);
            if (isdo)                                       /* DO command? */
            {
                if (flag >= MAX_DO_NEST_LVL)                /* nest too deep? */
                    stat = SCPE_NEST;
                else
                {
                    sim_dflt_rscx->cpu_unit = sim_dflt_cpu;
                    stat = do_cmd (flag + 1, cptr);         /* exec DO cmd */
                    sim_dflt_rscx->cpu_unit = sim_dflt_cpu;
                }
            }
            else
            {
                sim_dflt_rscx->cpu_unit = sim_dflt_cpu;
                stat = cmdp->action (cmdp->arg, cptr);      /* exec other cmd */
                sim_dflt_rscx->cpu_unit = sim_dflt_cpu;
            }
        }
        else
        {
            stat = SCPE_UNK;                                /* bad cmd given */
        }
        staying = (stat != SCPE_EXIT) &&                    /* decide if staying */
                  (stat != SCPE_AFAIL) &&
                  (!errabort || (stat < SCPE_BASE) || (stat == SCPE_STEP));

        if (stat != SCPE_OK || cmdp->action != &echo_cmd)
            sim_last_cmd_stat = stat;                       /* save command error status */

        if ((stat >= SCPE_BASE) && (stat != SCPE_EXIT) &&   /* error from cmd? */
            (stat != SCPE_STEP))
        {
            if (!echo && !sim_quiet &&                      /* report if not echoing */
                (!isdo || (stat & SCPE_DOFAILED)))          /* and not from DO return */
            {
                smp_printf("%s> %s\n", do_arg[0], ocptr);
                if (sim_log)
                    fprintf (sim_log, "%s> %s\n", do_arg[0], ocptr);
            }
            stat = stat & ~SCPE_DOFAILED;                   /* remove possible flag */
        }
        if ((staying || !interactive) &&                    /* report error if staying */
            (stat >= SCPE_BASE) && !isdo)                   /* or in cmdline file */
        {
            smp_printf ("%s\n", sim_error_text (stat));
            if (sim_log)
                fprintf (sim_log, "%s\n", sim_error_text (stat));
        }
        if (sim_vm_post != NULL)
            (*sim_vm_post) (TRUE);
    }
    while (staying);

    fclose (fpin);                                          /* close file */
    sim_do_echo = 0;

    return stat;
}

t_stat do_script (int32 flag, sim_cstream* script, const char* prompt)
{
    char *cptr, cbuf[CBUFSIZE], gbuf[CBUFSIZE];
    CTAB *cmdp;
    int32 echo, errabort;
    t_bool isdo, staying;
    t_stat stat;
    char *ocptr;

    stat = SCPE_OK;
    staying = TRUE;
    echo = sim_switches & SWMASK ('V');                     /* -v means echo */
    errabort = sim_switches & SWMASK ('E');                 /* -e means abort on error */

    do
    {
        stat = process_new_brk_actions (flag);              /* process break actions if pending in the CPUs */
        if (stat == SCPE_EXIT)
            return stat;

        ocptr = cptr = script->read_line (cbuf, CBUFSIZE);   /* get cmd line */
        if (cptr == NULL)                                    /* EOF? */
        {
            stat = SCPE_OK;                                 /* set good return */
            break;
        }
        if (*cptr == 0)                                     /* ignore blank */
            continue;
        if (echo)                                           /* echo if -v */
        {
            print_prompt (smp_stdout, prompt);
            smp_printf("%s\n", cptr);
        }
        if (echo && sim_log)
        {
            print_prompt (sim_log, prompt);
            fprintf(sim_log, "%s\n", cptr);
        }
        cptr = get_glyph (cptr, gbuf, 0);                   /* get command glyph */
        sim_switches = 0;                                   /* init switches */
        isdo = FALSE;
        if (cmdp = find_cmd (gbuf))                         /* lookup command */
        {
            isdo = (cmdp->action == &do_cmd);
            if (isdo)                                       /* DO command? */
            {
                if (flag >= MAX_DO_NEST_LVL)                /* nest too deep? */
                    stat = SCPE_NEST;
                else
                {
                    sim_dflt_rscx->cpu_unit = sim_dflt_cpu;
                    stat = do_cmd (flag + 1, cptr);         /* exec DO cmd */
                    sim_dflt_rscx->cpu_unit = sim_dflt_cpu;
                }
            }
            else
            {
                sim_dflt_rscx->cpu_unit = sim_dflt_cpu;
                stat = cmdp->action (cmdp->arg, cptr);      /* exec other cmd */
                sim_dflt_rscx->cpu_unit = sim_dflt_cpu;
            }
        }
        else
        {
            stat = SCPE_UNK;                                /* bad cmd given */
        }
        staying = (stat != SCPE_EXIT) &&                    /* decide if staying */
                  (stat != SCPE_AFAIL) &&
                  (!errabort || (stat < SCPE_BASE) || (stat == SCPE_STEP));
        if ((stat >= SCPE_BASE) && (stat != SCPE_EXIT) &&   /* error from cmd? */
            (stat != SCPE_STEP))
        {
            if (!echo && !sim_quiet &&                      /* report if not echoing */
                (!isdo || (stat & SCPE_DOFAILED)))          /* and not from DO return */
            {
                print_prompt (smp_stdout, prompt);
                smp_printf("%s\n", ocptr);
                if (sim_log)
                {
                    print_prompt (sim_log, prompt);
                    fprintf(sim_log, "%s\n", ocptr);
                }
            }
            stat = stat & ~SCPE_DOFAILED;                   /* remove possible flag */
        }
        if (stat >= SCPE_BASE)
        {
            smp_printf ("%s\n", sim_error_text (stat));
            if (sim_log)
                fprintf (sim_log, "%s\n", sim_error_text (stat));
        }
        if (sim_vm_post != NULL)
            (*sim_vm_post) (TRUE);
    }
    while (staying);

    return stat;
}

/* Substitute_args - replace %n tokens in 'instr' with the do command's arguments
                     and other enviroment variables

   Calling sequence
   instr        =       input string
   tmpbuf       =       temp buffer
   maxstr       =       min (len (instr), len (tmpbuf))
   do_arg[10]   =       arguments

   Token "%0" represents the command file name.

   The input sequence "\%" represents a literal "%", and "\\" represents a
   literal "\".  All other character combinations are rendered literally.

   Omitted parameters result in null-string substitutions.
*/

void sub_args (char *instr, char *tmpbuf, int32 maxstr, char *do_arg[])
{
char *ip, *op, *ap, *oend = tmpbuf + maxstr - 2;

for (ip = instr, op = tmpbuf; *ip && (op < oend); ) {
    if ((ip [0] == '\\') &&                             /* literal escape? */
        ((ip [1] == '%') || (ip [1] == '\\'))) {        /*   and followed by '%' or '\'? */
        ip++;                                           /* skip '\' */
        *op++ = *ip++;                                  /* copy escaped char */
        }
    else
        if (*ip == '%') {                               /* sub? */
            if ((ip[1] >= '0') && (ip[1] <= ('9'))) {   /* %n = sub */
                ap = do_arg[ip[1] - '0'];
                ip = ip + 2;
                }
            else {                                      /* environment variable */
                char gbuf[CBUFSIZE];

                ap = NULL;
                get_glyph_gen (ip+1, gbuf, '%', FALSE);
                ip += 1 + strlen (gbuf);
                if (*ip == '%') ++ip;
                ap = getenv(gbuf);
                if (!ap) {
                    static char rbuf[CBUFSIZE];
                    time_t now;
                    struct tm *tmnow;

                    time(&now);
                    tmnow = localtime(&now);
                    if (!strcmp ("DATE", gbuf)) {
                        sprintf (rbuf, "%4d/%02d/%02d", tmnow->tm_year+1900, tmnow->tm_mon+1, tmnow->tm_mday);
                        ap = rbuf;
                        }
                    else if (!strcmp ("TIME", gbuf)) {
                        sprintf (rbuf, "%02d:%02d:%02d", tmnow->tm_hour, tmnow->tm_min, tmnow->tm_sec);
                        ap = rbuf;
                        }
                    else if (!strcmp ("CTIME", gbuf)) {
                        strcpy (rbuf, ctime(&now));
                        rbuf[strlen (rbuf)-1] = '\0';    /* remove trailing \n */
                        ap = rbuf;
                        }
                    else if (!strcmp ("STATUS", gbuf)) {
                        sprintf (rbuf, "%08X", sim_last_cmd_stat);
                        ap = rbuf;
                        }
                    else if (!strcmp ("TSTATUS", gbuf)) {
                        sprintf (rbuf, "%s", sim_error_text (sim_last_cmd_stat));
                        ap = rbuf;
                        }
                    }
                }
            if (ap) {                                   /* non-null arg? */
                while (*ap && (op < oend))              /* copy the argument */
                    *op++ = *ap++;
                }
            }
        else
            *op++ = *ip++;                              /* literal character */
    }
*op = 0;                                                /* term buffer */
strcpy (instr, tmpbuf);
return;
}

/* Assert command
   
   Syntax: ASSERT {<dev>} <reg>{<logical-op><value>}<conditional-op><value>

   If <dev> is not specified, CPU is assumed.  <value> is expressed in the radix
   specified for <reg>.  <logical-op> and <conditional-op> are the same as that
   allowed for examine and deposit search specifications. */

t_stat assert_cmd (int32 flag, char *cptr)
{
char gbuf[CBUFSIZE], *gptr, *tptr;
REG *rptr;
uint32 idx;
t_value val;
t_stat r;

cptr = get_sim_opt (CMD_OPT_SW|CMD_OPT_DFT, cptr, &r);  /* get sw, default */
if (*cptr == 0)                                         /* must be more */
    return SCPE_2FARG;
cptr = get_glyph (cptr, gbuf, 0);                       /* get register */
rptr = find_reg (gbuf, &gptr, sim_dfdev);               /* parse register */
if (!rptr)                                              /* not there */
    return SCPE_NXREG;
if (*gptr == '[') {                                     /* subscript? */
    if (rptr->depth <= 1)                               /* array register? */
        return SCPE_ARG;
    idx = (uint32) strtotv (++gptr, &tptr, 10);         /* convert index */
    if ((gptr == tptr) || (*tptr++ != ']'))
        return SCPE_ARG;
    gptr = tptr;                                        /* update */
    }
else idx = 0;                                           /* not array */
if (idx >= rptr->depth)                                 /* validate subscript */
    return SCPE_SUB;
if (*gptr != 0)                                         /* more? must be search */
    get_glyph (gptr, gbuf, 0);
else {
    if (*cptr == 0)                                     /* must be more */
            return SCPE_2FARG;
    cptr = get_glyph (cptr, gbuf, 0);                   /* get search cond */
    }
if (*cptr != 0)                                         /* must be done */
    return SCPE_2MARG;
if (!get_search (gbuf, rptr->radix, &sim_stab))         /* parse condition */
    return SCPE_MISVAL;
val = get_rval (rptr, idx);                             /* get register value */
if (test_search (val, &sim_stab))                       /* test condition */
    return SCPE_OK;
return SCPE_AFAIL;                                      /* condition fails */
}

/* Set command */

static CTAB set_glob_tab[] = {
    { "CONSOLE", &sim_set_console, 0 },
    { "BREAK", &brk_cmd, SSH_ST },
    { "NOBREAK", &brk_cmd, SSH_CL },
    { "TELNET", &sim_set_telnet, 0 },                   /* deprecated */
    { "NOTELNET", &sim_set_notelnet, 0 },               /* deprecated */
    { "LOG", &sim_set_logon, 0 },                       /* deprecated */
    { "NOLOG", &sim_set_logoff, 0 },                    /* deprecated */
    { "DEBUG", &sim_set_debon, 0 },                     /* deprecated */
    { "NODEBUG", &sim_set_deboff, 0 },                  /* deprecated */
    { "THROTTLE", &sim_set_throt, 1 },
    { "NOTHROTTLE", &sim_set_throt, 0 },
    { "ASYNCH", &sim_set_asynch, 1 },
    { "NOASYNCH", &sim_set_asynch, 0 },
    { "ENV", &sim_set_environment, 1 },
    { NULL, NULL, 0 }
    };

static C1TAB set_dev_tab[] = {
    { "OCTAL", &set_dev_radix, 8 },
    { "DECIMAL", &set_dev_radix, 10 },
    { "HEX", &set_dev_radix, 16 },
    { "ENABLED", &set_dev_enbdis, 1 },
    { "DISABLED", &set_dev_enbdis, 0 },
    { "DEBUG", &set_dev_debug, 1 },
    { "NODEBUG", &set_dev_debug, 0 },
    { NULL, NULL, 0 }
    };

static C1TAB set_unit_tab[] = {
    { "ENABLED", &set_unit_enbdis, 1 },
    { "DISABLED", &set_unit_enbdis, 0 },
    { NULL, NULL, 0 }
    };


/* Set asynch/noasynch routine */

t_stat sim_set_asynch (int32 flag, char *cptr)
{
    if (cptr && (*cptr != 0))                               /* now eol? */
        return SCPE_2MARG;
    if (flag == sim_asynch_enabled)                         /* already set correctly? */
        return SCPE_OK;
    sim_asynch_enabled = flag;
    if (1)
    {
        uint32 i, j;
        DEVICE *dptr;
        UNIT *uptr;

        /* Call unit flush routines to report asynch status change to device layer */
        for (i = 1; (dptr = sim_devices[i]) != NULL; i++)   /* flush attached files */
        {
            for (j = 0; j < dptr->numunits; j++)            /* if not buffered in mem */
            {
                uptr = dptr->units[j];
                if ((uptr->flags & UNIT_ATT) &&             /* attached, */
                    !(uptr->flags & UNIT_BUF) &&            /* not buffered, */
                    (uptr->fileref))                        /* real file, */
                    if (uptr->io_flush)                     /* unit specific flush routine */
                        uptr->io_flush (uptr);
            }
        }
    }
    if (!sim_quiet)
        smp_printf ("Asynchronous I/O %sabled\n", sim_asynch_enabled ? "en" : "dis");
    if (sim_log)
        fprintf (sim_log, "Asynchronous I/O %sabled\n", sim_asynch_enabled ? "en" : "dis");
    return SCPE_OK;
}

/* Show asynch routine */

t_stat sim_show_asynch (SMP_FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, char *cptr)
{
    if (cptr && (*cptr != 0))
        return SCPE_2MARG;
    fprintf (st, "Asynchronous I/O is %sabled\n", (sim_asynch_enabled) ? "en" : "dis");
    return SCPE_OK;
}

/* Set environment routine */

t_stat sim_set_environment (int32 flag, char *cptr)
{
    char varname[CBUFSIZE];

    if ((!cptr) || (*cptr == 0))                            /* now eol? */
        return SCPE_2FARG;
    cptr = get_glyph_gen (cptr, varname, '=', FALSE);       /* get environment variable name */
    setenv(varname, cptr, 1);
    return SCPE_OK;
}

t_stat set_cmd (int32 flag, char *cptr)
{
    int32 lvl;
    t_stat r;
    char gbuf[CBUFSIZE], *cvptr, *svptr;
    DEVICE *dptr;
    UNIT *uptr;
    MTAB *mptr;
    CTAB *gcmdp;
    C1TAB *ctbr, *glbr;

    GET_SWITCHES (cptr);                                    /* get switches */
    if (*cptr == 0)                                         /* must be more */
        return SCPE_2FARG;
    cptr = get_glyph (cptr, gbuf, 0);                       /* get glob/dev/unit */

    if (dptr = find_dev (gbuf))                             /* device match? */
    {
        if (dptr == &cpu_dev && sim_dflt_cpu)
            uptr = sim_dflt_cpu;                            /* selected CPU unit */
        else
            uptr = dptr->units[0];                          /* unit 0 */
        ctbr = set_dev_tab;                                 /* global table */
        lvl = MTAB_VDV;                                     /* device match */
    }
    else if (dptr = find_unit (gbuf, &uptr))                /* unit match? */
    {
        if (uptr == NULL)                                   /* invalid unit */
            return SCPE_NXUN;
        ctbr = set_unit_tab;                                /* global table */
        lvl = MTAB_VUN;                                     /* unit match */
    }
    else if (gcmdp = find_ctab (set_glob_tab, gbuf))        /* global? */
        return gcmdp->action (gcmdp->arg, cptr);            /* do the rest */
    else return SCPE_NXDEV;                                 /* no match */
    if (*cptr == 0)                                         /* must be more */
        return SCPE_2FARG;

    check_select_cpu(uptr);

    while (*cptr != 0)                                      /* do all mods */
    {
        cptr = get_glyph (svptr = cptr, gbuf, ',');         /* get modifier */
        if (cvptr = strchr (gbuf, '='))                     /* = value? */
            *cvptr++ = 0;
        for (mptr = dptr->modifiers; mptr && mptr->mask != 0; mptr++)
        {
            if ((mptr->mstring) &&                          /* match string */
                (MATCH_CMD (gbuf, mptr->mstring) == 0))     /* matches option? */
            {
                if (mptr->mask & MTAB_XTD)                  /* extended? */
                {
                    if ((lvl & mptr->mask) == 0)
                        return SCPE_ARG;
                    if ((lvl & MTAB_VUN) && (uptr->flags & UNIT_DIS))
                        return SCPE_UDIS;                   /* unit disabled? */
                    if (mptr->valid)                        /* validation rtn? */
                    {
                        if (cvptr && (mptr->mask & MTAB_NC))
                        {
                            get_glyph_nc (svptr, gbuf, ',');
                            if (cvptr = strchr (gbuf, '='))
                                *cvptr++ = 0;
                        }
                        r = mptr->valid (uptr, mptr->match, cvptr, mptr->desc);
                        if (r != SCPE_OK)
                            return r;
                    }
                    else if (!mptr->desc)                   /* value desc? */
                        break;
    //                else if (mptr->mask & MTAB_VAL) {     /* take a value? */
    //                    if (!cvptr) return SCPE_MISVAL;   /* none? error */
    //                    r = dep_reg (0, cvptr, (REG *) mptr->desc, 0);
    //                    if (r != SCPE_OK) return r;
    //                    }
                    else if (cvptr)                         /* = value? */
                        return SCPE_ARG;
                    else 
                        *((int32 *) mptr->desc) = mptr->match;
                }                                           /* end if xtd */
                else                                        /* old style */
                {
                    if (cvptr)                              /* = value? */
                        return SCPE_ARG;
                    if (uptr->flags & UNIT_DIS)             /* disabled? */
                         return SCPE_UDIS;
                    if ((mptr->valid) &&                    /* invalid? */
                        ((r = mptr->valid (uptr, mptr->match, cvptr, mptr->desc)) != SCPE_OK))
                        return r;
                    uptr->flags = (uptr->flags & ~(mptr->mask)) |
                        (mptr->match & mptr->mask);         /* set new value */
                    if (uptr->is_cpu())
                        cpu_sync_flags((CPU_UNIT*) uptr);
                }                                           /* end else xtd */
                break;                                      /* terminate for */
            }                                               /* end if match */
        }                                                   /* end for */

        if (!mptr || (mptr->mask == 0))                     /* no match? */
        {
            if (glbr = find_c1tab (ctbr, gbuf))             /* global match? */
            {
                r = glbr->action (dptr, uptr, glbr->arg, cvptr);    /* do global */
                if (r != SCPE_OK)
                    return r;
            }
            else if (!dptr->modifiers)                      /* no modifiers? */
                return SCPE_NOPARAM;
            else 
                return SCPE_NXPAR;
        }                                                   /* end if no mat */
    }                                                       /* end while */
    return SCPE_OK;                                         /* done all */
}

/* Match CTAB/CTAB1 name */

CTAB *find_ctab (CTAB *tab, const char *gbuf)
{
for (; tab->name != NULL; tab++) {
    if (MATCH_CMD (gbuf, tab->name) == 0)
        return tab;
    }
return NULL;
}

C1TAB *find_c1tab (C1TAB *tab, const char *gbuf)
{
for (; tab->name != NULL; tab++) {
    if (MATCH_CMD (gbuf, tab->name) == 0)
        return tab;
    }
return NULL;
}

/* Set device data radix routine */

t_stat set_dev_radix (DEVICE *dptr, UNIT *uptr, int32 flag, char *cptr)
{
if (cptr)
    return SCPE_ARG;
dptr->dradix = flag & 037;
return SCPE_OK;
}

/* Set device enabled/disabled routine */

t_stat set_dev_enbdis (DEVICE *dptr, UNIT *uptr, int32 flag, char *cptr)
{
    UNIT *up;
    uint32 i;

    if (cptr)
        return SCPE_ARG;

    if ((dptr->flags & DEV_DISABLE) == 0)                   /* allowed? */
        return SCPE_NOFNC;

    if (dptr->flags & DEV_PERCPU)                           /* safety check */
        return SCPE_NOFNC;

    if (flag)                                               /* enable? */
    {
        if ((dptr->flags & DEV_DIS) == 0)                   /* already enb? ok */
            return SCPE_OK;
        dptr->flags = dptr->flags & ~DEV_DIS;               /* no, enable */
    }
    else
    {
        if (dptr->flags & DEV_DIS)                          /* already dsb? ok */
            return SCPE_OK;
        for (i = 0; i < dptr->numunits; i++)                /* check units */
        {
            up = dptr->units[i];                            /* att or active? */
            if ((up->flags & UNIT_ATT) || sim_is_active (up))
                return SCPE_NOFNC;                          /* can't do it */
        }
        dptr->flags = dptr->flags | DEV_DIS;                /* disable */
    }

    return reset_dev_allcpus (dptr);                        /* reset device */
}

/* Set unit enabled/disabled routine */

t_stat set_unit_enbdis (DEVICE *dptr, UNIT *uptr, int32 flag, char *cptr)
{
    if (cptr)
        return SCPE_ARG;
    if (!(uptr->flags & UNIT_DISABLE))                      /* allowed? */
        return SCPE_NOFNC;
    if (flag)                                               /* enb? enable */
        uptr->flags = uptr->flags & ~UNIT_DIS;
    else {
        if ((uptr->flags & UNIT_ATT) ||                     /* dsb */
            sim_is_active (uptr))                           /* more tests */
            return SCPE_NOFNC;
        uptr->flags = uptr->flags | UNIT_DIS;               /* disable */
        }
    return SCPE_OK;
}

/* Set device debug enabled/disabled routine */

t_stat set_dev_debug (DEVICE *dptr, UNIT *uptr, int32 flag, char *cptr)
{
char gbuf[CBUFSIZE];
DEBTAB *dep;

if ((dptr->flags & DEV_DEBUG) == 0)
    return SCPE_NOFNC;
if (cptr == NULL) {                                     /* no arguments? */
    dptr->dctrl = flag;                                 /* disable/enable w/o table */
    if (flag && dptr->debflags) {                       /* enable with table? */
        for (dep = dptr->debflags; dep->name != NULL; dep++)
            dptr->dctrl = dptr->dctrl | dep->mask;      /* set all */
        }
    return SCPE_OK;
    }
if (dptr->debflags == NULL)                             /* must have table */
    return SCPE_ARG;
while (*cptr) {
    cptr = get_glyph (cptr, gbuf, ';');                 /* get debug flag */
    for (dep = dptr->debflags; dep->name != NULL; dep++) {
        if (strcmp (dep->name, gbuf) == 0) {            /* match? */
            if (flag)
                dptr->dctrl = dptr->dctrl | dep->mask;
            else dptr->dctrl = dptr->dctrl & ~dep->mask;
            break;
            }
        }                                               /* end for */
    if (dep->mask == 0)                                 /* no match? */
        return SCPE_ARG;
    }                                                   /* end while */
return SCPE_OK;
}

/* Show command */

t_stat show_cmd (int32 flag, char *cptr)
{
t_stat r;

cptr = get_sim_opt (CMD_OPT_SW|CMD_OPT_OF, cptr, &r);   /* get sw, ofile */
if (!cptr)                                              /* error? */
    return r;
if (sim_ofile) {                                        /* output file? */
    r = show_cmd_fi (sim_ofile, flag, cptr);            /* do show */
    fclose (sim_ofile);
    }
else {
    r = show_cmd_fi (smp_stdout, flag, cptr);           /* no, stdout, log */
    if (sim_log)
        show_cmd_fi (sim_log, flag, cptr);
    }
return r;
}

static SHTAB show_glob_tab[] = {
    { "CONFIGURATION", &show_config, 0 },
    { "DEVICES", &show_config, 1 },
    { "QUEUE", &show_queue, 0 },
    { "TIME", &show_time, 0 },
    { "MODIFIERS", &show_mod_names, 0 },
    { "NAMES", &show_log_names, 0 },
    { "SHOW", &show_show_commands, 0 },
    { "VERSION", &show_version, 1 },
    { "CONSOLE", &sim_show_console, 0 },
    { "BREAK", &show_break, 0 },
    { "LOG", &sim_show_log, 0 },                        /* deprecated */
    { "TELNET", &sim_show_telnet, 0 },                  /* deprecated */
    { "DEBUG", &sim_show_debug, 0 },                    /* deprecated */
    { "THROTTLE", &sim_show_throt, 0 },
    { "ASYNCH", &sim_show_asynch, 0 },
    { NULL, NULL, 0 }
    };

static SHTAB show_dev_tab[] = {
    { "RADIX", &show_dev_radix, 0 },
    { "DEBUG", &show_dev_debug, 0 },
    { "MODIFIERS", &show_dev_modifiers, 0 },
    { "NAMES", &show_dev_logicals, 0 },
    { "SHOW", &show_dev_show_commands, 0 },
    { NULL, NULL, 0 }
    };

static SHTAB show_unit_tab[] = {
    { NULL, NULL, 0 }
    };


t_stat show_cmd_fi (SMP_FILE *ofile, int32 flag, char *cptr)
{
    int32 lvl;
    char gbuf[CBUFSIZE], *cvptr;
    DEVICE *dptr;
    UNIT *uptr;
    MTAB *mptr;
    SHTAB *shtb, *shptr;

    GET_SWITCHES (cptr);                                    /* get switches */
    if (*cptr == 0)                                         /* must be more */
        return SCPE_2FARG;
    cptr = get_glyph (cptr, gbuf, 0);                       /* get next glyph */
    if (shptr = find_shtab (show_glob_tab, gbuf))           /* global? */
        return shptr->action (ofile, NULL, NULL, shptr->arg, cptr);

    if (dptr = find_dev (gbuf))                             /* device match? */
    {
        if (dptr == &cpu_dev && sim_dflt_cpu)
            uptr = sim_dflt_cpu;                            /* selected CPU unit */
        else
            uptr = dptr->units[0];                          /* unit 0 */
        shtb = show_dev_tab;                                /* global table */
        lvl = MTAB_VDV;                                     /* device match */
    }
    else if (dptr = find_unit (gbuf, &uptr))                /* unit match? */
    {
        if (uptr == NULL)                                   /* invalid unit */
            return SCPE_NXUN;
        if (uptr->flags & UNIT_DIS)                         /* disabled? */
            return SCPE_UDIS;
        shtb = show_unit_tab;                               /* global table */
        lvl = MTAB_VUN;                                     /* unit match */
    }
    else
        return SCPE_NXDEV;                                  /* no match */

    if (*cptr == 0)                                         /* now eol? */
    {
        return (lvl == MTAB_VDV)?
            show_device (ofile, dptr, 0):
            show_unit (ofile, dptr, uptr, -1);
    }

    if (dptr->modifiers == NULL)                            /* any modifiers? */
        return SCPE_NOPARAM;

    check_select_cpu(uptr);

    while (*cptr != 0)                                      /* do all mods */
    {
        cptr = get_glyph (cptr, gbuf, ',');                 /* get modifier */
        if (cvptr = strchr (gbuf, '='))                     /* = value? */
            *cvptr++ = 0;
        for (mptr = dptr->modifiers; mptr->mask != 0; mptr++)
        {
            if (((mptr->mask & MTAB_XTD)?                   /* right level? */
                (mptr->mask & lvl): (MTAB_VUN & lvl)) && 
                ((mptr->disp && mptr->pstring &&            /* named disp? */
                (MATCH_CMD (gbuf, mptr->pstring) == 0))
     //           ||
     //           ((mptr->mask & MTAB_VAL) &&                 /* named value? */
     //           mptr->mstring &&
     //           (MATCH_CMD (gbuf, mptr->mstring) == 0)))
                ))
            {
                if (cvptr && !(mptr->mask & MTAB_SHP))
                    return SCPE_ARG;
                show_one_mod (ofile, dptr, uptr, mptr, cvptr, 1);
                break;
            }                                               /* end if */
        }                                                   /* end for */

        if (mptr->mask == 0)                                /* no match? */
        {
            if (shptr = find_shtab (shtb, gbuf))            /* global match? */
                shptr->action (ofile, dptr, uptr, shptr->arg, cptr);
            else
                return SCPE_ARG;
        }                                                   /* end if */
    }                                                       /* end while */
    return SCPE_OK;
}

SHTAB *find_shtab (SHTAB *tab, const char *gbuf)
{
    for (; tab->name != NULL; tab++) {
        if (MATCH_CMD (gbuf, tab->name) == 0)
            return tab;
        }
    return NULL;
}

/* Show device and unit */

t_stat show_device (SMP_FILE *st, DEVICE *dptr, int32 flag)
{
    uint32 j, udbl, ucnt;
    UNIT *uptr;

    fprintf (st, "%s", sim_dname (dptr));                   /* print dev name */
    if (qdisable (dptr)) {                                  /* disabled? */
        fprintf (st, ", disabled\n");
        return SCPE_OK;
        }
    for (j = ucnt = udbl = 0; j < dptr->numunits; j++) {    /* count units */
        uptr = dptr->units[j];
        if (uptr->flags & UNIT_DISABLE)
            udbl++;
        if (!(uptr->flags & UNIT_DIS))
            ucnt++;
        }
    check_select_cpu(dptr->units[0]);
    show_all_mods (st, dptr, dptr->units[0], MTAB_VDV);     /* show dev mods */
    if (dptr->numunits == 0)
        fprintf (st, "\n");
    else {
        if (udbl && (ucnt == 0))
            fprintf (st, ", all units disabled\n");
        else if (ucnt > 1)
            fprintf (st, ", %d units\n", ucnt);
        else if (flag)
            fprintf (st, "\n");
        }
    if (flag)                                               /* dev only? */
        return SCPE_OK;
    for (j = 0; j < dptr->numunits; j++) {                  /* loop thru units */
        uptr = dptr->units[j];
        if ((uptr->flags & UNIT_DIS) == 0)
            show_unit (st, dptr, uptr, ucnt);
        }
    return SCPE_OK;
}

t_stat show_unit (SMP_FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag)
{
    check_select_cpu(uptr);
    int32 u = sim_unit_index (uptr);

    if (flag > 1)
        fprintf (st, "  %s%d", sim_dname (dptr), u);
    else if (flag < 0)
        fprintf (st, "%s%d", sim_dname (dptr), u);
    if (uptr->flags & UNIT_FIX) {
        fprintf (st, ", ");
        fprint_capac (st, dptr, uptr);
        }
    if (uptr->flags & UNIT_ATT) {
        fprintf (st, ", attached to %s", uptr->filename);
        if (uptr->flags & UNIT_RO)
            fprintf (st, ", read only");
        }
    else if (uptr->flags & UNIT_ATTABLE)
        fprintf (st, ", not attached");
    show_all_mods (st, dptr, uptr, MTAB_VUN);               /* show unit mods */ 
    if (uptr->is_cpu())
    {
        CPU_UNIT* cpu_unit = (CPU_UNIT*) uptr;
        fprintf (st, ", %s", cpu_describe_state(cpu_unit));
    }
    fprintf (st, "\n");
    return SCPE_OK;
}

void fprint_capac (SMP_FILE *st, DEVICE *dptr, UNIT *uptr)
{
    t_addr kval = (uptr->flags & UNIT_BINK)? 1024: 1000;
    t_addr mval = kval * kval;
    t_addr psize = uptr->capac;
    char scale, width;

    if ((dptr->dwidth / dptr->aincr) > 8)
        width = 'W';
    else width = 'B';
    if (uptr->capac < (kval * 10))
        scale = 0;
    else if (uptr->capac < (mval * 10)) {
        scale = 'K';
        psize = psize / kval;
        }
    else {
        scale = 'M';
        psize = psize / mval;
        }
    fprint_val (st, (t_value) psize, 10, T_ADDR_W, PV_LEFT);
    if (scale)
        fputc (scale, st);
    fputc (width, st);
    return;
}

/* Show <global name> processors  */

t_stat show_version (SMP_FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, char *cptr)
{
    int32 vmaj = SIM_MAJOR, vmin = SIM_MINOR, vpat = SIM_PATCH, vdelt = SIM_DELTA;

    if (cptr && (*cptr != 0))
        return SCPE_2MARG;
    fprintf (st, "%s simulator V%d.%d-%d", sim_name, vmaj, vmin, vpat);
    if (vdelt)
        fprintf (st, "(%d)", vdelt);
    if (flag)
        fprintf (st, " [%s, %s, %s]", sim_si64, sim_sa64, sim_snet);
    fprintf (st, "\n");


#if defined(VM_VAX_MP)
    t_bool print_nl = FALSE;

# if defined(VSMP_REVISION)
    fprintf(st, "vSMP revision: %d", VSMP_REVISION);
    print_nl = TRUE;
# endif

    const char* ilock = "portable only";
# if SMP_NATIVE_INTERLOCKED
    ilock = use_native_interlocked ? "native/portable" : "portable/native";
# endif
    fprintf(st, "%sprimary interlock: %s",  print_nl ? ", " : "", ilock);
    print_nl = TRUE;

    if (print_nl)  fprintf(st, "\n");
#endif

    smp_show_thread_priority_info(st);

    return SCPE_OK;
}

t_stat show_config (SMP_FILE *st, DEVICE *dnotused, UNIT *unotused, int32 flag, char *cptr)
{
    int32 i;
    DEVICE *dptr;

    if (cptr && (*cptr != 0))
        return SCPE_2MARG;
    fprintf (st, "%s simulator configuration\n\n", sim_name);
    for (i = 0; (dptr = sim_devices[i]) != NULL; i++)
        show_device (st, dptr, flag);
    return SCPE_OK;
}

t_stat show_log_names (SMP_FILE *st, DEVICE *dnotused, UNIT *unotused, int32 flag, char *cptr)
{
    int32 i;
    DEVICE *dptr;

    if (cptr && (*cptr != 0))
        return SCPE_2MARG;
    for (i = 0; (dptr = sim_devices[i]) != NULL; i++)
        show_dev_logicals (st, dptr, NULL, 1, cptr);
    return SCPE_OK;
}

t_stat show_dev_logicals (SMP_FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, char *cptr)
{
    if (dptr->lname)
        fprintf (st, "%s -> %s\n", dptr->lname, dptr->name);
    else if (!flag)
        fputs ("no logical name assigned\n", st);
    return SCPE_OK;
}

t_stat show_queue (SMP_FILE *st, DEVICE *dnotused, UNIT *unotused, int32 flag, char *cptr)
{
    /*
     * Unlike SIMH 3.8 we do not display asynch IO queue here since it is flushed on Ctrl/E before
     * control is passed to the console, so when show_queue is invoked, asynch IO queue will be empty.
     */

    DEVICE *dptr;
    int32 accum;

    if (cptr && *cptr)
        return SCPE_2MARG;

    for (uint32 k = 0;  k < sim_ncpus;  k++)
    {
        CPU_UNIT* cpu_unit = cpu_units[k];
        clock_queue_entry* cqe;

        if (sim_ncpus == 1)
        {
            fprintf(st, "%s", sim_name);
        }
        else
        {
            fprintf(st, "CPU%d", cpu_unit->cpu_id);
        }

        if (!cpu_unit->is_running() && !cpu_unit->is_primary_cpu())
        {
            fprintf (st, " %s\n", cpu_describe_state(cpu_unit));
            continue;
        }

        if (cpu_unit->clock_queue == NULL)
        {
            fprintf (st, " event queue empty, time = %.0f\n", cpu_unit->sim_time);
            continue;
        }

        fprintf (st, " event queue status, time = %.0f\n", cpu_unit->sim_time);
        accum = 0;
        t_bool clk_printed = FALSE;

        for (cqe = cpu_unit->clock_queue; cqe != NULL; cqe = cqe->next)
        {
            if (cqe->clk_cosched && cpu_unit->clk_active && !clk_printed)
            {
                fprintf (st, "  CLK at next SYNCLK tick\n");
                clk_printed = TRUE;
            }

            if (cqe->uptr == &sim_throt_unit)
            {
                fprintf (st, "  Throttle timer");
            }
            else if ((dptr = find_dev_from_unit (cqe->uptr)) != NULL)
            {
                fprintf (st, "  %s", sim_dname (dptr));
                if (dptr->numunits > 1)  fprintf (st, " unit %d", sim_unit_index (cqe->uptr));
            }
            else
            {
                fprintf (st, "  Unknown");
            }

            // ToDo: sort by clk_cosched
            if (cqe->clk_cosched == 0)
                fprintf (st, " at %d", accum + cqe->time);
            else if (cqe->clk_cosched == 1)
                fprintf (st, " at next CLK tick");
            else
                fprintf (st, " after %d CLK ticks", cqe->clk_cosched);

            if (! (IS_PERCPU_UNIT(cqe->uptr) || cqe->uptr->clock_queue_cpu == cpu_unit))
            {
                if (cqe->uptr->clock_queue_cpu)
                    fprintf (st, " [invalidated by CPU%02d]", cqe->uptr->clock_queue_cpu->cpu_id);
                else
                    fprintf (st, " [invalidated by CPUxx]");
            }

            fprintf(st, "\n");
       
            accum = accum + cqe->time;
        }

        if (use_clock_thread && cpu_unit->clk_active && !clk_printed)
            fprintf (st, "  CLK at next SYNCLK tick\n");
    }

    return SCPE_OK;
}

t_stat show_time (SMP_FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, char *cptr)
{
    RUN_SCOPE;
    if (cptr && *cptr)
        return SCPE_2MARG;
    fprintf (st, "Time:\t%.0f\n", cpu_unit->sim_time);
    return SCPE_OK;
}

t_stat show_break (SMP_FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, char *cptr)
{
    t_stat r;

    if (cptr && (*cptr != 0))
        r = ssh_break (st, cptr, 1);  /* more? */
    else r = sim_brk_showall (st, sim_switches);
    return r;
}

t_stat show_dev_radix (SMP_FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, char *cptr)
{
    fprintf (st, "Radix=%d\n", dptr->dradix);
    return SCPE_OK;
}

t_stat show_dev_debug (SMP_FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, char *cptr)
{
int32 any = 0;
DEBTAB *dep;

if (dptr->flags & DEV_DEBUG) {
    if (dptr->dctrl == 0)
        fputs ("Debugging disabled", st);
    else if (dptr->debflags == NULL)
        fputs ("Debugging enabled", st);
    else {
        fputs ("Debug=", st);
        for (dep = dptr->debflags; dep->name != NULL; dep++) {
            if (dptr->dctrl & dep->mask) {
                if (any)
                    fputc (';', st);
                fputs (dep->name, st);
                any = 1;
                }
            }
        }
    fputc ('\n', st);
    return SCPE_OK;
    }
else return SCPE_NOFNC;
}

/* Show modifiers */

t_stat show_mod_names (SMP_FILE *st, DEVICE *dnotused, UNIT *unotused, int32 flag, char *cptr)
{
    int32 i;
    DEVICE *dptr;

    if (cptr && (*cptr != 0))                               /* now eol? */
        return SCPE_2MARG;
    for (i = 0; (dptr = sim_devices[i]) != NULL; i++) 
        show_dev_modifiers (st, dptr, NULL, flag, cptr);
    return SCPE_OK;
}

t_stat show_dev_modifiers (SMP_FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, char *cptr)
{
    int32 any, enb;
    MTAB *mptr;
    DEBTAB *dep;

    any = enb = 0;
    if (dptr->modifiers) {
        for (mptr = dptr->modifiers; mptr->mask != 0; mptr++) {
            if (mptr->mstring) {
                if (strcmp (mptr->mstring, "ENABLED") == 0)
                    enb = 1;
                if (any++)
                    fprintf (st, ", %s", mptr->mstring);
                else fprintf (st, "%s\t%s", sim_dname (dptr), mptr->mstring);
                }
            }
        }
    if (dptr->flags & DEV_DEBUG) {
        if (any++)
            fprintf (st, ", DEBUG, NODEBUG");
        else fprintf (st, "%s\tDEBUG, NODEBUG", sim_dname (dptr));
        }
    if (!enb && (dptr->flags & DEV_DISABLE)) {
        if (any++)
            fprintf (st, ", ENABLED, DISABLED");
        else fprintf (st, "%s\tENABLED, DISABLED", sim_dname (dptr));
        }
    if (any) fprintf (st, "\n");
    if ((dptr->flags & DEV_DEBUG) && dptr->debflags) {
        fprintf (st, "%s\tDEBUG=", sim_dname (dptr));
        for (dep = dptr->debflags; dep->name != NULL; dep++)
            fprintf (st, "%s%s", ((dep == dptr->debflags) ? "" : ";"), dep->name);
        fprintf (st, "\n");
        }
    return SCPE_OK;
}

t_stat show_all_mods (SMP_FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag)
{
    MTAB *mptr;

    if (dptr->modifiers == NULL)
        return SCPE_OK;
    for (mptr = dptr->modifiers; mptr->mask != 0; mptr++) {
        if (mptr->pstring && ((mptr->mask & MTAB_XTD)?
            ((mptr->mask & flag) && !(mptr->mask & MTAB_NMO)): 
            ((MTAB_VUN & flag) && ((uptr->flags & mptr->mask) == mptr->match)))) {
            fputs (", ", st);
            show_one_mod (st, dptr, uptr, mptr, NULL, 0);
            }
        }
    return SCPE_OK;
}

t_stat show_one_mod (SMP_FILE *st, DEVICE *dptr, UNIT *uptr, MTAB *mptr,
    char *cptr, int32 flag)
{
    //t_value val;

    if (mptr->disp)
        mptr->disp (st, uptr, mptr->match, cptr ? cptr: mptr->desc);
    //else if ((mptr->mask & MTAB_XTD) && (mptr->mask & MTAB_VAL)) {
    //    REG *rptr = (REG *) mptr->desc;
    //    fprintf (st, "%s=", mptr->pstring);
    //    val = get_rval (rptr, 0);
    //    fprint_val (st, val, rptr->radix, rptr->width,
    //        rptr->flags & REG_FMT);
    //    }
    else
        fputs (mptr->pstring, st);
    if (flag && !((mptr->mask & MTAB_XTD) && (mptr->mask & MTAB_NMO)))
        fputc ('\n', st);
    return SCPE_OK;
}

/* Show show commands */

t_stat show_show_commands (SMP_FILE *st, DEVICE *dnotused, UNIT *unotused, int32 flag, char *cptr)
{
    int32 i;
    DEVICE *dptr;

    if (cptr && (*cptr != 0))                               /* now eol? */
        return SCPE_2MARG;
    for (i = 0; (dptr = sim_devices[i]) != NULL; i++) 
        show_dev_show_commands (st, dptr, NULL, flag, cptr);
    return SCPE_OK;
}

t_stat show_dev_show_commands (SMP_FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, char *cptr)
{
    int32 any, enb;
    MTAB *mptr;

    any = enb = 0;
    if (dptr->modifiers) {
        for (mptr = dptr->modifiers; mptr->mask != 0; mptr++) {
            if ((!mptr->disp) || (!mptr->pstring))
                continue;
            if (any++)
                fprintf (st, ", %s", mptr->pstring);
            else fprintf (st, "sh{ow} %s\t%s", sim_dname (dptr), mptr->pstring);
            }
        }
    if (any)
        fprintf (st, "\n");
    return SCPE_OK;
}

static SHTAB cpu_cmd_tab[] = {
    { "MULTIPROCESSOR", &cpu_cmd_multiprocessor, 0 },
    { "ID", &cpu_cmd_id, 0 },
    { "SMT", &cpu_cmd_smt, 0 },
    { "INFO", &cpu_cmd_info, 0 },
    { NULL, NULL, 0 }
};

t_stat cpu_cmd (int32 flag, char *cptr)
{
    char gbuf[CBUFSIZE];
    SHTAB* shptr;

    // GET_SWITCHES (cptr);                                 /* get switches */

    if (*cptr == 0)                                         /* must be more */
        return SCPE_2FARG;
    cptr = get_glyph (cptr, gbuf, 0);                       /* get next glyph */

    if (shptr = find_shtab (cpu_cmd_tab, gbuf))
        return shptr->action (sim_ofile ? sim_ofile : smp_stdout, NULL, NULL, shptr->arg, cptr);

    return SCPE_ARG;
}

t_stat cpu_cmd_multiprocessor (SMP_FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, char *cptr)
{
    t_stat r;

    if (! (cptr && *cptr))
        return SCPE_2FARG;
    uint32 ncpus = (uint32) get_uint (cptr, 10, SIM_MAX_CPUS, &r);
    if (r != SCPE_OK)
        return SCPE_ARG;
    if (ncpus == sim_ncpus)  return SCPE_OK;
    if (ncpus < sim_ncpus || ncpus > SIM_MAX_CPUS)  return SCPE_ARG;

    /*
     * Check if SMT level is acceptable
     */
    if (smp_smt_factor_set)
    {
        smp_printf("Using override SMT (Hyper-Threading) slow-down factor %g\n", smp_smt_factor);
        if (sim_log)
            fprintf(sim_log, "Using override SMT (Hyper-Threading) slow-down factor %g\n", smp_smt_factor);
    }
    else if (smp_nsmt_per_core <= 0)
    {
        smp_printf("%s was unable to query the number of SMT (Hyper-Threading) units per core on the host system\n", sim_name);
        if (sim_log)
            fprintf(sim_log, "%s was unable to query the number of SMT (Hyper-Threading) units per core on the host system\n", sim_name);
        return SCPE_AFAIL;
    }
    else if (smp_nsmt_per_core > 2)
    {
        smp_printf("This version of %s is not designed for host systems with more than 2-way SMT (Hyper-Threading) per core\n", sim_name);
        if (sim_log)
            fprintf(sim_log, "This version of %s is not designed for host systems with more than 2-way SMT (Hyper-Threading) per core\n", sim_name);
        return SCPE_NOFNC;
    }

    if (ncpus > (uint32) smp_ncpus)
    {
        smp_printf("Warning!!! Number of configured virtual processors (%d) exceeds the number of\n", ncpus);
        smp_printf("           physical or logical processors on the host system (%d).\n", smp_ncpus);
        smp_printf("           Configured %s system may be unstable or not runnable.\n", sim_name);
        if (sim_log)
        {
            fprintf(sim_log, "Warning!!! Number of configured virtual processors (%d) exceeds the number of\n", ncpus);
            fprintf(sim_log, "           physical or logical processors on the host system (%d).\n", smp_ncpus);
            fprintf(sim_log, "           Configured %s system may be unstable or not runnable.\n", sim_name);
        }
    }

    if (cpu_stop_history ())
    {
        smp_printf("Warning: CPU MULTI[PROCESSOR] command disabled CPU HISTORY recording, reenable if required\n");
        if (sim_log)
            fprintf (sim_log, "Warning: CPU MULTI[PROCESSOR] command disabled CPU HISTORY recording, reenable if required\n");
    }

    /*
     * Check if process is able to execute in desired priority range.
     *
     * We must provide guest operating system with capability to properly calibrate its timing loops,
     * such as TENUSEC and UBADELAY based loops in case of VMS. This is essential even for the uniprocessor case
     * but is truly critical for SMP mode.
     *
     * Priority elevation is also essential for:
     *
     *    * lock holders preemtion avoidance and loss of efficiency due to busy-waiting while spinlock holder
     *      is pre-empted;
     *
     *    * similary, to avoid convoying problem if VM-critical lock holder thread is pre-empted;
     *
     *    * avoidance of pre-emption of VCPU thread while it is processing IPI request, and loss of efficiency
     *      while requesting processor is waiting for a response to IPI synchronous request;
     *
     *    * prompt and timely processing of clock interrupts to maintain satisfactory synchronicty in the system
     *      and also to avoid significant system time drifts.
     *
     */
    t_bool ok_prio = smp_set_thread_priority(SIMH_THREAD_PRIORITY_CPU_RUN) &&
                     smp_set_thread_priority(SIMH_THREAD_PRIORITY_CONSOLE_RUN) &&
                     smp_set_thread_priority(SIMH_THREAD_PRIORITY_CONSOLE_PAUSED) &&
                     smp_set_thread_priority(SIMH_THREAD_PRIORITY_CPU_CRITICAL_OS) &&
                     smp_set_thread_priority(SIMH_THREAD_PRIORITY_CPU_CRITICAL_OS_HI) &&
                     smp_set_thread_priority(SIMH_THREAD_PRIORITY_CPU_CRITICAL_VM) &&
                     smp_set_thread_priority(SIMH_THREAD_PRIORITY_CLOCK) &&
                     smp_set_thread_priority(SIMH_THREAD_PRIORITY_CPU_CALIBRATION) &&
                     smp_set_thread_priority(SIMH_THREAD_PRIORITY_CPU_RUN) &&
                     smp_set_thread_priority(SIMH_THREAD_PRIORITY_CONSOLE_RUN) &&
                     smp_set_thread_priority(SIMH_THREAD_PRIORITY_CONSOLE_PAUSED) &&
                     smp_set_thread_priority(SIMH_THREAD_PRIORITY_CPU_CRITICAL_OS) &&
                     smp_set_thread_priority(SIMH_THREAD_PRIORITY_CPU_CRITICAL_OS_HI) &&
                     smp_set_thread_priority(SIMH_THREAD_PRIORITY_CPU_CRITICAL_VM) &&
                     smp_set_thread_priority(SIMH_THREAD_PRIORITY_CLOCK) &&
                     smp_set_thread_priority(SIMH_THREAD_PRIORITY_CPU_CALIBRATION);
    smp_set_thread_priority(SIMH_THREAD_PRIORITY_CONSOLE_PAUSED);
    if (! ok_prio)
    {
        smp_printf("Warning!!! %s is unable to control thread priority. It is strongly recommened to configure host system\n", sim_name);
        smp_printf("           as described in %s manual, otherwise guest operating system may be unstable in SMP mode.\n", sim_name);
        if (sim_log)
        {
            fprintf(sim_log, "Warning!!! %s is unable to control thread priority. It is strongly recommened to configure host system\n", sim_name);
            fprintf(sim_log, "           as described in %s manual, otherwise guest operating system may be unstable in SMP mode.\n", sim_name);
        }
    }

    return cpu_create_cpus(ncpus) ? SCPE_OK : SCPE_AFAIL;
}

t_stat cpu_cmd_id (SMP_FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, char *cptr)
{
    if (cptr == NULL || *cptr == '\0')
    {
        smp_printf ("Current default CPU: cpu%d\n", sim_dflt_cpu->cpu_id);
        if (sim_log)
            fprintf (sim_log, "Current default CPU: cpu%d\n", sim_dflt_cpu->cpu_id);
    }
    else
    {
        t_stat r;
        uint32 ncpu = (uint32) get_uint (cptr, 10, sim_ncpus - 1, &r);
        if (r != SCPE_OK)
            return SCPE_ARG;
        if (ncpu >= sim_ncpus || cpu_units[ncpu] == NULL)  return SCPE_ARG;
        sim_dflt_rscx->cpu_unit = sim_dflt_cpu = cpu_units[ncpu];
    }

    return SCPE_OK;
}

t_stat cpu_cmd_smt (SMP_FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, char *cptr)
{
    if (cptr == NULL || *cptr == '\0')
    {
        if (smp_nsmt_per_core > 0)
        {
            smp_printf ("Detected SMT (Hyper-Threading) units per host processor core: %d\n", smp_nsmt_per_core);
            if (sim_log)
                fprintf (sim_log, "Detected SMT (Hyper-Threading) units per host processor core: %d\n", smp_nsmt_per_core);
        }
        else
        {
            smp_printf ("Detected SMT (Hyper-Threading) units per host processor core: unknown\n");
            if (sim_log)
                fprintf (sim_log, "Detected SMT (Hyper-Threading) units per host processor core: unknown\n");
        }

        if (smp_smt_factor_set)
        {
            smp_printf ("Override SMT (Hyper-Threading) slow-down factor: %g\n", smp_smt_factor);
            if (sim_log)
                fprintf (sim_log, "Override SMT (Hyper-Threading) slow-down factor: %g\n", smp_smt_factor);
        }
    }
    else
    {
        t_stat r = get_double(cptr, & smp_smt_factor);
        if (r != SCPE_OK)
            return r;
        smp_smt_factor_set = TRUE;
    }

    return SCPE_OK;
}

/* Breakpoint commands */

t_stat brk_cmd (int32 flg, char *cptr)
{
    GET_SWITCHES (cptr);                                    /* get switches */
    return ssh_break (NULL, cptr, flg);                     /* call common code */
}

t_stat ssh_break (SMP_FILE *st, char *cptr, int32 flg)
{
    if (sim_brk_types == 0) 
        return SCPE_NOFNC;
    DEVICE* dptr = sim_dflt_dev;
    if (dptr == NULL)
        return SCPE_IERR;
    UNIT* uptr = dptr->units[0];
    if (uptr == NULL)
        return SCPE_IERR;

    char gbuf[CBUFSIZE], *tptr, *t1ptr, *aptr;
    t_stat r;
    t_addr lo, hi, max = uptr->capac - 1;
    int32 cnt;

    if (aptr = strchr (cptr, ';'))                          /* ;action? */
    {
        if (flg != SSH_ST)                                  /* only on SET */
            return SCPE_ARG;
        *aptr++ = 0;                                        /* separate strings */
    }

    if (*cptr == 0)                                         /* no argument? */
    {
        lo = (t_addr) get_rval (sim_PC, 0);                 /* use PC */
        return ssh_break_one (st, flg, lo, 0, aptr);
    }

    while (*cptr)
    {
        cptr = get_glyph (cptr, gbuf, ',');
        tptr = get_range (dptr, gbuf, &lo, &hi, dptr->aradix, max, 0);
        if (tptr == NULL)
            return SCPE_ARG;
        if (*tptr == '[')
        {
            cnt = (int32) strtotv (tptr + 1, &t1ptr, 10);
            if ((tptr == t1ptr) || (*t1ptr != ']') || (flg != SSH_ST))
                return SCPE_ARG;
            tptr = t1ptr + 1;
        }
        else
        {
            cnt = 0;
        }
        if (*tptr != 0)
            return SCPE_ARG;
        if (lo == 0 && hi == max)
        {
            if (flg == SSH_CL)
                sim_brk_clrall (sim_switches);
            else if (flg == SSH_SH)
                sim_brk_showall (st, sim_switches);
            else return SCPE_ARG;
        }
        else
        {
            for ( ; lo <= hi; lo = lo + 1)
            {
                r = ssh_break_one (st, flg, lo, cnt, aptr);
                if (r != SCPE_OK)
                    return r;
            }
        }
    }

    return SCPE_OK;
}

t_stat ssh_break_one (SMP_FILE *st, int32 flg, t_addr lo, int32 cnt, char *aptr)
{
switch (flg) {

    case SSH_ST:
        return sim_brk_set (lo, sim_switches, cnt, aptr);
        break;

    case SSH_CL:
        return sim_brk_clr (lo, sim_switches);
        break;

    case SSH_SH:
        return sim_brk_show (st, lo, sim_switches);
        break;

    default:
        return SCPE_ARG;
    }
}

/* Reset command and routines

   re[set]              reset all devices
   re[set] all          reset all devices
   re[set] device       reset specific device
*/

#if defined (VM_VAX)
extern DEVICE qba_dev;
#endif

t_stat reset_cmd (int32 flag, char *cptr)
{
    RUN_SCOPE;
    char gbuf[CBUFSIZE];
    DEVICE *dptr;

    GET_SWITCHES (cptr);                                    /* get switches */
    if (*cptr == 0)                                         /* reset(cr) */
        return (reset_all (0));
    cptr = get_glyph (cptr, gbuf, 0);                       /* get next glyph */
    if (*cptr != 0)                                         /* now eol? */
        return SCPE_2MARG;
    if (strcmp (gbuf, "ALL") == 0)
        return (reset_all (0));
    dptr = find_dev (gbuf);                                 /* locate device */
    if (dptr == NULL)                                       /* found it? */
        return SCPE_NXDEV;

    if (dptr == &cpu_dev)
    {
        // if per-unit reset were provided, should call reset_all(0) for CPU0,
        // for any other CPUs should call reset_cpu_and_its_devices(cpu_unit)
        return reset_all(0);
    }
    else if (dptr->flags & DEV_PERCPU)                      /* allowed? */
    {
        return SCPE_NOFNC;
    }
#if defined (VM_VAX)
    // we could rather emit this error message directly from qba_reset,
    // but the only path it can "legally" happen starts here, so better catch it early
    else if (dptr == &qba_dev && !cpu_unit->is_primary_cpu())
    {
        smp_printf ("QBA can be reset only by the primary CPU (cpu0)\n");
        if (sim_log)
            fprintf (sim_log, "QBA can be reset only by the primary CPU (cpu0)\n");
        return SCPE_NOFNC;
    }
#endif
    else
    {
        return reset_dev_thiscpu (dptr);
    }
}

/* Reset devices start..end

   Inputs:
        start   =       number of starting device
   Outputs:
        status  =       error status
*/

t_stat reset_all (uint32 start)
{
    DEVICE *dptr;
    t_stat reason = SCPE_OK;

    for (uint32 i = 0; i < start; i++)
    {
        if (sim_devices[i] == NULL)
            return SCPE_IERR;
    }

    if (start == 0)
    {
        for (uint32 k = 0;  k < sim_ncpus;  k++)
        {
            reason = reset_cpu_and_its_devices (cpu_units[k]);
            if (reason != SCPE_OK)  break;
        }
    }
    else for (uint32 i = start; (dptr = sim_devices[i]) != NULL; i++)
    {
        if (dptr->flags & DEV_PERCPU)
            reason = reset_dev_allcpus (dptr);
        else
            reason = reset_dev_thiscpu (dptr);

        if (reason != SCPE_OK)
            break;
    }

    return reason;
}

/* Reset to powerup state

   Inputs:
        start   =       number of starting device
   Outputs:
        status  =       error status
*/

t_stat reset_all_p (uint32 start)
{
    t_stat r;
    int32 old_sw = sim_switches;

    sim_switches = SWMASK ('P');
    r = reset_all (start);
    sim_switches = old_sw;
    return r;
}

/* reset device, will reset per-CPU device only on the current CPU, not all CPUs */
t_stat reset_dev_thiscpu (DEVICE* dptr)
{
    if (dptr->reset != NULL)
        return dptr->reset (dptr);
    else
        return SCPE_OK;
}

/* reset device, will reset per-CPU device on all CPUs */
t_stat reset_dev_allcpus (DEVICE* dptr)
{
    t_stat res = SCPE_OK;

    if (dptr->reset != NULL)
    {
        if (dptr->flags & DEV_PERCPU)
        {
            run_scope_context* rscx = run_scope_context::get_current();
            CPU_UNIT* sv_cpu_unit = rscx->cpu_unit;

            for (uint32 k = 0;  k < sim_ncpus;  k++)
            {
                rscx->cpu_unit = cpu_units[k];
                t_stat rc = dptr->reset (dptr);
                if (rc != SCPE_OK && res == SCPE_OK)
                    res = rc;
            }

            rscx->cpu_unit = sv_cpu_unit;
        }
        else
        {
            res = dptr->reset (dptr);
        }
    }

    return res;
}


/* Load and dump commands

   lo[ad] filename {arg}        load specified file
   du[mp] filename {arg}        dump to specified file
*/

t_stat load_cmd (int32 flag, char *cptr)
{
    RUN_SCOPE;
    char gbuf[CBUFSIZE];
    SMP_FILE *loadfile;
    t_stat reason;

    GET_SWITCHES (cptr);                                    /* get switches */
    if (*cptr == 0)                                         /* must be more */
        return SCPE_2FARG;
    cptr = get_glyph_nc (cptr, gbuf, 0);                    /* get file name */
    loadfile = sim_fopen (gbuf, flag? "wb": "rb");          /* open for wr/rd */
    if (loadfile == NULL)
        return SCPE_OPENERR;
    GET_SWITCHES (cptr);                                    /* get switches */
    reason = sim_load (RUN_PASS, loadfile, cptr, gbuf, flag);         /* load or dump */
    fclose (loadfile);
    return reason;
}

/* Attach command

   at[tach] unit file   attach specified unit to file
*/

t_stat attach_cmd (int32 flag, char *cptr)
{
    char gbuf[CBUFSIZE];
    DEVICE *dptr;
    UNIT *uptr;
    t_stat r;

    GET_SWITCHES (cptr);                                    /* get switches */
    if (*cptr == 0)                                         /* must be more */
        return SCPE_2FARG;
    cptr = get_glyph (cptr, gbuf, 0);                       /* get next glyph */
    GET_SWITCHES (cptr);                                    /* get switches */
    if (*cptr == 0)                                         /* now eol? */
        return SCPE_2FARG;
    dptr = find_unit (gbuf, &uptr);                         /* locate unit */
    if (dptr == NULL)                                       /* found dev? */
        return SCPE_NXDEV;
    if (uptr == NULL)                                       /* valid unit? */
        return SCPE_NXUN;
    if (uptr->flags & UNIT_ATT) {                           /* already attached? */
        r = scp_detach_unit (dptr, uptr);                   /* detach it */
        if (r != SCPE_OK)                                   /* error? */
            return r;
        }
    sim_trim_endspc (cptr);                                 /* trim trailing spc */
    return scp_attach_unit (dptr, uptr, cptr);              /* attach */
}

/* Call device-specific or file-oriented attach unit routine */

t_stat scp_attach_unit (DEVICE *dptr, UNIT *uptr, char *cptr)
{
    if (dptr->attach != NULL)                               /* device routine? */
        return dptr->attach (uptr, cptr);                   /* call it */
    return attach_unit (uptr, cptr);                        /* no, std routine */
}

/* Attach unit to file */

t_stat attach_unit (UNIT *uptr, char *cptr)
{
    DEVICE *dptr;

    if (uptr->flags & UNIT_DIS)                             /* disabled? */
        return SCPE_UDIS;
    if (!(uptr->flags & UNIT_ATTABLE))                      /* not attachable? */
        return SCPE_NOATT;
    if ((dptr = find_dev_from_unit (uptr)) == NULL)
        return SCPE_NOATT;
    if (dptr->flags & DEV_RAWONLY)                          /* raw mode only? */
        return SCPE_NOFNC;
    uptr->filename = (char *) calloc (CBUFSIZE, sizeof (char)); /* alloc name buf */
    if (uptr->filename == NULL)
        return SCPE_MEM;
    strncpy (uptr->filename, cptr, CBUFSIZE);               /* save name */
    if (sim_switches & SWMASK ('R')) {                      /* read only? */
        if ((uptr->flags & UNIT_ROABLE) == 0)               /* allowed? */
            return attach_err (uptr, SCPE_NORO);            /* no, error */
        uptr->fileref = sim_fopen (cptr, "rb");             /* open rd only */
        if (uptr->fileref == NULL)                          /* open fail? */
            return attach_err (uptr, SCPE_OPENERR);         /* yes, error */
        uptr->flags = uptr->flags | UNIT_RO;                /* set rd only */
        if (!sim_quiet)
            smp_printf ("%s: unit is read only\n", sim_dname (dptr));
        }
    else {                                                  /* normal */
        uptr->fileref = sim_fopen (cptr, "rb+");            /* open r/w */
        if (uptr->fileref == NULL) {                        /* open fail? */
            if ((errno == EROFS) || (errno == EACCES)) {    /* read only? */
                if ((uptr->flags & UNIT_ROABLE) == 0)       /* allowed? */
                    return attach_err (uptr, SCPE_NORO);    /* no error */
                uptr->fileref = sim_fopen (cptr, "rb");     /* open rd only */
                if (uptr->fileref == NULL)                  /* open fail? */
                    return attach_err (uptr, SCPE_OPENERR); /* yes, error */
                uptr->flags = uptr->flags | UNIT_RO;        /* set rd only */
                if (!sim_quiet)
                    smp_printf ("%s: unit is read only\n", sim_dname (dptr));
                }
            else {                                          /* doesn't exist */
                if (sim_switches & SWMASK ('E'))            /* must exist? */
                    return attach_err (uptr, SCPE_OPENERR); /* yes, error */
                uptr->fileref = sim_fopen (cptr, "wb+");    /* open new file */
                if (uptr->fileref == NULL)                  /* open fail? */
                    return attach_err (uptr, SCPE_OPENERR); /* yes, error */
                if (!sim_quiet) smp_printf ("%s: creating new file\n", sim_dname (dptr));
                }
            }                                               /* end if null */
        }                                                   /* end else */
    if (uptr->flags & UNIT_BUFABLE) {                       /* buffer? */
        uint32 cap = ((uint32) uptr->capac) / dptr->aincr;  /* effective size */
        if (uptr->flags & UNIT_MUSTBUF)                     /* dyn alloc? */
            uptr->filebuf = calloc (cap, SZ_D (dptr));      /* allocate */
        if (uptr->filebuf == NULL)                          /* no buffer? */
            return attach_err (uptr, SCPE_MEM);             /* error */
        if (!sim_quiet) smp_printf ("%s: buffering file in memory\n", sim_dname (dptr));
        uptr->hwmark = (uint32) sim_fread (uptr->filebuf,    /* read file */
            SZ_D (dptr), cap, uptr->fileref);
        uptr->flags = uptr->flags | UNIT_BUF;               /* set buffered */
        }
    uptr->flags = uptr->flags | UNIT_ATT;
    uptr->pos = 0;
    return SCPE_OK;
}

t_stat attach_err (UNIT *uptr, t_stat stat)
{
    free (uptr->filename);
    uptr->filename = NULL;
    return stat;
}

/* Detach command

   det[ach] all         detach all units
   det[ach] unit        detach specified unit
*/

t_stat detach_cmd (int32 flag, char *cptr)
{
    char gbuf[CBUFSIZE];
    DEVICE *dptr;
    UNIT *uptr;

    GET_SWITCHES (cptr);                                    /* get switches */
    if (*cptr == 0)                                         /* must be more */
        return SCPE_2FARG;
    cptr = get_glyph (cptr, gbuf, 0);                       /* get next glyph */
    if (*cptr != 0)                                         /* now eol? */
        return SCPE_2MARG;
    if (strcmp (gbuf, "ALL") == 0)
        return (detach_all (0, FALSE));
    dptr = find_unit (gbuf, &uptr);                         /* locate unit */
    if (dptr == NULL)                                       /* found dev? */
        return SCPE_NXDEV;
    if (uptr == NULL)                                       /* valid unit? */
        return SCPE_NXUN;
    return scp_detach_unit (dptr, uptr);                    /* detach */
}

/* Detach devices start..end

   Inputs:
        start   =       number of starting device
        shutdown =      TRUE if simulator shutting down
   Outputs:
        status  =       error status

   Note that during shutdown, detach routines for non-attachable devices
   will be called.  These routines can implement simulator shutdown.  Error
   returns during shutdown are ignored.
*/

t_stat detach_all (int32 start, t_bool shutdown)
{
    uint32 i, j;
    DEVICE *dptr;
    UNIT *uptr;
    t_stat r;

    if ((start < 0) || (start > 1))
        return SCPE_IERR;
    for (i = start; (dptr = sim_devices[i]) != NULL; i++) { /* loop thru dev */
        for (j = 0; j < dptr->numunits; j++) {              /* loop thru units */
            uptr = dptr->units[j];
            if ((uptr->flags & UNIT_ATT) ||                 /* attached? */
                (shutdown && dptr->detach &&                /* shutdown, spec rtn, */
                !(uptr->flags & UNIT_ATTABLE))) {           /* !attachable? */
                r = scp_detach_unit (dptr, uptr);           /* detach unit */

                if ((r != SCPE_OK) && !shutdown)            /* error and not shutting down? */
                    return r;                               /* bail out now with error status */
                }
            }
        }
    return SCPE_OK;
}

/* Call device-specific or file-oriented detach unit routine */

t_stat scp_detach_unit (DEVICE *dptr, UNIT *uptr)
{
    if (dptr->detach != NULL)                               /* device routine? */
        return dptr->detach (uptr);
    return detach_unit (uptr);                              /* no, standard */
}

/* Detach unit from file */

t_stat detach_unit (UNIT *uptr)
{
    DEVICE *dptr;

    if (uptr == NULL)
        return SCPE_IERR;
    if (!(uptr->flags & UNIT_ATTABLE))                      /* attachable? */
        return SCPE_NOATT;
    if (!(uptr->flags & UNIT_ATT))                          /* attached? */
        return SCPE_OK;
    if ((dptr = find_dev_from_unit (uptr)) == NULL)
        return SCPE_OK;
    if (uptr->flags & UNIT_BUF) {
        uint32 cap = (uptr->hwmark + dptr->aincr - 1) / dptr->aincr;
        if (uptr->hwmark && ((uptr->flags & UNIT_RO) == 0)) {
            if (!sim_quiet)
                smp_printf ("%s: writing buffer to file\n", sim_dname (dptr));
            rewind (uptr->fileref);
            sim_fwrite (uptr->filebuf, SZ_D (dptr), cap, uptr->fileref);
            if (ferror (uptr->fileref))
                smp_perror ("I/O error");
            }
        if (uptr->flags & UNIT_MUSTBUF) {                   /* dyn alloc? */
            free (uptr->filebuf);                           /* free buf */
            uptr->filebuf = NULL;
            }
        uptr->flags = uptr->flags & ~UNIT_BUF;
        }
    uptr->flags = uptr->flags & ~(UNIT_ATT | UNIT_RO);
    free (uptr->filename);
    uptr->filename = NULL;
    if (fclose (uptr->fileref) == EOF)
        return SCPE_IOERR;
    return SCPE_OK;
}

/* Assign command

   as[sign] device name assign logical name to device
*/

t_stat assign_cmd (int32 flag, char *cptr)
{
    char gbuf[CBUFSIZE];
    DEVICE *dptr;

    GET_SWITCHES (cptr);                                    /* get switches */
    if (*cptr == 0)                                         /* must be more */
        return SCPE_2FARG;
    cptr = get_glyph (cptr, gbuf, 0);                       /* get next glyph */
    GET_SWITCHES (cptr);                                    /* get switches */
    if (*cptr == 0)                                         /* now eol? */
        return SCPE_2FARG;
    dptr = find_dev (gbuf);                                 /* locate device */
    if (dptr == NULL)                                       /* found dev? */
        return SCPE_NXDEV;
    cptr = get_glyph (cptr, gbuf, 0);                       /* get next glyph */
    if (*cptr != 0)                                         /* must be eol */
        return SCPE_2MARG;
    if (find_dev (gbuf))                                    /* name in use */
        return SCPE_ARG;
    deassign_device (dptr);                                 /* release current */
    return assign_device (dptr, gbuf);
}

t_stat assign_device (DEVICE *dptr, char *cptr)
{
    dptr->lname = (char *) calloc (CBUFSIZE, sizeof (char));
    if (dptr->lname == NULL)
        return SCPE_MEM;
    strncpy (dptr->lname, cptr, CBUFSIZE);
    return SCPE_OK;
}

/* Deassign command

   dea[ssign] device    deassign logical name
*/

t_stat deassign_cmd (int32 flag, char *cptr)
{
    char gbuf[CBUFSIZE];
    DEVICE *dptr;

    GET_SWITCHES (cptr);                                    /* get switches */
    if (*cptr == 0)                                         /* must be more */
        return SCPE_2FARG;
    cptr = get_glyph (cptr, gbuf, 0);                       /* get next glyph */
    if (*cptr != 0)                                         /* now eol? */
        return SCPE_2MARG;
    dptr = find_dev (gbuf);                                 /* locate device */
    if (dptr == NULL)                                       /* found dev? */
        return SCPE_NXDEV;
    return deassign_device (dptr);
}

t_stat deassign_device (DEVICE *dptr)
{
    if (dptr->lname)
        free (dptr->lname);
    dptr->lname = NULL;
    return SCPE_OK;
}

/* Get device display name */

const char *sim_dname (DEVICE *dptr)
{
    return (dptr->lname ? dptr->lname : dptr->name);
}

/* Save command

   sa[ve] filename              save state to specified file
*/

t_stat save_cmd (int32 flag, char *cptr)
{
#if 1
    smp_printf ("Save/Restore functionality is not implemented yet in VAX MP simulator\n");
    if (sim_log)
        fprintf (sim_log, "Save/Restore functionality is not implemented yet in VAX MP simulator\n");
    return SCPE_NOFNC;
#else
    SMP_FILE *sfile;
    t_stat r;
    GET_SWITCHES (cptr);                                    /* get switches */
    if (*cptr == 0)                                         /* must be more */
        return SCPE_2FARG;
    sim_trim_endspc (cptr);
    if ((sfile = sim_fopen (cptr, "wb")) == NULL)
        return SCPE_OPENERR;
    r = sim_save (sfile);
    fclose (sfile);
    return r;
#endif
}

t_stat sim_save (SMP_FILE *sfile)
{
// ToDo: reimplement for VAX MP, accounting for additional fields in UNIT, CPU_UNIT, 
//       multiple CPUs, CPU database and other global variables
RUN_SCOPE;
void *mbuf;
int32 l, t;
uint32 i, j;
t_addr k, high;
t_value val;
t_stat r;
t_bool zeroflg;
size_t sz;
DEVICE *dptr;
UNIT *uptr;
REG *rptr;

#define WRITE_I(xx) sim_fwrite (&(xx), sizeof (xx), 1, sfile)

fprintf (sfile, "%s\n%s\n%s\n%s\n%s\n%.0f\n",
    save_vercur,                                        /* [V2.5] save format */
    sim_name,                                           /* sim name */
    sim_si64, sim_sa64, sim_snet,                       /* [V3.5] options */
    cpu_unit->sim_time);                                /* [V3.2] sim time (ToDo) */
WRITE_I (cpu_unit->sim_rtime);                          /* [V2.6] sim rel time (ToDo) */

for (i = 0; (dptr = sim_devices[i]) != NULL; i++) {     /* loop thru devices */
    fputs (dptr->name, sfile);                          /* device name */
    fputc ('\n', sfile);
    if (dptr->lname)                                    /* [V3.0] logical name */
        fputs (dptr->lname, sfile);
    fputc ('\n', sfile);
    WRITE_I (dptr->flags);                              /* [V2.10] flags */
    for (j = 0; j < dptr->numunits; j++) {
        uptr = dptr->units[j];
        t = sim_is_active (uptr);                       /* ToDo: redo for SMP */
        WRITE_I (j);                                    /* unit number */
        WRITE_I (t);                                    /* activation time */
        WRITE_I (uptr->u3);                             /* unit specific */
        WRITE_I (uptr->u4);
        WRITE_I (uptr->u5);                             /* [V3.0] more unit */
        WRITE_I (uptr->u6);
        WRITE_I (uptr->flags);                          /* [V2.10] flags */
        WRITE_I (uptr->capac);                          /* [V3.5] capacity */
        if (uptr->flags & UNIT_ATT)
            fputs (uptr->filename, sfile);
        fputc ('\n', sfile);
        if (((uptr->flags & (UNIT_FIX + UNIT_ATTABLE)) == UNIT_FIX) &&
             (dptr->examine != NULL) &&
             ((high = uptr->capac) != 0)) {             /* memory-like unit? */
            WRITE_I (high);                             /* [V2.5] write size */
            sz = SZ_D (dptr);
            if ((mbuf = calloc (SRBSIZ, sz)) == NULL) {
                fclose (sfile);
                return SCPE_MEM;
                }
            for (k = 0; k < high; ) {                   /* loop thru mem */
                zeroflg = TRUE;
                for (l = 0; (l < SRBSIZ) && (k < high); l++,
                     k = k + (dptr->aincr)) {           /* check for 0 block */
                    r = dptr->examine (&val, k, uptr, SIM_SW_REST);
                    if (r != SCPE_OK)
                        return r;
                    if (val) zeroflg = FALSE;
                    SZ_STORE (sz, val, mbuf, l);
                    }                                   /* end for l */
                if (zeroflg) {                          /* all zero's? */
                    l = -l;                             /* invert block count */
                    WRITE_I (l);                        /* write only count */
                    }
                else {
                    WRITE_I (l);                        /* block count */
                    sim_fwrite (mbuf, sz, l, sfile);
                    }
                }                                       /* end for k */
            free (mbuf);                                /* dealloc buffer */
            }                                           /* end if mem */
        else {                                          /* no memory */
            high = 0;                                   /* write 0 */
            WRITE_I (high);
            }                                           /* end else mem */
        }                                               /* end unit loop */
    t = -1;                                             /* end units */
    WRITE_I (t);                                        /* write marker */
    for (rptr = dptr->registers; (rptr != NULL) &&      /* loop thru regs */
         (rptr->name != NULL); rptr++) {
        fputs (rptr->name, sfile);                      /* name */
        fputc ('\n', sfile);
        WRITE_I (rptr->depth);                          /* [V2.10] depth */
        for (j = 0; j < rptr->depth; j++) {             /* loop thru values */
            val = get_rval (rptr, j);                   /* get value */
            WRITE_I (val);                              /* store */
            }
        }
    fputc ('\n', sfile);                                /* end registers */
    }
fputc ('\n', sfile);                                    /* end devices */
return (ferror (sfile))? SCPE_IOERR: SCPE_OK;           /* error during save? */
}

/* Restore command

   re[store] filename           restore state from specified file
*/

t_stat restore_cmd (int32 flag, char *cptr)
{
#if 1
    smp_printf ("Save/Restore functionality is not implemented yet in VAX MP simulator\n");
    if (sim_log)
        fprintf (sim_log, "Save/Restore functionality is not implemented yet in VAX MP simulator\n");
    return SCPE_NOFNC;
#else
    SMP_FILE *rfile;
    t_stat r;

    GET_SWITCHES (cptr);                                    /* get switches */
    if (*cptr == 0)                                         /* must be more */
        return SCPE_2FARG;
    sim_trim_endspc (cptr);
    if ((rfile = sim_fopen (cptr, "rb")) == NULL)
        return SCPE_OPENERR;
    r = sim_rest (rfile);
    fclose (rfile);
    return r;
#endif
}

t_stat sim_rest (SMP_FILE *rfile)
{
// ToDo: reimplement for VAX MP, accounting for additional fields in UNIT, CPU_UNIT, 
//       multiple CPUs, CPU database and other global variables
RUN_SCOPE;
char buf[CBUFSIZE];
void *mbuf;
int32 j, blkcnt, limit, unitno, time, flg;
uint32 us, depth;
t_addr k, high, old_capac;
t_value val, mask;
t_stat r;
size_t sz;
t_bool v35, v32;
DEVICE *dptr;
UNIT *uptr;
REG *rptr;

#define READ_S(xx) if (read_line ((xx), CBUFSIZE, rfile) == NULL) \
    return SCPE_IOERR;
#define READ_I(xx) if (sim_fread (&xx, sizeof (xx), 1, rfile) == 0) \
    return SCPE_IOERR;

READ_S (buf);                                           /* [V2.5+] read version */
v35 = v32 = FALSE;
if (strcmp (buf, save_vercur) == 0)                     /* version 3.5? */
    v35 = v32 = TRUE;  
else if (strcmp (buf, save_ver32) == 0)                 /* version 3.2? */
    v32 = TRUE;
else if (strcmp (buf, save_ver30) != 0) {               /* version 3.0? */
    smp_printf ("Invalid file version: %s\n", buf);
    return SCPE_INCOMP;
    }
READ_S (buf);                                           /* read sim name */
if (strcmp (buf, sim_name)) {                           /* name match? */
    smp_printf ("Wrong system type: %s, not %s\n", buf, sim_name);
    return SCPE_INCOMP;
    }
if (v35) {                                              /* [V3.5+] options */
    READ_S (buf);                                       /* integer size */
    if (strcmp (buf, sim_si64) != 0) {
        smp_printf ("Incompatible integer size, save file = %s\n", buf);
        return SCPE_INCOMP;
        }
    READ_S (buf);                                       /* address size */
    if (strcmp (buf, sim_sa64) != 0) {
        smp_printf ("Incompatible address size, save file = %s\n", buf);
        return SCPE_INCOMP;
        }
    READ_S (buf);                                       /* Ethernet */
    }
if (v32) {                                              /* [V3.2+] time as string */
    READ_S (buf);
    sscanf (buf, "%lf", &cpu_unit->sim_time);
    }
else READ_I (cpu_unit->sim_time);                       /* sim time (ToDo)*/
READ_I (cpu_unit->sim_rtime);                           /* [V2.6+] sim rel time (ToDo) */

for ( ;; ) {                                            /* device loop */
    READ_S (buf);                                       /* read device name */
    if (buf[0] == 0)                                    /* last? */
        break;
    if ((dptr = find_dev (buf)) == NULL) {              /* locate device */
        smp_printf ("Invalid device name: %s\n", buf);
        return SCPE_INCOMP;
        }
    READ_S (buf);                                       /* [V3.0+] logical name */
    deassign_device (dptr);                             /* delete old name */
    if ((buf[0] != 0) && 
        ((r = assign_device (dptr, buf)) != SCPE_OK))
        return r;
    READ_I (flg);                                       /* [V2.10+] ctlr flags */
    if (!v32)
        flg = ((flg & DEV_UFMASK_31) << (DEV_V_UF - DEV_V_UF_31)) |
            (flg & ~DEV_UFMASK_31);                     /* [V3.2+] flags moved */
    dptr->flags = (dptr->flags & ~DEV_RFLAGS) |         /* restore ctlr flags */
         (flg & DEV_RFLAGS);
    for ( ;; ) {                                        /* unit loop */
        sim_switches = SIM_SW_REST;                     /* flag rstr, clr RO */
        READ_I (unitno);                                /* unit number */
        if (unitno < 0)                                 /* end units? */
            break;
        if ((uint32) unitno >= dptr->numunits) {        /* too big? */
            smp_printf ("Invalid unit number: %s%d\n", sim_dname (dptr), unitno);
            return SCPE_INCOMP;
            }
        READ_I (time);                                  /* event time */
        uptr = dptr->units[unitno];
        sim_cancel (uptr);
        if (time > 0)
            sim_activate (uptr, time - 1);
        READ_I (uptr->u3);                              /* device specific */
        READ_I (uptr->u4);
        READ_I (uptr->u5);                              /* [V3.0+] more dev spec */
        READ_I (uptr->u6);
        READ_I (flg);                                   /* [V2.10+] unit flags */
        old_capac = uptr->capac;                        /* save current capacity */
        if (v35) {                                      /* [V3.5+] capacity */
            READ_I (uptr->capac);
            }
        if (!v32)
            flg = ((flg & UNIT_UFMASK_31) << (UNIT_V_UF - UNIT_V_UF_31)) |
                (flg & ~UNIT_UFMASK_31);                /* [V3.2+] flags moved */
        uptr->flags = (uptr->flags & ~UNIT_RFLAGS) |
            (flg & UNIT_RFLAGS);                        /* restore */
        READ_S (buf);                                   /* attached file */
        if ((uptr->flags & UNIT_ATT) &&                 /* unit currently attached? */
            !(dptr->flags & DEV_NET)) {                 /*  and not a net device? */
            r = scp_detach_unit (dptr, uptr);           /* detach it */
            if (r != SCPE_OK)
                return r;
            }
        if ((buf[0] != '\0') &&                         /* unit to be reattached? */
            !(dptr->flags & DEV_NET) &&                 /*  and not a net device? */
            ((uptr->flags & UNIT_ATTABLE) ||            /*  and unit is attachable */
             (dptr->attach != NULL))) {                 /*    or VM attach routine provided? */
            uptr->flags = uptr->flags & ~UNIT_DIS;      /* ensure device is enabled */
            if (flg & UNIT_RO)                          /* [V2.10+] saved flgs & RO? */
                sim_switches |= SWMASK ('R');           /* RO attach */
            r = scp_attach_unit (dptr, uptr, buf);      /* reattach unit */
            if (r != SCPE_OK)
                return r;
            }
        READ_I (high);                                  /* memory capacity */
        if (high > 0) {                                 /* [V2.5+] any memory? */
            if (((uptr->flags & (UNIT_FIX + UNIT_ATTABLE)) != UNIT_FIX) ||
                 (dptr->deposit == NULL)) {
                smp_printf ("Can't restore memory: %s%d\n", sim_dname (dptr), unitno);
                return SCPE_INCOMP;
                }
            if (high != old_capac) {                    /* size change? */
                uptr->capac = old_capac;                /* temp restore old */
                if ((dptr->flags & DEV_DYNM) &&
                    ((dptr->msize == NULL) ||
                     (dptr->msize (uptr, (int32) high, NULL, NULL) != SCPE_OK))) {
                    smp_printf ("Can't change memory size: %s%d\n",
                        sim_dname (dptr), unitno);
                    return SCPE_INCOMP;
                    }
                uptr->capac = high;                     /* new memory size */
                smp_printf ("Memory size changed: %s%d = ", sim_dname (dptr), unitno);
                fprint_capac (smp_stdout, dptr, uptr);
                smp_printf ("\n");
                }
            sz = SZ_D (dptr);                           /* allocate buffer */
            if ((mbuf = calloc (SRBSIZ, sz)) == NULL)
                return SCPE_MEM;
            for (k = 0; k < high; ) {                   /* loop thru mem */
                READ_I (blkcnt);                        /* block count */
                if (blkcnt < 0)                         /* compressed? */
                    limit = -blkcnt;
                else limit = (int32) sim_fread (mbuf, sz, blkcnt, rfile);
                if (limit <= 0)                         /* invalid or err? */
                    return SCPE_IOERR;
                for (j = 0; j < limit; j++, k = k + (dptr->aincr)) {
                    if (blkcnt < 0)                     /* compressed? */
                        val = 0;
                    else SZ_LOAD (sz, val, mbuf, j);    /* saved value */
                    r = dptr->deposit (val, k, uptr, SIM_SW_REST);
                    if (r != SCPE_OK)
                        return r;
                    }                                   /* end for j */
                }                                       /* end for k */
            free (mbuf);                                /* dealloc buffer */
            }                                           /* end if high */
        }                                               /* end unit loop */
    for ( ;; ) {                                        /* register loop */
        READ_S (buf);                                   /* read reg name */
        if (buf[0] == 0)                                /* last? */
            break;
        READ_I (depth);                                 /* [V2.10+] depth */
        if ((rptr = find_reg (buf, NULL, dptr)) == NULL) {
            smp_printf ("Invalid register name: %s %s\n", sim_dname (dptr), buf);
            for (us = 0; us < depth; us++) {            /* skip values */
                READ_I (val);
                }
            continue;
            }
        if (depth != rptr->depth)                       /* [V2.10+] mismatch? */
            smp_printf ("Register depth mismatch: %s %s, file = %d, sim = %d\n",
                sim_dname (dptr), buf, depth, rptr->depth);
        mask = width_mask[rptr->width];                 /* get mask */
        for (us = 0; us < depth; us++) {                /* loop thru values */
            READ_I (val);                               /* read value */
            if (val > mask)                             /* value ok? */
                smp_printf ("Invalid register value: %s %s\n", sim_dname (dptr), buf);
            else if (us < rptr->depth)                  /* in range? */
                put_rval (rptr, us, val);
            }
        }
    }                                                   /* end device loop */
return SCPE_OK;
}

/* Run, go, cont, step commands

   ru[n] [new PC]       reset and start simulation
   go [new PC]          start simulation
   co[nt]               start simulation
   s[tep] [step limit]  start simulation for 'limit' instructions
   b[oot] device        bootstrap from device and start simulation
*/

t_stat run_cmd (int32 flag, char *cptr)
{
    RUN_SCOPE;
    char *tptr, gbuf[CBUFSIZE];
    uint32 i, j;
    int32 unitno;
    t_value pcv;
    t_stat r;
    DEVICE *dptr;
    UNIT *uptr;

    if (sim_brk_is_in_action ())
    {
        const char* verb;
        t_bool can = FALSE;

        switch (flag)
        {
        case RU_RUN:     verb = "RUN";  break;
        case RU_GO:      verb = "GO";  break;
        case RU_BOOT:    verb = "BOOT";  break;
        case RU_STEP:    verb = "STEP";  can = TRUE; break;
        case RU_CONT:    verb = "CONTINUE";  can = TRUE; break;
        default:         verb = "the";  can = TRUE; break;
        }

        if (! can)
        {
            smp_printf ("Cannot execute %s command from breakpoint action or script\n", verb);
            if (sim_log)
                fprintf (sim_log, "Cannot execute %s command from breakpoint action or script\n", verb);
            return SCPE_ARG;
        }
    }

    if (sim_ws_settings_changed)
        sim_ws_setup();

    GET_SWITCHES (cptr);                                    /* get switches */
    sim_step = 0;
    if (flag == RU_RUN || flag == RU_GO)                    /* run or go */
    {
        switch (cpu_unit->cpu_state)
        {
        case CPU_STATE_RUNNABLE:
        case CPU_STATE_RUNNING:
            break;
        default:
            smp_printf ("Cannot execute %s command since cpu%d is in %s state\n", flag == RU_RUN ? "RUN" : "GO", cpu_unit->cpu_id, cpu_describe_state(cpu_unit));
            if (sim_log)
                fprintf (sim_log, "Cannot execute %s command since cpu%d is in %s state\n", flag == RU_RUN ? "RUN" : "GO", cpu_unit->cpu_id, cpu_describe_state(cpu_unit));
            return SCPE_ARG;
        }

        if (*cptr != 0)                                     /* argument? */
        {
            cptr = get_glyph (cptr, gbuf, 0);               /* get next glyph */
            if (*cptr != 0)                                 /* should be end */
                return SCPE_2MARG;
            if (sim_vm_parse_addr)                          /* address parser? */
                pcv = sim_vm_parse_addr (sim_dflt_dev, gbuf, &tptr);
            else pcv = strtotv (gbuf, &tptr, sim_PC->radix);/* parse PC */
            if ((tptr == gbuf) || (*tptr != 0) ||           /* error? */
                (pcv > width_mask[sim_PC->width]))
                return SCPE_ARG;
            put_rval (sim_PC, 0, pcv);
        }

        if ((flag == RU_RUN) &&                             /* run? */
            ((r = run_boot_prep ()) != SCPE_OK))            /* reset sim */
        {
            return r;
        }
    }
    else if (flag == RU_STEP)                               /* step */
    {
        switch (cpu_unit->cpu_state)
        {
        case CPU_STATE_RUNNABLE:
        case CPU_STATE_RUNNING:
            break;
        default:
            smp_printf ("Cannot execute STEP command since cpu%d is in %s state\n", cpu_unit->cpu_id, cpu_describe_state(cpu_unit));
            if (sim_log)
                fprintf (sim_log, "Cannot execute STEP command since cpu%d is in %s state\n", cpu_unit->cpu_id, cpu_describe_state(cpu_unit));
            return SCPE_ARG;
        }

        if (*cptr != 0)                                      /* argument? */
        {
             cptr = get_glyph (cptr, gbuf, 0);               /* get next glyph */
             if (*cptr != 0)                                 /* should be end */
                 return SCPE_2MARG;
             sim_step = (int32) get_uint (gbuf, 10, INT_MAX, &r);
             if (r != SCPE_OK || sim_step <= 0)              /* error? */
                 return SCPE_ARG;
         }
         else
         {
             sim_step = 1;
         }
    }
    else if (flag == RU_BOOT)                               /* boot */
    {
        if (*cptr == 0)                                     /* must be more */
            return SCPE_2FARG;
        cptr = get_glyph (cptr, gbuf, 0);                   /* get next glyph */
        if (*cptr != 0)                                     /* should be end */
            return SCPE_2MARG;
        dptr = find_unit (gbuf, &uptr);                     /* locate unit */
        if (dptr == NULL)                                   /* found dev? */
            return SCPE_NXDEV;
        if (uptr == NULL)                                   /* valid unit? */
            return SCPE_NXUN;
        if (dptr->boot == NULL)                             /* can it boot? */
            return SCPE_NOFNC;
        if (uptr->flags & UNIT_DIS)                         /* disabled? */
            return SCPE_UDIS;
        if ((uptr->flags & UNIT_ATTABLE) &&                 /* if attable, att? */
            !(uptr->flags & UNIT_ATT))
            return SCPE_UNATT;
        unitno = sim_unit_index (uptr);                     /* recover unit# */
        if (uptr->is_cpu() && uptr != &cpu_unit_0)
        {
            smp_printf ("Only primary CPU (cpu0) can boot\n");
            if (sim_log)
                fprintf (sim_log, "Only primary CPU (cpu0) can boot\n");
            return SCPE_NOFNC;
        }
        if (uptr->is_cpu())                                 /* set as current cpu */
        {
            run_scope_context* rscx = run_scope_context::get_current();
            rscx->cpu_unit = cpu_unit = (CPU_UNIT*) uptr;                                    
        }
        if ((r = run_boot_prep ()) != SCPE_OK)              /* reset sim */
            return r;
        if ((r = dptr->boot (unitno, dptr)) != SCPE_OK)     /* boot device */
            return r;
    }
    else if (flag == RU_CONT)
    {
        if (sim_brk_is_in_action ())
        {
            smp_printf ("CONTINUE command inside breakpoint action: postponed till the end of action\n");
            if (sim_log)
                fprintf (sim_log, "CONTINUE command inside breakpoint action: postponed till the end of action\n");
            sim_brk_continue = TRUE;
            return SCPE_OK;
        }
    }
    else
    {
        return SCPE_IERR;
    }

    for (i = 1; (dptr = sim_devices[i]) != NULL; i++)       /* reposition all */
    {
        for (j = 0; j < dptr->numunits; j++)                /* seq devices */
        {
            uptr = dptr->units[j];
            if ((uptr->flags & (UNIT_ATT + UNIT_SEQ)) == (UNIT_ATT + UNIT_SEQ))
                sim_fseek (uptr->fileref, uptr->pos, SEEK_SET);
        }
    }

    if ((r = run_cmd_core (RUN_PASS, flag)) != SCPE_OK)
        return r;

    if (sim_log)                                            /* flush console log */
        fflush (sim_log);
    if (sim_deb)                                            /* flush debug log */
        fflush (sim_deb);
    for (i = 1; (dptr = sim_devices[i]) != NULL; i++)       /* flush attached files */
    {
        for (j = 0; j < dptr->numunits; j++)                /* if not buffered in mem */
        {
            uptr = dptr->units[j];
            if ((uptr->flags & UNIT_ATT) &&                 /* attached, */
                !(uptr->flags & UNIT_BUF) &&                /* not buffered, */
                (uptr->fileref))                            /* real file, */
            {
                if (uptr->io_flush)                         /* unit specific flush routine */
                {
                    uptr->io_flush (uptr);
                }
                else if (!(uptr->flags & UNIT_RAW) &&        /* not raw, */
                         !(uptr->flags & UNIT_RO))           /* not read only? */
                {
                    fflush (uptr->fileref);
                }
            }
        }
    }

    sim_async_process_io_events_for_console();

#if defined (VMS)
    smp_printf ("\n");
#endif
    fprint_stopped (smp_stdout);                            /* print msg */
    if (sim_log)                                            /* log if enabled */
        fprint_stopped (sim_log);
    return SCPE_OK;
}

/* Common setup for RUN or BOOT */

t_stat run_boot_prep (void)
{
    /* reset queue and time */
    for (uint32 cpu_ix = 0;  cpu_ix < sim_ncpus;  cpu_ix++)
    {
        CPU_UNIT* cpu_unit = cpu_units[cpu_ix];
        // sim_interval = 0;  // redundant: will be reset in all CPU units by reset_all
        cpu_unit->noqueue_time = 0;
        cpu_unit->sim_time = cpu_unit->sim_rtime = 0;
    }

    /* reset_all will also reset clock event queues on all CPUs */
    return reset_all (0);
}

static t_stat run_cmd_core (RUN_DECL, int32 runcmd)
{
    run_scope_context* rscx = run_scope_context::get_current();
    CPU_UNIT* current_cpu_unit = cpu_unit;
    t_stat r;

    stop_cpus = 0;
    hlt_pin = 0;

    if (use_clock_thread && !sim_clock_thread_created)
    {
        sim_try
        {
            smp_create_thread(sim_clock_thread_proc, NULL, & sim_clock_thread);
            sim_clock_thread_created = TRUE;
        }
        sim_catch (sim_exception_SimError, exc)
        {
            fprintf(smp_stderr, "\nFailed to create CLOCK thread: %s\n", exc->get_message());
            return SCPE_IERR;
        }
        sim_end_try
    }

    fflush(smp_stdout);                                     /* flush stdout */
    if (sim_log)                                            /* flush log if enabled */
        fflush (sim_log);

    if (signal (SIGINT, int_handler) == SIG_ERR)            /* set WRU */
    {
        return SCPE_SIGERR;
    }

#ifdef SIGHUP
    if (signal (SIGHUP, int_handler) == SIG_ERR)            /* set WRU */
        return SCPE_SIGERR;
#endif

    if (signal (SIGTERM, int_handler) == SIG_ERR)           /* set WRU */
        return SCPE_SIGERR;

    if (sim_ttrun () != SCPE_OK)                            /* set console mode */
    {
        sim_ttcmd ();
        return SCPE_TTYERR;
    }

    sim_ttrun_mode = TRUE;

    if ((r = sim_check_console (30)) != SCPE_OK)            /* check console, error? */
    {
        sim_ttcmd ();
        sim_ttrun_mode = FALSE;
        return r;
    }
    if ((r = build_dib_tab ()) != SCPE_OK)                  /* build, chk dib_tab */
    {
        sim_ttcmd ();
        sim_ttrun_mode = FALSE;
        return r;
    }

    /************************************************************
    *  set up VCPUs affinity                                    *
    ************************************************************/

    if (sim_vcpu_per_core)
    {
        if (smp_can_alloc_per_core(sim_ncpus))
        {
            sim_vcpu_affinity = SMP_AFFINITY_PER_CORE;
        }
        else
        {
            sim_vcpu_affinity = SMP_AFFINITY_ALL;
            sim_vcpu_per_core = FALSE;
            smp_printf("Warning!!! %s is unable to assign VCPUs to distinct PCPU cores.\n", sim_name);
            if (sim_log)
                fprintf(sim_log, "Warning!!! %s is unable to assign VCPUs to distinct PCPU cores.\n", sim_name);
        }
    }
    else
    {
        sim_vcpu_affinity = SMP_AFFINITY_ALL;
    }

    /************************************************************
    *  prepare to launch CPUs                                   *
    ************************************************************/

    cpu_unit->sim_step = (uint32) sim_step;                 /* set step counter */
    cpu_unit->sim_instrs = 0;                               /* ... */
    cpu_unit->cpu_state = CPU_STATE_RUNNING;                /* mark CPU state change */
    cpu_running_set.set(cpu_unit->cpu_id);                  /* ... */
    sim_mp_active_update();                                 /* ... */
    sim_brk_clract();                                       /* defang actions */
    syncw_resuming();                                       /* reset syncw wait data */

    for (uint32 cpu_ix = 0;  cpu_ix < sim_ncpus;  cpu_ix++)
    {
        rscx->cpu_unit = cpu_unit = cpu_units[cpu_ix];
        if (cpu_unit->cpu_state != CPU_STATE_RUNNING)  continue;
        if (sim_step && cpu_unit != current_cpu_unit)  continue;
        sim_throt_sched ();                                                          /* set throttle */
        sim_rtcn_init_all (RUN_PASS, runcmd == RU_CONT || runcmd == RU_STEP) ;       /* re-init clocks */
    }
    rscx->cpu_unit = cpu_unit = current_cpu_unit;

    cpu_attention->clear();
    cpu_pause_sync_barrier->set_count(sim_ncpus + 1 + (use_clock_thread ? 1 : 0));   /* all threads incl. self */
    smp_wmb();    // redundant before locking primitives, but let it be
    smp_set_thread_priority(SIMH_THREAD_PRIORITY_CONSOLE_RUN);

    /************************************************************
    *  launch CPUs and optional clock thread                    *
    ************************************************************/

    cpu_database_lock->lock();

    if (use_clock_thread)
        cpu_clock_run_gate->release(1);

    for (uint32 cpu_ix = 0;  cpu_ix < sim_ncpus;  cpu_ix++)
    {
        CPU_UNIT* xcpu = cpu_units[cpu_ix];
        if (xcpu->cpu_state != CPU_STATE_RUNNING)  continue;
        if (sim_step && xcpu != current_cpu_unit)  continue;
        xcpu->cpu_run_gate->release(1);
    }

    cpu_database_lock->unlock();

    /************************************************************
    *  run console loop during CPUs execution                   *
    ************************************************************/

    smp_pollable_synch_object* objs[2];
    objs[0] = cpu_attention;
    objs[1] = smp_pollable_console_keyboard::get();

    while (! weak_read(stop_cpus))
    {
        int wres = smp_wait_any(objs, 2, -1);

        if (wres == 1)           /* cpu_attention */
            break;

        if (wres == 2)           /* console keyboard */
        {
            for (;;)
            {
                r = sim_os_poll_kbd ();
                if (r == SCPE_OK)                      /* no keyboard input */
                {
                    break;
                }
                else if (r == SCPE_STOP)               /* Ctrl/E */
                {
                    stop_cpus = 1;
                    break;
                }
                else 
                {
                    sim_con_rcv_char ((int32) r);
                }
            }
        }
    }

    /************************************************************
    *  pause CPUs for pending console action                    *
    ************************************************************/

    stop_cpus = 1;
    smp_wmb();

    cpu_database_lock->lock();

    int nbarrier = 1;                                       /* count self */

    for (uint32 cpu_ix = 0;  cpu_ix < sim_ncpus;  cpu_ix++)
    {
        CPU_UNIT* xcpu = cpu_units[cpu_ix];
        if (xcpu->cpu_state != CPU_STATE_RUNNING)  continue;
        if (sim_step && xcpu != current_cpu_unit)  continue;
        wakeup_cpu(xcpu);
        /* 
         * Wakeup CPUs out of sync window sleep. Do not reset syncw.cpu[].waitset
         * nor xcpu->syncw_wait_cpu_id since they will be used by console commands
         * to display syncw state information.
         */
        xcpu->syncw_wait_event->set();
        nbarrier++;
    }

    cpu_pause_sync_barrier->set_count(nbarrier + (use_clock_thread ? 1 : 0));

    cpu_database_lock->unlock();
    cpu_pause_sync_barrier->wait();

    /************************************************************
    *  CPUs had been paused                                     *
    ************************************************************/
 
    smp_mb();                                               /* redundant after sync primitives, but let it be */
    
    cpu_unit->sim_step = 0;                                 /* reset step pending indicator */

    smp_set_thread_priority(SIMH_THREAD_PRIORITY_CONSOLE_PAUSED);

    sim_ttcmd ();                                           /* restore console */
    sim_ttrun_mode = FALSE;

    signal (SIGINT, SIG_DFL);                               /* cancel WRU */
#ifdef SIGHUP
    signal (SIGHUP, SIG_DFL);                               /* cancel WRU */
#endif
    signal (SIGTERM, SIG_DFL);                              /* cancel WRU */

    for (uint32 cpu_ix = 0;  cpu_ix < sim_ncpus;  cpu_ix++)
    {
        rscx->cpu_unit = cpu_unit = cpu_units[cpu_ix];
        if (cpu_unit->cpu_state != CPU_STATE_RUNNING)  continue;

        sim_throt_cancel ();                                /* cancel throttle */
        UPDATE_CPU_SIM_TIME();                              /* update sim time */
    }
    rscx->cpu_unit = cpu_unit = current_cpu_unit;

    // drain console keyboard typeahead
    while (sim_os_poll_kbd () != SCPE_OK) {}

    // clear pending tti typeahead
    tti_clear_pending_typeahead();

    return SCPE_OK;
}

SMP_THREAD_ROUTINE_DECL sim_cpu_work_thread_proc (void* arg)
{
    CPU_UNIT* cpu_unit = (CPU_UNIT*) arg;
    char tname[8];
    smp_affinity_kind_t affinity = SMP_AFFINITY_ALL;

    sim_try
    {
        smp_thread_init();

        run_scope_context* rscx = new run_scope_context(cpu_unit, SIM_THREAD_TYPE_CPU, cpu_unit->cpu_thread);
        rscx->thread_cpu_id = cpu_unit->cpu_id;
        rscx->set_current();

        if (cpu_unit->is_primary_cpu())
            cpu_set_thread_priority(RUN_PASS, SIMH_THREAD_PRIORITY_CPU_RUN);
        else
            cpu_set_thread_priority(RUN_PASS, SIMH_THREAD_PRIORITY_CPU_CRITICAL_OS_HI);

        sprintf(tname, "CPU%02d", cpu_unit->cpu_id);
        smp_set_thread_name(tname);

        for (;;)
        {
            cpu_unit->cpu_run_gate->wait();

            smp_rmb();                                      /* redundant after sync primitive, but let it be */

            if (affinity != sim_vcpu_affinity)
            {
                affinity = sim_vcpu_affinity;
                smp_set_affinity(cpu_unit->cpu_thread, affinity);
            }

            /*
             * reevaluate thread priority either on initial VCPU start to match VCPU state
             * or when resuming from console to reflect any changes done by console commands
             */
            cpu_unit->cpu_thread_priority = SIMH_THREAD_PRIORITY_INVALID;
            cpu_reevaluate_thread_priority(RUN_PASS);
            if (! must_control_prio())
            {
                /*
                 * If VCPU thread priority should not be controlled, always set it to CPU_RUN level.
                 * Priority is not controlled for performance reasons when executing in uniprocessor
                 * mode (only one VCPU is active) or when executing on a dedicated host machine.
                 *
                 * Note that even when priority is not controlled, it is still controlled for calibration purposes,
                 * i.e. thread priority is allowed to raise to CALIBRATION level and drop down from it.
                 * Therefore normally we should not knock down prority of a thread that is at CALIBRATION level, 
                 * however if VCPU thread was at CALIBRATION level and was interrupted by the console,
                 * pending calibration is as good as a dead fish, so no useful point trying to care about
                 * maintaining CALIBRATION level here. Therefore always set it CPU_RUN unconditionally.
                 */
                smp_set_thread_priority(SIMH_THREAD_PRIORITY_CPU_RUN);
            }

            /* check if should re-enter sync window sleep */
            cpu_unit->cpu_stop_code = syncw_checkinterval(RUN_PASS, TRUE);

            if (cpu_unit->cpu_stop_code == SCPE_OK)
                cpu_unit->cpu_stop_code = sim_instr(RUN_PASS);

            smp_wmb();                                      /* redundant before sync primitive, but let it be */

            t_bool join_console = TRUE;

            if (cpu_unit->is_primary_cpu() && (cpu_unit->cpu_stop_code == STOP_HALT || cpu_unit->cpu_stop_code == STOP_LOOP))
            {
                /*
                 * Halting primary processor because it executed HALT instructions or infinite non-interruptable loop.
                 * All secondaries should have been shut down by operating system by now.
                 * In case guest OS failed, perform emergency force-shutdown of secondaries.
                 */
                syncw_leave_all(RUN_PASS, SYNCW_OVERRIDE_ALL | SYNCW_DISABLE_CPU);
                cpu_stopping_ips_rate_update(RUN_PASS);
                cpu_shutdown_secondaries(RUN_PASS);
            }

            if (cpu_unit->is_secondary_cpu() && (cpu_unit->cpu_stop_code == STOP_HALT || cpu_unit->cpu_stop_code == STOP_LOOP))
            {
                /*
                 * Halting secondary processor because it executed HALT instructions or infinite non-interruptable loop
                 */
                cpu_database_lock->lock();

                smp_rmb();                                  /* redundant after locking primitive, but let it be */

                if (stop_cpus)
                {
                    /*
                     * Pause-to-console request is pending.
                     *
                     * Cannot stop the CPU right now because console thread already expects this CPU thread to join the barrier.
                     * Will join the barrier and stop CPU later, when instruction that caused stop condition 
                     * (HALT or non-interruptable infinite branch to self) will be re-executed when this CPU is resumed.
                     */
                    if (cpu_unit->cpu_stop_code == STOP_HALT)
                        cpu_backstep_pc(RUN_PASS);

                    cpu_unit->cpu_stop_code = SCPE_STOP;
                }
                else
                {
                    join_console = FALSE;
                    cpu_unit->cpu_state = CPU_STATE_STANDBY;
                    cpu_running_set.clear(cpu_unit->cpu_id);
                    sim_mp_active_update();
                    syncw_leave_all(RUN_PASS, SYNCW_OVERRIDE_ALL | SYNCW_DISABLE_CPU);
                    cpu_stopping_ips_rate_update(RUN_PASS);

                    /* update sim time */
                    UPDATE_CPU_SIM_TIME();

                    /*
                     * If there are events for system-wide devices pending in this CPU's event queue,
                     * ask the primary CPU to transfer them over to the primary CPU's queue.
                     */
                    if (sim_cpu_has_syswide_events(RUN_PASS))
                        cpu_unit->cpu_requeue_syswide_pending = TRUE;

                    /* notify the primary that secondary is shutting down */
                    interrupt_set_int(&cpu_unit_0, IPL_SECEXIT, INT_V_SECEXIT);
                }

                cpu_database_lock->unlock();

                if (! join_console)
                    cpu_set_thread_priority(RUN_PASS, SIMH_THREAD_PRIORITY_CPU_CRITICAL_OS_HI);
            }

            if (join_console)
            {
                cpu_attention->release();
                cpu_pause_sync_barrier->wait();
            }
        }
    }
    sim_catch (sim_exception_SimError, exc)
    {
        fprintf(smp_stderr, "\nFatal error in %s simulator, unexpected exception while executing CPU%d\n", sim_name, cpu_unit->cpu_id);
        fprintf(smp_stderr, "Exception cause: %s\n", exc->get_message());
        fprintf(smp_stderr, "Terminating the simulator abnormally...\n");
        exit(1);
    }
    sim_end_try

#if defined(USE_C_TRY_CATCH)
    /* relax compiler false warning */
    SMP_THREAD_ROUTINE_END;
#endif
}

SMP_THREAD_ROUTINE_DECL sim_clock_thread_proc (void* arg)
{
    sim_try
    {
        smp_thread_init();

        run_scope_context* rscx = new run_scope_context(NULL, SIM_THREAD_TYPE_CLOCK, sim_clock_thread);
        rscx->set_current();

        smp_set_thread_priority(SIMH_THREAD_PRIORITY_CLOCK);
        smp_set_thread_name("CLOCK");

        uint32 ms = 1000 / clk_tps;
        cpu_set synclk_set;

        for (;;)
        {
            cpu_clock_run_gate->wait();
            for (;;)
            {
                /* sleep one tick */
                sim_os_ms_sleep(ms);

                /* broadcast clock strobe interrupts (SYNCLK) */
                cpu_database_lock->lock();
                synclk_set = cpu_running_set;
                cpu_database_lock->unlock();

                for (uint32 ix = 0;  ix < sim_ncpus;  ix++)
                {
                    if (synclk_set.is_set(ix))
                    {
                        interrupt_set_int(cpu_units[ix], IPL_SYNCLK, INT_V_SYNCLK);
                    }
                }

                if (weak_read(stop_cpus)) break;
            }
            cpu_pause_sync_barrier->wait();
        }
    }
    sim_catch (sim_exception_SimError, exc)
    {
        fprintf(smp_stderr, "\nFatal error in %s simulator, unexpected exception while executing clock thread\n", sim_name);
        fprintf(smp_stderr, "Exception cause: %s\n", exc->get_message());
        fprintf(smp_stderr, "Terminating the simulator abnormally...\n");
        exit(1);
    }
    sim_end_try

#if defined(USE_C_TRY_CATCH)
    /* relax compiler false warning */
    SMP_THREAD_ROUTINE_END;
#endif
}

SIM_INLINE static t_bool cpu_should_actually_change_thread_priority(RUN_DECL, sim_thread_priority_t prio, sim_thread_priority_t* actprio)
{
    if (must_control_prio())
        return TRUE;

    /*
     * If thread priority control is disabled, let through only calls that raise to CALIBRATION level
     * and drop down from it
     */

    if (unlikely(prio == SIMH_THREAD_PRIORITY_CPU_CALIBRATION))
    {
        /* allow to raise to CALIBRATION */
        return TRUE;
    }

    if (cpu_unit->cpu_thread_priority == SIMH_THREAD_PRIORITY_CPU_CALIBRATION && prio < cpu_unit->cpu_thread_priority)
    {
        /* allow to drop down from CALIBRATION -- drop down always to RUN */
        *actprio = SIMH_THREAD_PRIORITY_CPU_RUN;
        return TRUE;
    }

    if (cpu_unit->cpu_thread_priority == SIMH_THREAD_PRIORITY_INVALID)
    {
        /* current priority is unknown -- always force to RUN */
        *actprio = SIMH_THREAD_PRIORITY_CPU_RUN;
        return TRUE;
    }

    return FALSE;
}

void cpu_set_thread_priority(RUN_DECL, sim_thread_priority_t prio)
{
    if (cpu_unit->cpu_thread_priority != prio)
    {
        sim_thread_priority_t actprio = prio;

        if (cpu_should_actually_change_thread_priority(RUN_PASS, prio, &actprio))
        {
            cpu_unit->cpu_thread_priority = prio;
            smp_set_thread_priority(actprio);
        }
        else
        {
            cpu_unit->cpu_thread_priority = prio;
        }
    }
}

void cpu_set_thread_priority(RUN_RSCX_DECL, sim_thread_priority_t prio)
{
    if (cpu_unit->cpu_id == rscx->thread_cpu_id &&
        rscx->thread_type == SIM_THREAD_TYPE_CPU &&
        cpu_unit->cpu_thread_priority != prio)
    {
        sim_thread_priority_t actprio = prio;

        if (cpu_should_actually_change_thread_priority(RUN_PASS, prio, &actprio))
        {
            cpu_unit->cpu_thread_priority = prio;
            smp_set_thread_priority(actprio);
        }
        else
        {
            cpu_unit->cpu_thread_priority = prio;
        }
    }
}


/* Print stopped message */

void fprint_stopped (SMP_FILE *st)
{
    fprint_stopped_gen (st, sim_PC, sim_dflt_dev);
}

const char* sim_stop_code_message(t_stat stop_code)
{
    if (stop_code >= SCPE_BASE)
        return sim_error_text (stop_code);
    else
        return sim_stop_messages[stop_code];
}

void fprint_stopped_gen (SMP_FILE *st, REG *pc, DEVICE *dptr)
{
    RUN_SCOPE_RSCX;
    const char* msg;

    /* handle "STEP" command case */
    if (sim_step && cpu_unit->cpu_stop_code == SCPE_STEP)
    {
        /* if step = 1 (common case), prevailing chances are that nothing had been output
           to console by the command, so no need for newline  */
        if (sim_step != 1)
            fprintf (st, "\n");

        msg = (sim_step == 1) ? NULL : sim_stop_code_message(cpu_unit->cpu_stop_code);
        fprint_stopped_instr(RUN_PASS, st, msg, pc, dptr);
        return;
    }

    CPU_UNIT* sv_cpu_unit = rscx->cpu_unit;
    t_stat prev_stop_code = SCPE_OK;                        /* init to suppress false GCC warning */
    t_bool printed = FALSE;

    fprintf (st, "\n");

    for (uint32 cpu_ix = 0;  cpu_ix < sim_ncpus;  cpu_ix++)
    {
        rscx->cpu_unit = cpu_unit = cpu_units[cpu_ix];

        if (cpu_unit->cpu_state != CPU_STATE_RUNNABLE &&
            cpu_unit->cpu_state != CPU_STATE_RUNNING)
        {
            msg = NULL;
            fprint_stopped_instr_or_state(RUN_PASS, st, msg, pc, dptr);
            printed = TRUE;
            continue;
        }

        t_stat stop_code = cpu_unit->cpu_stop_code;
        msg = sim_stop_code_message(stop_code);

        if (stop_code == SCPE_STOP || stop_code == SCPE_SIGERR)
        {
            if (printed && stop_code == prev_stop_code)
                msg = NULL;
        }

        fprint_stopped_instr_or_state(RUN_PASS, st, msg, pc, dptr);
        prev_stop_code = stop_code;
        printed = TRUE;
    }

    /* if nothing had been printed, print for the primary VCPU */
    if (! printed)
    {
        rscx->cpu_unit = cpu_unit = & cpu_unit_0;
        msg = sim_stop_code_message(cpu_unit->cpu_stop_code);
        fprint_stopped_instr_or_state (RUN_PASS, st, msg, pc, dptr);
    }

    rscx->cpu_unit = sv_cpu_unit;
}

void fprint_stopped_instr_or_state (RUN_DECL, SMP_FILE *st, const char* msg, REG *pc, DEVICE *dptr)
{
    if (cpu_unit->cpu_state == CPU_STATE_RUNNABLE ||
        cpu_unit->cpu_state == CPU_STATE_RUNNING)
    {
        fprint_stopped_instr (RUN_PASS, st, msg, pc, dptr);
    }
    else
    {
        if (msg != NULL)
            fprintf (st, "%s\n", msg);

        if (sim_ncpus != 1)
            fprintf (st, "CPU %02d ", cpu_unit->cpu_id);

        fprintf (st, "%s\n", cpu_describe_state(cpu_unit));
    }
}

void fprint_stopped_instr (RUN_DECL, SMP_FILE *st, const char* msg, REG *pc, DEVICE *dptr)
{
    int32 i;
    t_stat r = 0;
    t_addr k;

    if (msg != NULL)
        fprintf (st, "%s\n", msg);

    if (sim_ncpus != 1)
        fprintf (st, "CPU %02d ", cpu_unit->cpu_id);

    fprintf (st, "%s: ", pc->name);

    t_value pcval = get_rval (pc, 0);

    if (sim_vm_fprint_addr)
        sim_vm_fprint_addr (st, dptr, (t_addr) pcval);
    else
        fprint_val (st, pcval, pc->radix, pc->width, pc->flags & REG_FMT);

    if (dptr != NULL && dptr->examine != NULL)
    {
        for (i = 0; i < sim_emax; i++)
            sim_eval[i] = 0;

        for (i = 0, k = (t_addr) pcval; i < sim_emax; i++, k = k + dptr->aincr)
        {
            if ((r = dptr->examine (&sim_eval[i], k, dptr == &cpu_dev ? cpu_unit : dptr->units[0], SWMASK ('V'))) != SCPE_OK)
                break;
        }

        if (r == SCPE_OK || i > 0)
        {
            fprintf (st, "  ");
            if (fprint_sym (st, (t_addr) pcval, sim_eval, NULL, SWMASK('M')|SIM_SW_STOP) > 0)
                fprint_val (st, sim_eval[0], dptr->dradix, dptr->dwidth, PV_RZRO);
        }
    }

    fprintf (st, "\n");
    
}

/* Signal handler for ^C signal - set stop simulation flag */

void int_handler (int sig)
{
    stop_cpus = 1;
}

/* Examine/deposit commands

   ex[amine] [modifiers] list           examine
   de[posit] [modifiers] list val       deposit
   ie[xamine] [modifiers] list          interactive examine
   id[eposit] [modifiers] list          interactive deposit

   modifiers
        @filename                       output file
        -letter(s)                      switches
        devname'n                       device name and unit number
        [{&|^}value]{=|==|!|!=|>|>=|<|<=} value search specification

   list                                 list of addresses and registers
        addr[:addr|-addr]               address range
        ALL                             all addresses
        register[:register|-register]   register range
        STATE                           all registers
*/

t_stat exdep_cmd (int32 flag, char *cptr)
{
    char gbuf[CBUFSIZE], *gptr, *tptr;
    int32 opt;
    t_addr low, high;
    t_stat reason;
    DEVICE *tdptr;
    REG *lowr, *highr;
    SMP_FILE *ofile;

    opt = CMD_OPT_SW|CMD_OPT_SCH|CMD_OPT_DFT;               /* options for all */
    if (flag == EX_E)                                       /* extra for EX */
        opt = opt | CMD_OPT_OF;
    cptr = get_sim_opt (opt, cptr, &reason);                /* get cmd options */
    if (!cptr)                                              /* error? */
        return reason;
    if (*cptr == 0)                                         /* must be more */
        return SCPE_2FARG;
    if (sim_dfunit == NULL)                                 /* got a unit? */
        return SCPE_NXUN;
    check_select_cpu(sim_dfunit);
    cptr = get_glyph (cptr, gbuf, 0);                       /* get list */
    if ((flag == EX_D) && (*cptr == 0))                     /* deposit needs more */
        return SCPE_2FARG;
    ofile = sim_ofile? sim_ofile: smp_stdout;               /* no ofile? use stdout */

    for (gptr = gbuf, reason = SCPE_OK;
        (*gptr != 0) && (reason == SCPE_OK); gptr = tptr)
    {
        tdptr = sim_dfdev;                                  /* working dptr */
        if (strncmp (gptr, "STATE", strlen ("STATE")) == 0)
        {
            tptr = gptr + strlen ("STATE");
            if (*tptr && (*tptr++ != ',')) 
                return SCPE_ARG;
            if ((lowr = sim_dfdev->registers) == NULL)
                return SCPE_NXREG;
            for (highr = lowr; highr->name != NULL; highr++) ;
            sim_switches = sim_switches | SIM_SW_HIDE;
            reason = exdep_reg_loop (ofile, sim_schptr, flag, cptr,
                lowr, --highr, 0, 0);
            continue;
        }

        if ((lowr = find_reg (gptr, &tptr, tdptr)) ||       /* local reg or */
            (!(sim_opt_out & CMD_OPT_DFT) &&                /* no dflt, global? */
            (lowr = find_reg_glob (gptr, &tptr, &tdptr))))
        {
            low = high = 0;
            if ((*tptr == '-') || (*tptr == ':'))
            {
                highr = find_reg (tptr + 1, &tptr, tdptr);
                if (highr == NULL)
                    return SCPE_NXREG;
            }
            else
            {
                highr = lowr;
                if (*tptr == '[')
                {
                    if (lowr->depth <= 1)
                        return SCPE_ARG;
                    tptr = get_range (NULL, tptr + 1, &low, &high,
                        10, lowr->depth - 1, ']');
                    if (tptr == NULL)
                        return SCPE_ARG;
                }
            }
            if (*tptr && (*tptr++ != ','))
                return SCPE_ARG;
            reason = exdep_reg_loop (ofile, sim_schptr, flag, cptr,
                lowr, highr, (uint32) low, (uint32) high);
            continue;
        }

        tptr = get_range (sim_dfdev, gptr, &low, &high, sim_dfdev->aradix,
            (((sim_dfunit->capac == 0) || (flag == EX_E))? 0:
            sim_dfunit->capac - sim_dfdev->aincr), 0);
        if (tptr == NULL)
            return SCPE_ARG;
        if (*tptr && (*tptr++ != ','))
            return SCPE_ARG;
        reason = exdep_addr_loop (ofile, sim_schptr, flag, cptr, low, high,
            sim_dfdev, sim_dfunit);
    }

    if (sim_ofile)                                          /* close output file */
        fclose (sim_ofile);

    return reason;
}

/* Loop controllers for examine/deposit

   exdep_reg_loop       examine/deposit range of registers
   exdep_addr_loop      examine/deposit range of addresses
*/

t_stat exdep_reg_loop (SMP_FILE *ofile, SCHTAB *schptr, int32 flag, char *cptr, 
    REG *lowr, REG *highr, uint32 lows, uint32 highs)
{
    t_stat reason;
    uint32 idx;
    t_value val;
    REG *rptr;

    if (lowr == NULL || highr == NULL)
        return SCPE_IERR;
    if (lowr > highr)
        return SCPE_ARG;
    for (rptr = lowr; rptr <= highr; rptr++)
    {
        if ((sim_switches & SIM_SW_HIDE) &&
            (rptr->flags & REG_HIDDEN))
            continue;

        for (idx = lows; idx <= highs; idx++)
        {
            if (idx >= rptr->depth)
                return SCPE_SUB;
            val = get_rval (rptr, idx);
            if (schptr && !test_search (val, schptr))
                continue;
            if (flag != EX_D)
            {
                reason = ex_reg (ofile, val, flag, rptr, idx);
                if (reason != SCPE_OK)
                    return reason;
                if (sim_log && ofile == smp_stdout)
                    ex_reg (sim_log, val, flag, rptr, idx);
            }
            if (flag != EX_E)
            {
                reason = dep_reg (flag, cptr, rptr, idx);
                if (reason != SCPE_OK)
                    return reason;
            }
        }
    }

    return SCPE_OK;
}

t_stat exdep_addr_loop (SMP_FILE *ofile, SCHTAB *schptr, int32 flag, char *cptr,
    t_addr low, t_addr high, DEVICE *dptr, UNIT *uptr)
{
t_addr i, mask;
t_stat reason;

if (uptr->flags & UNIT_DIS)                             /* disabled? */
    return SCPE_UDIS;
mask = (t_addr) width_mask[dptr->awidth];
if ((low > mask) || (high > mask) || (low > high))
    return SCPE_ARG;
for (i = low; i <= high; ) {                            /* all paths must incr!! */
    reason = get_aval (i, dptr, uptr);                  /* get data */
    if (reason != SCPE_OK)                              /* return if error */
        return reason;
    if (schptr && !test_search (sim_eval[0], schptr))
        i = i + dptr->aincr;                            /* sch fails, incr */
    else {                                              /* no sch or success */
        if (flag != EX_D) {                             /* ex, ie, or id? */
            reason = ex_addr (ofile, flag, i, dptr, uptr);
            if (reason > SCPE_OK)
                return reason;
            if (sim_log && (ofile == smp_stdout))
                ex_addr (sim_log, flag, i, dptr, uptr);
            }
        else reason = 1 - dptr->aincr;                  /* no, dflt incr */
        if (flag != EX_E) {                             /* ie, id, or d? */
            reason = dep_addr (flag, cptr, i, dptr, uptr, reason);
            if (reason > SCPE_OK)
                return reason;
            }
        i = i + (1 - reason);                           /* incr */
        }
    }
return SCPE_OK;
}

/* Examine register routine

   Inputs:
        ofile   =       output stream
        val     =       current register value
        flag    =       type of ex/mod command (ex, iex, idep)
        rptr    =       pointer to register descriptor
        idx     =       index
   Outputs:
        return  =       error status
*/

t_stat ex_reg (SMP_FILE *ofile, t_value val, int32 flag, REG *rptr, uint32 idx)
{
    RUN_SCOPE;
    int32 rdx;

    if (rptr == NULL)
        return SCPE_IERR;

    if (sim_ncpus != 1)
    {
        int rcpuid = -1;
        if (rptr->locinfo.loctype == REG_LOCTYPE_CPU)
        {
            rcpuid = cpu_unit->cpu_id;
        }
        else if (rptr->locinfo.loctype == REG_LOCTYPE_DYN)
        {
            if (rptr->locinfo.get_vcpu)
                rcpuid = (*rptr->locinfo.get_vcpu)(rptr, idx);
        }

        if (rcpuid >= 0)
        {
            fprintf (ofile, "[cpu%d]\t", rcpuid);
        }
        else
        {
            fprintf (ofile, "\t");
        }
    }

    if (rptr->depth > 1)
        fprintf (ofile, "%s[%d]:\t", rptr->name, idx);
    else
        fprintf (ofile, "%s:\t", rptr->name);
    if (!(flag & EX_E))
        return SCPE_OK;
    GET_RADIX (rdx, rptr->radix);
    if ((rptr->flags & REG_VMAD) && sim_vm_fprint_addr)
        sim_vm_fprint_addr (ofile, sim_dflt_dev, (t_addr) val);
    else if (!(rptr->flags & REG_VMIO) ||
            fprint_sym (ofile, rdx, &val, NULL, sim_switches | SIM_SW_REG) > 0)
    {
        fprint_val (ofile, val, rdx, rptr->width, rptr->flags & REG_FMT);
    }
    if (flag & EX_I)
        fprintf (ofile, "\t");
    else
        fprintf (ofile, "\n");
    return SCPE_OK;
}

/* Get register value

   Inputs:
        rptr    =       pointer to register descriptor
        idx     =       index
   Outputs:
        return  =       register value
*/

t_value get_rval (REG *rptr, uint32 idx)
{
    RUN_SCOPE;
    size_t sz = SZ_R (rptr);
    t_value val;

    if (rptr->locinfo.loctype == REG_LOCTYPE_DYN)
    {
        val = (*rptr->locinfo.get_rvalue)(rptr, idx);
        return val & width_mask[rptr->width];
    }

    if (rptr->depth > 1 && (rptr->flags & REG_CIRC))
    {
        idx = idx + rptr->getqptr(RUN_PASS);
        if (idx >= rptr->depth) idx = idx - rptr->depth;
    }

    if (rptr->depth > 1 && (rptr->flags & REG_UNIT))
    {
        void* vptr = rptr->getloc_unit_idx(RUN_PASS, idx);
#if defined (USE_INT64)
        if (sz <= sizeof (uint32))
            val = *((uint32 *) vptr);
        else
            val = *((t_uint64 *) vptr);
#else
        val = *((uint32 *) vptr);
#endif
    }
    else if (((rptr->depth > 1) || (rptr->flags & REG_FIT)) &&
        (sz == sizeof (uint8)))
        val = *(((uint8 *) rptr->getloc(RUN_PASS)) + idx);
    else if (((rptr->depth > 1) || (rptr->flags & REG_FIT)) &&
        (sz == sizeof (uint16)))
        val = *(((uint16 *) rptr->getloc(RUN_PASS)) + idx);
#if defined (USE_INT64)
    else if (sz <= sizeof (uint32))
         val = *(((uint32 *) rptr->getloc(RUN_PASS)) + idx);
    else
        val = *(((t_uint64 *) rptr->getloc(RUN_PASS)) + idx);
#else
    else
        val = *(((uint32 *) rptr->getloc(RUN_PASS)) + idx);
#endif

    val = (val >> rptr->offset) & width_mask[rptr->width];

    return val;
}

/* Deposit register routine

   Inputs:
        flag    =       type of deposit (normal/interactive)
        cptr    =       pointer to input string
        rptr    =       pointer to register descriptor
        idx     =       index
   Outputs:
        return  =       error status
*/

t_stat dep_reg (int32 flag, char *cptr, REG *rptr, uint32 idx)
{
    t_stat r;
    t_value val, mask;
    int32 rdx;
    char *tptr, gbuf[CBUFSIZE];

    if (cptr == NULL || rptr == NULL)
        return SCPE_IERR;
    if (rptr->flags & REG_RO)
        return SCPE_RO;
    if (flag & EX_I)
    {
        cptr = read_line (gbuf, CBUFSIZE, smp_stdin);
        if (sim_log)
            fprintf (sim_log, (cptr? "%s\n": "\n"), cptr);
        if (cptr == NULL)                                   /* force exit */
            return 1;
        if (*cptr == 0)                                     /* success */
            return SCPE_OK;
    }
    mask = width_mask[rptr->width];
    GET_RADIX (rdx, rptr->radix);
    if ((rptr->flags & REG_VMAD) && sim_vm_parse_addr)      /* address form? */
    {
        val = sim_vm_parse_addr (sim_dflt_dev, cptr, &tptr);
        if (tptr == cptr || *tptr != 0 || val > mask)
            return SCPE_ARG;
    }
    else if (!(rptr->flags & REG_VMIO) ||                   /* dont use sym? */
             parse_sym (cptr, rdx, NULL, &val, sim_switches | SIM_SW_REG) > SCPE_OK)
    {
        val = get_uint (cptr, rdx, mask, &r);
        if (r != SCPE_OK)
            return SCPE_ARG;
    }
    if ((rptr->flags & REG_NZ) && (val == 0))
        return SCPE_ARG;
    put_rval (rptr, idx, val);
    return SCPE_OK;
}

/* Put register value

   Inputs:
        rptr    =       pointer to register descriptor
        idx     =       index
        val     =       new value
        mask    =       mask
   Outputs:
        none
*/

void put_rval (REG *rptr, uint32 idx, t_value val)
{
    RUN_SCOPE;
    size_t sz;
    t_value mask;

#define PUT_RVAL(sz,rp,id,v,m) \
    *(((sz *) rp->getloc(RUN_PASS)) + id) = \
            (*(((sz *) rp->getloc(RUN_PASS)) + id) & \
            ~((m) << (rp)->offset)) | ((v) << (rp)->offset)

    if (rptr == sim_PC)
        sim_brk_npc (RUN_PASS, 0);
    sz = SZ_R (rptr);
    mask = width_mask[rptr->width];

    if (rptr->locinfo.loctype == REG_LOCTYPE_DYN)
    {
        (*rptr->locinfo.set_rvalue)(rptr, idx, val & mask);
        return;
    }

    if (rptr->depth > 1 && (rptr->flags & REG_CIRC))
    {
        idx = idx + rptr->getqptr(RUN_PASS);
        if (idx >= rptr->depth)
            idx = idx - rptr->depth;
    }
    if (rptr->depth > 1 && (rptr->flags & REG_UNIT))
    {
        void* vptr = rptr->getloc_unit_idx(RUN_PASS, idx);
#if defined (USE_INT64)
        if (sz <= sizeof (uint32))
            *((uint32 *) vptr) = (*((uint32 *) vptr) &
            ~(((uint32) mask) << rptr->offset)) | 
            (((uint32) val) << rptr->offset);
        else *((t_uint64 *) vptr) = (*((t_uint64 *) vptr)
            & ~(mask << rptr->offset)) | (val << rptr->offset);
#else
        *((uint32 *) vptr) = (*((uint32 *) vptr) &
            ~(((uint32) mask) << rptr->offset)) | 
            (((uint32) val) << rptr->offset);
#endif
    }
    else if (((rptr->depth > 1) || (rptr->flags & REG_FIT)) &&
        (sz == sizeof (uint8)))
        PUT_RVAL (uint8, rptr, idx, (uint32) val, (uint32) mask);
    else if (((rptr->depth > 1) || (rptr->flags & REG_FIT)) &&
        (sz == sizeof (uint16)))
        PUT_RVAL (uint16, rptr, idx, (uint32) val, (uint32) mask);
#if defined (USE_INT64)
    else if (sz <= sizeof (uint32))
        PUT_RVAL (uint32, rptr, idx, (int32) val, (uint32) mask);
    else PUT_RVAL (t_uint64, rptr, idx, val, mask);
#else
    else PUT_RVAL (uint32, rptr, idx, val, mask);
#endif
}

/* Examine address routine

   Inputs: (sim_eval is an implicit argument)
        ofile   =       output stream
        flag    =       type of ex/mod command (ex, iex, idep)
        addr    =       address to examine
        dptr    =       pointer to device
        uptr    =       pointer to unit
   Outputs:
        return  =       if > 0, error status
                        if <= 0,-number of extra addr units retired
*/

t_stat ex_addr (SMP_FILE *ofile, int32 flag, t_addr addr, DEVICE *dptr, UNIT *uptr)
{
t_stat reason;
int32 rdx;

if (sim_vm_fprint_addr)
    sim_vm_fprint_addr (ofile, dptr, addr);
else fprint_val (ofile, addr, dptr->aradix, dptr->awidth, PV_LEFT);
fprintf (ofile, ":\t");
if (!(flag & EX_E))
    return (1 - dptr->aincr);

GET_RADIX (rdx, dptr->dradix);
if ((reason = fprint_sym (ofile, addr, sim_eval, uptr, sim_switches)) > 0) {
    fprint_val (ofile, sim_eval[0], rdx, dptr->dwidth, PV_RZRO);
    reason = 1 - dptr->aincr;
    }
if (flag & EX_I)
    fprintf (ofile, "\t");
else fprintf (ofile, "\n");
return reason;
}

/* Get address routine

   Inputs:
        flag    =       type of ex/mod command (ex, iex, idep)
        addr    =       address to examine
        dptr    =       pointer to device
        uptr    =       pointer to unit
   Outputs: (sim_eval is an implicit output)
        return  =       error status
*/

t_stat get_aval (t_addr addr, DEVICE *dptr, UNIT *uptr)
{
int32 i;
t_value mask;
t_addr j, loc;
size_t sz;
t_stat reason = SCPE_OK;

if ((dptr == NULL) || (uptr == NULL))
    return SCPE_IERR;
mask = width_mask[dptr->dwidth];
for (i = 0; i < sim_emax; i++)
    sim_eval[i] = 0;
for (i = 0, j = addr; i < sim_emax; i++, j = j + dptr->aincr) {
    if (dptr->examine != NULL) {
        reason = dptr->examine (&sim_eval[i], j, uptr, sim_switches);
        if (reason != SCPE_OK)
            break;
        }
    else {
        if (!(uptr->flags & UNIT_ATT))
            return SCPE_UNATT;
        if (uptr->flags & UNIT_RAW)
            return SCPE_NOFNC;
        if ((uptr->flags & UNIT_FIX) && (j >= uptr->capac)) {
            reason = SCPE_NXM;
            break;
            }
        sz = SZ_D (dptr);
        loc = j / dptr->aincr;
        if (uptr->flags & UNIT_BUF) {
            SZ_LOAD (sz, sim_eval[i], uptr->filebuf, loc);
            }
        else {
            sim_fseek (uptr->fileref, (t_addr) (sz * loc), SEEK_SET);
            sim_fread (&sim_eval[i], sz, 1, uptr->fileref);
            if ((feof (uptr->fileref)) &&
               !(uptr->flags & UNIT_FIX)) {
                reason = SCPE_EOF;
                break;
                }
            else if (ferror (uptr->fileref)) {
                clearerr (uptr->fileref);
                reason = SCPE_IOERR;
                break;
                }
            }
        }
    sim_eval[i] = sim_eval[i] & mask;
    }
if ((reason != SCPE_OK) && (i == 0))
    return reason;
return SCPE_OK;
}

/* Deposit address routine

   Inputs:
        flag    =       type of deposit (normal/interactive)
        cptr    =       pointer to input string
        addr    =       address to examine
        dptr    =       pointer to device
        uptr    =       pointer to unit
        dfltinc =       value to return on cr input
   Outputs:
        return  =       if > 0, error status
                        if <= 0, -number of extra address units retired
*/

t_stat dep_addr (int32 flag, char *cptr, t_addr addr, DEVICE *dptr,
    UNIT *uptr, int32 dfltinc)
{
int32 i, count, rdx;
t_addr j, loc;
t_stat r, reason;
t_value mask;
size_t sz;
char gbuf[CBUFSIZE];

if (dptr == NULL)
    return SCPE_IERR;
if (flag & EX_I) {
    cptr = read_line (gbuf, CBUFSIZE, smp_stdin);
    if (sim_log)
        fprintf (sim_log, (cptr? "%s\n": "\n"), cptr);
    if (cptr == NULL)                                   /* force exit */
        return 1;
    if (*cptr == 0)                                     /* success */
        return dfltinc;
    }
if (uptr->flags & UNIT_RO)                              /* read only? */
    return SCPE_RO;
mask = width_mask[dptr->dwidth];

GET_RADIX (rdx, dptr->dradix);
if ((reason = parse_sym (cptr, addr, uptr, sim_eval, sim_switches)) > 0) {
    sim_eval[0] = get_uint (cptr, rdx, mask, &reason);
    if (reason != SCPE_OK)
        return reason;
    reason = dfltinc;
    }
count = (1 - reason + (dptr->aincr - 1)) / dptr->aincr;

for (i = 0, j = addr; i < count; i++, j = j + dptr->aincr) {
    sim_eval[i] = sim_eval[i] & mask;
    if (dptr->deposit != NULL) {
        r = dptr->deposit (sim_eval[i], j, uptr, sim_switches);
        if (r != SCPE_OK)
            return r;
        }
    else {
        if (!(uptr->flags & UNIT_ATT))
            return SCPE_UNATT;
        if (uptr->flags & UNIT_RAW) 
            return SCPE_NOFNC;
        if ((uptr->flags & UNIT_FIX) && (j >= uptr->capac))
            return SCPE_NXM;
        sz = SZ_D (dptr);
        loc = j / dptr->aincr;
        if (uptr->flags & UNIT_BUF) {
            SZ_STORE (sz, sim_eval[i], uptr->filebuf, loc);
            if (loc >= uptr->hwmark) 
                uptr->hwmark = (uint32) loc + 1;
            }
        else {
            sim_fseek (uptr->fileref, (t_addr) (sz * loc), SEEK_SET);
            sim_fwrite (&sim_eval[i], sz, 1, uptr->fileref);
            if (ferror (uptr->fileref)) {
                clearerr (uptr->fileref);
                return SCPE_IOERR;
                }
            }
        }
    }
return reason;
}

/* Evaluate command */

t_stat eval_cmd (int32 flg, char *cptr)
{
DEVICE *dptr = sim_dflt_dev;
int32 i, rdx, a, lim;
t_stat r;

GET_SWITCHES (cptr);
GET_RADIX (rdx, dptr->dradix);
for (i = 0; i < sim_emax; i++)
sim_eval[i] = 0;
if (*cptr == 0)
    return SCPE_2FARG;
if ((r = parse_sym (cptr, 0, dptr->units[0], sim_eval, sim_switches)) > 0) {
    sim_eval[0] = get_uint (cptr, rdx, width_mask[dptr->dwidth], &r);
    if (r != SCPE_OK)
        return r;
    }
lim = 1 - r;
for (i = a = 0; a < lim; ) {
    smp_printf ("%d:\t", a);
    if ((r = fprint_sym (smp_stdout, a, &sim_eval[i], dptr->units[0], sim_switches)) > 0)
        r = fprint_val (smp_stdout, sim_eval[i], rdx, dptr->dwidth, PV_RZRO);
    smp_printf ("\n");
    if (sim_log) {
        fprintf (sim_log, "%d\t", i);
        if ((r = fprint_sym (sim_log, a, &sim_eval[i], dptr->units[0], sim_switches)) > 0)
            r = fprint_val (sim_log, sim_eval[i], rdx, dptr->dwidth, PV_RZRO);
        fprintf (sim_log, "\n");
        }
    if (r < 0)
        a = a + 1 - r;
    else a = a + dptr->aincr;
    i = a / dptr->aincr;
    }
return SCPE_OK;
}

/* String processing routines

   read_line            read line

   Inputs:
        cptr    =       pointer to buffer
        size    =       maximum size
        stream  =       pointer to input stream
   Outputs:
        optr    =       pointer to first non-blank character
                        NULL if EOF
*/

char *read_line (char *cptr, int32 size, SMP_FILE *stream)
{
    return read_line_p (NULL, cptr, size, stream);
}

/* read_line_p          read line with prompt

   Inputs:
        prompt  =       pointer to prompt string
        cptr    =       pointer to buffer
        size    =       maximum size
        stream  =       pointer to input stream
   Outputs:
        optr    =       pointer to first non-blank character
                        NULL if EOF
*/

char *read_line_p (char *prompt, char *cptr, int32 size, SMP_FILE *stream)
{
    char *tptr;
#if defined(HAVE_DLOPEN)
    static int initialized = 0;
    static char *(*p_readline)(const char *) = NULL;
    static void (*p_add_history)(const char *) = NULL;

    if (!initialized)
    {
        initialized = 1;
        void *handle;

#define __STR_QUOTE(tok) #tok
#define __STR(tok) __STR_QUOTE(tok)
        handle = dlopen("libncurses." __STR(HAVE_DLOPEN), RTLD_NOW|RTLD_GLOBAL);
        handle = dlopen("libcurses." __STR(HAVE_DLOPEN), RTLD_NOW|RTLD_GLOBAL);
        handle = dlopen("libreadline." __STR(HAVE_DLOPEN), RTLD_NOW|RTLD_GLOBAL);
        if (handle)
        {
            p_readline = (char *(*)(const char *)) dlsym(handle, "readline");
            p_add_history = (void (*)(const char *)) dlsym(handle, "add_history");
        }
    }

    if (prompt)                                             /* interactive? */
    {
        char *tmpc;

        if (p_readline)
        {
            char *tmpc = p_readline (prompt);               /* get cmd line */
            if (tmpc == NULL)                               /* bad result? */
                cptr = NULL;
            else
            {
                strncpy (cptr, tmpc, size);                 /* copy result */
                free (tmpc);                                /* free temp */
            }
        }
        else
        {
            smp_printf ("%s", prompt);                      /* display prompt */
            cptr = fgets (cptr, size, stream);              /* get cmd line */
        }
    }
    else 
    {
        cptr = fgets (cptr, size, stream);                  /* get cmd line */
    }
#else
    if (prompt)                                             /* interactive? */
        smp_printf ("%s", prompt);                          /* display prompt */
    cptr = fgets (cptr, size, stream);                      /* get cmd line */
#endif

    if (cptr == NULL)
    {
        clearerr (stream);                                  /* clear error */
        return NULL;                                        /* ignore EOF */
    }
    for (tptr = cptr; tptr < (cptr + size); tptr++)         /* remove cr or nl */
    {
        if (*tptr == '\n' || *tptr == '\r' ||
            tptr == cptr + size - 1)                        /* str max length? */
        {
            *tptr = 0;                                      /* terminate */
            break;
        }
    }
    while (isspace (*cptr))                                 /* trim leading spc */
        cptr++;
    if (*cptr == ';')                                       /* ignore comment */
    {
        char pmt[32];
        make_prompt (pmt, "do");
        if (sim_do_echo)                                    /* echo comments if -v */
            smp_printf("%s%s\n", pmt, cptr);
        if (sim_do_echo && sim_log)
            fprintf (sim_log, "%s%s\n", pmt, cptr);
        *cptr = 0;
    }

#if defined (HAVE_DLOPEN)
    if (prompt && p_add_history && *cptr)                   /* Save non blank lines in history */
        p_add_history (cptr);
#endif

    return cptr;
}

/* get_glyph            get next glyph (force upper case)
   get_glyph_nc         get next glyph (no conversion)
   get_glyph_gen        get next glyph (general case)

   Inputs:
        iptr    =       pointer to input string
        optr    =       pointer to output string
        mchar   =       optional end of glyph character
        flag    =       TRUE for convert to upper case (_gen only)
   Outputs
        result  =       pointer to next character in input string
*/

char *get_glyph_gen (char *iptr, char *optr, char mchar, t_bool uc)
{
    while (!isspace(*iptr) && *iptr && *iptr != mchar)
    {
        if (islower (*iptr) && uc)
            *optr = toupper (*iptr);
        else
            *optr = *iptr;
        iptr++; optr++;
    }
    *optr = 0;
    if (mchar && *iptr == mchar)                            /* skip terminator */
        iptr++;
    while (isspace (*iptr))                                 /* absorb spaces */
        iptr++;
    return iptr;
}

char *get_glyph (char *iptr, char *optr, char mchar)
{
    return get_glyph_gen (iptr, optr, mchar, TRUE);
}

char *get_glyph_nc (char *iptr, char *optr, char mchar)
{
    return get_glyph_gen (iptr, optr, mchar, FALSE);
}

/* Trim trailing spaces from a string

    Inputs:
        cptr    =       pointer to string
    Outputs:
        cptr    =       pointer to string
*/

char *sim_trim_endspc (char *cptr)
{
char *tptr;

tptr = cptr + strlen (cptr);
while ((--tptr >= cptr) && isspace (*tptr))
    *tptr = 0;
return cptr;
}

/* get_yn               yes/no question

   Inputs:
        cptr    =       pointer to question
        deflt   =       default answer
   Outputs:
        result  =       true if yes, false if no
*/

t_stat get_yn (char *ques, t_stat deflt)
{
char cbuf[CBUFSIZE], *cptr;

printf ("%s ", ques);
cptr = read_line (cbuf, CBUFSIZE, smp_stdin);
if ((cptr == NULL) || (*cptr == 0))
    return deflt;
if ((*cptr == 'Y') || (*cptr == 'y'))
    return TRUE;
return FALSE;
}

/* get_uint             unsigned number

   Inputs:
        cptr    =       pointer to input string
        radix   =       input radix
        max     =       maximum acceptable value
        *status =       pointer to error status
   Outputs:
        val     =       value
*/

t_value get_uint (char *cptr, uint32 radix, t_value max, t_stat *status)
{
    t_value val;
    char *tptr;

    *status = SCPE_OK;
    val = strtotv (cptr, &tptr, radix);
    if (cptr == tptr || val > max)
        *status = SCPE_ARG;
    else
    {
        while (isspace (*tptr)) tptr++;
        if (*tptr)
            *status = SCPE_ARG;
    }
    return val;
}

/* get_uint             unsigned number

   Inputs:
        cptr    =       pointer to input string
        *status =       pointer to error status
   Outputs:
        val     =       value
*/

t_stat get_double(char *cptr, double* pval)
{
    double val = 0;

    if (*cptr != '.' && !isdigit(*cptr))
        return SCPE_ARG;

    while (isdigit(*cptr))
        val = val * 10 + (*cptr++ - '0');

    if (*cptr == '.')
    {
        cptr++;
        double scale = 0.1;
        while (isdigit(*cptr))
        {
            val += (*cptr++ - '0') * scale;
            scale *= 0.1;
        }
    }

    while (isspace(*cptr)) cptr++;

    if (*cptr)  return SCPE_ARG;

    *pval = val;

    return SCPE_OK;
}

/* get_range            range specification

   Inputs:
        dptr    =       pointer to device (NULL if none)
        cptr    =       pointer to input string
        *lo     =       pointer to low result
        *hi     =       pointer to high result
        aradix  =       radix
        max     =       default high value
        term    =       terminating character, 0 if none
   Outputs:
        tptr    =       input pointer after processing
                        NULL if error
*/

char *get_range (DEVICE *dptr, char *cptr, t_addr *lo, t_addr *hi,
    uint32 rdx, t_addr max, char term)
{
char *tptr;

if (max && strncmp (cptr, "ALL", strlen ("ALL")) == 0) { /* ALL? */
    tptr = cptr + strlen ("ALL");
    *lo = 0;
    *hi = max;
    }
else {
    if (dptr && sim_vm_parse_addr)                      /* get low */
        *lo = sim_vm_parse_addr (dptr, cptr, &tptr);
    else *lo = (t_addr) strtotv (cptr, &tptr, rdx);
    if (cptr == tptr)                                   /* error? */
            return NULL;
    if ((*tptr == '-') || (*tptr == ':')) {             /* range? */
        cptr = tptr + 1;
        if (dptr && sim_vm_parse_addr)                  /* get high */
            *hi = sim_vm_parse_addr (dptr, cptr, &tptr);
        else *hi = (t_addr) strtotv (cptr, &tptr, rdx);
        if (cptr == tptr)
            return NULL;
        if (*lo > *hi)
            return NULL;
        }
    else if (*tptr == '/') {                            /* relative? */
        cptr = tptr + 1;
        *hi = (t_addr) strtotv (cptr, &tptr, rdx);      /* get high */
        if ((cptr == tptr) || (*hi == 0))
            return NULL;
        *hi = *lo + *hi - 1;
        }
    else *hi = *lo;
    }
if (term && (*tptr++ != term))
    return NULL;
return tptr;
}

/* get_ipaddr           IP address:port

   Inputs:
        cptr    =       pointer to input string
   Outputs:
        ipa     =       pointer to IP address (may be NULL), 0 = none
        ipp     =       pointer to IP port (may be NULL), 0 = none
        result  =       status
*/

t_stat get_ipaddr (char *cptr, uint32 *ipa, uint32 *ipp)
{
char gbuf[CBUFSIZE];
char *addrp, *portp, *octetp;
uint32 i, addr, port, octet;
t_stat r;

if ((cptr == NULL) || (*cptr == 0))
    return SCPE_ARG;
strncpy (gbuf, cptr, CBUFSIZE);
addrp = gbuf;                                           /* default addr */
if (portp = strchr (gbuf, ':'))                         /* x:y? split */
    *portp++ = 0;
else if (strchr (gbuf, '.'))                            /* x.y...? */
    portp = NULL;
else {
    portp = gbuf;                                       /* port only */
    addrp = NULL;                                       /* no addr */
    }
if (portp) {                                            /* port string? */
    if (ipp == NULL)                                    /* not wanted? */
        return SCPE_ARG;
    port = (int32) get_uint (portp, 10, 65535, &r);
    if ((r != SCPE_OK) || (port == 0))
        return SCPE_ARG;
    }
else port = 0;
if (addrp) {                                            /* addr string? */
    if (ipa == NULL)                                    /* not wanted? */
        return SCPE_ARG;
    for (i = addr = 0; i < 4; i++) {                    /* four octets */
        octetp = strchr (addrp, '.');                   /* find octet end */
        if (octetp != NULL)                             /* split string */
            *octetp++ = 0;
        else if (i < 3)                                 /* except last */
            return SCPE_ARG;
        octet = (int32) get_uint (addrp, 10, 255, &r);
        if (r != SCPE_OK)
            return SCPE_ARG;
        addr = (addr << 8) | octet;
        addrp = octetp;
        }
    if (((addr & 0377) == 0) || ((addr & 0377) == 255))
        return SCPE_ARG;
    }
else addr = 0;
if (ipp)                                                /* return req values */
    *ipp = port;
if (ipa)
    *ipa = addr;
return SCPE_OK;   
}

/* Find_device          find device matching input string

   Inputs:
        cptr    =       pointer to input string
   Outputs:
        result  =       pointer to device
*/

DEVICE *find_dev (char *cptr)
{
    int32 i;
    DEVICE *dptr;

    for (i = 0; (dptr = sim_devices[i]) != NULL; i++)
    {
        if ((strcmp (cptr, dptr->name) == 0) ||
            (dptr->lname &&
            (strcmp (cptr, dptr->lname) == 0)))
        {
            return dptr;
        }
    }
    return NULL;
}

DEVICE *find_dev (DIB* dibp)
{
    DEVICE *dptr;

    if (dibp == NULL)
        return NULL;

    for (int i = 0; (dptr = sim_devices[i]) != NULL; i++)
    {
        if (dptr->ctxt == dibp)
            return dptr;
    }
    return NULL;
}

/* Find_unit            find unit matching input string

   Inputs:
        cptr    =       pointer to input string
        uptr    =       pointer to unit pointer
   Outputs:
        result  =       pointer to device (null if no dev)
        *iptr   =       pointer to unit (null if nx unit)
*/

DEVICE *find_unit (char *cptr, UNIT **uptr)
{
    uint32 i, u;
    const char* nptr;
    char* tptr;
    t_stat r;
    DEVICE *dptr;

    if (uptr == NULL)                                       /* arg error? */
        return NULL;

    if (dptr = find_dev (cptr))                             /* exact match? */
    {
        if (qdisable (dptr))                                /* disabled? */
            return NULL;
        if (dptr == &cpu_dev && sim_dflt_cpu)
            *uptr = sim_dflt_cpu;                           /* selected CPU unit */
        else
            *uptr = dptr->units[0];                         /* unit 0 */
        return dptr;
    }

    for (i = 0; (dptr = sim_devices[i]) != NULL; i++)       /* base + unit#? */
    {
        if (dptr->numunits &&                               /* any units? */
            (((nptr = dptr->name) &&
              (strncmp (cptr, nptr, strlen (nptr)) == 0)) ||
             ((nptr = dptr->lname) &&
              (strncmp (cptr, nptr, strlen (nptr)) == 0))))
        {
            tptr = cptr + strlen (nptr);
            if (isdigit (*tptr))
            {
                if (qdisable (dptr))                        /* disabled? */
                    return NULL;
                u = (uint32) get_uint (tptr, 10, dptr->numunits - 1, &r);
                if (r != SCPE_OK)                           /* error? */
                    *uptr = NULL;
                else
                    *uptr = dptr->units[u];
                return dptr;
            }
        }
    }

    return NULL;
}

/* Find_dev_from_unit   find device for unit

   Inputs:
        uptr    =       pointer to unit
   Outputs:
        result  =       pointer to device
*/

DEVICE *find_dev_from_unit (UNIT *uptr)
{
    return uptr ? uptr->device : NULL;
}

/* Test for disabled device */

t_bool qdisable (DEVICE *dptr)
{
    return (dptr->flags & DEV_DIS? TRUE: FALSE);
}

/* find_reg_glob        find globally unique register

   Inputs:
        cptr    =       pointer to input string
        optr    =       pointer to output pointer (can be null)
        gdptr   =       pointer to global device
   Outputs:
        result  =       pointer to register, NULL if error
        *optr   =       pointer to next character in input string
        *gdptr  =       pointer to device where found
*/

REG *find_reg_glob (char *cptr, char **optr, DEVICE **gdptr)
{
int32 i;
DEVICE *dptr;
REG *rptr, *srptr = NULL;

for (i = 0; (dptr = sim_devices[i]) != 0; i++) {        /* all dev */
    if (dptr->flags & DEV_DIS)                          /* skip disabled */
        continue;
    if (rptr = find_reg (cptr, optr, dptr)) {           /* found? */
        if (srptr)                                      /* ambig? err */
            return NULL;
        srptr = rptr;                                   /* save reg */
        *gdptr = dptr;                                  /* save unit */
        }
    }
return srptr;
}

/* find_reg             find register matching input string

   Inputs:
        cptr    =       pointer to input string
        optr    =       pointer to output pointer (can be null)
        dptr    =       pointer to device
   Outputs:
        result  =       pointer to register, NULL if error
        *optr   =       pointer to next character in input string
*/

REG *find_reg (char *cptr, char **optr, DEVICE *dptr)
{
char *tptr;
REG *rptr;
size_t slnt;

if ((cptr == NULL) || (dptr == NULL) || (dptr->registers == NULL))
    return NULL;
tptr = cptr;
do {
    tptr++;
    } while (isalnum (*tptr) || (*tptr == '*') || (*tptr == '_'));
slnt = (unsigned int) (tptr - cptr);
for (rptr = dptr->registers; rptr->name != NULL; rptr++) {
    if ((slnt == strlen (rptr->name)) &&
        (strncmp (cptr, rptr->name, slnt) == 0)) {
        if (optr != NULL)
            *optr = tptr;
        return rptr;
        }
    }
return NULL;
}

/* get_switches         get switches from input string

   Inputs:
        cptr    =       pointer to input string
   Outputs:
        sw      =       switch bit mask
                        0 if no switches, -1 if error
*/

int32 get_switches (char *cptr)
{
int32 sw;

if (*cptr != '-')
    return 0;
sw = 0;
for (cptr++; (isspace (*cptr) == 0) && (*cptr != 0); cptr++) {
    if (isalpha (*cptr) == 0)
        return -1;
    sw = sw | SWMASK (toupper (*cptr));
    }
return sw;
}

/* get_sim_sw           accumulate sim_switches

   Inputs:
        cptr    =       pointer to input string
   Outputs:
        ptr     =       pointer to first non-string glyph
                        NULL if error
*/

char *get_sim_sw (char *cptr)
{
int32 lsw;
char gbuf[CBUFSIZE];

while (*cptr == '-') {                                  /* while switches */
    cptr = get_glyph (cptr, gbuf, 0);                   /* get switch glyph */
    lsw = get_switches (gbuf);                          /* parse */
    if (lsw <= 0)                                       /* invalid? */
        return NULL;
    sim_switches = sim_switches | lsw;                  /* accumulate */
    }
return cptr;
}

/* get_sim_opt          get simulator command options

   Inputs:
        opt     =       command options
        cptr    =       pointer to input string
   Outputs:
        ptr     =       pointer to next glypsh, NULL if error
        *stat   =       error status
*/

char *get_sim_opt (int32 opt, char *cptr, t_stat *st)
{
int32 t;
char *svptr, gbuf[CBUFSIZE];
DEVICE *tdptr;
UNIT *tuptr;

sim_switches = 0;                                       /* no switches */
sim_ofile = NULL;                                       /* no output file */
sim_schptr = NULL;                                      /* no search */
sim_stab.logic = SCH_OR;                                /* default search params */
sim_stab.boolop = SCH_GE;
sim_stab.mask = 0;
sim_stab.comp = 0;
sim_dfdev = sim_dflt_dev;
if (sim_dfdev == &cpu_dev)
{
    sim_dfunit = sim_dflt_cpu;
}
else
{
    sim_dfunit = sim_dfdev->units[0];
}
sim_opt_out = 0;                                        /* no options yet */
*st = SCPE_OK;
while (*cptr) {                                         /* loop through modifiers */
    svptr = cptr;                                       /* save current position */
    if ((opt & CMD_OPT_OF) && (*cptr == '@')) {         /* output file spec? */
        if (sim_ofile) {                                /* already got one? */
            fclose (sim_ofile);                         /* one per customer */
            *st = SCPE_ARG;
            return NULL;
            }
        cptr = get_glyph_nc (cptr + 1, gbuf, 0);
        sim_ofile = sim_fopen (gbuf, "a");              /* open for append */
        if (sim_ofile == NULL) {                        /* open failed? */
            *st = SCPE_OPENERR;                        
            return NULL;
            }
        sim_opt_out |= CMD_OPT_OF;                      /* got output file */
        continue;
        }
    cptr = get_glyph (cptr, gbuf, 0);
    if ((t = get_switches (gbuf)) != 0) {               /* try for switches */
        if (t < 0) {                                    /* err if bad switch */
            *st = SCPE_INVSW;
            return NULL;
            }
        sim_switches = sim_switches | t;                /* or in new switches */
        }
    else if ((opt & CMD_OPT_SCH) &&                     /* if allowed, */
        get_search (gbuf, sim_dfdev->dradix, &sim_stab)) { /* try for search */
        sim_schptr = &sim_stab;                         /* set search */
        sim_opt_out |= CMD_OPT_SCH;                     /* got search */
        }
    else if ((opt & CMD_OPT_DFT) &&                     /* default allowed? */
        ((sim_opt_out & CMD_OPT_DFT) == 0) &&           /* none yet? */
        (tdptr = find_unit (gbuf, &tuptr)) &&           /* try for default */
        (tuptr != NULL)) {
        sim_dfdev = tdptr;                              /* set as default */
        sim_dfunit = tuptr;
        sim_opt_out |= CMD_OPT_DFT;                     /* got default */
        }
    else return svptr;                                  /* not rec, break out */
    }
return cptr;
}

/* Match file extension

   Inputs:
        fnam    =       file name
        ext     =       extension, without period
   Outputs:
        cp      =       pointer to final '.' if match, NULL if not
*/

char *match_ext (char *fnam, char *ext)
{
char *pptr, *fptr, *eptr;

if ((fnam == NULL) || (ext == NULL))                    /* bad arguments? */
     return NULL;
pptr = strrchr (fnam, '.');                             /* find last . */
if (pptr) {                                             /* any? */
    for (fptr = pptr + 1, eptr = ext;                   /* match characters */
#if defined (VMS)                                       /* VMS: stop at ; or null */
    (*fptr != 0) && (*fptr != ';');
#else
    *fptr != 0;                                         /* others: stop at null */
#endif
    fptr++, eptr++) {
        if (toupper (*fptr) != toupper (*eptr))
            return NULL;
        }
    if (*eptr != 0)                                     /* ext exhausted? */
        return NULL;
    }
return pptr;
}

/* Get search specification

   Inputs:
        cptr    =       pointer to input string
        radix   =       radix for numbers
        schptr =        pointer to search table
   Outputs:
        return =        NULL if error
                        schptr if valid search specification
*/

SCHTAB *get_search (char *cptr, int32 radix, SCHTAB *schptr)
{
int32 c;
int32 logop, cmpop;
t_value logval, cmpval;
char *sptr, *tptr;
const char logstr[] = "|&^", cmpstr[] = "=!><";

logval = cmpval = 0;
if (*cptr == 0)                                         /* check for clause */
    return NULL;
for (logop = cmpop = -1; c = *cptr++; ) {               /* loop thru clauses */
    if (sptr = (char*) strchr (logstr, c)) {            /* check for mask */
        logop = (int) (sptr - logstr);
        logval = strtotv (cptr, &tptr, radix);
        if (cptr == tptr)
            return NULL;
        cptr = tptr;
        }
    else if (sptr = (char*) strchr (cmpstr, c)) {               /* check for boolop */
        cmpop = (int) (sptr - cmpstr);
        if (*cptr == '=') {
            cmpop = cmpop + (int) strlen (cmpstr);
            cptr++;
            }
        cmpval = strtotv (cptr, &tptr, radix);
        if (cptr == tptr)
            return NULL;
        cptr = tptr;
        }
    else return NULL;
    }                                                   /* end for */
if (logop >= 0) {
    schptr->logic = logop;
    schptr->mask = logval;
    }
if (cmpop >= 0) {
    schptr->boolop = cmpop;
    schptr->comp = cmpval;
    }
return schptr;
}

/* Test value against search specification

   Inputs:
        val     =       value to test
        schptr =        pointer to search table
   Outputs:
        return =        1 if value passes search criteria, 0 if not
*/

int32 test_search (t_value val, SCHTAB *schptr)
{
if (schptr == NULL) return 0;

switch (schptr->logic) {                                /* case on logical */

    case SCH_OR:
        val = val | schptr->mask;
        break;

    case SCH_AND:
        val = val & schptr->mask;
        break;

    case SCH_XOR:
        val = val ^ schptr->mask;
        break;
        }

switch (schptr->boolop) {                                       /* case on comparison */

    case SCH_E: case SCH_EE:
        return (val == schptr->comp);

    case SCH_N: case SCH_NE:
        return (val != schptr->comp);

    case SCH_G:
        return (val > schptr->comp);

    case SCH_GE:
        return (val >= schptr->comp);

    case SCH_L:
        return (val < schptr->comp);

    case SCH_LE:
        return (val <= schptr->comp);
        }

return 0;
}

/* Radix independent input/output package

   strtotv - general radix input routine

   Inputs:
        inptr   =       string to convert
        endptr  =       pointer to first unconverted character
        radix   =       radix for input
   Outputs:
        value   =       converted value

   On an error, the endptr will equal the inptr.
*/

t_value strtotv (char *inptr, char **endptr, uint32 radix)
{
int32 nodigit;
t_value val;
uint32 c, digit;

*endptr = inptr;                                        /* assume fails */
if ((radix < 2) || (radix > 36))
    return 0;
while (isspace (*inptr))                                /* bypass white space */
    inptr++;
val = 0;
nodigit = 1;
for (c = *inptr; isalnum(c); c = *++inptr) {            /* loop through char */
    if (islower (c))
        c = toupper (c);
    if (isdigit (c))                                    /* digit? */
        digit = c - (uint32) '0';
    else if (radix <= 10)                               /* stop if not expected */
        break;
    else digit = c + 10 - (uint32) 'A';                 /* convert letter */
    if (digit >= radix)                                 /* valid in radix? */
        return 0;
    val = (val * radix) + digit;                        /* add to value */
    nodigit = 0;
    }
if (nodigit)                                            /* no digits? */
    return 0;
*endptr = inptr;                                        /* result pointer */
return val;
}

/* fprint_val - general radix printing routine

   Inputs:
        stream  =       stream designator
        val     =       value to print
        radix   =       radix to print
        width   =       width to print
        format  =       leading zeroes format
   Outputs:
        status  =       error status
*/

t_stat fprint_val (SMP_FILE *stream, t_value val, uint32 radix,
    uint32 width, uint32 format)
{
#define MAX_WIDTH ((int) (CHAR_BIT * sizeof (t_value)))
t_value owtest, wtest;
int32 d, digit, ndigits;
char dbuf[MAX_WIDTH + 1];

for (d = 0; d < MAX_WIDTH; d++)
    dbuf[d] = (format == PV_RZRO)? '0': ' ';
dbuf[MAX_WIDTH] = 0;
d = MAX_WIDTH;
do {
    d = d - 1;
    digit = (int32) (val % radix);
    val = val / radix;
    dbuf[d] = (digit <= 9)? '0' + digit: 'A' + (digit - 10);
    } while ((d > 0) && (val != 0));

if (format != PV_LEFT) {
    wtest = owtest = radix;
    ndigits = 1;
    while ((wtest < width_mask[width]) && (wtest >= owtest)) {
        owtest = wtest;
        wtest = wtest * radix;
        ndigits = ndigits + 1;
        }
    if ((MAX_WIDTH - ndigits) < d)
        d = MAX_WIDTH - ndigits;
    }
if (fputs (&dbuf[d], stream) == EOF)
    return SCPE_IOERR;
return SCPE_OK;
}

/* Event queue package

        sim_activate            add entry to event queue
        sim_cancel              remove entry from event queue
        sim_process_event       process entries on event queue
        sim_is_active           see if entry is on event queue
        sim_atime               return absolute time for an entry
        sim_gtime               return global time

   Asynchronous events are set up by queueing a unit data structure
   to the event queue with a timeout (in simulator units, relative
   to the current time).  Each simulator 'times' these events by
   counting down interval counter sim_interval.  When this reaches
   zero the simulator calls sim_process_event to process the event
   and to see if further events need to be processed, or sim_interval
   reset to count the next one.

   The event queue is maintained in clock order; entry timeouts are
   RELATIVE to the time in the previous entry.

   sim_process_event - process event

   Inputs:
        none
   Outputs:
        reason  =       reason code returned by any event processor,
                        or 0 (SCPE_OK) if no exceptions
*/

t_stat sim_process_event (RUN_DECL)
{
    t_stat reason;
    t_bool elevated = FALSE;
    t_bool check_elevate = TRUE;

    if (weak_read(stop_cpus))                               /* stop CPU? */
        return SCPE_STOP;

    if (cpu_unit->clock_queue == NULL)                      /* queue empty? */
    {
        UPDATE_SIM_TIME (cpu_unit->noqueue_time);                /* update sim time */
        sim_interval = cpu_unit->noqueue_time = NOQUEUE_WAIT;    /* flag queue empty */
        return SCPE_OK;
    }

    RUN_SCOPE_RSCX_ONLY;

    UPDATE_SIM_TIME (cpu_unit->clock_queue->time);          /* update sim time */

    do
    {
        clock_queue_entry* cqe = cpu_unit->clock_queue;     /* get first */

        if (unlikely(cqe->clk_cosched))
        {
            /* 
             * change time on all cosched entries
             */
            sim_reschedule_cosched(RUN_PASS, RescheduleCosched_RequeueOnProcessEvent);
            if (sim_interval)
                return SCPE_OK;
            cqe = cpu_unit->clock_queue;
            if (unlikely(cqe->clk_cosched))
                panic("Unexpected prematurely expired CLK COSCHED entry in the clock queue");
        }

        UNIT *uptr = cqe->uptr;

        cpu_unit->clock_queue = cqe->next;                  /* remove from active list */

        cqe->next = cpu_unit->clock_queue_freelist;         /* place on freelist */
        cpu_unit->clock_queue_freelist = cqe;

        if (cpu_unit->clock_queue != NULL)
            sim_interval = cpu_unit->clock_queue->time;
        else
            sim_interval = cpu_unit->noqueue_time = NOQUEUE_WAIT;

        /*
         * We are about to call device handler which is likely to acquire device lock
         * as a part of event processing.
         *
         * We elevate thread priority to avoid acquiring device lock at low thread priority
         * and thus to reduce probability of lock holder preemption.
         *
         * Exception is made for per-CPU devices since they normally do not acquire shared locks.
         * However some per-CPU devices (for VAX MP, TTI and TTO) do acquire shared locks,
         * and for these devices thread priority must be elevated too. It would have been possible
         * for them to raise and lower thread priority internally, however we try to minimize
         * overall number of thread priority changes to at most one pair per loop.
         */
        if (!elevated && check_elevate)
        {
            if (unlikely(!is_os_running(RUN_PASS)) ||
                cpu_unit->cpu_thread_priority != SIMH_THREAD_PRIORITY_INVALID &&
                cpu_unit->cpu_thread_priority >= SIMH_THREAD_PRIORITY_CPU_CRITICAL_OS_HI)
            {
                check_elevate = FALSE;
            }
            else
            {
                t_bool device_match = !IS_PERCPU_UNIT(uptr);
#if defined(VM_VAX_MP)
                if (! device_match)
                {
                    extern DEVICE tti_dev, tto_dev;
                    if (uptr->device == &tti_dev || uptr->device == &tto_dev)
                        device_match = TRUE;
                }
#endif
                if (device_match && rscx->os_hi_critical_locks == 0)
                {
                    rscx->os_hi_critical_locks++;
                    cpu_reevaluate_thread_priority(RUN_PASS);
                    elevated = TRUE;
                }
            }
        }

        if (uptr->action != NULL)
            reason = uptr->action (RUN_PASS, uptr);
        else
            reason = SCPE_OK;
    }
    while (reason == SCPE_OK && sim_interval == 0);

    /*
     * Reset thread priority. Note that we cannot simply restore priority to that at the
     * entrance to sim_process_event, since priority might have been purposefully altered
     * by events like:
     *
     *     - event handler changing priority, e.g. in case of VAX SSC clock expiration
     *     - event handler sending interrupt to this processor
     *     - interrupt was sent to this processor by external source
     *
     * In addition, event handler may temporary change priority inside itself by acquiring
     * and releasing locks with non-null criticality level.
     *
     */
    if (elevated && --rscx->os_hi_critical_locks == 0)
        cpu_reevaluate_thread_priority(RUN_PASS);

    return reason;
}

/*
 * sim_activate - activate (queue) event
 *
 * If device is system-wide (not per-CPU), caller should hold device lock.
 *
 * Inputs:
 *      uptr    =       pointer to unit
 *      event_time =    relative timeout (only valid if nticks == 0)
 *      nticks =        number of clock ticks for cosched activation (>=1), 1 = next tick
 * Outputs:
 *      reason  =       result (SCPE_OK if ok)
 */

#define CLK_COSCHED_DUMMY_TIME (INT32_MAX / 4)

t_stat sim_activate (UNIT *uptr, int32 event_time, int32 nticks)
{
    RUN_SCOPE;

    if (event_time < 0)
        return SCPE_IERR;

    if (nticks)
        event_time = CLK_COSCHED_DUMMY_TIME;

    if (uptr == &clk_unit && use_clock_thread)
    {
        cpu_unit->clk_active = TRUE;
        return SCPE_OK;
    }

    clock_queue_entry* cqe;
    clock_queue_entry* prev;
    int32 accum;

    if (sim_is_active (uptr))                               /* already active? */
    {
        return SCPE_OK;
    }

    UPDATE_CPU_SIM_TIME();                                  /* update sim time */

    prev = NULL;
    accum = 0;
    for (cqe = cpu_unit->clock_queue; cqe != NULL; cqe = cqe->next)
    {
        if (event_time < accum + cqe->time)
            break;
        accum = accum + cqe->time;
        prev = cqe;
    }

    /* allocate entry off free list and fill it */
    cqe = cpu_unit->clock_queue_freelist;
    if (cqe == NULL)
    {
        panic("Unable to allocate clock queue entry");
    }
    cpu_unit->clock_queue_freelist = cqe->next;
    cqe->time = event_time - accum;
    cqe->uptr = uptr;
    cqe->clk_cosched = nticks;

    /* insert it */
    if (prev == NULL)                                       /* insert at head */
    {
        cqe->next = cpu_unit->clock_queue;
        cpu_unit->clock_queue = cqe;
    }
    else
    {
        cqe->next = prev->next;                             /* insert at prev */
        prev->next = cqe;
    }

    if (cqe->next != NULL)
        cqe->next->time -= cqe->time;

    sim_interval = cpu_unit->clock_queue->time;

    if (! IS_PERCPU_UNIT(uptr))
        uptr->clock_queue_cpu = cpu_unit;

    return SCPE_OK;
}

/*
 * sim_activate_abs - activate (queue) event even if event already scheduled
 *
 * If device is system-wide (not per-CPU), caller should hold device lock.
 *
 * Inputs:
 *      uptr    =       pointer to unit
 *      event_time =    relative timeout
 * Outputs:
 *      reason  =       result (SCPE_OK if ok)
 */

t_stat sim_activate_abs (UNIT *uptr, int32 event_time)
{
    sim_cancel (uptr);
    return sim_activate (uptr, event_time);
}

t_stat sim_activate_clk_cosched (UNIT *uptr, int32 nticks)
{
    if (use_clock_thread)
    {
        /* 
         * Mark it as "clk_cosched" and time-mark for remote future.
         * Once SYNCLK event is received (nticks-th time), it will reschedule all "clk_cosched" entries for immediate execution.
         */
        return sim_activate (uptr, 0, nticks);
    }
    else
    {
        int32 t = sim_is_active (&clk_unit);
        if (t)
            t = t - 1 + (nticks - 1) * weak_read_var(tmr_poll);
        else
            t = nticks * weak_read_var(tmr_poll);
        return sim_activate (uptr, t);
    }
}

t_stat sim_activate_clk_cosched_abs (UNIT *uptr, int32 nticks)
{
    sim_cancel (uptr);
    return sim_activate_clk_cosched (uptr, nticks);
}

/*
 * sim_cancel - cancel (dequeue) event
 *
 * If device is system-wide (not per-CPU), caller should hold device lock.
 *
 * Inputs:
 *      uptr    =       pointer to unit
 * Outputs:
 *      reason  =       result (SCPE_OK if ok)
 */

t_stat sim_cancel (UNIT *uptr)
{
    RUN_SCOPE;

    if (uptr == &clk_unit && use_clock_thread)
    {
        if (cpu_unit->clk_active)
        {
            cpu_unit->clk_active = FALSE;
            /* 
             * convert all pending co-sched entries to regular ones with scheduled execution time 
             * at estimated next clock tick 
             */
            sim_reschedule_cosched(RUN_PASS, RescheduleCosched_OnCancelClock);
        }
        return SCPE_OK;
    }

    if (IS_PERCPU_UNIT(uptr))
    {
        /*
         * Per-CPU unit, queue entry is always kept on local processor,
         * will dequeue the entry below.
         */
    }
    else if (uptr->clock_queue_cpu == cpu_unit)
    {
        /*
         * System-wide unit queued on local processor.
         * Mark unit not queued, then dequeue clock queue entry below.
         */
        uptr->clock_queue_cpu = NULL;
    }
    else
    {
        /*
         * System-wide unit queued on remote processor.
         * Mark entry not queued.
         */
        uptr->clock_queue_cpu = NULL;
        return SCPE_OK;
    }

    if (cpu_unit->clock_queue == NULL)
    {
        return SCPE_OK;
    }

    UPDATE_SIM_TIME (cpu_unit->clock_queue->time);          /* update sim time */

    clock_queue_entry* cqe = NULL;
    clock_queue_entry* prev = NULL;

    for (cqe = cpu_unit->clock_queue;  cqe != NULL;  cqe = cqe->next)
    {
        if (cqe->uptr == uptr)
        {
            if (prev == NULL)
            {
                cpu_unit->clock_queue = cqe->next;
            }
            else
            {
                prev->next = cqe->next;
            }
            break;
        }
        prev = cqe;
    }

    if (cqe != NULL)
    {
        if (cqe->next != NULL)
            cqe->next->time += cqe->time;

        /* release queue entry */
        cqe->next = cpu_unit->clock_queue_freelist;
        cpu_unit->clock_queue_freelist = cqe;

        if (cpu_unit->clock_queue != NULL)
            sim_interval = cpu_unit->clock_queue->time;
        else
            sim_interval = cpu_unit->noqueue_time = NOQUEUE_WAIT;
    }
    
    return SCPE_OK;
}

/* 
 * sim_is_active - test for entry in queue, return activation time
 *
 * If device is system-wide (not per-CPU), caller should hold device lock.
 *
 *  Inputs:
 *       uptr   =   pointer to unit
 *  Outputs:
 *       result =  0 if inactive
 *                 otherwise if active:
 *                     for per-CPU devices:  absolute activation time + 1
 *                     for system-wide (non per-CPU) units queued on the local CPU:  absolute activation time + 1
 *                     for system-wide (non per-CPU) units queued on the remote CPU:  1
 *
 * Note that for system-wide devices queued on remote CPU this function will return 1,
 * not activation time value.
 */

int32 sim_is_active (UNIT *uptr)
{
    RUN_SCOPE;

    if (uptr == &clk_unit && use_clock_thread)
    {
        if (cpu_unit->clk_active)
            return synclk_expected_next(RUN_PASS) + 1;
        else
            return 0;
    }

    if (! IS_PERCPU_UNIT(uptr))
    {
        if (uptr->clock_queue_cpu == NULL)
            return 0;
        if (uptr->clock_queue_cpu != cpu_unit)
            return 1;
    }

    int32 accum = 0;

    for (clock_queue_entry *cqe = cpu_unit->clock_queue;  cqe != NULL;  cqe = cqe->next)
    {
        if (cqe == cpu_unit->clock_queue)
        {
            if (sim_interval > 0)
                accum += sim_interval;
        }
        else 
        {
            accum += cqe->time;
        }

        if (cqe->uptr == uptr)
        {
            if (cqe->clk_cosched)
            {
                /* implies use_clock_thread, provide just an estimate / boolean flag */
                return synclk_expected_next(RUN_PASS) + (cqe->clk_cosched - 1) * weak_read_var(tmr_poll) + 1;
            }
            else
            {
                return accum + 1;
            }
        }
    }

    return 0;
}

/*
 * Take all CLK cosched entries out of the queue and requque them back according to 'how'.
 * Updates sim_interval.
 */
void sim_reschedule_cosched(RUN_DECL, RescheduleCoschedHow how)
{
    if (cpu_unit->clock_queue == NULL)
        return;

    UPDATE_SIM_TIME (cpu_unit->clock_queue->time);

    clock_queue_entry* cosched_list = NULL;
    clock_queue_entry* unlinked_list = NULL;
    clock_queue_entry* cosched_later_list = NULL;
    clock_queue_entry* unlinked_later_list = NULL;
    clock_queue_entry* cqe;
    clock_queue_entry* xqe;
    clock_queue_entry* prev = NULL;

    /* 
     * move all clk_cosched entries from CPU clock_queue to unlinked_list/unlinked_later_list
     */
    for (cqe = cpu_unit->clock_queue;  cqe; )
    {
        if (cqe->clk_cosched)
        {
            xqe = cqe;

            if (cqe->next)
                cqe->next->time += cqe->time;

            if (prev == NULL)
                cpu_unit->clock_queue = cqe->next;
            else
                prev->next = cqe->next;

            cqe = cqe->next;

            if (how == RescheduleCosched_OnSynClk && xqe->clk_cosched > 1)
            {
                xqe->next = unlinked_later_list;
                unlinked_later_list = xqe;
            }
            else
            {
                xqe->next = unlinked_list;
                unlinked_list = xqe;
            }
        }
        else
        {
            prev = cqe;
            cqe = cqe->next;
        }
    }

    switch (how)
    {
    case RescheduleCosched_RequeueOnProcessEvent:
        if (unlinked_list)
        {
            /* move CLK-coscheduled entries towards the tail of the queue */
            cosched_list = reverse_cqe_list(unlinked_list);
            setup_cotimed_cqe_list(cosched_list, -1);
            insert_cotimed_cqe_list(RUN_PASS, cosched_list, CLK_COSCHED_DUMMY_TIME);
        }
        break;

    case RescheduleCosched_OnCancelClock:
        if (unlinked_list)
        {
            /* convert CLK-coscheduled entries to regular entries scheduled at estimated clock expiration time */
            cosched_list = reverse_cqe_list(unlinked_list);
            int32 till_next_tick = synclk_expected_next(RUN_PASS);
            int32 tick_length = weak_read_var(tmr_poll);
            while (cqe = cosched_list)
            {
                cosched_list = cqe->next;
                cqe->next = NULL;
                int32 newtime = till_next_tick + tick_length * (cqe->clk_cosched - 1);
                cqe->clk_cosched = 0;
                insert_cotimed_cqe_list(RUN_PASS, cqe, newtime);
            }
        }
        break;

    case RescheduleCosched_OnSynClk:
        if (unlinked_list)
        {
            /* convert entries with (clk_cosched == 1) to regular entries scheduled for immediate processing */
            cosched_list = reverse_cqe_list(unlinked_list);
            setup_cotimed_cqe_list(cosched_list, 0);
            insert_cotimed_cqe_list(RUN_PASS, cosched_list, 0);
        }
        if (unlinked_later_list)
        {
            /* for entries with (clk_cosched > 1) decerement clk_cosched and requeue them far end of the queue */
            cosched_later_list = reverse_cqe_list(unlinked_later_list);
            for (cqe = cosched_later_list;  cqe;  cqe = cqe->next)
            {
                cqe->clk_cosched--;
                cqe->time = 0;
            }
            insert_cotimed_cqe_list(RUN_PASS, cosched_later_list, CLK_COSCHED_DUMMY_TIME);
        }
        break;
    }

    sim_interval = cpu_unit->clock_queue->time;
}

static clock_queue_entry* reverse_cqe_list(clock_queue_entry* list)
{
    clock_queue_entry* result = NULL;
    clock_queue_entry* cqe;

    while ((cqe = list) != NULL)
    {
        list = cqe->next;
        cqe->next = result;
        result = cqe;
    }

    return result;
}

static void setup_cotimed_cqe_list(clock_queue_entry* list, int32 nticks)
{
    for (clock_queue_entry* cqe = list;  cqe;  cqe = cqe->next)
    {
        if (nticks >= 0)
            cqe->clk_cosched = nticks;
        cqe->time = 0;
    }
}

static void insert_cotimed_cqe_list(RUN_DECL, clock_queue_entry* list, int32 newtime)
{
    clock_queue_entry* prev = NULL;
    clock_queue_entry* last;
    clock_queue_entry* cqe;

    if (list == NULL)
        return;

    for (cqe = list;  cqe;  cqe = cqe->next)
        last = cqe;

    /* insert entries at newtime */

    int32 accum = 0;

    for (cqe = cpu_unit->clock_queue; cqe != NULL; cqe = cqe->next)
    {
        if (newtime < accum + cqe->time)
            break;
        accum = accum + cqe->time;
        prev = cqe;
    }

    if (prev == NULL)
    {
        /* insert at head */
        last->next = cpu_unit->clock_queue;
        cpu_unit->clock_queue = list;
        cpu_unit->clock_queue->time = newtime;

        if (last->next)
            last->next->time -= newtime;
    }
    else
    {
        /* insert at prev */
        last->next = prev->next;
        prev->next = list;
        list->time = newtime - accum;
        if (last->next)
            last->next->time -= list->time;
    }
}

/*
 * Drop entries off the front of clock queue that had been migrated to another processor.
 * Updates sim_interval.
 */
void sim_flush_migrated_clock_queue_entries(RUN_DECL)
{
    for (;;)
    {
        /* 
         * check if front element had been migrated
         */

        clock_queue_entry* cqe = cpu_unit->clock_queue;
        if (cqe == NULL)
            break;
        UNIT* uptr = cqe->uptr;
        if (IS_PERCPU_UNIT(uptr) || uptr->clock_queue_cpu == cpu_unit)
            break;

        /* 
         * remove front element
         */

        /* update sim time */
        UPDATE_SIM_TIME (cqe->time);

        cpu_unit->clock_queue = cqe->next;

        if (cqe->next != NULL)
            cqe->next->time += cqe->time;

        /* update remaining interval count */
        if (cpu_unit->clock_queue != NULL)
            sim_interval = cpu_unit->clock_queue->time;
        else
            sim_interval = cpu_unit->noqueue_time = NOQUEUE_WAIT;

        /* release queue entry */
        cqe->next = cpu_unit->clock_queue_freelist;
        cpu_unit->clock_queue_freelist = cqe;
    }
}

/*
 * Called when SYNCLK is received.
 *
 * Calculate interval of protection for device activity during which a subsequent SYNCLK
 * will not be processed immediately if received, but instead will be postponed till after
 * the end of interval.
 */
int32 sim_calculate_device_activity_protection_interval(RUN_DECL)
{
    int32 accum = 0;
    int32 res = 0;

    for (clock_queue_entry *cqe = cpu_unit->clock_queue;  cqe != NULL;  cqe = cqe->next)
    {
        if (cqe == cpu_unit->clock_queue)
        {
            if (sim_interval > 0)
                accum += sim_interval;
        }
        else 
        {
            accum += cqe->time;
        }

        if ((uint32) accum > synclk_safe_cycles)
            break;

        if (cqe->clk_cosched || cqe->uptr == cpu_unit || cqe->uptr == &sim_throt_unit)
            continue;

        res = accum;
    }

    return res;
}

/*
 * Check if current CPU has pending events for system-wide (non-percpu) devices
 */
t_bool sim_cpu_has_syswide_events(RUN_DECL)
{
    for (clock_queue_entry *cqe = cpu_unit->clock_queue;  cqe != NULL;  cqe = cqe->next)
    {
        UNIT* uptr = cqe->uptr;
        if (!IS_PERCPU_UNIT(uptr) && uptr->clock_queue_cpu == cpu_unit)
            return TRUE;
    }
    return FALSE;
}

/*
 * Called on the primary processor after a secondary had been shut down.
 * Requeue any pending events for system-wide devices from halted secondary VCPU's event queue
 * (that is no longer processed) by moving them to the primary's queue.
 */
void sim_requeue_syswide_events(RUN_DECL)
{
    int nentries;
    t_bool locked = FALSE;

    if (!cpu_unit->is_primary_cpu())
        panic("sim_requeue_syswide_events called on secondary CPU");

    /* scan all secondary CPUs and handle those who requested requeueing service */
    for (uint32 cpu_ix = 0;  cpu_ix < sim_ncpus;  cpu_ix++)
    {
        /* (re)acquire cpu db lock and process next CPU */
        if (! locked)
        {
            cpu_database_lock->lock();
            locked = TRUE;
        }

        if (cpu_running_set.is_set(cpu_ix))
            continue;
        CPU_UNIT* xcpu = cpu_units[cpu_ix];
        if (xcpu == cpu_unit || !xcpu->cpu_requeue_syswide_pending || xcpu->cpu_state != CPU_STATE_STANDBY)
            continue;

        /* copy matching clock event queue entries info to temporary buffer */
        nentries = 0;
        int32 qtime =  cpu_sim_interval(xcpu);
        for (clock_queue_entry *cqe = xcpu->clock_queue;  cqe != NULL;  cqe = cqe->next)
        {
            UNIT* uptr = cqe->uptr;
            if (cqe != xcpu->clock_queue)
                qtime += cqe->time;
            if (! IS_PERCPU_UNIT(uptr))
            {
                sim_requeue_info[nentries].uptr = cqe->uptr;
                sim_requeue_info[nentries].time = qtime;
                sim_requeue_info[nentries].clk_cosched = cqe->clk_cosched;
                nentries++;
            }
        }
        xcpu->cpu_requeue_syswide_pending = FALSE;
        if (nentries == 0)  continue;

        /* temporarily release cpu db lock */
        cpu_database_lock->unlock();
        locked = FALSE;

        /* process entries */
        for (int k = 0;  k < nentries;  k++)
        {
            clock_queue_entry_info* cq = sim_requeue_info + k;
            UNIT* uptr = cq->uptr;
            if (uptr->lock)  uptr->lock->lock();
            if (uptr->clock_queue_cpu == xcpu)
            {
                if (cq->clk_cosched)
                    sim_activate_clk_cosched (cq->uptr, cq->clk_cosched);
                else
                    sim_activate (cq->uptr, cq->time);
            }
            if (uptr->lock)  uptr->lock->unlock();
        }
    }

    if (locked)
        cpu_database_lock->unlock();
}

/* sim_gtime - return global time
   sim_grtime - return global time with rollover

   Inputs: none
   Outputs:
        time    =       global time
*/

double sim_gtime (RUN_DECL)
{
    UPDATE_CPU_SIM_TIME();
    return cpu_unit->sim_time;
}

uint32 sim_grtime (RUN_DECL)
{
    UPDATE_CPU_SIM_TIME();
    return cpu_unit->sim_rtime;
}

/*
 * Bind specified lock for all units of the device
 */
void sim_bind_devunits_lock(DEVICE* dptr, smp_lock* lock)
{
    for (uint32 k = 0; k < dptr->numunits; k++)
        dptr->units[k]->lock = lock;
}

/* Breakpoint package.  This module replaces the VM-implemented one
   instruction breakpoint capability.

   Breakpoints are stored in table sim_brk_tab, which is ordered by address for
   efficient binary searching.  A breakpoint consists of a four entry structure:

        addr                    address of the breakpoint
        type                    types of breakpoints set on the address
                                a bit mask representing letters A-Z
        cnt                     number of iterations before breakp is taken
        action                  pointer command string to be executed
                                when break is taken

   sim_brk_summ is a summary of the types of breakpoints that are currently set (it
   is the bitwise OR of all the type fields).  A simulator need only check for
   a breakpoint of type X if bit SWMASK('X') is set in sim_brk_sum.

   The package contains the following public routines:

        sim_brk_init            initialize
        sim_brk_set             set breakpoint
        sim_brk_clr             clear breakpoint
        sim_brk_clrall          clear all breakpoints
        sim_brk_show            show breakpoint
        sim_brk_showall         show all breakpoints
        sim_brk_test            test for breakpoint
        sim_brk_npc             PC has been changed
        sim_brk_clract          clear pending actions in CPUs

   Initialize breakpoint system.
*/

t_stat sim_brk_init (void)
{
    RUN_SCOPE;
    sim_brk_lnt = SIM_BRK_INILNT;
    sim_brk_tab = (BRKTAB *) calloc (sim_brk_lnt, sizeof (BRKTAB));
    if (sim_brk_tab == NULL)
        return SCPE_MEM;
    sim_brk_ent = sim_brk_ins = 0;
    // sim_brk_act = NULL;
    sim_brk_npc (RUN_PASS, 0);
    return SCPE_OK;
}

/* Search for a breakpoint in the sorted breakpoint table */

BRKTAB *sim_brk_fnd (t_addr loc)
{
    int32 lo, hi, p;
    BRKTAB *bp;

    if (sim_brk_ent == 0)                                   /* table empty? */
    {
        sim_brk_ins = 0;                                    /* insrt at head */
        return NULL;                                        /* sch fails */
    }
    lo = 0;                                                 /* initial bounds */
    hi = sim_brk_ent - 1;

    do
    {
        p = (lo + hi) >> 1;                                 /* probe */
        bp = sim_brk_tab + p;                               /* table addr */
        if (loc == bp->addr)                                /* match? */
            return bp;
        else if (loc < bp->addr)                            /* go down? p is upper */
            hi = p - 1;
        else
            lo = p + 1;                                     /* go up? p is lower */
    }
    while (lo <= hi);

    if (loc < bp->addr)                                     /* insrt before or */
        sim_brk_ins = p;
    else
        sim_brk_ins = p + 1;                                /* after last sch */
    return NULL;
}

/* Insert a breakpoint */

BRKTAB *sim_brk_new (t_addr loc)
{
    int32 i, t;
    BRKTAB *bp, *newp;

    if (sim_brk_ins < 0)
        return NULL;
    if (sim_brk_ent >= sim_brk_lnt)                         /* out of space? */
    {
        t = sim_brk_lnt + SIM_BRK_INILNT;                   /* new size */
        newp = (BRKTAB *) calloc (t, sizeof (BRKTAB));      /* new table */
        if (newp == NULL)                                   /* can't extend */
            return NULL;
        for (i = 0; i < sim_brk_lnt; i++)                   /* copy table */
            *(newp + i) = *(sim_brk_tab + i);
        free (sim_brk_tab);                                 /* free old table */
        sim_brk_tab = newp;                                 /* new base, lnt */
        sim_brk_lnt = t;
    }
    if (sim_brk_ins != sim_brk_ent)                         /* move needed? */
    {
        for (bp = sim_brk_tab + sim_brk_ent;
             bp > sim_brk_tab + sim_brk_ins; bp--)
            *bp = *(bp - 1);
    }
    bp = sim_brk_tab + sim_brk_ins;
    bp->addr = loc;
    bp->typ = 0;
    bp->cnt = 0;
    bp->act = NULL;
    sim_brk_ent = sim_brk_ent + 1;
    return bp;
}

/* Set a breakpoint of type sw */

t_stat sim_brk_set (t_addr loc, int32 sw, int32 ncnt, char *act)
{
    BRKTAB *bp;

    if (sw == 0) sw = sim_brk_dflt;
    if ((sim_brk_types & sw) == 0)
        return SCPE_NOFNC;
    bp = sim_brk_fnd (loc);                                 /* present? */
    if (!bp)                                                /* no, allocate */
        bp = sim_brk_new (loc);
    if (!bp)                                                /* still no? mem err */
        return SCPE_MEM;
    bp->typ = sw;                                           /* set type */
    bp->cnt = ncnt;                                         /* set count */
    if (bp->act != NULL && act != NULL)                     /* replace old action? */
    {
        free (bp->act);                                     /* deallocate */
        bp->act = NULL;                                     /* now no action */
    }
    if (act != NULL && *act != 0)                           /* new action? */
    {
        char *newp = (char *) calloc (CBUFSIZE, sizeof (char)); /* alloc buf */
        if (newp == NULL)                                   /* mem err? */
            return SCPE_MEM;
        strncpy (newp, act, CBUFSIZE);                      /* copy action */
        bp->act = newp;                                     /* set pointer */
    }
    sim_brk_summ = sim_brk_summ | sw;
    return SCPE_OK;
}

/* Clear a breakpoint */

t_stat sim_brk_clr (t_addr loc, int32 sw)
{
    BRKTAB *bp = sim_brk_fnd (loc);

    if (!bp)                                                /* not there? ok */
        return SCPE_OK;
    if (sw == 0)
        sw = SIM_BRK_ALLTYP;
    bp->typ = bp->typ & ~sw;
    if (bp->typ)                                            /* clear all types? */
        return SCPE_OK;
    if (bp->act != NULL)                                    /* deallocate action */
        free (bp->act);
    for ( ; bp < (sim_brk_tab + sim_brk_ent - 1); bp++)     /* erase entry */
        *bp = *(bp + 1);
    sim_brk_ent = sim_brk_ent - 1;                          /* decrement count */
    sim_brk_summ = 0;                                       /* recalc summary */
    for (bp = sim_brk_tab; bp < (sim_brk_tab + sim_brk_ent); bp++)
        sim_brk_summ = sim_brk_summ | bp->typ;
    return SCPE_OK;
}

/* Clear all breakpoints */

t_stat sim_brk_clrall (int32 sw)
{
    BRKTAB *bp;

    if (sw == 0) sw = SIM_BRK_ALLTYP;
    for (bp = sim_brk_tab; bp < (sim_brk_tab + sim_brk_ent); )
    {
        if (bp->typ & sw)
            sim_brk_clr (bp->addr, sw);
        else bp++;
    }
    return SCPE_OK;
}

/* Show a breakpoint */

t_stat sim_brk_show (SMP_FILE *st, t_addr loc, int32 sw)
{
    BRKTAB *bp = sim_brk_fnd (loc);
    DEVICE *dptr;
    int32 i, any;

    if (sw == 0)
        sw = SIM_BRK_ALLTYP;
    if (!bp || (!(bp->typ & sw)))
        return SCPE_OK;
    dptr = sim_dflt_dev;
    if (dptr == NULL)
        return SCPE_OK;
    if (sim_vm_fprint_addr)
        sim_vm_fprint_addr (st, dptr, loc);
    else fprint_val (st, loc, dptr->aradix, dptr->awidth, PV_LEFT);
    fprintf (st, ":\t");
    for (i = any = 0; i < 26; i++) {
        if ((bp->typ >> i) & 1) {
            if (any)
                fprintf (st, ", ");
            fputc (i + 'A', st);
            any = 1;
            }
        }
    if (bp->cnt > 0)
        fprintf (st, " [%d]", bp->cnt);
    if (bp->act != NULL)
        fprintf (st, "; %s", bp->act);
    fprintf (st, "\n");
    return SCPE_OK;
}

/* Show all breakpoints */

t_stat sim_brk_showall (SMP_FILE *st, int32 sw)
{
    BRKTAB *bp;

    if (sw == 0)
        sw = SIM_BRK_ALLTYP;
    for (bp = sim_brk_tab; bp < (sim_brk_tab + sim_brk_ent); bp++)
    {
        if (bp->typ & sw)
            sim_brk_show (st, bp->addr, sw);
    }
    return SCPE_OK;
}

/* Test for breakpoint */

uint32 sim_brk_test (RUN_DECL, t_addr loc, uint32 btyp)
{
    uint32 spc = (btyp >> SIM_BKPT_V_SPC) & (SIM_BKPT_N_SPC - 1);
    BRKTAB* bp = sim_brk_fnd (loc);

    if (bp && (btyp & bp->typ))                             /* in table, type match? */
    {
        /* previous location? */
        if (cpu_unit->sim_brk_pend[spc] && (loc == cpu_unit->sim_brk_ploc[spc]))
        {
            return 0;
        }
        if (--bp->cnt > 0)                                  /* count > 0? */
        {
            return 0;
        }
        bp->cnt = 0;                                        /* reset count */
        cpu_unit->sim_brk_ploc[spc] = loc;                  /* save location */
        cpu_unit->sim_brk_pend[spc] = TRUE;                 /* don't do twice */
        cpu_unit->sim_brk_act = bp->act;                    /* set up actions */
        return (btyp & bp->typ);
    }
    cpu_unit->sim_brk_pend[spc] = FALSE;
    return 0;
}

static t_bool sim_brk_is_action_pending ()
{
    for (uint32 k = 0;  k < sim_ncpus;  k++)
    {
        if (cpu_units[k]->sim_brk_act)
            return TRUE;
    }
    return FALSE;
}

t_bool sim_brk_is_in_action ()
{
    return sim_brk_action_stack.depth() != 0;
}

/* Get next pending action, if any */

sim_cstream* sim_brk_get_action_script ()
{
    if (sim_brk_is_action_pending ())
    {
        if (sim_brk_action_stack.depth() >= 8)
        {
            smp_printf ("Breakpoint action script exceeded safety depth limit, ignoring breakpoint action commands\n");
            if (sim_log)
                fprintf (sim_log, "Breakpoint action script exceeded safety depth limit, ignoring breakpoint action commands\n");
            sim_brk_continue = FALSE;
            return NULL;
        }

        /*
         * build script of the structure:
         *
         *    CPU ID n1
         *    commands from action for n1
         *    ....
         *    CPU ID nx
         *    commands from action for nx
         *    CPU ID original
         *
         */
        sim_cstream* sim_try_volatile script = NULL;
        char* sim_try_volatile xact = NULL;
        sim_try
        {
            t_bool restore_cpu = FALSE;
            script = new sim_cstream();
            char cmd[100];
            for (uint32 k = 0;  k < sim_ncpus;  k++)
            {
                CPU_UNIT* xcpu = cpu_units[k];
                if (xcpu->sim_brk_act)
                {
                    t_bool did_setcpu = FALSE;
                    /* accumulate action commands */
                    if (xact)  free(xact);
                    xact = dupstr_exc(xcpu->sim_brk_act);
                    char* cp = strtok(xact, ";");
                    while (cp)
                    {
                        while (isspace(*cp)) cp++;
                        if (*cp)
                        {
                            if (!did_setcpu && sim_ncpus > 1)
                            {
                                sprintf(cmd, "CPU ID %d\n", xcpu->cpu_id);
                                script->append(cmd);
                                did_setcpu = TRUE;
                                restore_cpu = TRUE;
                            }
                            script->append(cp);
                            script->append('\n');
                        }
                        cp = strtok(NULL, ";");
                    }
                }
            }

            if (restore_cpu)
            {
                sprintf(cmd, "CPU ID %d\n", sim_dflt_cpu->cpu_id);
                script->append(cmd);
            }

            sim_brk_action_stack.push((sim_cstream_ptr_t) script);
            sim_brk_clract ();
            if (xact)
            {
                free(xact);
                xact = NULL;
            }
        }
        sim_catch (sim_exception_SimError, exc)
        {
            fprintf (smp_stderr, "Error while processing breakpoint actions: %s\n", exc->get_message());
            exc->checkAutoDelete();
            delete script;
            if (xact)  free(xact);
            sim_brk_continue = FALSE;
            return NULL;
        }
        sim_end_try
    }

    if (sim_brk_action_stack.depth() == 0)
        return NULL;

    return sim_brk_action_stack.peek();
}

const char* sim_brk_end_action_script (sim_cstream* script)
{
    if (sim_brk_action_stack.depth() != 0 &&
        sim_brk_action_stack.peek() == script)
    {
        sim_brk_action_stack.pop();
    }
    else
    {
        panic("Internal consistency check failure: unmatched breakpoint script stack");
    }

    if (sim_brk_action_stack.depth() == 0 && sim_brk_continue)
    {
        sim_brk_continue = FALSE;
        return "cont";
    }

    return NULL;
}

/* Clear pending actions */

void sim_brk_clract (void)
{
    for (uint32 k = 0;  k < sim_ncpus;  k++)
    {
        CPU_UNIT* cpu_unit = cpu_units[k];
        cpu_unit->sim_brk_act = NULL;
    }
}

/* New PC */

void sim_brk_npc (RUN_DECL, uint32 cnt)
{
    uint32 i;

    if (cnt == 0 || cnt > SIM_BKPT_N_SPC)
        cnt = SIM_BKPT_N_SPC;

    for (i = 0; i < cnt; i++)
    {
        cpu_unit->sim_brk_pend[i] = FALSE;
        cpu_unit->sim_brk_ploc[i] = 0;
    }
}

#if 0
/* Clear breakpoint space (unused routine) */
void sim_brk_clrspc (RUN_DECL, uint32 spc)
{
    if (spc < SIM_BKPT_N_SPC)
    {
        cpu_unit->sim_brk_pend[spc] = FALSE;
        cpu_unit->sim_brk_ploc[spc] = 0;
    }
}
#endif

volatile t_bool sim_dbg_wait = FALSE;
static void dbg_wait_int_handler(int sig);

static void sim_wait_debugger(int* pargc, char* argv[])
{
    t_bool cancel_sigint_handler = FALSE;

    for (int k = 0;  k < *pargc;  k++)
    {
        if (streqi(argv[k], "--dbg-wait"))
        {
            sim_dbg_wait = TRUE;
            for (int j = k + 1;  j < *pargc;  j++)
                argv[j - 1] = argv[j];
            k--;
            (*pargc)--;
        }
    }

    if (sim_dbg_wait)
    {
        signal(SIGINT, dbg_wait_int_handler);
        cancel_sigint_handler = TRUE;
        fprintf(stderr, "Waiting for debugger to connect, press Ctrl/C to resume...\n");
    }

#if defined(_WIN32)
    while (sim_dbg_wait)
        _sleep(250);
#else
    while (sim_dbg_wait)
        sleep(1);
#endif

    if (cancel_sigint_handler)
    {
        signal(SIGINT, SIG_DFL);
        fprintf(stderr, "\nResuming...\n");
    }
}

static void dbg_wait_int_handler(int sig)
{
    sim_dbg_wait = FALSE;
}

/* Message Text */

AUTO_TLS(sim_error_text_msg_key);
const char *sim_error_text (t_stat stat)
{
    static char s_msgbuf[64];

    stat &= ~(SCPE_KFLAG|SCPE_BREAK);                       /* remove any flags */
    if (stat == SCPE_OK)
        return "No Error";
    if (stat >= SCPE_BASE && stat <= SCPE_MAX_ERR)
        return scp_errors[stat - SCPE_BASE].message;
    char* msgbuf = (char*) tls_get_value(sim_error_text_msg_key);
    if (msgbuf == NULL)
    {
        msgbuf = (char*) malloc(sizeof(s_msgbuf));
        tls_set_value(sim_error_text_msg_key, msgbuf);
    }
    if (msgbuf == NULL)  msgbuf = s_msgbuf;
    sprintf(msgbuf, "Error %d", stat);
    return msgbuf;
}

t_stat sim_string_to_stat (char *cptr, t_stat *stat)
{
    char gbuf[CBUFSIZE];
    int32 cond;

    *stat = SCPE_ARG;
    cptr = get_glyph (cptr, gbuf, 0);
    if (0 == memcmp("SCPE_", gbuf, 5))
        strcpy (gbuf, gbuf + 5);   /* skip leading SCPE_ */
    for (cond = 0;  cond <= SCPE_MAX_ERR - SCPE_BASE;  cond++)
    {
        if (0 == strcmp(scp_errors[cond].code, gbuf))
        {
            cond += SCPE_BASE;
            break;
        }
    }
    if (cond > SCPE_MAX_ERR - SCPE_BASE)          /* not found? */
    {
        if (0 == (cond = strtol(gbuf, NULL, 0)))  /* try explicit number */
            return SCPE_ARG;
    }
    if (cond > SCPE_MAX_ERR)
        return SCPE_ARG;
    *stat = cond;
    return SCPE_OK;    
}

/* Debug printout routines, from Dave Hittner */

const char* debug_bstates = "01_^";
const char* debug_fmt     = "DBG> %s %s: ";
int32 debug_unterm  = 0;

/* Finds debug phrase matching bitmask from from device DEBTAB table */

static char* get_dbg_verb (uint32 dbits, DEVICE* dptr)
{
    static char* debtab_none    = "DEBTAB_ISNULL";
    static char* debtab_nomatch = "DEBTAB_NOMATCH";
    int32 offset = 0;

    if (dptr->debflags == 0)
        return debtab_none;

    /* Find matching words for bitmask */

    while (dptr->debflags[offset].name && (offset < 32))
    {
        if (dptr->debflags[offset].mask & dbits)
            return dptr->debflags[offset].name;
        offset++;
    }
    return debtab_nomatch;
}

/* Prints standard debug prefix unless previous call unterminated */

static void sim_debug_prefix (uint32 dbits, DEVICE* dptr)
{
    if (!debug_unterm)
    {
        char* debug_type = get_dbg_verb (dbits, dptr);
        fprintf(sim_deb, debug_fmt, dptr->name, debug_type);
    }
}

/* Prints state of a register: bit translation + state (0,1,_,^)
   indicating the state and transition of the bit. States:
   0=steady(0->0), 1=steady(1->1), _=falling(1->0), ^=rising(0->1) */

void sim_debug_u16(uint32 dbits, DEVICE* dptr, const char* const* bitdefs,
    uint16 before, uint16 after, int terminate)
{
    if (sim_deb && (dptr->dctrl & dbits))
    {
        int32 i;

        sim_debug_prefix(dbits, dptr);                      /* print prefix if required */
        for (i = 15; i >= 0; i--)                           /* print xlation, transition */
        {
            int off = ((after >> i) & 1) + (((before ^ after) >> i) & 1) * 2;
            fprintf(sim_deb, "%s%c ", bitdefs[i], debug_bstates[off]);
        }
        if (terminate)
            fprintf(sim_deb, "\r\n");
        debug_unterm = terminate ? 0 : 1;                   /* set unterm for next */
    }
}

#if defined (_WIN32)
#define vsnprintf _vsnprintf
#endif
#if defined (__DECC) && defined (__VMS) && (defined (__VAX) || (__CRTL_VER <= 70311000))
#define NO_vsnprintf
#endif
#if defined( NO_vsnprintf)
#define STACKBUFSIZE 16384
#else
#define STACKBUFSIZE 2048
#endif

/* Inline debugging - will print debug message if debug file is
   set and the bitmask matches the current device debug options.
   Extra returns are added for un*x systems, since the output
   device is set into 'raw' mode when the cpu is booted,
   and the extra returns don't hurt any other systems. 
   
   Callers should be calling sim_debug() which is a macro
   defined in scp.h which evaluates the action condition before 
   incurring call overhead. */

void _sim_debug (uint32 dbits, DEVICE* dptr, const char* fmt, ...)
{
    if (sim_deb && (dptr->dctrl & dbits))
    {
        char stackbuf[STACKBUFSIZE];
        int32 bufsize = sizeof(stackbuf);
        char *buf = stackbuf;
        va_list arglist;
        int32 i, j, len;

        char* debug_type = get_dbg_verb (dbits, dptr);
        buf[bufsize-1] = '\0';
        // sim_debug_prefix(dbits, dptr);                      /* print prefix if required */

        while (1)                                           /* format passed string, args */
        {
            va_start (arglist, fmt);
#if defined(NO_vsnprintf)
#if defined(HAS_vsprintf_void)

    /* Note, this could blow beyond the buffer, and we couldn't tell */
    /* That is a limitation of the C runtime library available on this platform */

            vsprintf (buf, fmt, arglist);
            for (len = 0; len < bufsize-1; len++)
                if (buf[len] == 0) break;
#else
            len = vsprintf (buf, fmt, arglist);
#endif                                                      /* HAS_vsprintf_void */
#else                                                       /* NO_vsnprintf */
#if defined(HAS_vsnprintf_void)
            vsnprintf (buf, bufsize-1, fmt, arglist);
            for (len = 0; len < bufsize-1; len++)
                if (buf[len] == 0) break;
#else
            len = vsnprintf (buf, bufsize-1, fmt, arglist);
#endif                                                      /* HAS_vsnprintf_void */
#endif                                                      /* NO_vsnprintf */
            va_end (arglist);

    /* If it didn't fit into the buffer, then grow it and try again */

            if (len < 0 || len >= bufsize - 1)
            {
                if (buf != stackbuf)
                    free (buf);
                bufsize = bufsize * 2;
                buf = (char *) malloc (bufsize);
                if (buf == NULL)                            /* out of memory */
                    return;
                buf[bufsize-1] = '\0';
                continue;
            }
            break;
        }

    /* Output the formatted data expanding newlines where they exist */

        for (i = j = 0; i < len; ++i)
        {
            if ('\n' == buf[i])
            {
                if (i > j)
                {
                    if (debug_unterm)
                        fprintf (sim_deb, "%.*s\r\n", i-j, &buf[j]);
                    else                                    /* print prefix when required */
                        fprintf (sim_deb, "DBG> %s %s: %.*s\r\n", dptr->name, debug_type, i-j, &buf[j]);
                    debug_unterm = 0;
                }
                j = i + 1;
            }
        }
        if (i > j)
            fwrite (&buf[j], 1, i-j, sim_deb);

    /* Set unterminated flag for next time */

        debug_unterm = (len && (buf[len-1]=='\n')) ? 0 : 1;
        if (buf != stackbuf)
            free (buf);
    }
}

enum perf_object_kind
{
    PERF_OBJECT_NONE = 0,
    PERF_OBJECT_SMP_LOCK = 1
};

class perf_object
{
public:
    perf_object_kind kind;
    const char* name;
private:
    void* object;
    t_bool copied_name;

public:
    perf_object()
        { kind = PERF_OBJECT_NONE;  name = NULL;  copied_name = FALSE; object = NULL;  }
    void set(const char* name, t_bool copied_name, smp_lock* object)
        { this->kind = PERF_OBJECT_SMP_LOCK;  this->name = name;  this->copied_name = copied_name;  this->object = object; }
    void unset()
        { kind = PERF_OBJECT_NONE;  if (name && copied_name) free((void*)name);  name = NULL;  copied_name = FALSE; object = NULL; }
    smp_lock* get_smp_lock()
        { return kind == PERF_OBJECT_SMP_LOCK ? (smp_lock*) object : NULL; }
    void* get_object()
        { return object; }
};

#define MAX_PERF_OBJECTS 200
static perf_object perf_objects[MAX_PERF_OBJECTS];
int perf_objects_count = 0;
t_bool perf_objects_overflow = FALSE;

void perf_register_object(const char* name, smp_lock* object, t_bool copyname)
{
    if (copyname)
    {
        name = dupstr(name);
        if (name == NULL)
        {
            if (smp_stderr == NULL)
                fprintf(stderr, "%s: Warning: Not enough memory to register performance object\n", sim_name);
            else
                fprintf(smp_stderr, "%s: Warning: Not enough memory to register performance object\n", sim_name);
            return;
        }
    }

    if (perf_objects_count < MAX_PERF_OBJECTS)
    {
        perf_objects[perf_objects_count++].set(name, copyname, object);
    }
    else if (! perf_objects_overflow)
    {
        if (smp_stderr == NULL)
            fprintf(stderr, "%s: Warning: Performance objects list overflow\n", sim_name);
        else
            fprintf(smp_stderr, "%s: Warning: Performance objects list overflow\n", sim_name);
        perf_objects_overflow = TRUE;
    }
}

perf_object* perf_find_object(const char* name)
{
    for (int k = 0;  k < perf_objects_count;  k++)
    {
        perf_object* po = perf_objects + k;
        if (po->kind != PERF_OBJECT_NONE && po->name && streqi(po->name, name))
            return po;
    }
    return NULL;
}

void perf_unregister_object(smp_lock* object)
{
    if (object == NULL)
        return;

    for (int k = 0;  k < perf_objects_count;  k++)
    {
        perf_object* po = perf_objects + k;
        if (po->kind != PERF_OBJECT_NONE && po->get_object() == object)
            po->unset();
    }
}

typedef enum __tag_perf_cmd_verb
{
    PERF_CMD_VERB_NONE = 0,
    PERF_CMD_VERB_ON = 1,
    PERF_CMD_VERB_OFF = 2,
    PERF_CMD_VERB_RESET = 3,
    PERF_CMD_VERB_SHOW = 4
}
perf_cmd_verb;

t_stat perf_cmd (int32 flag, char *cptr)
{
    char gbuf[CBUFSIZE];
    perf_cmd_verb verb = PERF_CMD_VERB_NONE;
    perf_object* xpo = NULL;

    cptr = get_glyph (cptr, gbuf, 0);

    if (streqi(gbuf, "SHOW"))
        verb = PERF_CMD_VERB_SHOW;
    else if (streqi(gbuf, "ON"))
        verb = PERF_CMD_VERB_ON;
    else if (streqi(gbuf, "OFF"))
        verb = PERF_CMD_VERB_OFF;
    else if (streqi(gbuf, "RESET"))
        verb = PERF_CMD_VERB_RESET;
    else if (streqi(gbuf, ""))
        verb = PERF_CMD_VERB_NONE;
    else
        return SCPE_ARG;

    if (verb == PERF_CMD_VERB_NONE)
    {
        verb = PERF_CMD_VERB_SHOW;
    }
    else
    {
        cptr = get_glyph (cptr, gbuf, 0);
        if (gbuf[0])
        {
            xpo = perf_find_object(gbuf);
            if (xpo == NULL)  return SCPE_ARG;
        }
    }

    for (int k = 0;  k < perf_objects_count;  k++)
    {
        perf_object* po = perf_objects + k;

        if (po->kind == PERF_OBJECT_NONE || po->name == NULL)
            break;

        if (xpo && po != xpo)
            continue;

        smp_lock* pcs = po->get_smp_lock();

        switch (verb)
        {
        case PERF_CMD_VERB_ON:
            if (pcs)  pcs->set_perf_collect(TRUE);
            break;

        case PERF_CMD_VERB_OFF :
            if (pcs)  pcs->set_perf_collect(FALSE);
            break;

        case PERF_CMD_VERB_RESET :
            if (pcs)  pcs->perf_reset();
            break;

        case PERF_CMD_VERB_SHOW:
            if (pcs)
            {
                pcs->perf_show(smp_stdout, po->name);
                if (sim_log)  pcs->perf_show(sim_log, po->name);
            }
            break;

        case PERF_CMD_VERB_NONE:
            /* cannot happen; include here just to suppress false GCC warning */
            break;
        }
    }

    return SCPE_OK;
}

void* malloc_aligned(size_t size, size_t alignment)
{
#if defined(_WIN32)
    void* p = _aligned_malloc(size, alignment);
#elif defined(__GNUC__)
    void* p = NULL;
    if (posix_memalign(&p, alignment, size))
        p = NULL;
    /* GNU LIBC requires alignment to be multiple of sizeof(void*),
       hence alignment=4 will fail with GNU LIBC on x64 systems */
    if (p == NULL && alignment < sizeof(void*))
        p = malloc(size);
#else
#  error Unimplemented
#endif
    if ((alignment - 1) & (t_addr_val) p)
    {
        free_aligned(p);
        p = NULL;
    }
    return p;
}

void* calloc_aligned (size_t num, size_t elsize, size_t alignment)
{
    size_t xsize = num * elsize;
    void* p = malloc_aligned(xsize, alignment);
    if (p)  memset(p, 0, xsize);
    return p;
}

void free_aligned(void* p)
{
#if defined(_WIN32)
    return _aligned_free(p);
#elif defined(__GNUC__)
    free(p);
#else
#  error Unimplemented
#endif
}

void* operator_new_aligned(size_t size, size_t alignment)
{
    void* p = malloc_aligned(size, alignment);
    if (! p)  panic("Unable to allocate aligned memory");
    return p;
}

void operator_delete_aligned(void* p)
{
    if (p)  free_aligned(p);
}

void throw_sim_exception_ABORT(RUN_DECL, t_stat code)
{
    sim_exception_ABORT* sa;
    if ((sa = cpu_unit->cpu_exception_ABORT) == NULL)
    {
        sa = new sim_exception_ABORT(code, TRUE);
    }
    else
    {
        cpu_unit->cpu_exception_ABORT = NULL;
        sa->code = code;
    }
    sim_throw(sa);
}

void panic(const char* cause)
{
    sim_throw(new sim_exception_SimError(cause));
}

t_value ws_min_rd(REG* r, uint32 idx)
{
    return sim_ws_min;
}

void ws_min_wr(REG* r, uint32 idx, t_value value)
{
    sim_ws_min = (uint32) value;
    sim_ws_lock = (sim_ws_min != 0 || sim_ws_max != 0);
    sim_ws_settings_changed = TRUE;
}

t_value ws_max_rd(REG* r, uint32 idx)
{
    return sim_ws_max;
}

void ws_max_wr(REG* r, uint32 idx, t_value value)
{
    sim_ws_max = (uint32) value;
    sim_ws_lock = (sim_ws_min != 0 || sim_ws_max != 0);
    sim_ws_settings_changed = TRUE;
}

t_value ws_lock_rd(REG* r, uint32 idx)
{
    return sim_ws_lock;
}

void ws_lock_wr(REG* r, uint32 idx, t_value value)
{
    sim_ws_lock = (t_bool) value;
    if (! sim_ws_lock)
        sim_ws_min = sim_ws_max = 0;
    sim_ws_settings_changed = TRUE;
}

/* Called when locking a critical object */
void critical_lock(sim_lock_criticality_t criticality)
{
    RUN_SCOPE_RSCX_ONLY;

    if (rscx->thread_type == SIM_THREAD_TYPE_CPU)
    {
        CPU_UNIT* cpu_unit = rscx->cpu_unit;

        switch (criticality)
        {
        case SIM_LOCK_CRITICALITY_VM:
            if (++rscx->vm_critical_locks == 1 && must_control_prio())
                cpu_reevaluate_thread_priority(RUN_PASS);
            break;

        case SIM_LOCK_CRITICALITY_OS_HI:
            if (++rscx->os_hi_critical_locks == 1 && must_control_prio())
                cpu_reevaluate_thread_priority(RUN_PASS);
            break;

        case SIM_LOCK_CRITICALITY_NONE:
            break;
        }
    }
    else if (rscx->thread_type == SIM_THREAD_TYPE_CONSOLE ||
             rscx->thread_type == SIM_THREAD_TYPE_IOP)
    {
        switch (criticality)
        {
        case SIM_LOCK_CRITICALITY_VM:
            if (++rscx->vm_critical_locks == 1 && must_control_prio())
                sim_reevaluate_noncpu_thread_priority(rscx);
            break;

        case SIM_LOCK_CRITICALITY_OS_HI:
            if (++rscx->os_hi_critical_locks == 1 && must_control_prio())
                sim_reevaluate_noncpu_thread_priority(rscx);
            break;

        case SIM_LOCK_CRITICALITY_NONE:
            break;
        }
    }

    /*
     * We do not adjust priority for CLOCK thread since it is always
     * executing at very high priority (and is meant to).
     */
}

/* Called when unlocking a critical object */
void critical_unlock(sim_lock_criticality_t criticality)
{
    RUN_SCOPE_RSCX_ONLY;

    if (rscx->thread_type == SIM_THREAD_TYPE_CPU)
    {
        CPU_UNIT* cpu_unit = rscx->cpu_unit;

        switch (criticality)
        {
        case SIM_LOCK_CRITICALITY_VM:
            if (--rscx->vm_critical_locks == 0 && must_control_prio())
                cpu_reevaluate_thread_priority(RUN_PASS);
            break;

        case SIM_LOCK_CRITICALITY_OS_HI:
            if (--rscx->os_hi_critical_locks == 0 && must_control_prio())
                cpu_reevaluate_thread_priority(RUN_PASS);
            break;

        case SIM_LOCK_CRITICALITY_NONE:
            break;
        }
    }
    else if (rscx->thread_type == SIM_THREAD_TYPE_CONSOLE ||
             rscx->thread_type == SIM_THREAD_TYPE_IOP)
    {
        switch (criticality)
        {
        case SIM_LOCK_CRITICALITY_VM:
            if (--rscx->vm_critical_locks == 0 && must_control_prio())
                sim_reevaluate_noncpu_thread_priority(rscx);
            break;

        case SIM_LOCK_CRITICALITY_OS_HI:
            if (--rscx->os_hi_critical_locks == 0 && must_control_prio())
                sim_reevaluate_noncpu_thread_priority(rscx);
            break;

        case SIM_LOCK_CRITICALITY_NONE:
            break;
        }
    }

    /*
     * We do not adjust priority for CLOCK thread since it is always
     * executing at very high priority (and is meant to).
     */
}

void sim_reevaluate_noncpu_thread_priority(run_scope_context* rscx)
{
    sim_thread_priority_t prio;

    if (rscx->vm_critical_locks)
        prio = SIMH_THREAD_PRIORITY_CPU_CRITICAL_VM;
    else if (rscx->os_hi_critical_locks)
        prio = SIMH_THREAD_PRIORITY_CPU_CRITICAL_OS_HI;
    else
    {
        prio = rscx->bprio;
        if (unlikely(prio == SIMH_THREAD_PRIORITY_INVALID))
            return;
    }

    if (prio != rscx->cprio)
    {
        rscx->reevaluating_prio = TRUE;
        smp_set_thread_priority(prio);
        rscx->reevaluating_prio = FALSE;

        rscx->cprio = prio;
    }
}


/*
 * Update sim_mp_active from cpu_running_set.
 * Must be called either by any VCPU thread holding cpu_database_lock or console thread when VCPUs are paused.
 */
void sim_mp_active_update()
{
    int ncpus = 0;

    for (uint32 cpu_ix = 0;  cpu_ix < sim_ncpus;  cpu_ix++)
    {
        if (cpu_running_set.is_set(cpu_ix))
        {
            if (++ncpus >= 2)
            {
                sim_mp_active = TRUE;
                return;
            }
        }
    }

    sim_mp_active = FALSE;
}
