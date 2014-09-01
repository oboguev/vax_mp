/* scp.h: simulator control program headers

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

   Except as contained in this notice, the name of Robert M Supnik shall not
   be used in advertising or otherwise to promote the sale, use or other dealings
   in this Software without prior written authorization from Robert M Supnik.

   05-Dec-10    MP      Added macro invocation of sim_debug 
   09-Aug-06    JDB     Added assign_device and deassign_device
   14-Jul-06    RMS     Added sim_activate_abs
   06-Jan-06    RMS     Added fprint_stopped_gen
                        Changed arg type in sim_brk_test
   07-Feb-05    RMS     Added ASSERT command
   09-Sep-04    RMS     Added reset_all_p
   14-Feb-04    RMS     Added debug prototypes (from Dave Hittner)
   02-Jan-04    RMS     Split out from SCP
*/

#ifndef _SIM_SCP_H_
#define _SIM_SCP_H_     0

/* run_cmd parameters */

#define RU_RUN          0                               /* run */
#define RU_GO           1                               /* go */
#define RU_STEP         2                               /* step */
#define RU_CONT         3                               /* continue */
#define RU_BOOT         4                               /* boot */

/* get_sim_opt parameters */

#define CMD_OPT_SW      001                             /* switches */
#define CMD_OPT_OF      002                             /* output file */
#define CMD_OPT_SCH     004                             /* search */
#define CMD_OPT_DFT     010                             /* defaults */

typedef enum
{
    RescheduleCosched_RequeueOnProcessEvent = 1,
    RescheduleCosched_OnCancelClock = 2,
    RescheduleCosched_OnSynClk = 3
}
RescheduleCoschedHow;

/* Command processors */

t_stat reset_cmd (int32 flag, char *ptr);
t_stat exdep_cmd (int32 flag, char *ptr);
t_stat eval_cmd (int32 flag, char *ptr);
t_stat load_cmd (int32 flag, char *ptr);
t_stat run_cmd (int32 flag, char *ptr);
t_stat attach_cmd (int32 flag, char *ptr);
t_stat detach_cmd (int32 flag, char *ptr);
t_stat assign_cmd (int32 flag, char *ptr);
t_stat deassign_cmd (int32 flag, char *ptr);
t_stat save_cmd (int32 flag, char *ptr);
t_stat restore_cmd (int32 flag, char *ptr);
t_stat exit_cmd (int32 flag, char *ptr);
t_stat set_cmd (int32 flag, char *ptr);
t_stat show_cmd (int32 flag, char *ptr);
t_stat perf_cmd (int32 flag, char *ptr);
t_stat cpu_cmd (int32 flag, char *ptr);
t_stat brk_cmd (int32 flag, char *ptr);
t_stat do_cmd (int32 flag, char *ptr);
t_stat do_script (int32 flag, sim_cstream* script, const char* prompt);
t_stat assert_cmd (int32 flag, char *ptr);
t_stat help_cmd (int32 flag, char *ptr);
t_stat spawn_cmd (int32 flag, char *ptr);
t_stat echo_cmd (int32 flag, char *ptr);
t_stat process_new_brk_actions (int flag);
t_stat process_brk_actions (int flag, char** ppcmd);

/* Utility routines */

t_stat sim_process_event (RUN_DECL);
t_stat sim_activate (UNIT *uptr, int32 interval, int32 nticks = 0);
t_stat sim_activate_abs (UNIT *uptr, int32 interval);
t_stat sim_activate_clk_cosched (UNIT *uptr, int32 nticks);
t_stat sim_activate_clk_cosched_abs (UNIT *uptr, int32 nticks);
t_stat sim_cancel (UNIT *uptr);
int32 sim_is_active (UNIT *uptr);
void sim_asynch_activate (UNIT *uptr, int32 interval);
void sim_asynch_activate_abs (UNIT *uptr, int32 interval);
void sim_reschedule_cosched(RUN_DECL, RescheduleCoschedHow how);
void sim_flush_migrated_clock_queue_entries(RUN_DECL);
int32 sim_calculate_device_activity_protection_interval(RUN_DECL);
void sim_requeue_syswide_events(RUN_DECL);
void sim_async_process_io_events(RUN_DECL, t_bool* any = NULL, t_bool current_only = FALSE);
void sim_async_post_io_event(UNIT* uptr);
void sim_async_process_io_events_for_console();
double sim_gtime (RUN_DECL);
uint32 sim_grtime (RUN_DECL);
void sim_bind_devunits_lock(DEVICE *dptr, smp_lock* lock);
t_stat attach_unit (UNIT *uptr, char *cptr);
t_stat detach_unit (UNIT *uptr);
t_stat assign_device (DEVICE *dptr, char *cptr);
t_stat deassign_device (DEVICE *dptr);
t_stat reset_all (uint32 start_device);
t_stat reset_all_p (uint32 start_device);
const char *sim_dname (DEVICE *dptr);
t_stat get_yn (char *ques, t_stat deflt);
char *get_sim_opt (int32 opt, char *cptr, t_stat *st);
char *get_glyph (char *iptr, char *optr, char mchar);
char *get_glyph_nc (char *iptr, char *optr, char mchar);
t_value get_uint (char *cptr, uint32 radix, t_value max, t_stat *status);
t_stat get_double(char *cptr, double* pval);
char *get_range (DEVICE *dptr, char *cptr, t_addr *lo, t_addr *hi,
    uint32 rdx, t_addr max, char term);
t_stat get_ipaddr (char *cptr, uint32 *ipa, uint32 *ipp);
t_value strtotv (char *cptr, char **endptr, uint32 radix);
t_stat fprint_val (SMP_FILE *stream, t_value val, uint32 rdx, uint32 wid, uint32 fmt);
CTAB *find_cmd (char *gbuf);
DEVICE *find_dev (char *ptr);
DEVICE *find_dev (DIB* dibp);
DEVICE *find_unit (char *ptr, UNIT **uptr);
DEVICE *find_dev_from_unit (UNIT *uptr);
REG *find_reg (char *ptr, char **optr, DEVICE *dptr);
CTAB *find_ctab (CTAB *tab, const char *gbuf);
C1TAB *find_c1tab (C1TAB *tab, const char *gbuf);
SHTAB *find_shtab (SHTAB *tab, const char *gbuf);
BRKTAB *sim_brk_fnd (t_addr loc);
uint32 sim_brk_test (RUN_DECL, t_addr bloc, uint32 btyp);
void sim_brk_clrspc (RUN_DECL, uint32 spc);
char *match_ext (char *fnam, char *ext);
const char *sim_error_text (t_stat stat);
t_stat sim_string_to_stat (char *cptr, t_stat *cond);
void sim_debug_u16 (uint32 dbits, DEVICE* dptr, const char* const* bitdefs, uint16 before, uint16 after, int terminate);

#if defined (__DECC) && defined (__VMS) && (defined (__VAX) || (__DECC_VER < 60590001))
#  define CANT_USE_MACRO_VA_ARGS 1
#endif

#ifdef CANT_USE_MACRO_VA_ARGS
#  define _sim_debug sim_debug
   void sim_debug (uint32 dbits, DEVICE* dptr, const char* fmt, ...);
#else
   void _sim_debug (uint32 dbits, DEVICE* dptr, const char* fmt, ...);
#  define sim_debug(dbits, dptr, ...) if (unlikely(sim_deb != NULL) && ((dptr)->dctrl & dbits)) _sim_debug (dbits, dptr, __VA_ARGS__)
#endif

void fprint_stopped_gen (SMP_FILE *st, REG *pc, DEVICE *dptr);
char* read_line (char *ptr, int32 size, SMP_FILE *stream);
SMP_THREAD_ROUTINE_DECL sim_cpu_work_thread_proc (void* arg);
SMP_THREAD_ROUTINE_DECL sim_clock_thread_proc (void* arg);
void sim_ws_setup();
void sim_prefault_memory();
t_stat xdev_cmd(int32 flag, char *ptr);

/* SIM <-> CPU routines */
t_stat sim_instr(RUN_DECL);
void cpu_backstep_pc(RUN_DECL);
void init_cpu_unit_0();
void sim_init_interrupt_info();
t_bool cpu_create_cpus(uint32 ncpus);
t_stat cpu_cmd_info (SMP_FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, char *cptr);
void cpu_sync_flags(CPU_UNIT* uptr);
void sim_mp_active_update();

void cpu_update_cycles_per_second(RUN_DECL, uint32 ips, t_bool valid, uint32 os_msec);
uint32 cpu_get_cycles_per_second(RUN_DECL);
void cpu_stopping_ips_rate_update(RUN_DECL);

/* CPU state database */

extern smp_lock* cpu_database_lock;
extern uint32 cpu_cycles_mark[SIM_MAX_CPUS];
extern uint32 cpu_cycles_sleepexit[SIM_MAX_CPUS];
extern cpu_set cpu_waitset[SIM_MAX_CPUS];
extern cpu_set cpu_running_set;
extern atomic_bool sim_mp_active;

/* CPU cycles per second rate */

extern smp_lock* cpu_cycles_per_second_lock;
extern atomic_uint32_var cpu_cycles_per_second;

/* Synchronization objects to control CPUs <-> console interaction */

extern smp_semaphore* cpu_attention;
extern smp_barrier* cpu_pause_sync_barrier;
extern smp_semaphore* cpu_clock_run_gate;
extern t_bool sim_ttrun_mode;
extern t_bool use_clock_thread;
extern t_bool sim_clock_thread_created;
extern smp_thread_t sim_clock_thread;

/* VCPU affinity control */

extern t_bool sim_vcpu_per_core;

/* other globals */
extern SMP_FILE* sim_deb;
extern uint32 sim_vsmp_os;

#endif
