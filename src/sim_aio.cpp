/*
 * sim_aio.cpp: support for asynchrnonous IO
 */

#include "sim_defs.h"

/******************************************************************************************
*  aio_context helper class for async disk and tape IO,                                   *
*  both disk_context and tape_context derive from aio_context                             *
******************************************************************************************/

aio_context::aio_context(UNIT* uptr)
{
    dptr = NULL;
    dbit = 0;
    this->uptr = uptr;

    asynch_io = FALSE;
    io_thread = SMP_THREAD_NULL;
    io_thread_created = FALSE;
    io_event = NULL;
    io_flush_ack = NULL;
    io_do_flush = FALSE;
}

aio_context::~aio_context()
{
    asynch_uninit();
}

void aio_context::asynch_init(smp_thread_routine_t start_routine, void* arg)
{
    if (io_event == NULL)
        io_event = smp_simple_semaphore::create(0);
    else
        io_event->clear();

    if (io_flush_ack == NULL)
        io_flush_ack = smp_event::create();

    if (! io_thread_created)
    {
        smp_create_thread(start_routine, arg, &io_thread);
        io_thread_created = TRUE;
    }
}

/*
 * asynch_uninit will be called either via sim_disk_unload by a VCPU thread holding uptr->lock
 * (perhaps sometimes console thread holding uptr->lock if manually writing to UQSSP registers)
 * or by console thread doing detach when all VCPU threads are paused.
 * In either case mutual exclusion between callers is observed.
 */
void aio_context::asynch_uninit()
{
    if (io_thread_created)
    {
        asynch_io = FALSE;
        io_event_signal();
        smp_wait_thread(io_thread);
        io_thread_created = FALSE;
    }

    asynch_io = FALSE;
    delete io_event;        io_event = NULL;
    delete io_flush_ack;    io_flush_ack = NULL;
}

void aio_context::thread_loop()
{
    for (;;)
    {
        io_event->wait();
        volatile t_bool was_asynch_io = asynch_io;
        volatile t_bool was_flush = io_do_flush;
        io_do_flush = FALSE;
        if (has_request())
        {
            /* after seeing operation code set, issue rmb to ensure 
               request paramaters are locally visible on this CPU */
            smp_rmb();
            perform_request();
        }
        if (was_flush)
        {
            perform_flush();
            io_flush_ack->set();
        }
        if (! was_asynch_io)
            break;
    }
}

/*
 * flush() will be normally called either by a VCPU thread holding uptr->lock
 * or by console thread doing io_flush when all VCPU threads are paused
 * or by console thread manually writing to UQSSP registers when all VCPU threads are paused
 * (and also holding uptr->lock, in addition).
 * In either case mutual exclusion between callers is observed.
 */
void aio_context::flush()
{
    if (asynch_io)
    {
        io_flush_ack->clear();
        io_do_flush = TRUE;
        io_event_signal();
        io_flush_ack->wait();
    }
    else
    {
        perform_flush();
    }
}

/******************************************************************************************
*  AIO event queue                                                                        *
******************************************************************************************/

/*
 * Events are posed to AIO event queue by IO processing (IOP) threads on completion of
 * asynchronous IO request.
 *
 * Events are fetched from the queue and processed by primary VCPU's thread, since on VAX
 * only primrary processor is handling the interrupts (under VMS anyway), therefore it would
 * be wasteful to spread AIO event handling to other CPUs so they almost immediatelly re-post
 * device interrupts to the primary processor. It is much more efficient to handle all of
 * AIO postprocessing within the context of primary processor only.
 *
 * Ocassionally requests can be fetched and processed also on the console thread, but
 * within the context of primary processor as well -- when all VCPUs are suspended and
 * simulator is entering console mode or executing in the console mode.
 *
 * Thus we have multiple producers (IOP threads) but only one consumer (either primary
 * VCPU thread or console thread, when VCPU threads are paused by the console).
 *
 * To reduce overhead and avoid preemption/convoying issues, we implement AIO event
 * queue as lock-free queue. 
 *
 * In our particular case (single consumer and multiple producers) we can get by with
 * CAS or LL/SC instructions.
 *
 * If there were multiple consumers, we'd have to use lock-free queue such as described
 * in Herlihy & Shavit's "The Art of Multiprocessor's Programming" (2008), pp. 230-237
 * or Michael & Scott's "Simple, Fast and Practical Non-Blocking and Blocking Concurrent
 * Queue Algorithms" (1996). These algorithms requuire CAS2 or LL/SC and cannot be
 * implemented with CAS alone. DCAS abstraction for various platforms is handily
 * defined in libldfs sources (libldfs.org, file abstraction_dcas.c).
 *
 * On host machines where CAS (or LL/SC) instructons are not available, AIO event queue
 * would have to be implemented as lockable queue, using something like
 *
 *     AUTO_INIT_LOCK(sim_asynch_queue_lock, SIM_LOCK_CRITICALITY_NONE, 200);
 *
 * and
 *
 *     sim_asynch_queue_lock->lock();
 *     sim_asynch_queue_lock->unlock();
 *
 * Current implementaton uses CAS.
 *
 * There is no ABA problem on inserting side because:
 *
 *     - UNIT object being inserted is "owned" by inserting IOP thread
 *     - value of head pointer being replaced is confirmed atomically by CAS
 *     - no other field is modified
 *
 * There is no ABA problem on removal side because:
 *
 *     - inserion and removal happens only at the head, so a_next link of
 *       head entry is not modified by IOP thread
 *     - head pointer value cannot cycle ABA except through dequeueing,
 *       by dequeueing is done only by single retrieval thread which is
 *       synchronized to itself
 *
 * Interlocked header insertion/retrieval works as LIFO. To maintain fairness
 * and prevent saturation of the queue by single device, we convert it to FIFO
 * after the entries had been removed off the interlocked queue. Retrieval side
 * maintains list private to retrieval thread. After copying all the entries 
 * from the interlocked LIFO list, it puts them on private LIFO list. Double
 * LIFO ordering creates FIFO.
 *
 * Note that current AIO queue code assumes that device handler can use
 * either sim_async_post_io_event or sim_asynch_activate[_abs], but not both
 * for the same UNIT.
 *
 */

#if !defined(VM_VAX_MP)
#  error review code: AIO_SIGNAL_CPU sends to primary CPU
#endif

/* events are put into sim_asynch_queue by IOP threads, retrieved by primary VCPU thread */
static smp_interlocked_addr_val_var sim_asynch_queue = smp_var_init(0);

/* "aqueue" is accessed by primary VCPU thread only */
static UNIT* aqueue = NULL;

static void init_aio_data()
{
    smp_check_aligned(& sim_asynch_queue);
}
ON_INIT_INVOKE(init_aio_data);

t_bool sim_async_io_queue_isempty()
{
    return smp_interlocked_cas_done_var(& sim_asynch_queue, (t_addr_val) 0, (t_addr_val) 0);
}

/* Insert new entry at the head, in LIFO order */
#define AIO_INSERT_QUEUE(uptr)                                                                                       \
    do                                                                                                               \
    {                                                                                                                \
        (uptr)->a_next = (UNIT*) smp_var(sim_asynch_queue);                                                          \
    }                                                                                                                \
    while (! smp_interlocked_cas_done_var(& sim_asynch_queue, (t_addr_val) (uptr)->a_next, (t_addr_val) (uptr)))

#define AIO_SIGNAL_CPU()  \
    interrupt_set_int(&cpu_unit_0, IPL_ASYNC_IO, INT_V_ASYNC_IO)

void sim_async_post_io_event(UNIT* uptr)
{
    smp_pre_interlocked_wmb();
    AIO_INSERT_QUEUE(uptr);
    AIO_SIGNAL_CPU();
}

static t_stat sim_async_activate_thunk(UNIT *uptr, int32 interval)
{
    return sim_activate(uptr, interval);
}

void sim_asynch_activate(UNIT *uptr, int32 interval)
{
    t_bool signal = FALSE;

    uptr->lock->lock();

    if (uptr->a_activate_call == NULL)
    {
        uptr->a_activate_call = sim_async_activate_thunk;
        uptr->a_sim_interval = interval;
        AIO_INSERT_QUEUE(uptr);
        signal = TRUE;
    }

    uptr->lock->unlock();

    if (signal)
        AIO_SIGNAL_CPU();
}

void sim_asynch_activate_abs(UNIT *uptr, int32 interval)
{
    uptr->lock->lock();

    t_bool onqueue = (uptr->a_activate_call != NULL);
    uptr->a_activate_call = sim_activate_abs;
    uptr->a_sim_interval = interval;
    if (! onqueue)  AIO_INSERT_QUEUE(uptr);

    uptr->lock->unlock();

    if (! onqueue)  AIO_SIGNAL_CPU();
}

/*
 * will normally be invoked at thread priority level VM_CRITICAL,
 * boosted up by sent interrupt and before interrupt processing
 * recalculates thread priority down
 */
void sim_async_process_io_events(RUN_DECL, t_bool* pany, t_bool current_only)
{
#if !defined(VM_VAX_MP)
#  error review code: assumes primary CPU context
#endif
    t_bool any = FALSE;

    for (;;)
    {
        UNIT* aq = NULL;
        UNIT* uptr;

        /*
         * Dequeue entries from AIO queue and transfer them to local queue "aq",
         * reversing order of entries from LIFO to FIFO.
         */
        for (;;)
        {
            t_addr_val qe = smp_var(sim_asynch_queue);
            uptr = (UNIT*) qe;
            if (uptr == NULL)  break;
            if (smp_interlocked_cas_done_var(& sim_asynch_queue, qe, (t_addr_val) uptr->a_next))
            {
                smp_post_interlocked_rmb();
                uptr->a_next = aq;
                aq = uptr;
            }
        }

        /*
         * One's first impulse is to process entries directly off "aq", but here is the problem:
         * a_check_completion can cause device reset, which in turn can call again sim_async_process_io_events,
         * recursively. This second recursive call should be able to drain events that we just picked off the queue.
         *
         * Therefore we put events fetched into "aq" on a static thread-local queue "aqueue" and process
         * events off this queue, which will be available to recursive invocations of this routine as well.
         */
        if (aqueue == NULL)
        {
            aqueue = aq;
        }
        else
        {
            /* maintain FIFO order */
            uptr = aq;
            while (uptr->a_next)
                uptr = uptr->a_next;
            uptr->a_next = aq;
        }

        /* Now process events off "aqueue" */
        while (aqueue != NULL)
        {
            uptr = aqueue;
            aqueue = uptr->a_next;
            uptr->a_next = NULL;
            any = TRUE;

            uptr->lock->lock();
            if (uptr->a_check_completion)
                (*uptr->a_check_completion)(uptr);
            if (uptr->a_activate_call)
            {
                (*uptr->a_activate_call)(uptr, uptr->a_sim_interval);
                uptr->a_activate_call = NULL;
            }
            uptr->lock->unlock();
        }

        /* anything left? */
        if (current_only || sim_async_io_queue_isempty())
            break;
    }

    if (pany)
        *pany = any;
}

/*
 * Called by console thread when VCPUs are paused to process pending async IO events
 * and to flush async IO queue.
 */
void sim_async_process_io_events_for_console()
{
#if !defined(VM_VAX_MP)
#  error review code: assumes events are processed in the primary CPU context
#endif
    /*
     * VAX-specific: do it in the context of primary CPU
     */
    RUN_SCOPE_RSCX;
    CPU_UNIT* sv_cpu_unit = rscx->cpu_unit;
    rscx->cpu_unit = cpu_unit = &cpu_unit_0;
    sim_async_process_io_events(RUN_PASS);
    rscx->cpu_unit = sv_cpu_unit;
}
