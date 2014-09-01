/*
 * sim_try framework is itended to provide a limited but efficient version of C++ like structured
 * exception handling on platforms where native C++ try/throw/catch sequence is costly, more costly
 * than setjmp/longjmp.
 *
 * If symbol USE_C_TRY_CATCH is not defined, framework code compiles to natvie C++ try/catch/finally
 * sequence.
 *
 * If symbol USE_C_TRY_CATCH is defined, framework code compiles to chained setjmp/longjmp sequence.
 *
 * Code sequence should be written as follows:
 *
 *     sim_try
 *     {
 *         ... code ...
 *     }
 *     sim_catch(T1, e1)
 *     {
 *         ... code ...
 *     }
 *     sim_catch(T2, e2)
 *     {
 *         ... code ...
 *     }
 *     sim_catch_all
 *     {
 *         ... code ...
 *     }
 *     sim_finally
 *     {
 *         ... code ...
 *     }
 *     sim_end_try
 *
 * Elements sim_catch, sim_catch_all and sim_finally are optional, but one of them should be present,
 * otherwise some C++ compilers will complain. Elements sim_try and sim_end_try are mandatory.
 *
 * To throw an exception use macro sim_throw(e).
 *
 * To rethrow an exception (from inside sim_catch or sim_catch_all block) use macro sim_rethrow.
 *
 * Do not intermix sim_try framework with C++ exception handling.
 * Do not intermix sim_try framework with setjmp/longjmp across sim_try frames.
 *
 * All automatic (on-stack) variables modified between the start of sim_try block and sim_throw whose values
 * will be used by the code later must be declared sim_try_volatile.
 *
 * All automatic (on-stack) variables modified between the start of sim_catch or sim_catch_all block and
 * sim_throw/sim_rethrow whose values will be used by the code later must be declared sim_try_volatile.
 *
 * All C++ class objects that must be destroyed on unwinding of stack frames by properly calling their
 * destructors must be derived from class sim_try_auto_destructable and have the following structure:
 *
 *     class AAA : public sim_try_auto_destructable
 *     {
 *     publuc:
 *         AAA()
 *         {
 *             ... regular AAA constructor code ...
 *             onConstructor();
 *         }
 *
 *         ~AAA()
 *         {
 *             onDestroy(FALSE);
 *         }
 *
 *         void onDestroy(t_bool unregistered)
 *         {
 *             if (onDestructor(unregistered))
 *             {
 *                 ... regular AAA destructor code ...
 *             }
 *         }
 *     };
 *
 * If USE_C_TRY_CATCH is defined and "break", "return" or "goto" statements are executed inside try or catch/catch_all block, 
 * that transfer control outside of the block, the finally block will not be executed.
 *
 * These statements are not meant to should never be used inside the finally block.
 *
 */

#if !defined(USE_C_TRY_CATCH)
#  define sim_try try
#  define sim_catch(T, e) catch(T* e)
#  define sim_catch_all catch (...)
#  define sim_throw(e) throw e
#  define sim_rethrow throw
#  define sim_finally finally
#  define sim_end_try

#  define sim_try_volatile
#  define sim_noreturn_int32

class sim_try_auto_destructable
{
public:
    void onConstructor() {}
    virtual void onDestroy(t_bool unregistered) = 0;
    t_bool onDestructor(t_bool unregistered) { return TRUE; }
};
#else // defined(USE_C_TRY_CATCH)
#  include <setjmp.h>
#  define sim_try_volatile volatile

class sim_seh_frame;
class sim_exception;

class sim_try_auto_destructable
{
private:
    t_bool _constructed;
    sim_seh_frame* _sim_seh_fp;
public:
    sim_try_auto_destructable()
    {
        _constructed = FALSE;
        _sim_seh_fp = NULL;
    }

    void onConstructor();
    virtual void onDestroy(t_bool unregistered) = 0;
    t_bool onDestructor(t_bool unregistered);
};

#define SIM_MAX_SEH_OBJECTS 16

#define SIM_SEH_STATE_DO_TRY          0
#define SIM_SEH_STATE_DO_CATCH        (1 << 0)
#define SIM_SEH_STATE_DO_FINALLY      (1 << 1)
#define SIM_SEH_STATE_DO_ENDTHROW     (1 << 2)

#define SIM_SEH_STATE_DOING_CATCH     (1 << 3)
#define SIM_SEH_STATE_DOING_FINALLY   (1 << 4)

class sim_seh_frame
{
public:
    jmp_buf jmpbuf;
    volatile int state;
    volatile sim_exception* ex;
private:
    volatile t_bool linked;
    volatile sim_seh_frame* prev;
    volatile int objcnt;
    sim_try_auto_destructable* volatile objlist[SIM_MAX_SEH_OBJECTS];
    static smp_tls_key tls;

public:
    SIM_INLINE static t_bool init()
    {
        return smp_tls_alloc(& tls);
    }

    SIM_INLINE static sim_seh_frame* get_current()
    {
        return (sim_seh_frame*) tls_get_value(tls);
    }

    sim_seh_frame()
    {
        linked = FALSE;
        ex = NULL;
        state = SIM_SEH_STATE_DO_TRY;
        objcnt = 0;
        prev = get_current();
        tls_set_value(tls, this);
        linked = TRUE;
    }

    SIM_INLINE ~sim_seh_frame()
    {
        unlink();
    }

    SIM_INLINE void register_object(sim_try_auto_destructable* obj)
    {
        if (unlikely(objcnt == SIM_MAX_SEH_OBJECTS))
            panic("Object count exceeeds SIM_MAX_SEH_OBJECTS");
        objlist[objcnt++] = obj;
    }

    void unregister_object(sim_try_auto_destructable* obj);
    void destroy_objects();
    SIM_INLINE void endthrow() { unlink();  do_throw((sim_exception*) this->ex); }
    static void do_throw(sim_exception* new_ex);
    static void do_rethrow();

private:
    SIM_INLINE void unlink()
    {
        if (likely(linked))
        {
            tls_set_value(tls, (void*) prev);
            linked = FALSE;
            /*
             * Note: We cannot perform destroy_objects here, since objects may have already
             *       gone off the stack (and usually will) at a point where unlink() is called.
             *       "objcnt" should normally be 0 when unlink is called. The only legitimately
             *       possible case it may be not 0 is if object destructor throws an exception
             *       when called from inside of destroy_objects() while unwinding.
             */
        }
    }
};

SIM_INLINE void sim_try_auto_destructable::onConstructor()
{
    sim_seh_frame* fp = sim_seh_frame::get_current();
    _sim_seh_fp = fp;
    _constructed = TRUE;
    if (likely(fp != NULL))
        fp->register_object(this);
}

SIM_INLINE t_bool sim_try_auto_destructable::onDestructor(t_bool unregistered)
{
    if (likely(_constructed))
    {
        if (unregistered == FALSE && _sim_seh_fp)
            _sim_seh_fp->unregister_object(this);
        _constructed = FALSE;
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

#  define sim_try                                               \
    {                                                           \
        sim_seh_frame _sim_seh_frame;                           \
        barrier();                                              \
        int _sim_seh_state = setjmp(_sim_seh_frame.jmpbuf);     \
        barrier();                                              \
        if (_sim_seh_state == SIM_SEH_STATE_DO_TRY)             \
        {

#  define sim_catch(T, e)                                                                                     \
        }                                                                                                     \
        if (unlikely((_sim_seh_state & SIM_SEH_STATE_DO_CATCH)) && ((sim_exception*) _sim_seh_frame.ex)->isType(T##_typeid))      \
        {                                                                                                     \
            _sim_seh_state &= ~(SIM_SEH_STATE_DO_CATCH | SIM_SEH_STATE_DO_ENDTHROW);                          \
            _sim_seh_frame.state |= SIM_SEH_STATE_DOING_CATCH;                                                \
            T* e = (T*) _sim_seh_frame.ex;

#  define sim_catch_all                                                                  \
        }                                                                                \
        if (unlikely(_sim_seh_state & SIM_SEH_STATE_DO_CATCH))                           \
        {                                                                                \
            _sim_seh_state &= ~(SIM_SEH_STATE_DO_CATCH | SIM_SEH_STATE_DO_ENDTHROW);     \
            _sim_seh_frame.state |= SIM_SEH_STATE_DOING_CATCH;

#  define sim_finally                                                   \
        }                                                               \
        if (_sim_seh_state & SIM_SEH_STATE_DO_FINALLY)                  \
        {                                                               \
            _sim_seh_state &= ~SIM_SEH_STATE_DO_CATCH;                  \
            _sim_seh_frame.state |= SIM_SEH_STATE_DOING_FINALLY;

#  define sim_end_try                                                   \
        }                                                               \
        if (unlikely(_sim_seh_state & SIM_SEH_STATE_DO_ENDTHROW))       \
        {                                                               \
            _sim_seh_frame.endthrow();                                  \
        }                                                               \
    }

#  define sim_throw(e) sim_seh_frame::do_throw(e)
#  define sim_rethrow sim_seh_frame::do_rethrow()
   /* to suppress compiler warning: */
#  define sim_noreturn_int32  return 0
#endif
