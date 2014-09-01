#include "sim_defs.h"

#if defined(USE_C_TRY_CATCH)
smp_tls_key sim_seh_frame::tls;

void sim_seh_frame::unregister_object(sim_try_auto_destructable* obj)
{
    int k, n;
    t_bool found = FALSE;

    for (n = objcnt - 1;  n >= 0;  n--)
    {
        if (objlist[n] == obj)
        {
            found = TRUE;
            break;
        }
    }

    if (likely(found))
    {
        objcnt--;

        for (k = n;  k < objcnt;  k++)
            objlist[k] = objlist[k + 1];
    }
}

void sim_seh_frame::destroy_objects()
{
    while (objcnt)
    {
        objcnt--;
        objlist[objcnt]->onDestroy(TRUE);
    }
    /*
     * If exception happens in a destructor, the intended behavior is:
     *     - sim_throw inside destructor for object inside sim_try => goto catch
     *     - sim_throw inside destructor for object inside sim_catch/sim_catch_all/sim_finally => unwind and handle in outer frame
     */
}

void sim_seh_frame::do_throw(sim_exception* new_ex)
{
    sim_seh_frame* fp = get_current();
    if (unlikely(fp == NULL))
    {
        fprintf(stderr, "\r\nUnhandled exception\r\n");
        abort();
    }
    if (unlikely(new_ex == NULL))
    {
        fprintf(stderr, "\r\nTrying to throw null exception\r\n");
        abort();
    }
    fp->ex = new_ex;
    fp->destroy_objects();
    if (fp->state == SIM_SEH_STATE_DO_TRY)
        fp->state = SIM_SEH_STATE_DO_CATCH | SIM_SEH_STATE_DO_FINALLY | SIM_SEH_STATE_DO_ENDTHROW;
    else if (fp->state & SIM_SEH_STATE_DOING_FINALLY)
        fp->state = SIM_SEH_STATE_DO_ENDTHROW;
    else if (fp->state & SIM_SEH_STATE_DOING_CATCH)
        fp->state = SIM_SEH_STATE_DO_FINALLY | SIM_SEH_STATE_DO_ENDTHROW;
    barrier();
    longjmp(fp->jmpbuf, fp->state);
}

void sim_seh_frame::do_rethrow()
{
    sim_seh_frame* fp = get_current();
    if (unlikely(fp == NULL))
    {
        fprintf(stderr, "\r\nUnhandled exception\r\n");
        abort();
    }
    if (unlikely(fp->state & SIM_SEH_STATE_DOING_FINALLY))
    {
        fprintf(stderr, "\r\nsim_rethrow in finally block\r\n");
        abort();
    }
    if (likely(fp->state & SIM_SEH_STATE_DOING_CATCH))
    {
        fp->destroy_objects();
        fp->state = SIM_SEH_STATE_DO_FINALLY | SIM_SEH_STATE_DO_ENDTHROW;
        barrier();
        longjmp(fp->jmpbuf, fp->state);
    }
    else
    {
        panic("sim_rethrow not in catch block");
    }
}
#endif
