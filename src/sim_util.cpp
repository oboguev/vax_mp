/*
 * Utility and data structure classes
 */

#include "sim_defs.h"

#define aligned_sizeof(T) ((unsigned int)(t_addr_off)(char*)(((T*) 0) + 1))

char* dupstr(const char* s, t_bool canFail)
{
    if (! s)  return NULL;
    char* p = (char*) malloc(strlen(s) + 1);
    if (p)
    {
        strcpy(p, s);
        return p;
    }
    else
    {
        if (! canFail)
            ThrowOutOfMemory();
        return NULL;
    }
}

void ThrowOutOfMemory()
{
    panic("Out of memory");
}

/* ================================================= sim_cstream ================================================= */

sim_cstream::sim_cstream()
{
    const size_t inisize = 512;
    m_bp = (char*) malloc(inisize);
    if (! m_bp)  ThrowOutOfMemory();
    *m_bp = 0;
    m_allocsize = inisize;
    m_w = 0;
    m_r = 0;
}

sim_cstream::~sim_cstream()
{
    if (m_bp)
        free(m_bp);
}

void sim_cstream::append(const char* s)
{
    size_t len = strlen(s);
    if (m_w + len + 1 < m_allocsize)
    {
        size_t size = m_allocsize * 2;
        char* bp = (char*) realloc(m_bp, size);
        if (! bp)  ThrowOutOfMemory();
        m_bp = bp;
        m_allocsize = size;
    }
    strcpy(m_bp + m_w, s);
    m_w += len;
}

void sim_cstream::append(char c)
{
    if (m_w + 2 < m_allocsize)
    {
        size_t size = m_allocsize * 2;
        char* bp = (char*) realloc(m_bp, size);
        if (! bp)  ThrowOutOfMemory();
        m_bp = bp;
        m_allocsize = size;
    }
    m_bp[m_w++] = c;
    m_bp[m_w + 1] = '\0';
}

char* sim_cstream::read_line(char *buf, int32 bufsize)
{
    char* cp = read_next();
    if (*cp == 0 || bufsize == 0)  return NULL;
    char* xp = cp;
    while (*xp && *xp != '\n' && *xp != '\r')  xp++;
    int len = (int) (xp - cp);
    if (len >= bufsize)
        len = bufsize - 1;
    if (len) strncpy(buf, cp, len);
    buf[len] = 0;
    read_setnext(*xp ? xp + 1 : xp);
    return buf;
}

void sim_cstream::rewind()
{
    m_r = 0;
}

/* ================================================= sim_stack ================================================= */

template <class T>
sim_stack<T>::sim_stack()
{
    const int inidepth = 16;
    m_stack = (T*) malloc(aligned_sizeof(T) * inidepth);
    if (! m_stack)  ThrowOutOfMemory();
    m_depth = 0;
    m_alloc = inidepth;
}

template <class T>
sim_stack<T>::~sim_stack()
{
    if (m_stack)
        free(m_stack);
}

template <class T>
void sim_stack<T>::push(const T& value)
{
    if (m_depth >= m_alloc)
    {
        int alloc = m_alloc * 2;
        T* p = (T*) realloc(m_stack, alloc * aligned_sizeof(T));
        if (! p)  ThrowOutOfMemory();
        m_stack = p;
        m_alloc = alloc;
    }
    m_stack[m_depth++] = value;
}

template <class T>
const T& sim_stack<T>::peek() const
{
    if (m_depth == 0)
        panic("Empty stack");
    return m_stack[m_depth - 1];
}

template <class T>
T sim_stack<T>::pop()
{
    if (m_depth == 0)
        panic("Empty stack");
    return m_stack[--m_depth];
}

template <class T>
void sim_stack<T>::reset()
{
    m_depth = 0;
}

/* ======================================= forced template instantiations ======================================= */

typedef sim_cstream* sim_cstream_ptr_t;
template class sim_stack<sim_cstream_ptr_t>;

char* sim_exception_SimError::nomem_msg = "Insufficient memory to record error information";