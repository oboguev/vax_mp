/*
 * Utility and data structure classes
 */

void ThrowOutOfMemory();

char* dupstr(const char* s, t_bool canFail = TRUE);
#define dupstr_exc(s)  dupstr((s), FALSE)

class sim_cstream
{
public:
    sim_cstream();
    ~sim_cstream();
    /* writer side */
    void append(const char* s);
    void append(char c);
    /* reader side */
    char* read_start() { return m_bp; }
    char* read_next() { return m_bp + m_r; }
    void read_setnext(const char* p) { m_r = p - m_bp; }
    char *read_line(char *buf, int32 bufsize);
    void rewind();

protected:
    char* m_bp;
    size_t m_w;
    size_t m_r;
    size_t m_allocsize;
};

/* Note: sim_stack is currently set up to handle only simple types
   with no constructors or destructors */
template <class T>
class sim_stack
{
public:
    sim_stack();
    ~sim_stack();
    void push(const T& value);
    const T& peek() const;
    T pop();
    int depth() const { return m_depth; }
    void reset();
protected:
    T* m_stack;
    int m_depth;
    int m_alloc;
};

template <class T, int rsize>
class sim_ring_buffer
{
    T buf[rsize];
    int maxsize;
    int count;
    int rd;
    int wr;
public:
    sim_ring_buffer()
    {
        maxsize = rsize;
        count = 0;
        rd = wr = 0;
    }
    void clear()
    {
        count = rd = wr = 0;
    }
    t_bool is_empty()
    {
        return count == 0;
    }
    t_bool put(const T& c)
    {
        if (count == maxsize)
            return FALSE;
        buf[wr] = c;
        wr = (wr + 1) % maxsize;
        count++;
        return TRUE;
    }
    t_bool get(T* c)
    {
        if (count == 0)
            return FALSE;
        *c = buf[rd];
        rd = (rd + 1) % maxsize;
        count--;
        return TRUE;
    }
};