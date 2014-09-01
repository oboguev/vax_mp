/*
 * Thread-safe file I/O
 */

#define SIM_THREADS_H_FULL_INCLUDE
#include "sim_defs.h"

SMP_FILE* smp_stdin = NULL;
SMP_FILE* smp_stdout = NULL;
SMP_FILE* smp_stderr = NULL;

class smp_file_critical_section : public smp_lock_impl
{
public:
    // If there is an access to file from concurrent threads,
    // give it just enough spin-wait time to format the string and enter characters in FILE memory buffer (e.g. when writing log files),
    // but hibernate at OS level for actual physical file IO.
    // This produces an estimate of spin-wait cycles used below.
    smp_file_critical_section()
    {
        init(5000);
    }
};

// class SMP_FILE
// {
// public:
//     SMP_FILE* stream;
//     smp_file_critical_section* lock_cs;
// };

#define DynLock(sfd)                             \
    sfd->lock_cs->lock();                        \
    DynUnlocker _dyn_unlocker(sfd->lock_cs);

class DynUnlocker
{
private:
    smp_lock* lock;
public:
    DynUnlocker(smp_lock* lock)
    {
        this->lock = lock;
    }
    ~DynUnlocker()
    {
        lock->unlock();
    }
};

SMP_FILE* smp_file_wrap(FILE* fp)
{
    SMP_FILE* sfd = new SMP_FILE();
    sfd->stream = fp;
    sfd->lock_cs = new smp_file_critical_section();
    return sfd;
}

SMP_FILE* smp_fopen(const char* filename, const char* mode)
{
    FILE* fd = fopen(filename, mode);
    return (fd == NULL) ? NULL : smp_file_wrap(fd);
}

#if defined (USE_INT64) && defined (USE_ADDR64) && defined (__linux)
SMP_FILE* smp_fopen64(const char* filename, const char* mode)
{
    FILE* fd = fopen64(filename, mode);
    return (fd == NULL) ? NULL : smp_file_wrap(fd);
}
#endif

int fclose(SMP_FILE* sfd)
{
    sfd->lock_cs->lock();
    int res = fclose(sfd->stream);
    sfd->lock_cs->unlock();
    delete sfd->lock_cs;
    delete sfd;
    return res;
}

void smp_perror(const char* string)
{
    if (smp_stderr)  smp_stderr->lock_cs->lock();
    perror(string);
    if (smp_stderr)  smp_stderr->lock_cs->unlock();
}

int smp_putchar(int c)
{
    smp_stdout->lock_cs->lock();
    int res = putchar(c);
    smp_stdout->lock_cs->unlock();
    return res;
}

int smp_printf(const char* fmt, ...)
{
    size_t len;
    va_list va;
    va_start(va, fmt);
    if (smp_stdout)  smp_stdout->lock_cs->lock();
    if (sim_ttrun_mode && fmt[0] == '\n')
        printf("\r");
    int res = vfprintf(stdout, fmt, va);
    if (sim_ttrun_mode && (len = strlen(fmt)) && fmt[len - 1] == '\n')
        printf("\r");
    if (smp_stdout)  smp_stdout->lock_cs->unlock();
    va_end(va);
    return res;
}

int fflush(SMP_FILE* sfd)
{
    DynLock(sfd);
    return fflush(sfd->stream);
}

int fgetc(SMP_FILE* sfd)
{
    DynLock(sfd);
    return fgetc(sfd->stream);
}

char* fgets(char* string, int n, SMP_FILE* sfd)
{
    DynLock(sfd);
    return fgets(string, n, sfd->stream);
}

size_t fread(void* buffer, size_t size, size_t count, SMP_FILE* sfd)
{
    DynLock(sfd);
    return fread(buffer, size, count, sfd->stream);
}

int fputc(int c, SMP_FILE* sfd)
{
    DynLock(sfd);
    return fputc(c, sfd->stream);
}

int fputs(const char* string, SMP_FILE* sfd)
{
    DynLock(sfd);
    return fputs(string, sfd->stream);
}

size_t fwrite(const void* buffer, size_t size, size_t count, SMP_FILE* sfd)
{
    DynLock(sfd);
    return fwrite(buffer, size, count, sfd->stream);
}

int fseek(SMP_FILE* sfd, long offset, int origin)
{
    DynLock(sfd);
    return fseek(sfd->stream, offset, origin);
}

int fsetpos(SMP_FILE* sfd, const fpos_t* pos)
{
    DynLock(sfd);
    return fsetpos(sfd->stream, pos);
}

void rewind(SMP_FILE* sfd)
{
    DynLock(sfd);
    return rewind(sfd->stream);
}

int vfprintf(SMP_FILE* sfd, const char* format, va_list argptr)
{
    DynLock(sfd);
    return vfprintf(sfd->stream, format, argptr);
}

long ftell(SMP_FILE* sfd)
{
    DynLock(sfd);
    return ftell(sfd->stream);
}

int ferror(SMP_FILE* sfd)
{
    DynLock(sfd);
    return ferror(sfd->stream);
}

int feof(SMP_FILE* sfd)
{
    DynLock(sfd);
    return feof(sfd->stream);
}

int getc(SMP_FILE* sfd)
{
    DynLock(sfd);
    return getc(sfd->stream);
}

int putc(int c, SMP_FILE* sfd)
{
    DynLock(sfd);
    return putc(c, sfd->stream);
}

void clearerr(SMP_FILE* sfd)
{
    DynLock(sfd);
    return clearerr(sfd->stream);
}

int fgetpos(SMP_FILE* sfd, fpos_t* pos)
{
    DynLock(sfd);
    return fgetpos(sfd->stream, pos);
}

int _fileno(SMP_FILE* sfd)
{
    DynLock(sfd);
#if defined(_WIN32)
    return _fileno(sfd->stream);
#else
    return fileno(sfd->stream);
#endif
}

int setvbuf(SMP_FILE *sfd, char *buffer, int mode, size_t size)
{
    DynLock(sfd);
    return setvbuf(sfd->stream, buffer, mode, size);
}

int fprintf(SMP_FILE* sfd, const char* fmt, ...)
{
    size_t len;
    DynLock(sfd);
    va_list va;
    va_start(va, fmt);
    if (sim_ttrun_mode && sfd == smp_stdout && fmt[0] == '\n')
        printf("\r");
    int res = vfprintf(sfd->stream, fmt, va);
    if (sim_ttrun_mode && sfd == smp_stdout && (len = strlen(fmt)) && fmt[len - 1] == '\n')
        printf("\r");
    va_end(va);
    return res;
}

#if defined (__linux)
int fseeko64(SMP_FILE *sfd, off64_t offset, int whence)
{
    DynLock(sfd);
    return fseeko64(sfd->stream, offset, whence);
}

off64_t ftello64(SMP_FILE *sfd)
{
    DynLock(sfd);
    return ftello64(sfd->stream);
}
#endif

#if defined (__APPLE__) || defined (__FreeBSD__)
int fseeko(SMP_FILE *sfd, off_t offset, int whence)
{
    DynLock(sfd);
    return fseeko(sfd->stream, offset, whence);
}

off_t ftello(SMP_FILE *sfd)
{
    DynLock(sfd);
    return ftello(sfd->stream);
}
#endif

#if 0
// vfscanf is not implemented by Windows CRT.
// However, fscanf is not used by SIMH, so just leave it out.
int fscanf(SMP_FILE* sfd, const char* fmt, ...)
{
    DynLock(sfd);
    va_list va;
    va_start(va, fmt);
    int res = vfscanf(sfd->stream, fmt, va);
    va_end(va);
    return res;
}
#endif