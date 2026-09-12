/* Ground diagnostics only. Observe main-thread long nanosleeps unchanged.
 * Fixed memory; no trace I/O during calls. Dump at process shutdown. */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <errno.h>
#include <execinfo.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

static int (*next_sleep)(const struct timespec *, struct timespec *);
static struct record { double requested, start, end; int frames; void *stack[32]; char names[32][512]; } records[32];
static unsigned count, overflow;
static double now(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC,&t); return t.tv_sec+t.tv_nsec/1e9; }
__attribute__((constructor)) static void prepare(void) {
    next_sleep=dlsym(RTLD_NEXT,"nanosleep");
    void *warm[2]; backtrace(warm,2); /* Resolve unwind support before callbacks. */
}
int nanosleep(const struct timespec *request, struct timespec *remaining) {
    struct record *r=NULL;
    double seconds=request->tv_sec+request->tv_nsec/1e9;
    if (syscall(SYS_gettid)==getpid() && seconds>=0.1) {
        if (count<32) {
            r=&records[count++]; r->requested=seconds; r->start=now(); r->frames=backtrace(r->stack,32);
            for (int j=0;j<r->frames;j++) {
                Dl_info info;
                if (dladdr(r->stack[j],&info)) snprintf(r->names[j],512,"%s : %s + %td (module offset %td)",
                    info.dli_fname,info.dli_sname ? info.dli_sname : "unknown",
                    info.dli_saddr ? (char*)r->stack[j]-(char*)info.dli_saddr : 0,
                    (char*)r->stack[j]-(char*)info.dli_fbase);
            }
        }
        else { overflow++; }
    }
    int result=next_sleep(request,remaining), saved_errno=errno;
    if (r) { r->end=now(); }
    errno=saved_errno;
    return result;
}
__attribute__((destructor)) static void persist(void) {
    const char *path=getenv("GWM_P2_WAIT_PROBE_OUTPUT");
    if (!path) return;
    FILE *out=fopen(path,"wx"); if (!out) return;
    fprintf(out,"main_thread_wait_probe count=%u overflow=%u\n",count,overflow);
    for (unsigned i=0;i<count;i++) {
        struct record *r=&records[i];
        fprintf(out,"wait %u request=%.9f start=%.9f end=%.9f duration=%.9f\n",i,r->requested,r->start,r->end,r->end-r->start);
        for(int j=0;j<r->frames;j++) fprintf(out,"%s\n",r->names[j]);
    }
    fclose(out);
}
