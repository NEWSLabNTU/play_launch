// What does SCHED_BATCH cost a task that WAKES rather than computes?
//
// SCHED_BATCH's whole content is one thing: the kernel does not let the task
// preempt on wakeup. From kernel/sched/fair.c, `check_preempt_wakeup`:
//
//     /*
//      * Batch and idle tasks do not preempt non-idle tasks (their preemption
//      * is driven by the tick):
//      */
//     if (unlikely(p->policy != SCHED_NORMAL) || !sched_feat(WAKEUP_PREEMPTION))
//         return;
//
// So a batch task that becomes runnable waits for the current task to be
// preempted by the tick instead of taking the CPU immediately. For something
// that mostly computes, that is free — it was going to wait its turn anyway.
// For something that sleeps and must act on waking, it is added latency, and
// the name of the policy is the warning.
//
// This measures the second case: a "driver" woken at a fixed rate through a
// pipe while the machine is busy, reporting how late each wake was.
//
//   batch_wakeup <policy: other|batch> <hz> <seconds> <load_threads>
#define _GNU_SOURCE
#include <errno.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

static volatile int stop_load = 0;

static void *burn(void *arg)
{
  (void)arg;
  volatile double x = 1.0;
  while (!stop_load) {
    for (int i = 0; i < 100000; i++) x = x * 1.0000001 + 1.0;
  }
  return NULL;
}

static long long ns_now(void)
{
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (long long)t.tv_sec * 1000000000LL + t.tv_nsec;
}

static int cmp_ll(const void *a, const void *b)
{
  long long x = *(const long long *)a, y = *(const long long *)b;
  return (x > y) - (x < y);
}

int main(int argc, char **argv)
{
  if (argc < 5) {
    fprintf(stderr, "usage: %s other|batch <hz> <seconds> <load_threads>\n", argv[0]);
    return 2;
  }
  int batch = strcmp(argv[1], "batch") == 0;
  int hz = atoi(argv[2]);
  int secs = atoi(argv[3]);
  int nload = atoi(argv[4]);

  // Contention. Without it every wakeup finds an idle CPU and the policy makes
  // no difference at all — the measurement would say "no cost" for the wrong
  // reason.
  pthread_t *load = calloc(nload, sizeof(*load));
  for (int i = 0; i < nload; i++) pthread_create(&load[i], NULL, burn, NULL);

  int fds[2];
  if (pipe(fds) != 0) { perror("pipe"); return 1; }

  pid_t waker = fork();
  if (waker == 0) {
    // The "sensor": writes one byte per period, on time.
    close(fds[0]);
    long long period = 1000000000LL / hz;
    long long next = ns_now() + period;
    for (int i = 0; i < hz * secs; i++) {
      struct timespec ts = {next / 1000000000LL, next % 1000000000LL};
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
      long long sent = ns_now();
      if (write(fds[1], &sent, sizeof(sent)) < 0) break;
      next += period;
    }
    close(fds[1]);
    _exit(0);
  }
  close(fds[1]);

  // The "driver": blocks on the pipe, wakes, records how late it was. Policy is
  // set on THIS task only, so the load threads stay ordinary and the comparison
  // is about the waker alone.
  if (batch) {
    struct sched_param p = {0};
    if (sched_setscheduler(0, SCHED_BATCH, &p) != 0) perror("sched_setscheduler");
  }

  int n = hz * secs;
  long long *lat = calloc(n, sizeof(*lat));
  int got = 0;
  for (int i = 0; i < n; i++) {
    long long sent;
    ssize_t r = read(fds[0], &sent, sizeof(sent));
    if (r != (ssize_t)sizeof(sent)) break;
    lat[got++] = ns_now() - sent;
  }
  stop_load = 1;
  for (int i = 0; i < nload; i++) pthread_join(load[i], NULL);

  qsort(lat, got, sizeof(*lat), cmp_ll);
  double p50 = got ? lat[got / 2] / 1000.0 : 0;
  double p99 = got ? lat[(int)(got * 0.99)] / 1000.0 : 0;
  double mx = got ? lat[got - 1] / 1000.0 : 0;
  printf("  %-6s n=%-6d p50=%8.1fus  p99=%9.1fus  max=%10.1fus\n",
         batch ? "batch" : "other", got, p50, p99, mx);
  return 0;
}
