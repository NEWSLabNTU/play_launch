// A better model of "my desktop is laggy".
//
// The first proxy woke at 60 Hz and did nothing, and 144 CPU burners barely
// touched it: CFS wakes a sleeper promptly however deep the runqueue, so a task
// that needs microseconds always gets them. A compositor is not that. It wakes
// every ~16.6 ms and needs several MILLISECONDS of CPU to build a frame, and it
// competes for those milliseconds with everything else runnable.
//
// So the thing to measure is frame COMPLETION: wake on the deadline, do a fixed
// amount of work, and count how often the frame lands late. That is what a user
// sees as lag.
//
//   frame_budget <hz> <work_iters> <seconds>
//
// `work_iters` must be CALIBRATED ONCE ON AN IDLE MACHINE and then held fixed
// across arms. Calibrating inside the measured run was the first version's bug:
// under 144 burners the 50 ms calibration window itself only gets a twelfth of
// a CPU, so it concluded the machine was twelve times slower and shrank the
// "4 ms" frame to 0.3 ms — making the loaded arms look BETTER than idle.
// Pass `-c` to print a count for a given millisecond budget.
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

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
  if (argc > 1 && strcmp(argv[1], "-c") == 0) {
    // Calibration mode: how many iterations is <ms> of CPU on this machine?
    double ms = argc > 2 ? atof(argv[2]) : 4.0;
    volatile double y = 1.0;
    long long t0 = ns_now();
    long it = 0;
    while (ns_now() - t0 < 200000000LL) { for (int i = 0; i < 1000; i++) y = y * 1.0000001 + 1.0; it++; }
    printf("%ld\n", (long)((double)it / (double)(ns_now() - t0) * ms * 1000000.0));
    return 0;
  }
  int hz = argc > 1 ? atoi(argv[1]) : 60;
  long work_iters = argc > 2 ? atol(argv[2]) : 100000;
  int secs = argc > 3 ? atoi(argv[3]) : 8;

  long long period = 1000000000LL / hz;
  int n = hz * secs;
  long long *frame = calloc(n, sizeof(*frame));

  volatile double x = 1.0;

  long long next = ns_now() + period;
  int late = 0;
  for (int i = 0; i < n; i++) {
    struct timespec ts = {next / 1000000000LL, next % 1000000000LL};
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
    long long start = ns_now();
    for (long k = 0; k < work_iters; k++) { for (int j = 0; j < 1000; j++) x = x * 1.0000001 + 1.0; }
    long long done = ns_now();
    // How long the frame took from its deadline to completion. Over one period
    // means the next frame is already late — a dropped frame.
    frame[i] = done - next;
    if (frame[i] > period) late++;
    next += period;
  }

  qsort(frame, n, sizeof(*frame), cmp_ll);
  printf("  p50=%7.2fms  p99=%8.2fms  max=%8.2fms  late=%d/%d (%.1f%%)\n",
         frame[n / 2] / 1e6, frame[(int)(n * 0.99)] / 1e6, frame[n - 1] / 1e6,
         late, n, 100.0 * late / n);
  return 0;
}
