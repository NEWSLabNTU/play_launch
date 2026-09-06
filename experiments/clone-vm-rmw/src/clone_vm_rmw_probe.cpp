// Copyright 2026 play_launch developers
// Licensed under the Apache License, Version 2.0.
//
// Does an rclcpp executor spin correctly inside a clone(CLONE_VM) child, and
// does the answer depend on the RMW backend?
//
// This mirrors what a CLONE_VM container would do, in the order a container
// does it. Nodes are built in the PARENT -- exactly as ComponentManager does
// after dlopen'ing the plugin -- and only spin() runs in the cloned child.
//
// Two children, not one, and the second is loaded while the first is already
// spinning, with a dlopen in between. That ordering is the point: dlopen grows
// the DTV of every thread in the process, and the DTV is the structure the
// dynamic-TLS resolver walks. A container hits this on every load after the
// first, so a probe that loads everything up front proves nothing about it.
//
// Every stage is announced before it is attempted, so a hang names its own
// phase instead of leaving a silent process.
//
// See README.md for what this measured.

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <sched.h>
#include <dlfcn.h>
#include <locale.h>
#include <pthread.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <sys/wait.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64.hpp>

namespace
{

constexpr std::size_t kStackSize = 8u * 1024u * 1024u;
constexpr int kChildren = 2;

// ── glibc TLS layout, discovered rather than assumed ────────────────────────
//
// The two architectures do not merely differ in offsets, they use different
// TLS variants. On x86_64 (variant II) the thread pointer IS the struct
// pthread; on aarch64 (variant I) the struct sits below it -- 1984 bytes below
// on glibc 2.35. Hardcoding either set, as the abandoned container did, writes
// into whatever happens to live at that address on the other one.
//
// So both numbers are measured at startup: the gap by comparing pthread_self()
// against the thread pointer, the tid slot by scanning for our own tid.

long g_pd_from_tp = 0;
int g_tid_off = -1;

void * thread_pointer()
{
  void * tp = nullptr;
#if defined(__aarch64__)
  __asm__ volatile("mrs %0, tpidr_el0" : "=r"(tp));
#elif defined(__x86_64__)
  syscall(SYS_arch_prctl, 0x1003 /* ARCH_GET_FS */, &tp);
#else
#error "no thread-pointer accessor for this architecture"
#endif
  return tp;
}

void discover_tls_layout()
{
  g_pd_from_tp = reinterpret_cast<char *>(pthread_self()) -
                 reinterpret_cast<char *>(thread_pointer());
  const auto tid = static_cast<pid_t>(syscall(SYS_gettid));
  char * pd = reinterpret_cast<char *>(pthread_self());
  for (int off = 0; off < 2048; off += 4) {
    int v = 0;
    std::memcpy(&v, pd + off, sizeof v);
    if (v == tid) {
      g_tid_off = off;
      return;
    }
  }
}

// ── state shared with the children ──────────────────────────────────────────
//
// MAP_SHARED rather than plain heap: the children share the address space, so
// the heap would work, but mapping it explicitly says which state is meant to
// cross the boundary.

struct Slot
{
  rclcpp::Executor * exec;
  std::atomic<int> stage;
  std::atomic<long> received;
};

struct Shared
{
  Slot slot[kChildren];
};

Shared * g_shared = nullptr;

const char * const kStage[] = {
  "not entered", "clone entry",  "tls identity",   "locale/ctype",
  "signal reset", "entered spin()", "spin() returned",
};

const char * stage_of(int index)
{
  const int s = g_shared->slot[index].stage.load();
  return (s >= 0 && s < static_cast<int>(sizeof kStage / sizeof *kStage)) ? kStage[s] : "?";
}

int child_fn(void * arg)
{
  Slot & me = g_shared->slot[reinterpret_cast<intptr_t>(arg)];
  me.stage = 1;

  // Our own identity in the fresh TLS block. Without it glibc mutex ownership
  // checks and pthread_create fail, because the block still carries whatever
  // tid the allocation inherited.
  char * pd = reinterpret_cast<char *>(thread_pointer()) + g_pd_from_tp;
  const auto tid = static_cast<pid_t>(syscall(SYS_gettid));
  if (g_tid_off >= 0) {
    std::memcpy(pd + g_tid_off, &tid, sizeof tid);
  }
  me.stage = 2;

  // _dl_allocate_tls builds the block but does not do what start_thread would:
  // the locale pointer and the per-thread ctype tables are left null, and the
  // first printf("%f") or isalpha() then dereferences them.
  uselocale(LC_GLOBAL_LOCALE);
  using ctype_init_fn = void (*)();
  if (auto * f = reinterpret_cast<ctype_init_fn>(dlsym(RTLD_DEFAULT, "__ctype_init"))) {
    f();
  }
  me.stage = 3;

  // Without CLONE_SIGHAND the child owns its signal table, so resetting it
  // here affects only this child. Equivalent to CLONE_CLEAR_SIGHAND.
  struct sigaction sa
  {
  };
  sa.sa_handler = SIG_DFL;
  for (int sig = 1; sig < _NSIG; ++sig) {
    sigaction(sig, &sa, nullptr);
  }
  me.stage = 4;

  // A core-dumping signal (SIGSEGV, SIGABRT, SIGBUS) in a task that shares its
  // mm is not a per-task event: to take the dump the kernel needs exclusive
  // access to the address space, so do_coredump() zaps every other task using
  // that mm. In a CLONE_VM container that is the container and every sibling.
  //
  // Making the child undumpable makes do_coredump() bail before it gets there,
  // so the signal kills this child alone. Set with PROBE_NO_COREDUMP to compare
  // the two behaviours; the container mode does it unconditionally.
  if (std::getenv("PROBE_NO_COREDUMP")) {
    prctl(PR_SET_DUMPABLE, 0, 0, 0, 0);
    struct rlimit no_core{0, 0};
    setrlimit(RLIMIT_CORE, &no_core);
  }

  me.stage = 5;
  me.exec->spin();  // the thing under test
  me.stage = 6;
  return 0;
}

// Everything the parent does per child, in one place, so the second load runs
// the same path as the first instead of a copy that drifts from it.
pid_t spawn_child(int index)
{
  auto p_alloc = reinterpret_cast<void * (*)(void *)>(dlsym(RTLD_DEFAULT, "_dl_allocate_tls"));
  if (!p_alloc) {
    std::fprintf(stderr, "no _dl_allocate_tls\n");
    return -1;
  }
  void * tls = p_alloc(nullptr);
  if (!tls) {
    return -1;
  }
  // CLONE_SETTLS wants the value the thread pointer should take, which is the
  // struct-pthread address adjusted back across the variant gap.
  void * newtp = reinterpret_cast<char *>(tls) - g_pd_from_tp;

  char * stack = static_cast<char *>(
    mmap(nullptr, kStackSize, PROT_READ | PROT_WRITE,
         MAP_PRIVATE | MAP_ANONYMOUS | MAP_STACK, -1, 0));
  if (stack == MAP_FAILED) {
    return -1;
  }

  // No CLONE_THREAD: a separate thread group is the whole point, since that is
  // what gives the child its own PID and its own signal disposition, so a
  // SIGSEGV kills one node instead of the container.
  const int flags = CLONE_VM | CLONE_FS | CLONE_FILES | CLONE_SETTLS | SIGCHLD;
  return clone(
    child_fn, stack + kStackSize, flags,
    reinterpret_cast<void *>(static_cast<intptr_t>(index)), nullptr, newtp, nullptr);
}

struct Loaded
{
  rclcpp::Node::SharedPtr node;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr sub;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr exec;
  pid_t pid = -1;
};

}  // namespace

int main(int argc, char ** argv)
{
  const char * rmw = std::getenv("RMW_IMPLEMENTATION");
  std::printf("RMW_IMPLEMENTATION = %s\n", rmw ? rmw : "(default)");

  discover_tls_layout();
  std::printf("layout: struct pthread at TP%+ld, tid at +%d\n", g_pd_from_tp, g_tid_off);
  if (g_tid_off < 0) {
    std::printf("FAIL: could not locate the tid slot in struct pthread\n");
    return 1;
  }

  rclcpp::init(argc, argv);
  std::printf("stage: rclcpp::init OK\n");
  std::fflush(stdout);

  g_shared = static_cast<Shared *>(
    mmap(nullptr, sizeof(Shared), PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0));
  new (g_shared) Shared{};

  auto pub_node = std::make_shared<rclcpp::Node>("clone_vm_probe_pub");
  auto pub = pub_node->create_publisher<std_msgs::msg::Int64>("probe", 10);

  Loaded loaded[kChildren];

  // Load and spawn one composable, exactly as a container does: construct in
  // the parent, then hand the executor to a cloned child.
  auto load_one = [&](int i) -> bool {
    char name[64];
    std::snprintf(name, sizeof name, "clone_vm_probe_%d", i);
    loaded[i].node = std::make_shared<rclcpp::Node>(name);
    loaded[i].sub = loaded[i].node->create_subscription<std_msgs::msg::Int64>(
      "probe", 10,
      [i](std_msgs::msg::Int64::SharedPtr m) { g_shared->slot[i].received = m->data; });
    loaded[i].exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    loaded[i].exec->add_node(loaded[i].node);
    g_shared->slot[i].exec = loaded[i].exec.get();
    std::printf("stage: [%d] node built in parent, executor ready\n", i);
    std::fflush(stdout);

    loaded[i].pid = spawn_child(i);
    if (loaded[i].pid < 0) {
      std::printf("FAIL: [%d] clone: %s\n", i, std::strerror(errno));
      return false;
    }
    std::printf("stage: [%d] cloned child pid %d\n", i, loaded[i].pid);
    std::fflush(stdout);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::printf("stage: [%d] child reached '%s'\n", i, stage_of(i));
    std::fflush(stdout);
    return true;
  };

  if (!load_one(0)) {
    return 1;
  }

  // The container hazard: the second composable is dlopen'd while the first is
  // already spinning. dlopen grows every thread's DTV, which is what the
  // dynamic-TLS resolver walks, so this is the ordering that can break a child
  // that was fine a moment ago.
  std::printf("stage: dlopen while child 0 spins\n");
  std::fflush(stdout);
  const char * libs[] = {"libpcl_common.so", "libyaml-cpp.so.0.7", "libtbb.so.12",
                         "libOpenGL.so.0"};
  int opened = 0;
  for (auto * l : libs) {
    if (dlopen(l, RTLD_NOW | RTLD_GLOBAL)) {
      ++opened;
    }
  }
  std::printf("stage: dlopen'd %d libs; child 0 now at '%s'\n", opened, stage_of(0));
  std::fflush(stdout);

  if (!load_one(1)) {
    return 1;
  }

  // ── The claim the whole model rests on ──────────────────────────────────
  // A clone child has its own thread group, so a fatal signal should end that
  // child and leave the container and its siblings running. If that does not
  // hold there is no reason to prefer this over loading as threads, because
  // the segfault boundary is the only thing it buys over them.
  if (std::getenv("PROBE_SEGV_CHILD0")) {
    std::printf("stage: sending SIGSEGV to child 0 (pid %d)\n", loaded[0].pid);
    std::fflush(stdout);
    kill(loaded[0].pid, SIGSEGV);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    // kill(pid, 0) succeeds on a zombie, so it cannot tell "died" from "still
    // running" -- read the state out of /proc instead. A child that took the
    // signal is Z until the parent reaps it.
    auto proc_state = [](pid_t pid) -> char {
      char path[64];
      std::snprintf(path, sizeof path, "/proc/%d/stat", pid);
      FILE * f = std::fopen(path, "r");
      if (!f) {
        return '-';  // gone entirely
      }
      char state = '?';
      // "pid (comm) state ..." -- comm can contain spaces, so scan past ')'.
      char buf[512] = {0};
      if (std::fgets(buf, sizeof buf, f)) {
        if (const char * close = std::strrchr(buf, ')')) {
          std::sscanf(close + 1, " %c", &state);
        }
      }
      std::fclose(f);
      return state;
    };
    const char s0 = proc_state(loaded[0].pid);
    const char s1 = proc_state(loaded[1].pid);
    const bool c0_dead = (s0 == 'Z' || s0 == '-' || s0 == 'X');
    const bool c1_alive = (s1 == 'S' || s1 == 'R' || s1 == 'D');
    std::printf("ISOLATION parent=alive child0=%c(%s) child1=%c(%s) -> %s\n",
                s0, c0_dead ? "dead" : "alive", s1, c1_alive ? "alive" : "dead",
                (c0_dead && c1_alive) ? "HOLDS" : "BROKEN");
    std::fflush(stdout);
    // child0 is gone; do not wait on it below.
    int st = 0;
    waitpid(loaded[0].pid, &st, WNOHANG);
    loaded[0].pid = -1;
    loaded[0].exec.reset();
  }

  // Both children must receive, not just one: a second child that spins but
  // never delivers is the failure this is looking for.
  long sent = 0;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  auto all_received = [&] {
    for (auto & s : g_shared->slot) {
      if (s.received == 0) {
        return false;
      }
    }
    return true;
  };
  while (std::chrono::steady_clock::now() < deadline && !all_received()) {
    std_msgs::msg::Int64 m;
    m.data = ++sent;
    pub->publish(m);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  std::printf("\nRESULT rmw=%s sent=%ld", rmw ? rmw : "default", sent);
  for (int i = 0; i < kChildren; ++i) {
    std::printf("  child%d{received=%ld stage=%s}", i, g_shared->slot[i].received.load(),
                stage_of(i));
  }
  std::printf("  -> %s\n", all_received() ? "PASS" : "HANG/FAIL");
  std::fflush(stdout);

  // A clone child may only ever be stopped cooperatively. cancel() writes the
  // executor's interrupt guard condition, spin() returns, the clone function
  // returns and the kernel _exit()s it. A signal instead leaves whatever rclcpp
  // or DDS mutex the child held locked forever in the address space the parent
  // shares -- measured as every parent thread parked in futex_wait_queue_me.
  std::printf("stage: cancelling child executors\n");
  std::fflush(stdout);
  for (auto & l : loaded) {
    if (l.exec) {
      l.exec->cancel();
    }
  }
  for (int i = 0; i < kChildren; ++i) {
    if (loaded[i].pid < 0) {
      continue;
    }
    bool reaped = false;
    int st = 0;
    for (int t = 0; t < 50 && !reaped; ++t) {  // 5 s
      if (waitpid(loaded[i].pid, &st, WNOHANG) == loaded[i].pid) {
        reaped = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::printf("stage: [%d] child %s (stage=%s)\n", i,
                reaped ? "exited cleanly" : "DID NOT EXIT", stage_of(i));
    std::fflush(stdout);
    if (!reaped) {
      // Only as a last resort, and it is expected to deadlock the shutdown
      // below -- which is itself the finding, not an accident.
      kill(loaded[i].pid, SIGKILL);
      waitpid(loaded[i].pid, &st, 0);
    }
  }

  std::printf("stage: calling rclcpp::shutdown()\n");
  std::fflush(stdout);
  rclcpp::shutdown();
  std::printf("stage: shutdown returned\n");
  std::fflush(stdout);

  return all_received() ? 0 : 2;
}
