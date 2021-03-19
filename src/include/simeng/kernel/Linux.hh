#pragma once

#include <set>
#include <unordered_map>

#include "simeng/kernel/LinuxProcess.hh"

namespace simeng {
namespace kernel {

/** Fixed-width definition of `timeval`.
 * Defined by Linux kernel in include/uapi/asm-generic/stat.h */
struct stat {
  uint64_t dev;       // offset =   0
  uint64_t ino;       // offset =   8
  uint32_t mode;      // offset =  16
  uint32_t nlink;     // offset =  20
  uint32_t uid;       // offset =  24
  uint32_t gid;       // offset =  28
  uint64_t rdev;      // offset =  32
  uint64_t padding1;  // offset =  40
  int64_t size;       // offset =  48
  int32_t blksize;    // offset =  56
  uint32_t padding2;  // offset =  60
  int64_t blocks;     // offset =  64
  int64_t atime;      // offset =  72
  uint64_t padding3;  // offset =  80
  int64_t mtime;      // offset =  88
  uint64_t padding4;  // offset =  96
  int64_t ctime;      // offset = 104
  uint64_t padding5;  // offset = 112
  uint32_t padding6;  // offset = 116
  uint32_t padding7;  // offset = 124
};

/** Fixed-width definition of `termios`.
 * Defined by Linux kernel in `include/uapi/asm-generic/termbits.h` */
struct ktermios {
  uint32_t c_iflag;  // input mode flags
  uint32_t c_oflag;  // output mode flags
  uint32_t c_cflag;  // control mode flags
  uint32_t c_lflag;  // local mode flags
  uint8_t c_line;    // line discipline
  uint8_t c_cc[19];  // control characters
};

/** Fixed-width definition of `timeval` (from `<sys/time.h>`). */
struct timeval {
  int64_t tv_sec;   // seconds
  int64_t tv_usec;  // microseconds
};

/** A state container for a Linux process. */
struct LinuxProcessState {
  /** The process ID. */
  int64_t pid;
  /** The path of the executable that created this process. */
  std::string path;
  /** The address of the start of the heap. */
  uint64_t startBrk;
  /** The address of the current end of heap. */
  uint64_t currentBrk;
  /** The initial stack pointer. */
  uint64_t initialStackPointer;

  // Thread state
  // TODO: Support multiple threads per process
  /** The clear_child_tid value. */
  uint64_t clearChildTid = 0;

  /** The virtual file descriptor mapping table. Each entry contains the
   * associated host file descriptor and an associated PMU eventId where
   * necessary (a -1 represents no such association). */
  std::vector<std::pair<int64_t, int16_t>> fileDescriptorTable;
  /** Set of deallocated virtual file descriptors available for reuse. */
  std::set<int64_t> freeFileDescriptors;
};

/** Reconstruction of areas of interest in the `perf_event_attr` struct.
 * Original struct details derived from
 * https://man7.org/linux/man-pages/man2/perf_event_open.2.html. */
struct perfEventAttr {
  /* Type of event. */
  uint32_t type;
  /* Size of attribute structure. */
  uint32_t size;
  /* Type-specific configuration. */
  uint64_t config;
  /* Specifies values returned in read. */
  uint64_t readFormat;
  /* Configuration options altering the data collected. */
  uint64_t eventConfig;
};

/** A structure representing an event in the Linux Performance Monitor Unit
 * (PMU). */
struct pmuEntry {
  /** The perf_event_attr struct created on perfEventOpen call. */
  perfEventAttr eventInfo;
  /** Current state of the event, {enabled, disabled}. */
  bool state;
  /** The value of the event. */
  uint64_t value;
  /** The previous count of this event. */
  uint64_t prev;
  /** The id of the performance event counter. */
  uint8_t id;
  /** The group id assigned to the event to be later utilised when event
   * grouping is chosen. */
  uint8_t groupId;
};

/** A Linux kernel syscall emulation implementation, which mimics the responses
   to Linux system calls. */
class Linux {
 public:
  /** Create a new Linux process running above this kernel. */
  void createProcess(const LinuxProcess& process, const char* memory);

  /** Retrieve the initial stack pointer. */
  uint64_t getInitialStackPointer() const;

  /** brk syscall: change data segment size. Sets the program break to
   * `addr` if reasonable, and returns the program break. */
  int64_t brk(uint64_t addr);

  /** clock_gettime syscall: get the time of specified clock `clkId`, using
   * the system timer `systemTimer` (with nanosecond accuracy). Returns 0 on
   * success, and puts the retrieved time in the `seconds` and `nanoseconds`
   * arguments. */
  uint64_t clockGetTime(uint64_t clkId, uint64_t systemTimer, uint64_t& seconds,
                        uint64_t& nanoseconds);

  /** close syscall: close a file descriptor. */
  int64_t close(int64_t fd);

  /** fstat syscall: get file status. */
  int64_t fstat(int64_t fd, stat& out);

  /** getpid syscall: get the process owner's process ID. */
  int64_t getpid() const;
  /** getuid syscall: get the process owner's user ID. */
  int64_t getuid() const;
  /** geteuid syscall: get the process owner's effective user ID. */
  int64_t geteuid() const;
  /** getgid syscall: get the process owner's group ID. */
  int64_t getgid() const;
  /** getegid syscall: get the process owner's effective group ID. */
  int64_t getegid() const;

  /** gettimeofday syscall: get the current time, using the system timer
   * `systemTimer` (with nanosecond accuracy). Returns 0 on success, and puts
   * the seconds and microsconds elapsed since the Epoch in `tv`, while setting
   * the elements of `tz` to 0. */
  int64_t gettimeofday(uint64_t systemTimer, timeval* tv, timeval* tz);

  /** ioctl syscall: control device. When arg is treated as an in parameter,
   * each index of the vector represents a single bit. */
  int64_t ioctl(int64_t fd, uint64_t request, std::vector<char>& arg);

  /** lseek syscall: reposition read/write file offset. */
  uint64_t lseek(int64_t fd, uint64_t offset, int64_t whence);

  /** openat syscall: open/create a file. */
  int64_t openat(int64_t dirfd, const std::string& path, int64_t flags,
                 uint16_t mode);

  /** perf_event_open syscall: Create a pmuEntry and an associated
   * representation of a file descriptor for a specified performance event. The
   * representation is in the form of the index the newly created pmuEntry
   * occupies in the `pmu_` unordered_map. Return values are a -1 in the case
   * of an error or the file descriptor representation in the case of a success.
   */
  int64_t perfEventOpen(uint64_t attr, pid_t pid, int64_t cpu, int64_t group_fd,
                        uint64_t flags);

  /** Increment the value of the performance event held within the pmu_ map, if
  it exists, by the difference between the previous known count and the newly
  supplied value. */
  void pmuIncrement(uint16_t event, uint64_t value);

  /** readlinkat syscall: read value of a symbolic link. */
  int64_t readlinkat(int64_t dirfd, const std::string pathname, char* buf,
                     size_t bufsize) const;

  /** set_tid_address syscall: set clear_child_tid value for calling thread. */
  int64_t setTidAddress(uint64_t tidptr);

  /** read syscall: read buffer from a file. */
  int64_t read(int64_t fd, void* buf, uint64_t count);

  /** readv syscall: read buffers from a file. */
  int64_t readv(int64_t fd, const void* iovdata, int iovcnt);

  /** write syscall: write buffer to a file. */
  int64_t write(int64_t fd, const void* buf, uint64_t count);

  /** writev syscall: write buffers to a file. */
  int64_t writev(int64_t fd, const void* iovdata, int iovcnt);

  /** The maximum size of a filesystem path. */
  static const size_t LINUX_PATH_MAX = 4096;

 private:
  /** A read only array representing the process memory. */
  const char* memory_;

  /** The state of the user-space processes running above the kernel. */
  std::vector<LinuxProcessState> processStates_;

  /** The set of PMU events currently created. */
  std::unordered_map<uint8_t, pmuEntry> pmu_;

  /** Mapping between hardware events and event ids. */
  std::unordered_map<uint16_t, std::vector<uint8_t>> hwEvents_;

  /** Mapping between group ids and event ids. */
  std::vector<std::vector<uint8_t>> pmuGroups_;

  /** Vector of supported perf_open_event associated ioctl requests. */
  std::vector<uint64_t> ioctlPmuReqs = {0x2400, 0x2401, 0x2403, 0x80082407};
};

}  // namespace kernel
}  // namespace simeng
