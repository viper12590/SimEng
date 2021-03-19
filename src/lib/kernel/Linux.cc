#include "simeng/kernel/Linux.hh"

#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/termios.h>
#include <sys/uio.h>
#include <unistd.h>

#include <algorithm>
#include <cassert>
#include <cstring>
#include <iostream>

namespace simeng {
namespace kernel {

void Linux::createProcess(const LinuxProcess& process, const char* memory) {
  assert(process.isValid() && "Attempted to use an invalid process");
  assert(processStates_.size() == 0 && "Multiple processes not yet supported");
  processStates_.push_back({.pid = 0,  // TODO: create unique PIDs
                            .path = process.getPath(),
                            .startBrk = process.getHeapStart(),
                            .currentBrk = process.getHeapStart(),
                            .initialStackPointer = process.getStackPointer()});
  processStates_.back().fileDescriptorTable.push_back({STDIN_FILENO, -1});
  processStates_.back().fileDescriptorTable.push_back({STDOUT_FILENO, -1});
  processStates_.back().fileDescriptorTable.push_back({STDERR_FILENO, -1});
  // Store process memory array representation for read-only access where
  // appropriate
  memory_ = memory;
}

uint64_t Linux::getInitialStackPointer() const {
  assert(processStates_.size() > 0 &&
         "Attempted to retrieve a stack pointer before creating a process");

  return processStates_[0].initialStackPointer;
}

int64_t Linux::brk(uint64_t address) {
  assert(processStates_.size() > 0 &&
         "Attempted to move the program break before creating a process");

  auto& state = processStates_[0];
  // Move the break if it's within the heap region
  if (address > state.startBrk) {
    state.currentBrk = address;
  }
  return state.currentBrk;
}

uint64_t Linux::clockGetTime(uint64_t clkId, uint64_t systemTimer,
                             uint64_t& seconds, uint64_t& nanoseconds) {
  // TODO: Ideally this should get the system timer from the core directly
  // rather than having it passed as an argument.
  if (clkId == CLOCK_REALTIME) {
    seconds = systemTimer / 1e9;
    nanoseconds = systemTimer - (seconds * 1e9);
    return 0;
  } else {
    assert(false && "Unhandled clk_id in clock_gettime syscall");
    return -1;
  }
}

int64_t Linux::close(int64_t fd) {
  assert(fd < processStates_[0].fileDescriptorTable.size());
  int64_t hfd = processStates_[0].fileDescriptorTable[fd].first;
  if (hfd < 0) {
    return EBADF;
  }

  // Deallocate the virtual file descriptor
  assert(processStates_[0].freeFileDescriptors.count(fd) == 0);
  processStates_[0].freeFileDescriptors.insert(fd);
  processStates_[0].fileDescriptorTable[fd] = {-1, -1};

  return ::close(hfd);
}

int64_t Linux::fstat(int64_t fd, stat& out) {
  assert(fd < processStates_[0].fileDescriptorTable.size());
  int64_t hfd = processStates_[0].fileDescriptorTable[fd].first;
  if (hfd < 0) {
    return EBADF;
  }

  // Pass call through to host
  struct ::stat statbuf;
  int64_t retval = ::fstat(hfd, &statbuf);

  // Copy results to output struct
  out.dev = statbuf.st_dev;
  out.ino = statbuf.st_ino;
  out.mode = statbuf.st_mode;
  out.nlink = statbuf.st_nlink;
  out.uid = statbuf.st_uid;
  out.gid = statbuf.st_gid;
  out.rdev = statbuf.st_rdev;
  out.size = statbuf.st_size;
  out.blksize = statbuf.st_blksize;
  out.blocks = statbuf.st_blocks;
  out.atime = statbuf.st_atime;
  out.mtime = statbuf.st_mtime;
  out.ctime = statbuf.st_ctime;

  return retval;
}

int64_t Linux::getpid() const {
  assert(processStates_.size() > 0);
  return processStates_[0].pid;
}

int64_t Linux::getuid() const { return 0; }
int64_t Linux::geteuid() const { return 0; }
int64_t Linux::getgid() const { return 0; }
int64_t Linux::getegid() const { return 0; }

int64_t Linux::gettimeofday(uint64_t systemTimer, timeval* tv, timeval* tz) {
  // TODO: Ideally this should get the system timer from the core directly
  // rather than having it passed as an argument.
  if (tv) {
    tv->tv_sec = systemTimer / 1e9;
    tv->tv_usec = (systemTimer - (tv->tv_sec * 1e9)) / 1e3;
  }
  if (tz) {
    tz->tv_sec = 0;
    tz->tv_usec = 0;
  }
  return 0;
}

int64_t Linux::ioctl(int64_t fd, uint64_t request, std::vector<char>& arg) {
  assert(fd < processStates_[0].fileDescriptorTable.size());
  std::pair<int64_t, int16_t> fdEntry =
      processStates_[0].fileDescriptorTable[fd];
  int64_t hfd = fdEntry.first;
  // Check if the sys call is pmu associated
  if (std::find(ioctlPmuReqs.begin(), ioctlPmuReqs.end(), request) ==
      ioctlPmuReqs.end()) {
    if (hfd < 0) {
      return EBADF;
    }

    switch (request) {
      case 0x5401: {  // TCGETS
        struct ::termios hostResult;
        int64_t retval;
#ifdef __APPLE__
        retval = ::ioctl(hfd, TIOCGETA, &hostResult);
#else
        retval = ::ioctl(hfd, TCGETS, &hostResult);
#endif
        arg.resize(sizeof(ktermios));
        ktermios& result = *reinterpret_cast<ktermios*>(arg.data());
        result.c_iflag = hostResult.c_iflag;
        result.c_oflag = hostResult.c_oflag;
        result.c_cflag = hostResult.c_cflag;
        result.c_lflag = hostResult.c_lflag;
        // TODO: populate c_line and c_cc
        return retval;
      }
      case 0x5413:  // TIOCGWINSZ
        arg.resize(sizeof(struct winsize));
        ::ioctl(hfd, TIOCGWINSZ, arg.data());
        return 0;
      default:
        assert(false && "unimplemented ioctl request");
        return -1;
    }
  } else {
    // Check for existence of fd in pmu_
    if (hfd < 0 || fdEntry.second == -1) {
      assert(false && "PMU associated ioctl syscall has bad fd");
      return -1;
    }
    uint8_t eventId = (uint8_t)fdEntry.second;
    if (eventId >= pmu_.size()) {
      assert(false && "Non-existent eventId in PMU associated ioctl syscall");
      return -1;
    }
    // Change appropriate pmuEntry value(s) along with other entries in group
    // if PERF_IOC_FLAG_GROUP bit is set.
    std::vector<uint8_t> perfEvents = (arg.size() && arg[0])
                                          ? pmuGroups_[pmu_[eventId].groupId]
                                          : std::vector({eventId});
    // Assume default case of arg as `in` parameter and thus has nothing to
    // return in argp to ExceptionHandler caller
    arg.resize(0);
    for (uint8_t pe : perfEvents) {
      switch (request) {
        case 0x2400:  // PERF_EVENT_IOC_ENABLE
          pmu_[pe].state = true;
          break;
        case 0x2401:  // PERF_EVENT_IOC_DISABLE
          pmu_[pe].state = false;
          // If pe links to instruction count event (0x8), decrement its value
          // by one as the SVC instruction causing this ioctl should not be
          // accounted for in the event value.
          if (pmu_[pe].eventInfo.config == 0x8) {
            pmu_[pe].value--;
          }
          break;
        case 0x2403:  // PERF_EVENT_IOC_RESET
          pmu_[pe].value = 0;
          break;
        case 0x80082407:  // PERF_EVENT_IOC_ID
                          // Argp is an `out` parameter so resize and store ID
          arg.resize(1);
          arg[0] = pmu_[pe].id;
          return 0;
        default:
          assert(false && "unimplemented ioctl request");
          return -1;
      }
    }
    return 0;
  }
}

uint64_t Linux::lseek(int64_t fd, uint64_t offset, int64_t whence) {
  assert(fd < processStates_[0].fileDescriptorTable.size());
  int64_t hfd = processStates_[0].fileDescriptorTable[fd].first;
  if (hfd < 0) {
    return EBADF;
  }
  return ::lseek(hfd, offset, whence);
}

int64_t Linux::openat(int64_t dirfd, const std::string& pathname, int64_t flags,
                      uint16_t mode) {
  // Resolve absolute path to target file
  char absolutePath[LINUX_PATH_MAX];
  realpath(pathname.c_str(), absolutePath);

  // Check if path may be a special file, bail out if it is
  // TODO: Add support for special files
  for (auto prefix : {"/dev/", "/proc/", "/sys/"}) {
    if (strncmp(absolutePath, prefix, strlen(prefix)) == 0) {
      std::cerr << "ERROR: attempted to open special file: "
                << "'" << absolutePath << "'" << std::endl;
      exit(1);
    }
  }

  // Pass syscall through to host
  assert(dirfd == -100 && "unsupported dirfd argument in openat syscall");
  int64_t hfd = ::openat(AT_FDCWD, pathname.c_str(), flags, mode);
  if (hfd < 0) {
    return hfd;
  }

  LinuxProcessState& processState = processStates_[0];

  // Allocate virtual file descriptor and map to host file descriptor
  int64_t vfd;
  if (!processState.freeFileDescriptors.empty()) {
    // Take virtual descriptor from free pool
    auto first = processState.freeFileDescriptors.begin();
    vfd = processState.freeFileDescriptors.extract(first).value();
    processState.fileDescriptorTable[vfd] = {hfd, -1};
  } else {
    // Extend file descriptor table for a new virtual descriptor
    vfd = processState.fileDescriptorTable.size();
    processState.fileDescriptorTable.push_back({hfd, -1});
  }

  return vfd;
}

int64_t Linux::perfEventOpen(uint64_t attr, pid_t pid, int64_t cpu,
                             int64_t group_fd, uint64_t flags) {
  if (flags != 0) {
    // flags in perf_event_open syscall unsupported
    return -1;
  } else if (pid != 0 || cpu != -1) {
    // in perf_event_open syscall, only pid == 0 and cpu == -1 monitoring is
    // supported supported
    return -1;
  }

  // Construct perfEventAttr struct from values at `attr` offset in memory_
  const char* base = memory_ + attr;
  perfEventAttr newEvent = {
      .type = *reinterpret_cast<const uint32_t*>(base),
      .size = *reinterpret_cast<const uint32_t*>(base + 4),
      .config = *reinterpret_cast<const uint64_t*>(base + 8),
      .readFormat = *reinterpret_cast<const uint64_t*>(base + 32),
      .eventConfig = *reinterpret_cast<const uint64_t*>(base + 40)};

  // Extract exclude _{kernel,hv} bits in eventConfig as thye must be set
  if ((newEvent.eventConfig & 96) != 96) {
    // in perf_event_open syscall, only user_level monitoring is supported
    return -1;
  }
  // Logical AND eventConfig with mask exlcuding exclude_kernel, and exclude_hv
  // options
  if (newEvent.eventConfig & ~(97)) {
    // in perf_event_open syscall, only disabled, exclude_kernel, and exclude_hv
    // options are supported
    return -1;
  }

  // Generate Id data
  uint8_t eventId = pmu_.size();
  uint8_t groupId;
  if (group_fd > -1) {
    // Leader fd defined, check for existence
    assert(group_fd < processStates_[0].fileDescriptorTable.size());
    std::pair<int64_t, int16_t> group_hfd =
        processStates_[0].fileDescriptorTable[group_fd];
    if (group_hfd.first < 0 || group_hfd.second == -1) {
      // in perf_event_open syscall, group_fd file descriptor is not valid
      return -1;
    }
    // Leader fd exists, get its groupId and assign new event to it
    uint8_t leaderId = group_hfd.second;
    if (leaderId < pmu_.size()) {
      groupId = pmu_[leaderId].groupId;
      pmuGroups_[groupId].push_back(eventId);
    } else {
      // in perf_event_open syscall, the group_fd file descriptor PMU entry is
      // not valid
      return -1;
    }
  } else {
    // No leader fd defined so create new group
    groupId = pmuGroups_.size();
    pmuGroups_.push_back({eventId});
  }

  // Create pmu entry and add to `pmu_`
  pmuEntry newEntry = {.eventInfo = newEvent,
                       .state = !(newEvent.eventConfig & 1),
                       .value = 0,
                       .prev = 0,
                       .id = eventId,
                       .groupId = groupId};
  pmu_.insert({eventId, newEntry});
  // Create or append id to HW event mapping
  if (hwEvents_.find(newEvent.config) != hwEvents_.end()) {
    hwEvents_[newEvent.config].push_back(eventId);
  } else {
    hwEvents_[newEvent.config] = {eventId};
  }

  // Generate host file descriptor
  LinuxProcessState& processState = processStates_[0];
  int64_t hfd = 0;
  for (auto fd : processState.fileDescriptorTable) {
    if (hfd == fd.first) hfd++;
  }
  // Allocate virtual file descriptor and map to host file descriptor
  int64_t vfd;
  if (!processState.freeFileDescriptors.empty()) {
    // Take virtual descriptor from free pool
    auto first = processState.freeFileDescriptors.begin();
    vfd = processState.freeFileDescriptors.extract(first).value();
    processState.fileDescriptorTable[vfd] = {hfd, eventId};
  } else {
    // Extend file descriptor table for a new virtual descriptor
    vfd = processState.fileDescriptorTable.size();
    processState.fileDescriptorTable.push_back({hfd, eventId});
  }
  // Return the event Id as a representation of the fd
  return vfd;
}

void Linux::pmuIncrement(uint16_t event, uint64_t value) {
  // Check existence of event in hwEvents_ and increment associated performance
  // eventsby `value` if true
  if (hwEvents_.find(event) != hwEvents_.end()) {
    for (uint8_t pe : hwEvents_[event]) {
      pmu_[pe].value += pmu_[pe].state ? (value - pmu_[pe].prev) : 0;
      pmu_[pe].prev = value;
    }
  }
}

int64_t Linux::readlinkat(int64_t dirfd, const std::string pathname, char* buf,
                          size_t bufsize) const {
  const auto& processState = processStates_[0];
  if (pathname == "/proc/self/exe") {
    // Copy executable path to buffer
    // TODO: resolve path into canonical path
    std::strncpy(buf, processState.path.c_str(), bufsize);

    return std::min(processState.path.length(), bufsize);
  }

  // TODO: resolve symbolic link for other paths
  return -1;
}

int64_t Linux::read(int64_t fd, void* buf, uint64_t count) {
  assert(fd < processStates_[0].fileDescriptorTable.size());
  std::pair<int64_t, int16_t> fdEntry =
      processStates_[0].fileDescriptorTable[fd];
  int64_t hfd = fdEntry.first;
  // Check if read is from a non-PMU fd
  if (fdEntry.second < 0) {
    if (hfd < 0) {
      return EBADF;
    }
    return ::read(hfd, buf, count);
  } else {
    // Check for existence of fd in pmu_
    if (hfd < 0 || fdEntry.second == -1) {
      assert(false && "PMU associated read syscall has bad fd");
      return -1;
    }
    uint8_t eventId = (uint8_t)fdEntry.second;
    if (eventId >= pmu_.size()) {
      assert(false && "Non-existent eventId in PMU associated read syscall");
      return -1;
    }

    // Get read_format information
    bool groupRead = pmu_[eventId].eventInfo.readFormat & 8;
    bool withId = pmu_[eventId].eventInfo.readFormat & 4;
    // Get pe's to be read from
    std::vector<uint8_t> perfEvents =
        groupRead ? pmuGroups_[pmu_[eventId].groupId] : std::vector({eventId});
    // Track read byte quantities
    uint64_t bytesRead = 0;
    uint64_t bytesAvailable = count;
    size_t numBytes;
    // Clear block of memory to put read_format struct into
    memset(buf, 0, count);
    uint64_t* buf64 = static_cast<uint64_t*>(buf);

    // Read nr if PERF_FORMAT_GROUP bit was set
    if (groupRead) {
      numBytes = (bytesAvailable > 7) ? 8 : bytesAvailable;
      uint64_t nr = perfEvents.size();
      memcpy(buf64, &nr, numBytes);
      bytesRead += numBytes;
      buf64++;
      bytesAvailable -= numBytes;
    }
    // Read values and id if PERF_FORMAT_ID bit was set
    for (uint8_t pe : perfEvents) {
      numBytes = (bytesAvailable > 7) ? 8 : bytesAvailable;
      memcpy(buf64, &pmu_[pe].value, numBytes);
      bytesRead += numBytes;
      buf64++;
      bytesAvailable -= numBytes;
      if (withId) {
        numBytes = (bytesAvailable > 7) ? 8 : bytesAvailable;
        memcpy(buf64, &pmu_[pe].id, 1);
        bytesRead += numBytes;
        buf64++;
        bytesAvailable -= numBytes;
      }
    }
    return bytesRead;
  }
}

int64_t Linux::readv(int64_t fd, const void* iovdata, int iovcnt) {
  assert(fd < processStates_[0].fileDescriptorTable.size());
  int64_t hfd = processStates_[0].fileDescriptorTable[fd].first;
  if (hfd < 0) {
    return EBADF;
  }
  return ::readv(hfd, reinterpret_cast<const struct iovec*>(iovdata), iovcnt);
}

int64_t Linux::setTidAddress(uint64_t tidptr) {
  assert(processStates_.size() > 0);
  processStates_[0].clearChildTid = tidptr;
  return processStates_[0].pid;
}

int64_t Linux::write(int64_t fd, const void* buf, uint64_t count) {
  assert(fd < processStates_[0].fileDescriptorTable.size());
  int64_t hfd = processStates_[0].fileDescriptorTable[fd].first;
  if (hfd < 0) {
    return EBADF;
  }
  return ::write(hfd, buf, count);
}

int64_t Linux::writev(int64_t fd, const void* iovdata, int iovcnt) {
  assert(fd < processStates_[0].fileDescriptorTable.size());
  int64_t hfd = processStates_[0].fileDescriptorTable[fd].first;
  if (hfd < 0) {
    return EBADF;
  }
  return ::writev(hfd, reinterpret_cast<const struct iovec*>(iovdata), iovcnt);
}

}  // namespace kernel
}  // namespace simeng
