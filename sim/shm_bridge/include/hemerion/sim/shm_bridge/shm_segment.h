// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
// hemerion/sim/shm_bridge/shm_segment.h
//
// Cross-platform named shared-memory segment, host-only. Wraps shm_open()+
// mmap() on POSIX and CreateFileMapping()+MapViewOfFile() on Windows behind
// one RAII type -- every other shm_bridge type only needs a fixed-size block
// of bytes shared between two local processes, not the platform API used to
// get one. The platform handles live in the .cpp; this header stays free of
// <windows.h>/<sys/mman.h> so it is safe to include from either side.
// ------------------------------------------------------------------------------
#pragma once

#include <cstddef>
#include <optional>
#include <string>

namespace hemerion::sim::shm_bridge {

class ShmSegment {
 public:
  // Creates a new named segment sized to exactly `size_bytes`. Fails (returns
  // std::nullopt) if a segment with this name already exists -- the owner
  // (the FMI master) is expected to create it once per simulation run.
  [[nodiscard]] static std::optional<ShmSegment> create(const std::string& name, std::size_t size_bytes);

  // Opens a segment previously created by create() with a matching name.
  // Fails if no such segment exists yet -- callers should retry/backoff
  // rather than treat this as fatal, since the peer process may start first.
  [[nodiscard]] static std::optional<ShmSegment> open_existing(const std::string& name, std::size_t size_bytes);

  ShmSegment(const ShmSegment&) = delete;
  ShmSegment& operator=(const ShmSegment&) = delete;
  ShmSegment(ShmSegment&& other) noexcept;
  ShmSegment& operator=(ShmSegment&& other) noexcept;
  ~ShmSegment();

  [[nodiscard]] void* data() const noexcept { return data_; }
  [[nodiscard]] std::size_t size() const noexcept { return size_; }
  // True if this handle is the one that created the underlying OS object --
  // that handle is the one responsible for unlinking it (POSIX) on destruction.
  [[nodiscard]] bool owns_segment() const noexcept { return owns_segment_; }

 private:
  ShmSegment() = default;
  void reset() noexcept;

  void* data_ = nullptr;
  std::size_t size_ = 0;
  bool owns_segment_ = false;
  std::string name_;

#if defined(_WIN32)
  void* mapping_handle_ = nullptr;  // HANDLE, stored as void* to keep this header OS-agnostic
#else
  int fd_ = -1;
#endif
};

}  // namespace hemerion::sim::shm_bridge
