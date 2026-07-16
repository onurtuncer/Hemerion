// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
#include "hemerion/sim/shm_bridge/shm_segment.h"

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#else
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

#include <cstdint>
#include <utility>

namespace hemerion::sim::shm_bridge
{

namespace
{

#if defined(_WIN32)
// Windows kernel object names use '\' as a namespace separator; strip a
// POSIX-style leading '/' so the same caller-supplied name works unmodified
// on both platforms.
std::string platform_name(const std::string& name)
{
  return (!name.empty() && name.front() == '/') ? name.substr(1) : name;
}
#else
// shm_open() names must start with a single leading '/'.
std::string platform_name(const std::string& name)
{
  return (!name.empty() && name.front() == '/') ? name : "/" + name;
}
#endif

}  // namespace

std::optional<ShmSegment> ShmSegment::create(const std::string& name, std::size_t size_bytes)
{
  const std::string mapped_name = platform_name(name);

#if defined(_WIN32)
  const DWORD size_high = static_cast<DWORD>(static_cast<std::uint64_t>(size_bytes) >> 32);
  const DWORD size_low = static_cast<DWORD>(static_cast<std::uint64_t>(size_bytes) & 0xFFFFFFFFu);

  HANDLE handle =
      CreateFileMappingA(INVALID_HANDLE_VALUE, nullptr, PAGE_READWRITE, size_high, size_low, mapped_name.c_str());
  if (handle == nullptr)
  {
    return std::nullopt;
  }
  // CreateFileMapping() returns a handle to the *existing* section (rather
  // than failing) when one of this name is already open elsewhere; treat
  // that the same as a hard failure since create() must be exclusive.
  if (GetLastError() == ERROR_ALREADY_EXISTS)
  {
    CloseHandle(handle);
    return std::nullopt;
  }

  void* view = MapViewOfFile(handle, FILE_MAP_ALL_ACCESS, 0, 0, size_bytes);
  if (view == nullptr)
  {
    CloseHandle(handle);
    return std::nullopt;
  }

  ShmSegment segment;
  segment.data_ = view;
  segment.size_ = size_bytes;
  segment.owns_segment_ = true;
  segment.name_ = mapped_name;
  segment.mapping_handle_ = handle;
  return segment;
#else
  const int fd = shm_open(mapped_name.c_str(), O_CREAT | O_EXCL | O_RDWR, 0600);
  if (fd < 0)
  {
    return std::nullopt;
  }
  if (ftruncate(fd, static_cast<off_t>(size_bytes)) != 0)
  {
    close(fd);
    shm_unlink(mapped_name.c_str());
    return std::nullopt;
  }
  void* view = mmap(nullptr, size_bytes, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  if (view == MAP_FAILED)
  {
    close(fd);
    shm_unlink(mapped_name.c_str());
    return std::nullopt;
  }

  ShmSegment segment;
  segment.data_ = view;
  segment.size_ = size_bytes;
  segment.owns_segment_ = true;
  segment.name_ = mapped_name;
  segment.fd_ = fd;
  return segment;
#endif
}

std::optional<ShmSegment> ShmSegment::open_existing(const std::string& name, std::size_t size_bytes)
{
  const std::string mapped_name = platform_name(name);

#if defined(_WIN32)
  HANDLE handle = OpenFileMappingA(FILE_MAP_ALL_ACCESS, FALSE, mapped_name.c_str());
  if (handle == nullptr)
  {
    return std::nullopt;
  }
  void* view = MapViewOfFile(handle, FILE_MAP_ALL_ACCESS, 0, 0, size_bytes);
  if (view == nullptr)
  {
    CloseHandle(handle);
    return std::nullopt;
  }

  ShmSegment segment;
  segment.data_ = view;
  segment.size_ = size_bytes;
  segment.owns_segment_ = false;
  segment.name_ = mapped_name;
  segment.mapping_handle_ = handle;
  return segment;
#else
  const int fd = shm_open(mapped_name.c_str(), O_RDWR, 0600);
  if (fd < 0)
  {
    return std::nullopt;
  }
  void* view = mmap(nullptr, size_bytes, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  if (view == MAP_FAILED)
  {
    close(fd);
    return std::nullopt;
  }

  ShmSegment segment;
  segment.data_ = view;
  segment.size_ = size_bytes;
  segment.owns_segment_ = false;
  segment.name_ = mapped_name;
  segment.fd_ = fd;
  return segment;
#endif
}

ShmSegment::ShmSegment(ShmSegment&& other) noexcept { *this = std::move(other); }

ShmSegment& ShmSegment::operator=(ShmSegment&& other) noexcept
{
  if (this != &other)
  {
    reset();

    data_ = other.data_;
    size_ = other.size_;
    owns_segment_ = other.owns_segment_;
    name_ = std::move(other.name_);
#if defined(_WIN32)
    mapping_handle_ = other.mapping_handle_;
    other.mapping_handle_ = nullptr;
#else
    fd_ = other.fd_;
    other.fd_ = -1;
#endif
    other.data_ = nullptr;
    other.size_ = 0;
    other.owns_segment_ = false;
  }
  return *this;
}

ShmSegment::~ShmSegment() { reset(); }

void ShmSegment::reset() noexcept
{
#if defined(_WIN32)
  if (data_ != nullptr)
  {
    UnmapViewOfFile(data_);
  }
  if (mapping_handle_ != nullptr)
  {
    CloseHandle(static_cast<HANDLE>(mapping_handle_));
    mapping_handle_ = nullptr;
  }
#else
  if (data_ != nullptr)
  {
    munmap(data_, size_);
  }
  if (fd_ >= 0)
  {
    close(fd_);
    fd_ = -1;
  }
  // Only the creating side unlinks -- the peer's handle simply stops mapping
  // the segment, leaving it alive for as long as the owner needs it.
  if (owns_segment_ && !name_.empty())
  {
    shm_unlink(name_.c_str());
  }
#endif
  data_ = nullptr;
  size_ = 0;
  owns_segment_ = false;
}

}  // namespace hemerion::sim::shm_bridge
