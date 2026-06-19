#include "database_mapped_file.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>

#ifndef _WIN32
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace rbf::lect_database {

LectDatabase::EvidenceMappedFile::~EvidenceMappedFile() { close(); }

bool LectDatabase::EvidenceMappedFile::open_read_only(const std::filesystem::path& path) {
    close();
#ifdef _WIN32
    file_ = ::CreateFileW(path.c_str(),
                          GENERIC_READ,
                          FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
                          nullptr,
                          OPEN_EXISTING,
                          FILE_ATTRIBUTE_NORMAL | FILE_FLAG_RANDOM_ACCESS,
                          nullptr);
    if (file_ == INVALID_HANDLE_VALUE) {
        file_ = INVALID_HANDLE_VALUE;
        return false;
    }
    LARGE_INTEGER size_value{};
    if (!::GetFileSizeEx(file_, &size_value) || size_value.QuadPart <= 0) {
        close();
        return false;
    }
    if (static_cast<unsigned long long>(size_value.QuadPart) >
        static_cast<unsigned long long>(std::numeric_limits<std::size_t>::max())) {
        close();
        return false;
    }
    mapping_ = ::CreateFileMappingW(file_, nullptr, PAGE_READONLY, 0, 0, nullptr);
    if (mapping_ == nullptr) {
        close();
        return false;
    }
    view_ = ::MapViewOfFile(mapping_, FILE_MAP_READ, 0, 0, 0);
    if (view_ == nullptr) {
        close();
        return false;
    }
    data_ = static_cast<const std::byte*>(view_);
    size_ = static_cast<std::size_t>(size_value.QuadPart);
    return true;
#else
    fd_ = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
    if (fd_ < 0) {
        fd_ = -1;
        return false;
    }
    struct stat status {};
    if (::fstat(fd_, &status) != 0 || status.st_size <= 0) {
        close();
        return false;
    }
    if (static_cast<unsigned long long>(status.st_size) >
        static_cast<unsigned long long>(std::numeric_limits<std::size_t>::max())) {
        close();
        return false;
    }
    void* mapped = ::mmap(nullptr, static_cast<std::size_t>(status.st_size), PROT_READ, MAP_PRIVATE, fd_, 0);
    if (mapped == MAP_FAILED) {
        close();
        return false;
    }
#ifdef MADV_RANDOM
    ::madvise(mapped, static_cast<std::size_t>(status.st_size), MADV_RANDOM);
#endif
    data_ = static_cast<const std::byte*>(mapped);
    size_ = static_cast<std::size_t>(status.st_size);
    return true;
#endif
}

void LectDatabase::EvidenceMappedFile::close() {
#ifdef _WIN32
    if (view_ != nullptr) {
        ::UnmapViewOfFile(view_);
        view_ = nullptr;
    }
    if (mapping_ != nullptr) {
        ::CloseHandle(mapping_);
        mapping_ = nullptr;
    }
    if (file_ != INVALID_HANDLE_VALUE) {
        ::CloseHandle(file_);
        file_ = INVALID_HANDLE_VALUE;
    }
#else
    if (data_ != nullptr) {
        ::munmap(const_cast<std::byte*>(data_), size_);
    }
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
#endif
    data_ = nullptr;
    size_ = 0;
}

bool LectDatabase::EvidenceMappedFile::is_open() const noexcept {
    return data_ != nullptr && size_ > 0;
}

std::span<const std::byte> LectDatabase::EvidenceMappedFile::bytes() const noexcept {
    return is_open() ? std::span<const std::byte>(data_, size_) : std::span<const std::byte>{};
}

void LectDatabase::EvidenceMappedFile::prefetch(std::uint64_t offset, std::uint64_t size) const {
    if (!is_open() || offset >= size_) {
        return;
    }
    const auto safe_size = std::min<std::uint64_t>(size, static_cast<std::uint64_t>(size_) - offset);
    if (safe_size == 0) {
        return;
    }
#ifdef _WIN32
    using PrefetchVirtualMemoryFn = BOOL(WINAPI*)(HANDLE, ULONG_PTR, PVOID, ULONG);
    struct MemoryRangeEntry {
        PVOID VirtualAddress;
        SIZE_T NumberOfBytes;
    };
    static const auto prefetch_virtual_memory = []() -> PrefetchVirtualMemoryFn {
        HMODULE module = ::GetModuleHandleW(L"kernel32.dll");
        if (module == nullptr) {
            return nullptr;
        }
        return reinterpret_cast<PrefetchVirtualMemoryFn>(::GetProcAddress(module, "PrefetchVirtualMemory"));
    }();
    if (prefetch_virtual_memory == nullptr) {
        return;
    }
    MemoryRangeEntry range;
    range.VirtualAddress = const_cast<std::byte*>(data_ + offset);
    range.NumberOfBytes = static_cast<SIZE_T>(safe_size);
    prefetch_virtual_memory(::GetCurrentProcess(), 1, &range, 0);
#else
#ifdef MADV_WILLNEED
    const long raw_page_size = ::sysconf(_SC_PAGESIZE);
    const auto page_size = raw_page_size > 0 ? static_cast<std::size_t>(raw_page_size) : 4096u;
    const auto aligned_offset = (static_cast<std::size_t>(offset) / page_size) * page_size;
    const auto aligned_end = ((static_cast<std::size_t>(offset + safe_size) + page_size - 1) / page_size) * page_size;
    const auto advised_size = std::min<std::size_t>(aligned_end - aligned_offset, size_ - aligned_offset);
    ::madvise(const_cast<std::byte*>(data_ + aligned_offset), advised_size, MADV_WILLNEED);
#else
    (void)offset;
    (void)safe_size;
#endif
#endif
}

std::size_t LectDatabase::EvidenceMappedFile::size() const noexcept { return size_; }


}  // namespace rbf::lect_database
