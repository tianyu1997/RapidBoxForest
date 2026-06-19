#include "read_snapshot_mapped_file.h"

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace rbf::lect_database {

ReadSnapshotMappedFile::~ReadSnapshotMappedFile() {
    close();
}

bool ReadSnapshotMappedFile::open_read_only(const std::filesystem::path& path) {
    close();
#ifdef _WIN32
    file_ = ::CreateFileW(path.wstring().c_str(), GENERIC_READ, FILE_SHARE_READ, nullptr,
                          OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (file_ == INVALID_HANDLE_VALUE) {
        return false;
    }
    LARGE_INTEGER file_size;
    if (!::GetFileSizeEx(file_, &file_size) || file_size.QuadPart < 0) {
        close();
        return false;
    }
    size_ = static_cast<std::size_t>(file_size.QuadPart);
    if (size_ == 0) {
        return true;
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
    return true;
#else
    fd_ = ::open(path.c_str(), O_RDONLY);
    if (fd_ < 0) {
        return false;
    }
    struct stat statbuf;
    if (::fstat(fd_, &statbuf) != 0 || statbuf.st_size < 0) {
        close();
        return false;
    }
    size_ = static_cast<std::size_t>(statbuf.st_size);
    if (size_ == 0) {
        return true;
    }
    void* mapped = ::mmap(nullptr, size_, PROT_READ, MAP_PRIVATE, fd_, 0);
    if (mapped == MAP_FAILED) {
        close();
        return false;
    }
#ifdef MADV_RANDOM
    ::madvise(mapped, size_, MADV_RANDOM);
#endif
    data_ = static_cast<const std::byte*>(mapped);
    return true;
#endif
}

void ReadSnapshotMappedFile::close() {
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
    if (data_ != nullptr && size_ != 0) {
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

std::span<const std::byte> ReadSnapshotMappedFile::bytes() const noexcept {
    return data_ == nullptr ? std::span<const std::byte>{} : std::span<const std::byte>(data_, size_);
}

}  // namespace rbf::lect_database
