#pragma once

#include <cstddef>
#include <filesystem>
#include <span>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace rbf::lect_database {

class ReadSnapshotMappedFile {
public:
    ~ReadSnapshotMappedFile();

    ReadSnapshotMappedFile() = default;
    ReadSnapshotMappedFile(const ReadSnapshotMappedFile&) = delete;
    ReadSnapshotMappedFile& operator=(const ReadSnapshotMappedFile&) = delete;
    ReadSnapshotMappedFile(ReadSnapshotMappedFile&&) = delete;
    ReadSnapshotMappedFile& operator=(ReadSnapshotMappedFile&&) = delete;

    bool open_read_only(const std::filesystem::path& path);
    void close();

    std::span<const std::byte> bytes() const noexcept;
    std::size_t size() const noexcept { return size_; }

private:
#ifdef _WIN32
    HANDLE file_ = INVALID_HANDLE_VALUE;
    HANDLE mapping_ = nullptr;
    void* view_ = nullptr;
#else
    int fd_ = -1;
#endif
    const std::byte* data_ = nullptr;
    std::size_t size_ = 0;
};

}  // namespace rbf::lect_database
