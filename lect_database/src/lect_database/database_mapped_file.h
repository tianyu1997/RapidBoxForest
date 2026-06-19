#pragma once

#include <rbf/lect_database/database.h>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <span>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace rbf::lect_database {

struct LectDatabase::EvidenceMappedFile {
    EvidenceMappedFile() = default;
    EvidenceMappedFile(const EvidenceMappedFile&) = delete;
    EvidenceMappedFile& operator=(const EvidenceMappedFile&) = delete;
    EvidenceMappedFile(EvidenceMappedFile&&) noexcept = default;
    EvidenceMappedFile& operator=(EvidenceMappedFile&&) noexcept = default;
    ~EvidenceMappedFile();

    bool open_read_only(const std::filesystem::path& path);
    void close();
    bool is_open() const noexcept;
    std::span<const std::byte> bytes() const noexcept;
    void prefetch(std::uint64_t offset, std::uint64_t size) const;
    std::size_t size() const noexcept;

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
