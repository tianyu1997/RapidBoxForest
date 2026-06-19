#include <rbf/lect_database/database.h>

#include "database_file_layout.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <string>
#include <utility>

namespace rbf::lect_database {

using database_file::node_page_path;
using database_file::node_pages_path;
using database_file::parse_node_record;
using database_file::replace_file;
using database_file::serialize_node_record;

bool LectDatabase::has_node(NodeId node_id) const noexcept {
    return node_ids_.find(node_id) != node_ids_.end();
}

std::size_t LectDatabase::rows_per_node_page() const noexcept {
    constexpr std::size_t estimated_row_size = 160;
    return std::max<std::size_t>(1, static_cast<std::size_t>(config_.page_size_bytes) / estimated_row_size);
}

std::uint64_t LectDatabase::page_id_for_node(NodeId node_id) const noexcept {
    return static_cast<std::uint64_t>(node_id / rows_per_node_page());
}

NodeId LectDatabase::first_node_id_for_page(std::uint64_t page_id) const noexcept {
    return static_cast<NodeId>(page_id * rows_per_node_page());
}

std::size_t LectDatabase::node_offset_in_page(NodeId node_id) const noexcept {
    return static_cast<std::size_t>(node_id % rows_per_node_page());
}

LectDatabase::NodePage& LectDatabase::touch_node_page(std::uint64_t page_id) const {
    auto existing = node_pages_.find(page_id);
    if (existing != node_pages_.end()) {
        ++stats_.node_page_cache_hits;
        existing->second.last_access = ++node_page_clock_;
        update_resident_page_stats();
        return existing->second;
    }

    ++stats_.node_page_cache_misses;
    ++stats_.node_page_reads;
    NodePage page;
    page.page_id = page_id;
    page.first_node_id = first_node_id_for_page(page_id);
    page.last_access = ++node_page_clock_;
    const auto rows_per_page = rows_per_node_page();

    std::ifstream input(node_page_path(config_.path, page_id));
    std::string line;
    while (std::getline(input, line)) {
        if (line.empty()) {
            continue;
        }
        auto record = parse_node_record(line);
        if (!record || record->id < page.first_node_id) {
            continue;
        }
        const auto offset = static_cast<std::size_t>(record->id - page.first_node_id);
        if (offset >= rows_per_page) {
            continue;
        }
        if (offset >= page.rows.size()) {
            page.rows.resize(offset + 1);
        }
        page.rows[offset] = std::move(*record);
    }

    auto [inserted, _] = node_pages_.emplace(page_id, std::move(page));
    (void)_;
    evict_node_pages_if_needed();
    update_resident_page_stats();
    return node_pages_.at(inserted->first);
}

bool LectDatabase::flush_node_page(std::uint64_t page_id) const {
    auto it = node_pages_.find(page_id);
    if (it == node_pages_.end() || !it->second.dirty) {
        return true;
    }
    if (config_.open.read_only) {
        return false;
    }
    std::filesystem::create_directories(node_pages_path(config_.path));
    const auto path = node_page_path(config_.path, page_id);
    const auto tmp = path.string() + ".tmp";
    std::ofstream out(tmp, std::ios::trunc);
    if (!out) {
        return false;
    }
    for (auto& row : it->second.rows) {
        if (!valid_node_id(row.id) || !has_node(row.id)) {
            continue;
        }
        row.page_id = page_id;
        row.dirty = false;
        out << serialize_node_record(row) << '\n';
    }
    out.close();
    if (!static_cast<bool>(out) || !replace_file(tmp, path)) {
        return false;
    }
    it->second.dirty = false;
    ++stats_.node_page_writes;
    ++stats_.node_page_dirty_flushes;
    return true;
}

bool LectDatabase::flush_all_node_pages() const {
    std::vector<std::uint64_t> page_ids;
    page_ids.reserve(node_pages_.size());
    for (const auto& [page_id, page] : node_pages_) {
        (void)page;
        page_ids.push_back(page_id);
    }
    for (const auto page_id : page_ids) {
        if (!flush_node_page(page_id)) {
            return false;
        }
    }
    return true;
}

void LectDatabase::evict_node_pages_if_needed() const {
    const std::size_t limit = std::max<std::size_t>(1, config_.max_resident_pages);
    const std::size_t hot_page_count = std::min<std::size_t>(2, limit / 2);
    const auto is_hot_page = [&](std::uint64_t page_id) {
        return hot_page_count > 0 && page_id < hot_page_count;
    };
    while (node_pages_.size() > limit) {
        auto victim = node_pages_.end();
        for (auto it = node_pages_.begin(); it != node_pages_.end(); ++it) {
            if (is_hot_page(it->first)) {
                continue;
            }
            if (victim == node_pages_.end() || it->second.last_access < victim->second.last_access) {
                victim = it;
            }
        }
        if (victim == node_pages_.end()) {
            for (auto it = node_pages_.begin(); it != node_pages_.end(); ++it) {
                if (victim == node_pages_.end() || it->second.last_access < victim->second.last_access) {
                    victim = it;
                }
            }
        }
        if (victim == node_pages_.end()) {
            break;
        }
        const bool dirty = victim->second.dirty;
        if (dirty && !flush_node_page(victim->first)) {
            break;
        }
        if (dirty) {
            ++stats_.node_page_dirty_evictions;
        }
        ++stats_.node_page_evictions;
        node_pages_.erase(victim);
        update_resident_page_stats();
    }
}

void LectDatabase::update_resident_page_stats() const noexcept {
    stats_.resident_node_pages = static_cast<std::uint64_t>(node_pages_.size());
    stats_.max_resident_node_pages = std::max<std::uint64_t>(stats_.max_resident_node_pages,
                                                            stats_.resident_node_pages);
}

std::optional<NodeRecord> LectDatabase::read_node(NodeId node_id) const {
    if (!has_node(node_id)) {
        return std::nullopt;
    }
    const auto page_id = page_id_for_node(node_id);
    const auto offset = node_offset_in_page(node_id);
    auto& page = touch_node_page(page_id);
    if (offset >= page.rows.size() || page.rows[offset].id != node_id) {
        return std::nullopt;
    }
    return page.rows[offset];
}

NodeRecord* LectDatabase::mutable_node(NodeId node_id) {
    if (!has_node(node_id)) {
        return nullptr;
    }
    const auto page_id = page_id_for_node(node_id);
    const auto offset = node_offset_in_page(node_id);
    auto& page = touch_node_page(page_id);
    if (offset >= page.rows.size() || page.rows[offset].id != node_id) {
        return nullptr;
    }
    page.dirty = true;
    page.rows[offset].dirty = true;
    return &page.rows[offset];
}

bool LectDatabase::write_node_record(NodeRecord record) {
    if (!valid_node_id(record.id)) {
        return false;
    }
    if (!remember_node_record(record)) {
        return false;
    }
    record.page_id = page_id_for_node(record.id);
    const auto page_id = record.page_id;
    const auto offset = node_offset_in_page(record.id);
    auto& page = touch_node_page(page_id);
    if (offset >= page.rows.size()) {
        page.rows.resize(offset + 1);
    }
    record.dirty = true;
    page.rows[offset] = std::move(record);
    page.dirty = true;
    evict_node_pages_if_needed();
    return true;
}

bool LectDatabase::remember_node_id(NodeId node_id) {
    if (!valid_node_id(node_id)) {
        return false;
    }
    const auto inserted = node_ids_.insert(node_id).second;
    if (inserted) {
        ++node_count_;
    }
    max_node_id_ = std::max(max_node_id_, node_id);
    if (node_id < kInvalidNodeId - 1) {
        next_node_id_ = std::max(next_node_id_, node_id + 1);
    }
    return true;
}

bool LectDatabase::remember_node_record(const NodeRecord& record) {
    if (!remember_node_id(record.id)) {
        return false;
    }
    const auto existing = node_path_index_.find(record.path);
    if (existing != node_path_index_.end() && existing->second != record.id) {
        return false;
    }
    node_path_index_[record.path] = record.id;
    return true;
}

std::vector<NodeId> LectDatabase::sorted_node_ids() const {
    std::vector<NodeId> ids(node_ids_.begin(), node_ids_.end());
    std::sort(ids.begin(), ids.end());
    return ids;
}

}  // namespace rbf::lect_database
