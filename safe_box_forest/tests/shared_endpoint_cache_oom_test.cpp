// Standalone OOM-bound stress test for SharedEndpointEvidenceCache.
// Validates that LRU eviction fires when the entry/byte budget is exceeded,
// that size stays within the configured cap, and that hot (recently-used)
// entries survive eviction while cold entries are dropped.
#include <rbf/lect_database/evidence_source.h>

#include <cassert>
#include <cstdio>
#include <vector>

using rbf::lect_database::SharedEndpointEvidenceCache;
using rbf::lect_database::EvidenceKey;
using rbf::Interval;

static std::vector<Interval> make_box(double base) {
    std::vector<Interval> iv(3);
    iv[0] = Interval{base, base + 1.0};
    iv[1] = Interval{base + 0.25, base + 1.25};
    iv[2] = Interval{base + 0.5, base + 1.5};
    return iv;
}

static EvidenceKey make_key() {
    EvidenceKey k{};
    return k;
}

int main() {
    const std::size_t cap = 100;
    SharedEndpointEvidenceCache cache(cap, /*max_bytes=*/0);
    EvidenceKey key = make_key();

    // Insert far more entries than the cap.
    const std::size_t total = 1000;
    for (std::size_t i = 0; i < total; ++i) {
        cache.put(make_box(static_cast<double>(i)), key,
                  std::vector<float>(16, static_cast<float>(i)),
                  /*child_hull=*/false, /*unavailable=*/false);
    }

    printf("entries=%zu bytes=%zu evictions=%llu\n", cache.size(), cache.bytes(),
           static_cast<unsigned long long>(cache.evictions()));

    // Size must never exceed the cap.
    assert(cache.size() <= cap && "size exceeded entry cap");
    // Evictions must have fired (inserted 1000, cap 100).
    assert(cache.evictions() >= total - cap && "expected evictions did not fire");
    // The most recently inserted entry must still be present.
    auto hit = cache.endpoint_for_box_exact(make_box(static_cast<double>(total - 1)), key);
    assert(hit.has_value() && "most-recent entry was wrongly evicted");
    // A very old entry must have been evicted.
    auto miss = cache.endpoint_for_box_exact(make_box(0.0), key);
    assert(!miss.has_value() && "oldest entry should have been evicted");

    // Byte-budget path: tiny byte cap forces eviction regardless of entry count.
    SharedEndpointEvidenceCache bcache(/*max_entries=*/0, /*max_bytes=*/4096);
    for (std::size_t i = 0; i < total; ++i) {
        bcache.put(make_box(static_cast<double>(i)), key,
                   std::vector<float>(64, static_cast<float>(i)), false, false);
    }
    printf("byte-cap: entries=%zu bytes=%zu evictions=%llu\n", bcache.size(),
           bcache.bytes(), static_cast<unsigned long long>(bcache.evictions()));
    assert(bcache.bytes() <= 4096 && "byte budget exceeded");
    assert(bcache.evictions() > 0 && "byte-budget eviction did not fire");

    printf("OK: shared endpoint cache OOM bounds enforced\n");
    return 0;
}
