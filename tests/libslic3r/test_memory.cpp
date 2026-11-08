#include <catch2/catch_all.hpp>

#include <catch2/catch_test_macros.hpp>

#include "libslic3r/Memory.hpp"

#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <vector>

using namespace Slic3r;

// What a resident set size means depends on the platform and on what the allocator did before the
// test ran, so these tests assert the API's contract - a plausible value, a peak that bounds the
// current size, and a trim that is safe to call and repeat - never a specific number of bytes.
// Reclaim amounts are not asserted: they depend on the allocator, the OS and the timing of the
// page reclaim, none of which a test can pin down.

TEST_CASE("Process RSS queries report a resident set size", "[Memory]")
{
    const size_t current = current_process_rss_bytes();
    const size_t peak    = peak_process_rss_bytes();

#if defined(_WIN32) || defined(__linux__) || defined(__APPLE__)
    CHECK(current > 0);
    CHECK(peak > 0);
    // The peak happened at some point before "now", so it can never be below the current size.
    CHECK(peak >= current);
#else
    // Platforms without a memory query report 0 rather than a made-up number.
    CHECK(current == 0);
    CHECK(peak == 0);
#endif
}

TEST_CASE("Trimming the process memory is safe after a large allocation", "[Memory]")
{
    // A chunk large enough to leave the allocator holding real pages, then freed: exactly the shape
    // of working set a finished slice leaves behind.
    constexpr size_t chunk_size = 8 * 1024 * 1024;
    std::vector<char> chunk(chunk_size);
    std::memset(chunk.data(), 0x5A, chunk_size);
    chunk.clear();
    chunk.shrink_to_fit();

    CHECK_NOTHROW(trim_allocator_caches());
    CHECK_NOTHROW(trim_process_memory());
    // Trimming twice must not fail either: the second call finds nothing to release.
    CHECK_NOTHROW(trim_allocator_caches());
    CHECK_NOTHROW(trim_process_memory());

#if defined(_WIN32) || defined(__linux__) || defined(__APPLE__)
    // The process is still measurable after trimming.
    CHECK(current_process_rss_bytes() > 0);
#endif
}

TEST_CASE("Allocator initialization can be repeated", "[Memory]")
{
    // Configured from both entry points (GUI and command line), and a test binary may call it in
    // any order; it must therefore be harmless to run more than once.
    CHECK_NOTHROW(initialize_memory_allocator());
    CHECK_NOTHROW(initialize_memory_allocator());
}
