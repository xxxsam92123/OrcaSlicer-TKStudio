#include "Memory.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <fstream>

// oneTBB's scalable allocator ships in every build of the slicer: libslic3r links TBB::tbbmalloc
// (src/libslic3r/CMakeLists.txt), which deps/TBB/TBB.cmake builds statically. Calling the
// allocator's own command directly is therefore always available - no runtime lookup of a
// library that may not be loaded, and no way for the call to silently do nothing because a
// dynamic load failed.
#include <oneapi/tbb/scalable_allocator.h>

#if defined(_WIN32)
    // PSAPI_VERSION 2 maps GetProcessMemoryInfo() onto K32GetProcessMemoryInfo(), which lives in
    // kernel32 instead of psapi.dll; the pragma keeps the psapi.lib import satisfied for SDKs
    // that still declare the older entry point.
    #ifndef PSAPI_VERSION
        #define PSAPI_VERSION 2
    #endif
    #include <windows.h>
    #include <psapi.h>
    #pragma comment(lib, "psapi.lib")
#else
    #include <sys/resource.h>
    #include <sys/types.h>
    #include <unistd.h>
    #if defined(__APPLE__)
        #include <mach/mach.h>
        #include <malloc/malloc.h>
    #endif
    #if defined(__linux__)
        #include <malloc.h>
    #endif
#endif

namespace Slic3r {

size_t current_process_rss_bytes()
{
#if defined(_WIN32)
    PROCESS_MEMORY_COUNTERS_EX counters;
    if (GetProcessMemoryInfo(GetCurrentProcess(), reinterpret_cast<PROCESS_MEMORY_COUNTERS *>(&counters), sizeof(counters)))
        return size_t(counters.WorkingSetSize);
    return 0;
#elif defined(__APPLE__)
    mach_task_basic_info info;
    mach_msg_type_number_t info_count = MACH_TASK_BASIC_INFO_COUNT;
    if (task_info(mach_task_self(), MACH_TASK_BASIC_INFO, reinterpret_cast<task_info_t>(&info), &info_count) == KERN_SUCCESS)
        return size_t(info.resident_size);
    return 0;
#elif defined(__linux__)
    // statm is the cheapest RSS source that does not need a page-table walk: size, resident,
    // shared, all in pages.
    std::ifstream statm("/proc/self/statm");
    size_t total_pages = 0, resident_pages = 0, shared_pages = 0;
    if (statm && (statm >> total_pages >> resident_pages >> shared_pages)) {
        const size_t page_size = size_t(sysconf(_SC_PAGE_SIZE));
        return resident_pages * page_size;
    }
    return 0;
#else
    return 0;
#endif
}

size_t peak_process_rss_bytes()
{
    const size_t current = current_process_rss_bytes();
    size_t peak = 0;
#if defined(_WIN32)
    PROCESS_MEMORY_COUNTERS_EX counters;
    if (GetProcessMemoryInfo(GetCurrentProcess(), reinterpret_cast<PROCESS_MEMORY_COUNTERS *>(&counters), sizeof(counters)))
        peak = size_t(counters.PeakWorkingSetSize);
#elif defined(__linux__) || defined(__APPLE__)
    struct rusage usage;
    if (getrusage(RUSAGE_SELF, &usage) == 0) {
        peak = size_t(usage.ru_maxrss);
        // ru_maxrss is kilobytes on Linux and bytes on the BSDs and macOS.
        #if defined(__linux__)
            peak *= 1024;
        #endif
    }
#endif
    return std::max(peak, current);
}

bool trim_allocator_caches()
{
    // The allocator's per-thread caches hold freed blocks that never go back to the OS. Both the
    // TBB containers and, in a static TBB build, allocations routed to scalable_malloc sit here.
    bool invoked = scalable_allocation_command(TBBMALLOC_CLEAN_ALL_BUFFERS, nullptr) == TBBMALLOC_OK;

    // Neither call below can touch a live page: both only return free pages of the heap to the OS.
#if defined(__APPLE__)
    // Returns the number of bytes relieved; a zero simply means there was nothing to give back.
    malloc_zone_pressure_relief(nullptr, 0);
#elif defined(__linux__) && defined(__GLIBC__)
    // 0 means the heap had nothing above the break to give back, not a failure.
    malloc_trim(0);
#else
    // Windows: deliberately nothing. SetProcessWorkingSetSize() is the only Win32 call that
    // returns this process's unused pages, and it does so by evicting pages from the working set
    // whether or not they are in use - see trim_process_memory(). A pipeline that is about to
    // read the same layers again must not pay for that.
#endif

    return invoked;
}

bool trim_process_memory()
{
    bool invoked = trim_allocator_caches();

#if defined(_WIN32)
    // Windows does not trim the CRT heap on free: the pages stay in the working set until the
    // memory manager notices pressure. (SIZE_T)-1 for both sizes means "give back as much as
    // possible"; the pages are paged out, not lost, and fault back in on the next touch. That
    // includes pages still in use, which is why callers use this only at idle / finished points.
    if (SetProcessWorkingSetSize(GetCurrentProcess(), (SIZE_T)-1, (SIZE_T)-1) == 0)
        invoked = false;
#endif

    return invoked;
}

void initialize_memory_allocator()
{
    // Nothing is pinned here on purpose.
    //
    // The hand-back this file implements is explicit: oneTBB's clean-all-buffers plus
    // malloc_trim(0), and malloc_trim(0) trims unconditionally - it does not consult
    // M_TRIM_THRESHOLD at all. Writing M_TRIM_THRESHOLD = 128 KiB back would only repeat glibc's
    // own default and would not decide anything either: glibc raises that threshold dynamically
    // when it releases an mmap'd block, and mallopt(M_TRIM_THRESHOLD, ...) does not switch that
    // adjustment off. A global mallopt() therefore buys nothing here and changes allocator
    // behaviour for the whole process, so it is not done.
    //
    // M_MMAP_THRESHOLD is left alone for the same reason: setting it turns the dynamic mmap
    // threshold off, which pushes every mid-sized allocation in the process - not only the
    // short-lived geometry - onto mmap/munmap, and nothing measured justifies that.
    //
    // The function stays as the one place allocator tuning at process entry would go, and stays
    // harmless to call more than once (both entry points call it).
}

} // namespace Slic3r
