#pragma once

#include <cstddef>

// Process memory footprint and the operations that can give freed pages back to the OS.
//
// The slicer's working set is dominated by short-lived geometry: per-layer slices of every
// volume, fill surfaces, bridging working areas. Freeing those objects returns the memory to the
// allocator, but neither the C runtime heap nor oneTBB's allocator hands the pages back to the
// operating system on their own schedule, so a process that sliced a large model keeps that
// footprint until it exits.
//
// Two strengths of hand-back are offered, because the strongest one is not safe everywhere:
//
//   trim_allocator_caches()  releases only pages the allocators are holding unused. Safe to call
//                            at any point, including between two phases of one slice that are
//                            about to read the same layer data again.
//   trim_process_memory()    everything the above does, plus the platform's working-set reclaim,
//                            which on Windows can page out memory the process is still using. Only
//                            for points where the run is over: idle, a finished slice, or a plate
//                            whose result was released completely. A partial release - the preview
//                            payload is gone but the print still owns its PrintObjects - is not
//                            such a point: it may only call trim_allocator_caches().
namespace Slic3r {

// Resident set size of this process in bytes, 0 when the platform cannot report it.
size_t current_process_rss_bytes();

// Peak resident set size of this process since it started, in bytes; never below the current RSS.
size_t peak_process_rss_bytes();

// Hands back the pages the allocators are holding unused, without touching anything live:
//   oneTBB allocator: scalable_allocation_command(TBBMALLOC_CLEAN_ALL_BUFFERS)
//   glibc:            malloc_trim(0)
//   macOS:            malloc_zone_pressure_relief()
//   Windows:          only the oneTBB caches: the Win32 heap has no call that returns its unused
//                     pages without also evicting the process working set, which is what
//                     trim_process_memory() adds. The CRT heap therefore stays as it is here.
// Returns true when the oneTBB command succeeded. That, like every return value below, reports
// that the mechanism ran, never how much it reclaimed: malloc_trim(0) returns 0 when the heap had
// nothing to give, which is not a failure. Safe to call from any thread and at any point.
bool trim_allocator_caches();

// trim_allocator_caches() plus the platform's working-set reclaim:
//   Windows: SetProcessWorkingSetSize(-1, -1), which removes as many pages from the working set as
//            it can - including pages that are still mapped and in use, so the next touch of them
//            faults them back in. Callers therefore use this only where nothing of the run is
//            read again: while idle, when a slice has finished, or after a plate's result was
//            released completely (payload and sliced objects both gone). A partial release keeps
//            live PrintObjects and must stay on trim_allocator_caches().
//   glibc / macOS: nothing beyond trim_allocator_caches(); their mechanisms release free pages
//            only, so the two functions are equivalent there.
// Returns true when every mechanism of the platform ran successfully; on Windows a successful
// SetProcessWorkingSetSize does not mean any page was actually paged out.
bool trim_process_memory();

// One-time allocator tuning for the process entry points, before the first slice allocates.
// Currently only glibc has thresholds worth setting; other platforms do nothing.
void initialize_memory_allocator();

} // namespace Slic3r
