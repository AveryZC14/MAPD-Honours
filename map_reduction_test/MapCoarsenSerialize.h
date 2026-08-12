// MapCoarsenSerialize.h
// Save/load a `MultiLevelCoarsenedGraph` to/from a flat binary file, so a
// hierarchy built once for a map can be reused by later runs instead of
// being rebuilt from scratch every process start.

#pragma once

#include "MapCoarsenV1.h"

#include <string>

namespace MapReductionTest {

/**
 * Serialize `hierarchy` (as built for `env`) to `path`, overwriting any
 * existing file. Returns false (and leaves any partially-written file, so
 * a stale/truncated cache file just fails to load rather than corrupting a
 * previously-good one -- callers should treat a save failure as "cache not
 * written" and continue) on any I/O error or if `hierarchy`/`env` are
 * empty/null.
 */
bool save_hierarchy_to_file(const MultiLevelCoarsenedGraph& hierarchy,
                            const SharedEnvironment* env,
                            const std::string& path);

/**
 * Attempt to load a hierarchy previously written by `save_hierarchy_to_file`
 * into `out`. Fails closed: returns false and leaves `out` untouched unless
 * the file's format version, `env->rows`/`env->cols`, `compute_env_signature`
 * result, and (when `expected_num_levels >= 0`) level count all match --
 * this is what makes a stale/foreign/corrupt cache file safe to point at,
 * since a mismatch just means "cache not usable", not a crash or silent
 * wrong hierarchy.
 */
bool load_hierarchy_from_file(const std::string& path,
                              const SharedEnvironment* env,
                              int expected_num_levels,
                              MultiLevelCoarsenedGraph& out);

} // namespace MapReductionTest
