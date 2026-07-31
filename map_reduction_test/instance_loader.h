// instance_loader.h
// Shared instance-loading helper for the map_reduction_test standalone tools
// (run.cpp, validate_guide_paths.cpp). Extracted out of run.cpp so a second
// standalone executable can reuse it without duplicating the ~50-line
// parsing logic or linking against run.cpp's main().

#pragma once

#include <string>

#include "SharedEnv.h"

/**
 * Parse an instance JSON (map/agent/task file triple, same format as
 * src/driver.cpp reads for the full `lifelong` binary) and populate `env`
 * with its map, agent start locations, and tasks. All agents start free and
 * all tasks start newly-revealed, as they would be at timestep 0 of a real
 * run. Throws std::runtime_error if the file can't be opened or parsed.
 *
 * This differs from a real run in one important way: a real `lifelong` run
 * (src/TaskManager.cpp's `reveal_tasks`) caps the *open* task pool at
 * `numTasksReveal * teamSize` tasks and only ever reveals the small
 * incremental delta needed to top it back up each timestep -- it never
 * loads the whole task file into `env->task_pool` at once the way this
 * function does. `num_tasks_reveal_out`, if non-null, is set to the raw
 * `numTasksReveal` JSON field (default 1.0, matching driver.cpp's own
 * default) so a caller that wants to approximate the real capped pool size
 * can compute `numTasksReveal * teamSize` itself -- see
 * map_reduction_test/dump_guide_paths.cpp's `--match-runtime`.
 */
void populate_env_from_instance(const std::string& input_json, SharedEnvironment& env,
                                 float* num_tasks_reveal_out = nullptr);
