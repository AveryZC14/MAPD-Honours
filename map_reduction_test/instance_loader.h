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
 */
void populate_env_from_instance(const std::string& input_json, SharedEnvironment& env);
