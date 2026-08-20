project(lifelong)

cmake_minimum_required(VERSION 3.16)
set(CMAKE_CXX_STANDARD 17)

# Set the default value of PYTHON to false
option(PYTHON "Enable Python" OFF)

option(CXX "enable C++ compilation" ON)

if(CXX)
  enable_language(CXX)
endif()

if(PYTHON)
    set(PYTHON_FLAG 1)
else()
    set(PYTHON_FLAG 0)
endif()


# Print the value of PYTHON
message(STATUS "PYTHON: ${PYTHON}")

IF(NOT CMAKE_BUILD_TYPE)
    # SET(CMAKE_BUILD_TYPE "DEBUG")
    SET(CMAKE_BUILD_TYPE "RELEASE")
    #SET(CMAKE_BUILD_TYPE "RELWITHDEBINFO")
    #SET(CMAKE_BUILD_TYPE "MINSIZEREL")
ENDIF()

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

include_directories("inc")

# default planner
include_directories("default_planner")

# map reduction / coarsening headers (needed by utils/validation tools)
include_directories("map_reduction_test")

file(GLOB SOURCES "src/*.cpp" "default_planner/*.cpp")

#-------
# Ensure map reduction implementation is compiled into main targets so
# functions like reduce_map_2x2 are available to scheduler.cpp.
list(APPEND SOURCES "map_reduction_test/mapReductionV0.cpp")
list(APPEND SOURCES "map_reduction_test/MapCoarsenV1.cpp")
list(APPEND SOURCES "map_reduction_test/MapCoarsenSerialize.cpp")
list(APPEND SOURCES "map_reduction_test/instance_loader.cpp")
list(APPEND SOURCES "map_reduction_test/LocalNodeMatch.cpp")
list(APPEND SOURCES "map_reduction_test/EdgeAugmentedCoarsen.cpp")
#------


# Find Boost
ADD_DEFINITIONS(-DBOOST_LOG_DYN_LINK)
#SET(Boost_USE_STATIC_LIBS ON)
find_package( Boost 1.49.0 REQUIRED COMPONENTS program_options system filesystem log log_setup)

include_directories( ${Boost_INCLUDE_DIRS} )

# # find gurobi
# find_package(GUROBI REQUIRED)

# include_directories(${GUROBI_INCLUDE_DIRS})

# find lemon
find_package(LEMON REQUIRED)
include_directories(${LEMON_INCLUDE_DIRS})


IF(PYTHON)
    message(STATUS "Python support is enabled")

    find_package(pybind11 REQUIRED)
    set(CMAKE_CXX_STANDARD 17 CACHE STRING "C++ version selection")

     # check if python/tmp exists, raise an error if it does not
    if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/python/tmp")
        message(FATAL_ERROR "Track is not configured for python bindings. Please run python/set_track.bash <combined|planner|scheduler> to configure the track before compiling.")
    endif()

    include_directories("python/tmp")

    pybind11_add_module(MAPF "./src/Grid.cpp"   "./src/States.cpp" "./python/tmp/MAPFbinding.cpp")
    target_link_libraries(MAPF PRIVATE ) 

    file(GLOB PY_SOURCES "python/tmp/*.cpp")
    list(FILTER PY_SOURCES EXCLUDE REGEX ".*python/MAPFbinding\.cpp")

    add_executable(lifelong  ${PY_SOURCES} ${SOURCES})
    target_link_libraries(lifelong PRIVATE pybind11::embed ${Boost_LIBRARIES})
    target_compile_definitions(lifelong PRIVATE PYTHON=${PYTHON_FLAG})

    if (COPY_PY_PATH_CONFIG)
        file(COPY "config.json" DESTINATION ${CMAKE_BINARY_DIR})
    endif()


ELSE()
    message(STATUS "Python support is disabled")

    
    # message(${GUROBI_CXX_LIBRARY})

    add_executable(lifelong ${SOURCES} )
    # target_link_libraries(lifelong ${Boost_LIBRARIES} ${GUROBI_CXX_LIBRARY} ${GUROBI_LIBRARY} ${LEMON_LIBRARIES})
    target_link_libraries(lifelong ${Boost_LIBRARIES} ${LEMON_LIBRARIES})
    target_compile_definitions(lifelong PRIVATE PYTHON=${PYTHON_FLAG})
ENDIF()


#-----------
# Add small test target to compare schedule_plan_flow vs schedule_plan_flow_reduced
# Create a project-source list that excludes the original program's main (src/driver.cpp)
set(PROJECT_SOURCES ${SOURCES})
# remove any source file that contains driver.cpp to avoid duplicate main
list(FILTER PROJECT_SOURCES EXCLUDE REGEX ".*driver\.cpp$")

add_executable(map_reduction_test map_reduction_test/run.cpp ${PROJECT_SOURCES})
target_link_libraries(map_reduction_test ${Boost_LIBRARIES} ${LEMON_LIBRARIES})
target_compile_definitions(map_reduction_test PRIVATE PYTHON=${PYTHON_FLAG})
#-----------

#-----------
# Standalone guide-path format/validity checker: verifies solver 1's and
# solver 6's agent_guide_path output are structurally interchangeable (same
# type, valid start/end/adjacency, no empty entries) and can be fed to the
# low-level planner the same way. See ai/guide_path_metric.md.
add_executable(guide_path_validator utils/validation/validate_guide_paths.cpp ${PROJECT_SOURCES})
target_link_libraries(guide_path_validator ${Boost_LIBRARIES} ${LEMON_LIBRARIES})
target_compile_definitions(guide_path_validator PRIVATE PYTHON=${PYTHON_FLAG})
#-----------

#-----------
# Standalone tool: builds a hierarchy, saves it via MapCoarsenSerialize,
# reloads it, and asserts the two are structurally identical level-by-level
# (node coordinates/mappings, arcs, both bridge caches) plus that a
# mismatched signature/level-count is correctly rejected. See
# ai/hierarchy_cache.md.
add_executable(hierarchy_cache_validator utils/validation/validate_hierarchy_cache.cpp ${PROJECT_SOURCES})
target_link_libraries(hierarchy_cache_validator ${Boost_LIBRARIES} ${LEMON_LIBRARIES})
target_compile_definitions(hierarchy_cache_validator PRIVATE PYTHON=${PYTHON_FLAG})
#-----------

#-----------
# Standalone tool: dump solver 1 and solver 6 guide paths (for a handful of
# agents) to a CSV for visualisation, see map_reduction_test/visualisation/
# plot_guide_paths.py.
add_executable(dump_guide_paths map_reduction_test/dump_guide_paths.cpp ${PROJECT_SOURCES})
target_link_libraries(dump_guide_paths ${Boost_LIBRARIES} ${LEMON_LIBRARIES})
target_compile_definitions(dump_guide_paths PRIVATE PYTHON=${PYTHON_FLAG})
#-----------

#-----------
# Standalone tool: dump the map-coarsening hierarchy (per-level node
# positions, coarse graph arcs, per-cell component partitions) to CSV for
# visualisation, see map_reduction_test/visualisation/plot_coarsening.py.
add_executable(dump_coarsening map_reduction_test/dump_coarsening.cpp ${PROJECT_SOURCES})
target_link_libraries(dump_coarsening ${Boost_LIBRARIES} ${LEMON_LIBRARIES})
target_compile_definitions(dump_coarsening PRIVATE PYTHON=${PYTHON_FLAG})
#-----------

#-----------
# Standalone tool: quantifies how often agents/tasks collapse onto the same
# top-level coarse node at each hierarchy level, and the real (fine-grid)
# matched-distance gap between the FIFO pairing compute_reduced_assignment
# actually produces for same-node collisions and an achievable
# nearest-neighbor pairing -- tests whether coarse-node collisions explain
# the throughput regression at deep --flowSolveLevel values.
add_executable(analyze_coarse_collisions map_reduction_test/analyze_coarse_collisions.cpp ${PROJECT_SOURCES})
target_link_libraries(analyze_coarse_collisions ${Boost_LIBRARIES} ${LEMON_LIBRARIES})
target_compile_definitions(analyze_coarse_collisions PRIVATE PYTHON=${PYTHON_FLAG})
#-----------

#-----------
# Standalone tool: structural rigor check for solver 7's edge-node-augmented
# backbone (EdgeAugmentedCoarsen.{h,cpp}) -- degree/arc-count invariants plus
# a cross-check of the fast bounding-box sweep against a naive recursive
# reference, see ai/edge_node_representation.md.
add_executable(edge_augmented_validator utils/validation/validate_edge_augmented_graph.cpp ${PROJECT_SOURCES})
target_link_libraries(edge_augmented_validator ${Boost_LIBRARIES} ${LEMON_LIBRARIES})
target_compile_definitions(edge_augmented_validator PRIVATE PYTHON=${PYTHON_FLAG})
#-----------


add_custom_target(clean_all
    COMMAND ${CMAKE_BUILD_TOOL} clean
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/CMakeCache.txt
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/lifelong
    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/build
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/*.so
)


set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES
    "${CMAKE_BINARY_DIR}/lifelong;
     ${CMAKE_BINARY_DIR}/build;
     ${CMAKE_BINARY_DIR}/*.so"
)



