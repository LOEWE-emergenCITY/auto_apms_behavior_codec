# Patch tinycbor CMakeLists.txt to use CMAKE_CURRENT_BINARY_DIR instead of CMAKE_BINARY_DIR.
# When tinycbor is included via FetchContent, CMAKE_BINARY_DIR points to the top-level build
# directory rather than tinycbor's own build directory, causing install rules to fail.
file(READ CMakeLists.txt content)
string(REPLACE "\${CMAKE_BINARY_DIR}" "\${CMAKE_CURRENT_BINARY_DIR}" content "${content}")
file(WRITE CMakeLists.txt "${content}")
