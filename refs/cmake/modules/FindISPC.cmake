find_path(ISPC_DIR ispc HINTS /usr/bin)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ISPC REQUIRED_VARS ISPC_DIR)
