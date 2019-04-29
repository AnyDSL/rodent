find_path(Embree_DIR include/embree3/rtcore.h HINTS /usr)
find_path(Embree_INCLUDE_DIR embree3/rtcore.h HINTS ${Embree_DIR}/include)
find_library(Embree_LIBRARY NAMES embree3 PATHS ${Embree_DIR}/lib64 ${Embree_DIR}/lib)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Embree REQUIRED_VARS Embree_LIBRARY Embree_INCLUDE_DIR)
