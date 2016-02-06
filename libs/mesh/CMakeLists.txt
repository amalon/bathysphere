find_package(OpenGL REQUIRED)

set(aamesh_SRCS
  AdaptiveMesh.cpp
  Mesh.cpp
)

include_directories(
  ${OPENGL_INCLUDE_DIR}
)

add_library(aamesh STATIC ${aamesh_SRCS})
target_link_libraries(aamesh
  ${OPENGL_gl_LIBRARY}
)
