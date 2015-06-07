find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)

set(bathysphere_SRCS
  bathysphere.cpp
  boat.cpp
  bubble.cpp
  collisionresponse.cpp
  coral.cpp
  cord.cpp
  crane.cpp
  debug.cpp
  dirt.cpp
  environment.cpp
  kelp.cpp
  main.cpp
  naturalconstraint.cpp
  object.cpp
  observer.cpp
  octree.cpp
  octreecube.cpp
  orientableobject.cpp
  orientation.cpp
  primitives.cpp
  random.c
  rangeconstraint.cpp
  reef.cpp
  reefmesh.cpp
  rigidbody.cpp
  seabed.cpp
  searodcoral.cpp
  seasurface.cpp
  simplestrand.cpp
  skydome.cpp
  strand.cpp
  texload.cpp
)

include_directories(
  ${GLUT_INCLUDE_DIR}
  ${OPENGL_INCLUDE_DIR}
)

add_executable(bathysphere ${bathysphere_SRCS})
target_link_libraries(bathysphere
  aamaths
  aamesh
  aacollision
  ${GLUT_glut_LIBRARY}
  ${OPENGL_LIBRARIES}
)
