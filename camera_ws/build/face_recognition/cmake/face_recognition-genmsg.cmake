# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(FATAL_ERROR "Could not find messages which '/home/tucuman/camera_ws/devel/share/face_recognition/msg/FaceRecognitionAction.msg' depends on. Did you forget to specify generate_messages(DEPENDENCIES ...)?
Cannot locate message [Header] in package [std_msgs] with paths [['/home/tucuman/catkin_ws/devel/share/std_msgs/cmake/../msg']]")
message(FATAL_ERROR "Could not find messages which '/home/tucuman/camera_ws/devel/share/face_recognition/msg/FaceRecognitionActionGoal.msg' depends on. Did you forget to specify generate_messages(DEPENDENCIES ...)?
Cannot locate message [Header] in package [std_msgs] with paths [['/home/tucuman/catkin_ws/devel/share/std_msgs/cmake/../msg']]")
message(FATAL_ERROR "Could not find messages which '/home/tucuman/camera_ws/devel/share/face_recognition/msg/FaceRecognitionActionResult.msg' depends on. Did you forget to specify generate_messages(DEPENDENCIES ...)?
Cannot locate message [Header] in package [std_msgs] with paths [['/home/tucuman/catkin_ws/devel/share/std_msgs/cmake/../msg']]")
message(FATAL_ERROR "Could not find messages which '/home/tucuman/camera_ws/devel/share/face_recognition/msg/FaceRecognitionActionFeedback.msg' depends on. Did you forget to specify generate_messages(DEPENDENCIES ...)?
Cannot locate message [Header] in package [std_msgs] with paths [['/home/tucuman/catkin_ws/devel/share/std_msgs/cmake/../msg']]")
message(STATUS "face_recognition: 8 messages, 0 services")

set(MSG_I_FLAGS "-Iface_recognition:/home/tucuman/camera_ws/src/face_recognition/msg;-Iface_recognition:/home/tucuman/camera_ws/devel/share/face_recognition/msg;-Istd_msgs:/home/tucuman/catkin_ws/devel/share/std_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/melodic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(face_recognition_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/tucuman/camera_ws/devel/share/face_recognition/msg/FaceRecognitionFeedback.msg" NAME_WE)
add_custom_target(_face_recognition_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "face_recognition" "/home/tucuman/camera_ws/devel/share/face_recognition/msg/FaceRecognitionFeedback.msg" ""
)

get_filename_component(_filename "/home/tucuman/camera_ws/devel/share/face_recognition/msg/FaceRecognitionResult.msg" NAME_WE)
add_custom_target(_face_recognition_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "face_recognition" "/home/tucuman/camera_ws/devel/share/face_recognition/msg/FaceRecognitionResult.msg" ""
)

get_filename_component(_filename "/home/tucuman/camera_ws/devel/share/face_recognition/msg/FaceRecognitionGoal.msg" NAME_WE)
add_custom_target(_face_recognition_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "face_recognition" "/home/tucuman/camera_ws/devel/share/face_recognition/msg/FaceRecognitionGoal.msg" ""
)

get_filename_component(_filename "/home/tucuman/camera_ws/src/face_recognition/msg/FRClientGoal.msg" NAME_WE)
add_custom_target(_face_recognition_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "face_recognition" "/home/tucuman/camera_ws/src/face_recognition/msg/FRClientGoal.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(face_recognition
  "/home/tucuman/camera_ws/devel/share/face_recognition/msg/FaceRecognitionResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/face_recognition
)
_generate_msg_cpp(face_recognition
  "/home/tucuman/camera_ws/devel/share/face_recognition/msg/FaceRecognitionGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/face_recognition
)
_generate_msg_cpp(face_recognition
  "/home/tucuman/camera_ws/src/face_recognition/msg/FRClientGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/face_recognition
)
_generate_msg_cpp(face_recognition
  "/home/tucuman/camera_ws/devel/share/face_recognition/msg/FaceRecognitionFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/face_recognition
)

### Generating Services

### Generating Module File
_generate_module_cpp(face_recognition
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/face_recognition
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(face_recognition_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(face_recognition_generate_messages face_recognition_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tucuman/camera_ws/devel/share/face_recognition/msg/FaceRecognitionFeedback.msg" NAME_WE)
add_dependencies(face_recognition_generate_messages_cpp _face_recognition_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tucuman/camera_ws/devel/share/face_recognition/msg/FaceRecognitionResult.msg" NAME_WE)
add_dependencies(face_recognition_generate_messages_cpp _face_recognition_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tucuman/camera_ws/devel/share/face_recognition/msg/FaceRecognitionGoal.msg" NAME_WE)
add_dependencies(face_recognition_generate_messages_cpp _face_recognition_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tucuman/camera_ws/src/face_recognition/msg/FRClientGoal.msg" NAME_WE)
add_dependencies(face_recognition_generate_messages_cpp _face_recognition_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(face_recognition_gencpp)
add_dependencies(face_recognition_gencpp face_recognition_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS face_recognition_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(face_recognition
  "/home/tucuman/camera_ws/devel/share/face_recognition/msg/FaceRecognitionResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/face_recognition
)
_generate_msg_eus(face_recognition
  "/home/tucuman/camera_ws/devel/share/face_recognition/msg/FaceRecognitionGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/face_recognition
)
_generate_msg_eus(face_recognition
  "/home/tucuman/camera_ws/src/face_recognition/msg/FRClientGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/face_recognition
)
_generate_msg_eus(face_recognition
  "/home/tucuman/camera_ws/devel/share/face_recognition/msg/FaceRecognitionFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/face_recognition
)

### Generating Services

### Generating Module File
_generate_module_eus(face_recognition
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/face_recognition
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(face_recognition_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(face_recognition_generate_messages face_recognition_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tucuman/camera_ws/devel/share/face_recognition/msg/FaceRecognitionFeedback.msg" NAME_WE)
add_dependencies(face_recognition_generate_messages_eus _face_recognition_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tucuman/camera_ws/devel/share/face_recognition/msg/FaceRecognitionResult.msg" NAME_WE)
add_dependencies(face_recognition_generate_messages_eus _face_recognition_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tucuman/camera_ws/devel/share/face_recognition/msg/FaceRecognitionGoal.msg" NAME_WE)
add_dependencies(face_recognition_generate_messages_eus _face_recognition_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tucuman/camera_ws/src/face_recognition/msg/FRClientGoal.msg" NAME_WE)
add_dependencies(face_recognition_generate_messages_eus _face_recognition_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(face_recognition_geneus)
add_dependencies(face_recognition_geneus face_recognition_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS face_recognition_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(face_recognition
  "/home/tucuman/camera_ws/devel/share/face_recognition/msg/FaceRecognitionResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/face_recognition
)
_generate_msg_lisp(face_recognition
  "/home/tucuman/camera_ws/devel/share/face_recognition/msg/FaceRecognitionGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/face_recognition
)
_generate_msg_lisp(face_recognition
  "/home/tucuman/camera_ws/src/face_recognition/msg/FRClientGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/face_recognition
)
_generate_msg_lisp(face_recognition
  "/home/tucuman/camera_ws/devel/share/face_recognition/msg/FaceRecognitionFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/face_recognition
)

### Generating Services

### Generating Module File
_generate_module_lisp(face_recognition
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/face_recognition
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(face_recognition_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(face_recognition_generate_messages face_recognition_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tucuman/camera_ws/devel/share/face_recognition/msg/FaceRecognitionFeedback.msg" NAME_WE)
add_dependencies(face_recognition_generate_messages_lisp _face_recognition_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tucuman/camera_ws/devel/share/face_recognition/msg/FaceRecognitionResult.msg" NAME_WE)
add_dependencies(face_recognition_generate_messages_lisp _face_recognition_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tucuman/camera_ws/devel/share/face_recognition/msg/FaceRecognitionGoal.msg" NAME_WE)
add_dependencies(face_recognition_generate_messages_lisp _face_recognition_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tucuman/camera_ws/src/face_recognition/msg/FRClientGoal.msg" NAME_WE)
add_dependencies(face_recognition_generate_messages_lisp _face_recognition_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(face_recognition_genlisp)
add_dependencies(face_recognition_genlisp face_recognition_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS face_recognition_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(face_recognition
  "/home/tucuman/camera_ws/devel/share/face_recognition/msg/FaceRecognitionResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/face_recognition
)
_generate_msg_nodejs(face_recognition
  "/home/tucuman/camera_ws/devel/share/face_recognition/msg/FaceRecognitionGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/face_recognition
)
_generate_msg_nodejs(face_recognition
  "/home/tucuman/camera_ws/src/face_recognition/msg/FRClientGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/face_recognition
)
_generate_msg_nodejs(face_recognition
  "/home/tucuman/camera_ws/devel/share/face_recognition/msg/FaceRecognitionFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/face_recognition
)

### Generating Services

### Generating Module File
_generate_module_nodejs(face_recognition
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/face_recognition
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(face_recognition_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(face_recognition_generate_messages face_recognition_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tucuman/camera_ws/devel/share/face_recognition/msg/FaceRecognitionFeedback.msg" NAME_WE)
add_dependencies(face_recognition_generate_messages_nodejs _face_recognition_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tucuman/camera_ws/devel/share/face_recognition/msg/FaceRecognitionResult.msg" NAME_WE)
add_dependencies(face_recognition_generate_messages_nodejs _face_recognition_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tucuman/camera_ws/devel/share/face_recognition/msg/FaceRecognitionGoal.msg" NAME_WE)
add_dependencies(face_recognition_generate_messages_nodejs _face_recognition_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tucuman/camera_ws/src/face_recognition/msg/FRClientGoal.msg" NAME_WE)
add_dependencies(face_recognition_generate_messages_nodejs _face_recognition_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(face_recognition_gennodejs)
add_dependencies(face_recognition_gennodejs face_recognition_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS face_recognition_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(face_recognition
  "/home/tucuman/camera_ws/devel/share/face_recognition/msg/FaceRecognitionResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/face_recognition
)
_generate_msg_py(face_recognition
  "/home/tucuman/camera_ws/devel/share/face_recognition/msg/FaceRecognitionGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/face_recognition
)
_generate_msg_py(face_recognition
  "/home/tucuman/camera_ws/src/face_recognition/msg/FRClientGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/face_recognition
)
_generate_msg_py(face_recognition
  "/home/tucuman/camera_ws/devel/share/face_recognition/msg/FaceRecognitionFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/face_recognition
)

### Generating Services

### Generating Module File
_generate_module_py(face_recognition
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/face_recognition
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(face_recognition_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(face_recognition_generate_messages face_recognition_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tucuman/camera_ws/devel/share/face_recognition/msg/FaceRecognitionFeedback.msg" NAME_WE)
add_dependencies(face_recognition_generate_messages_py _face_recognition_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tucuman/camera_ws/devel/share/face_recognition/msg/FaceRecognitionResult.msg" NAME_WE)
add_dependencies(face_recognition_generate_messages_py _face_recognition_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tucuman/camera_ws/devel/share/face_recognition/msg/FaceRecognitionGoal.msg" NAME_WE)
add_dependencies(face_recognition_generate_messages_py _face_recognition_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tucuman/camera_ws/src/face_recognition/msg/FRClientGoal.msg" NAME_WE)
add_dependencies(face_recognition_generate_messages_py _face_recognition_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(face_recognition_genpy)
add_dependencies(face_recognition_genpy face_recognition_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS face_recognition_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/face_recognition)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/face_recognition
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(face_recognition_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET actionlib_msgs_generate_messages_cpp)
  add_dependencies(face_recognition_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/face_recognition)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/face_recognition
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(face_recognition_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET actionlib_msgs_generate_messages_eus)
  add_dependencies(face_recognition_generate_messages_eus actionlib_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/face_recognition)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/face_recognition
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(face_recognition_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET actionlib_msgs_generate_messages_lisp)
  add_dependencies(face_recognition_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/face_recognition)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/face_recognition
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(face_recognition_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET actionlib_msgs_generate_messages_nodejs)
  add_dependencies(face_recognition_generate_messages_nodejs actionlib_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/face_recognition)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/face_recognition\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/face_recognition
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(face_recognition_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET actionlib_msgs_generate_messages_py)
  add_dependencies(face_recognition_generate_messages_py actionlib_msgs_generate_messages_py)
endif()
