execute_process(COMMAND "/fetch_ws/build/joint_state_reader/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/fetch_ws/build/joint_state_reader/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
