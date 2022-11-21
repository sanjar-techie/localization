execute_process(COMMAND "/home/team6/catkin_ws/build/eurecarr_vehicle_sim/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/team6/catkin_ws/build/eurecarr_vehicle_sim/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
