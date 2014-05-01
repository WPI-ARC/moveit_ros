set(MOVEIT_LIB_NAME moveit_default_planning_request_adapter_plugins)

set(SOURCE_FILES
  src/empty.cpp
  src/fix_start_state_bounds.cpp
  src/fix_start_state_collision.cpp
  src/fix_start_state_path_constraints.cpp
  src/fix_workspace_bounds.cpp
  src/add_time_parameterization.cpp)


#include_directories ("/home/pr2/Desktop/source_code/IPI/include")

#set(reflexxes_LIBRARIES /home/pr2/Desktop/source_code/IPI/Linux/x86/debug/lib/shared/libReflexxesTypeIV.so)

add_library(${MOVEIT_LIB_NAME} ${SOURCE_FILES})
#target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${reflexxes_LIBRARIES})
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} )

#message("QUAN MESSAGE " ${catkin_LIBRARIES})
#message("QUAN MESSAGE " ${reflexxes_LIBRARIES})
#message("QUAN MESSAGE " ${Boost_LIBRARIES})

add_executable(moveit_list_request_adapter_plugins src/list.cpp)
#target_link_libraries(moveit_list_request_adapter_plugins ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${reflexxes_LIBRARIES})
target_link_libraries(moveit_list_request_adapter_plugins ${catkin_LIBRARIES} ${Boost_LIBRARIES} )

install(TARGETS ${MOVEIT_LIB_NAME} moveit_list_request_adapter_plugins
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

