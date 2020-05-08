add_executable(pandar_cloud_node pointcloudXYZIR.cpp convert.cpp cloud_node.cpp cloud_nodelet.cpp)

add_dependencies(pandar_cloud_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(pandar_cloud_node pandar_rawdata
        ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
install(TARGETS pandar_cloud_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_library(pandar_cloud_nodelet pointcloudXYZIR.cpp convert.cpp cloud_node.cpp cloud_nodelet.cpp)
add_dependencies(pandar_cloud_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(pandar_cloud_nodelet pandar_rawdata
        ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
install(TARGETS pandar_cloud_nodelet
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

