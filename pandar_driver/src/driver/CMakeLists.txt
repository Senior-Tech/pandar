# build the driver node
add_executable(pandar_driver_node pandar_node.cpp driver.cpp)
add_dependencies(pandar_driver_node pandar_driver_gencfg)
target_link_libraries(pandar_driver_node
        pandar_input
        ${catkin_LIBRARIES}
        ${libpcap_LIBRARIES}
        )

# build the nodelet version
add_library(pandar_driver_nodelet nodelet.cpp driver.cpp)
add_dependencies(pandar_driver_nodelet pandar_driver_gencfg)
target_link_libraries(pandar_driver_nodelet
        pandar_input
        ${catkin_LIBRARIES}
        ${libpcap_LIBRARIES}
        )

install(TARGETS pandar_driver_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        COMPONENT main
        )

install(TARGETS pandar_driver_nodelet
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )


