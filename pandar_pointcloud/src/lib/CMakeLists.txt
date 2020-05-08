add_library(pandar_rawdata rawdata.cpp calibration.cpp)
target_link_libraries(pandar_rawdata
        ${catkin_LIBRARIES}
        ${YAML_CPP_LIBRARIES})
install(TARGETS pandar_rawdata
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
