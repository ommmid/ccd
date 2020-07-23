# Generate a list of files and store them in variables CCD_SOURCE_CODE
# and CCD_HEADERS for source and headers
file(GLOB_RECURSE CCD_SOURCE_CODE robot_model/*.cpp robot_state/*.cpp util/*.cpp base/*.cpp)
file(GLOB_RECURSE CCD_HEADERS robot_model/*.h robot_state/*.h util/*.h base/*.h)

# make ccd library here
add_library(ccd SHARED ${CCD_SOURCE_CODE})
# target_link_libraries(ccd Eigen3::Eigen)

# install the library
install(TARGETS ccd 
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)


