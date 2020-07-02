


add_executable(rotconvert ${EbsdLibProj_SOURCE_DIR}/Source/Apps/rotconvert.cpp)
target_link_libraries(rotconvert EbsdLib)
target_include_directories(rotconvert PUBLIC ${EbsdLibProj_SOURCE_DIR}/Source)
