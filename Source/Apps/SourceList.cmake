add_executable(rotconvert ${EbsdLibProj_SOURCE_DIR}/Source/Apps/rotconvert.cpp)
target_link_libraries(rotconvert PUBLIC EbsdLib)
target_include_directories(rotconvert PUBLIC ${EbsdLibProj_SOURCE_DIR}/Source)

add_executable(make_ipf ${EbsdLibProj_SOURCE_DIR}/Source/Apps/make_ipf.cpp)
target_link_libraries(make_ipf PUBLIC EbsdLib)
target_include_directories(make_ipf PUBLIC ${EbsdLibProj_SOURCE_DIR}/Source)

add_executable(convert_orientations ${EbsdLibProj_SOURCE_DIR}/Source/Apps/ConvertOrientations.cpp)
target_link_libraries(convert_orientations PUBLIC EbsdLib)
target_include_directories(convert_orientations PUBLIC ${EbsdLibProj_SOURCE_DIR}/Source)

add_executable(gen_sym_code ${EbsdLibProj_SOURCE_DIR}/Source/Apps/gen_sym_code.cpp)
target_link_libraries(gen_sym_code PUBLIC EbsdLib)
target_include_directories(gen_sym_code PUBLIC ${EbsdLibProj_SOURCE_DIR}/Source)
