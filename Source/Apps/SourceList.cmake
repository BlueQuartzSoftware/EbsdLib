configure_file(${EbsdLibProj_SOURCE_DIR}/Source/Test/TestFileLocations.h.in
               ${EbsdLibProj_BINARY_DIR}/EbsdLib/Apps/EbsdLibFileLocations.h @ONLY IMMEDIATE)

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

add_executable(generate_ipf_legends ${EbsdLibProj_SOURCE_DIR}/Source/Apps/generate_ipf_legends.cpp)
target_link_libraries(generate_ipf_legends PUBLIC EbsdLib)
target_include_directories(generate_ipf_legends 
    PUBLIC 
        ${EbsdLibProj_SOURCE_DIR}/Source 
        ${EbsdLibProj_BINARY_DIR} 
    PRIVATE
        "${EbsdLibProj_SOURCE_DIR}/3rdParty/canvas_ity/src")


if(EbsdLib_INSTALL_FILES)
  install(FILES
    "${EbsdLibProj_SOURCE_DIR}/Data/IPF_Legend/Cubic m-3 (Th)/Cubic m-3 (Th).tiff"
    "${EbsdLibProj_SOURCE_DIR}/Data/IPF_Legend/Cubic m-3m (Oh)/Cubic m-3m (Oh).tiff"
    "${EbsdLibProj_SOURCE_DIR}/Data/IPF_Legend/Hexagonal 6|m (C6h)//Hexagonal 6|m (C6h).tiff"
    "${EbsdLibProj_SOURCE_DIR}/Data/IPF_Legend/Hexagonal 6|mmm (D6h)/Hexagonal 6|mmm (D6h).tiff"
    "${EbsdLibProj_SOURCE_DIR}/Data/IPF_Legend/Monoclinic 2|m (C2h)/Monoclinic 2|m (C2h).tiff"
    "${EbsdLibProj_SOURCE_DIR}/Data/IPF_Legend/Orthorhombic mmm (D2h)/Orthorhombic mmm (D2h).tiff"
    "${EbsdLibProj_SOURCE_DIR}/Data/IPF_Legend/Tetragonal 4|m (C4h)/Tetragonal 4|m (C4h).tiff"
    "${EbsdLibProj_SOURCE_DIR}/Data/IPF_Legend/Tetragonal 4|mmm (D4h)//Tetragonal 4|mmm (D4h).tiff"
    "${EbsdLibProj_SOURCE_DIR}/Data/IPF_Legend/Triclinic -1 (Ci)/Triclinic -1 (Ci).tiff"
    "${EbsdLibProj_SOURCE_DIR}/Data/IPF_Legend/Trigonal -3 (C3i)/Trigonal -3 (C3i).tiff"
    "${EbsdLibProj_SOURCE_DIR}/Data/IPF_Legend/Trigonal -3m (D3d)/Trigonal -3m (D3d).tiff"
    DESTINATION share/EbsdLib/Data
    COMPONENT Headers
  )
endif()
