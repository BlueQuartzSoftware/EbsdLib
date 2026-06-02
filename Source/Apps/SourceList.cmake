configure_file(${EbsdLibProj_SOURCE_DIR}/Source/Test/TestFileLocations.h.in
               ${EbsdLibProj_BINARY_DIR}/EbsdLib/Apps/EbsdLibFileLocations.h @ONLY IMMEDIATE)

add_executable(rotconvert ${EbsdLibProj_SOURCE_DIR}/Source/Apps/rotconvert.cpp)
target_link_libraries(rotconvert PUBLIC EbsdLib)
target_include_directories(rotconvert PUBLIC ${EbsdLibProj_SOURCE_DIR}/Source)
set_target_properties(rotconvert PROPERTIES FOLDER EbsdLib_Applications)


add_executable(make_ipf ${EbsdLibProj_SOURCE_DIR}/Source/Apps/make_ipf.cpp)
target_link_libraries(make_ipf PUBLIC EbsdLib)
target_include_directories(make_ipf PUBLIC ${EbsdLibProj_SOURCE_DIR}/Source)
set_target_properties(make_ipf PROPERTIES FOLDER EbsdLib_Applications)

add_executable(convert_orientations ${EbsdLibProj_SOURCE_DIR}/Source/Apps/ConvertOrientations.cpp)
target_link_libraries(convert_orientations PUBLIC EbsdLib)
target_include_directories(convert_orientations PUBLIC ${EbsdLibProj_SOURCE_DIR}/Source)
set_target_properties(convert_orientations PROPERTIES FOLDER EbsdLib_Applications)

add_executable(gen_sym_code ${EbsdLibProj_SOURCE_DIR}/Source/Apps/gen_sym_code.cpp)
target_link_libraries(gen_sym_code PUBLIC EbsdLib)
target_include_directories(gen_sym_code PUBLIC ${EbsdLibProj_SOURCE_DIR}/Source)
set_target_properties(gen_sym_code PROPERTIES FOLDER EbsdLib_Applications)


add_executable(eq_orientations ${EbsdLibProj_SOURCE_DIR}/Source/Apps/eq_orientations.cpp)
target_link_libraries(eq_orientations PUBLIC EbsdLib)
target_include_directories(eq_orientations PUBLIC ${EbsdLibProj_SOURCE_DIR}/Source)
set_target_properties(eq_orientations PROPERTIES FOLDER EbsdLib_Applications)


add_executable(generate_ipf_legends ${EbsdLibProj_SOURCE_DIR}/Source/Apps/generate_ipf_legends.cpp)
target_link_libraries(generate_ipf_legends PUBLIC EbsdLib)
target_include_directories(generate_ipf_legends
    PUBLIC
        ${EbsdLibProj_SOURCE_DIR}/Source
        ${EbsdLibProj_BINARY_DIR})
set_target_properties(generate_ipf_legends PROPERTIES FOLDER EbsdLib_Applications)

add_executable(generate_ipf_density ${EbsdLibProj_SOURCE_DIR}/Source/Apps/generate_ipf_density.cpp)
target_link_libraries(generate_ipf_density PUBLIC EbsdLib)
target_include_directories(generate_ipf_density
    PUBLIC
        ${EbsdLibProj_SOURCE_DIR}/Source
        ${EbsdLibProj_BINARY_DIR})
set_target_properties(generate_ipf_density PROPERTIES FOLDER EbsdLib_Applications)

add_executable(generate_ipf_from_file ${EbsdLibProj_SOURCE_DIR}/Source/Apps/generate_ipf_from_file.cpp)
target_link_libraries(generate_ipf_from_file PUBLIC EbsdLib)
target_include_directories(generate_ipf_from_file PUBLIC ${EbsdLibProj_SOURCE_DIR}/Source)
set_target_properties(generate_ipf_from_file PROPERTIES FOLDER EbsdLib_Applications)

add_executable(generate_pole_figure ${EbsdLibProj_SOURCE_DIR}/Source/Apps/generate_pole_figure.cpp)
target_link_libraries(generate_pole_figure PUBLIC EbsdLib)
target_include_directories(generate_pole_figure PUBLIC ${EbsdLibProj_SOURCE_DIR}/Source)
set_target_properties(generate_pole_figure PROPERTIES FOLDER EbsdLib_Applications)

add_executable(make_pole_figure ${EbsdLibProj_SOURCE_DIR}/Source/Apps/make_pole_figure.cpp)
target_link_libraries(make_pole_figure PUBLIC EbsdLib)
target_include_directories(make_pole_figure PUBLIC ${EbsdLibProj_SOURCE_DIR}/Source)
set_target_properties(make_pole_figure PROPERTIES FOLDER EbsdLib_Applications)

add_executable(render_ebsd
  ${EbsdLibProj_SOURCE_DIR}/Source/Apps/render_ebsd.cpp
  ${EbsdLibProj_SOURCE_DIR}/Source/Apps/render_ebsd.h
  ${EbsdLibProj_SOURCE_DIR}/Source/Apps/render_ebsd_main.cpp)
target_link_libraries(render_ebsd PUBLIC EbsdLib)
target_include_directories(render_ebsd PUBLIC ${EbsdLibProj_SOURCE_DIR}/Source)
set_target_properties(render_ebsd PROPERTIES FOLDER EbsdLib_Applications)

add_executable(ParseAztecProject ${EbsdLibProj_SOURCE_DIR}/Source/Apps/ParseAztecProject.cpp)
target_link_libraries(ParseAztecProject PUBLIC EbsdLib)
target_include_directories(ParseAztecProject PUBLIC ${EbsdLibProj_SOURCE_DIR}/Source)
set_target_properties(ParseAztecProject PROPERTIES FOLDER EbsdLib_Applications)


if(EbsdLib_INSTALL_FILES)
  install(FILES
    "${EbsdLibProj_SOURCE_DIR}/Data/IPF_Legend/Cubic m-3 (Th)/Cubic m-3 (Th).tiff"
    "${EbsdLibProj_SOURCE_DIR}/Data/IPF_Legend/Cubic m-3m (Oh)/Cubic m-3m (Oh).tiff"
    "${EbsdLibProj_SOURCE_DIR}/Data/IPF_Legend/Hexagonal 6_m (C6h)/Hexagonal 6_m (C6h).tiff"
    "${EbsdLibProj_SOURCE_DIR}/Data/IPF_Legend/Hexagonal 6_mmm (D6h)/Hexagonal 6_mmm (D6h).tiff"
    "${EbsdLibProj_SOURCE_DIR}/Data/IPF_Legend/Monoclinic 2_m (C2h)/Monoclinic 2_m (C2h).tiff"
    "${EbsdLibProj_SOURCE_DIR}/Data/IPF_Legend/Orthorhombic mmm (D2h)/Orthorhombic mmm (D2h).tiff"
    "${EbsdLibProj_SOURCE_DIR}/Data/IPF_Legend/Tetragonal 4_m (C4h)/Tetragonal 4_m (C4h).tiff"
    "${EbsdLibProj_SOURCE_DIR}/Data/IPF_Legend/Tetragonal 4_mmm (D4h)//Tetragonal 4_mmm (D4h).tiff"
    "${EbsdLibProj_SOURCE_DIR}/Data/IPF_Legend/Triclinic -1 (Ci)/Triclinic -1 (Ci).tiff"
    "${EbsdLibProj_SOURCE_DIR}/Data/IPF_Legend/Trigonal -3 (C3i)/Trigonal -3 (C3i).tiff"
    "${EbsdLibProj_SOURCE_DIR}/Data/IPF_Legend/Trigonal -3m (D3d)/Trigonal -3m (D3d).tiff"
    DESTINATION share/EbsdLib/Data
    COMPONENT Headers
  )
endif()
