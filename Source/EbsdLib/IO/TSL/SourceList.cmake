#-- Get the TSL Sources
set(TSL_SRCS
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/IO/TSL/AngReader.cpp
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/IO/TSL/AngPhase.cpp
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/IO/TSL/AngFields.cpp
)

set(TSL_HDRS
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/IO/TSL/AngConstants.h
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/IO/TSL/AngHeaderEntry.h
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/IO/TSL/AngReader.h
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/IO/TSL/AngPhase.h
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/IO/TSL/AngFields.h
)

if(EbsdLib_ENABLE_HDF5)
  set(TSL_SRCS ${TSL_SRCS}
    ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/IO/TSL/H5AngImporter.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/IO/TSL/H5AngReader.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/IO/TSL/H5AngVolumeReader.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/IO/TSL/H5OIMReader.cpp
  )
  set(TSL_HDRS ${TSL_HDRS}
    ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/IO/TSL/H5AngImporter.h
    ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/IO/TSL/H5AngReader.h
    ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/IO/TSL/H5AngVolumeReader.h
    ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/IO/TSL/H5OIMReader.h
  )
endif()

#cmp_IDE_SOURCE_PROPERTIES("TSL" "${TSL_HDRS}" "${TSL_SRCS}" ${EbsdLib_INSTALL_FILES})

if(EbsdLib_INSTALL_FILES)
  install(FILES ${TSL_HDRS}
    DESTINATION include/EbsdLib/IO/TSL
    COMPONENT Headers
  )
endif()
