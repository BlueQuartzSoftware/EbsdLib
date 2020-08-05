set(DIR_NAME IO)

set(EbsdLib_${DIR_NAME}_HDRS
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/EbsdReader.h         
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/EbsdImporter.h       
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/EbsdHeaderEntry.h    
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/AngleFileLoader.h
)

set(EbsdLib_${DIR_NAME}_SRCS
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/EbsdReader.cpp
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/AngleFileLoader.cpp
)

if(EbsdLib_ENABLE_HDF5)
  set(EbsdLib_${DIR_NAME}_HDRS
    ${EbsdLib_${DIR_NAME}_HDRS}
    ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/H5EbsdVolumeReader.h
    ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/H5EbsdVolumeInfo.h
  )
  set(EbsdLib_${DIR_NAME}_SRCS
    ${EbsdLib_${DIR_NAME}_SRCS}
    ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/H5EbsdVolumeInfo.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/H5EbsdVolumeReader.cpp
  )
endif()

#cmp_IDE_SOURCE_PROPERTIES("IO" "${EbsdLib_IO_HDRS}" "${EbsdLib_IO_SRCS}" "0")
if(EbsdLib_INSTALL_FILES)
  install(FILES ${EbsdLib_${DIR_NAME}_HDRS}
    DESTINATION include/EbsdLib/${DIR_NAME}
    COMPONENT Headers
  )
endif()

include(${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/BrukerNano/SourceList.cmake)
include(${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/HKL/SourceList.cmake)
include(${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/TSL/SourceList.cmake)

set(EbsdLib_${DIR_NAME}_HDRS 
  ${EbsdLib_${DIR_NAME}_HDRS}
  ${BRUKER_NANO_HDRS}
  ${HKL_HDRS}
  ${TSL_HDRS}
)

set(EbsdLib_${DIR_NAME}_SRCS
  ${EbsdLib_${DIR_NAME}_SRCS}
  ${BRUKER_NANO_SRCS}
  ${HKL_SRCS}
  ${TSL_SRCS}
)
