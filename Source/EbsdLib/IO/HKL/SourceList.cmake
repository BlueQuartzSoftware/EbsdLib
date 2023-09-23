#-- Get the HKL Sources
set(HKL_SRCS
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/IO/HKL/CtfReader.cpp
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/IO/HKL/CtfPhase.cpp
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/IO/HKL/CtfFields.cpp
)

set(HKL_HDRS
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/IO/HKL/CtfConstants.h
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/IO/HKL/CtfHeaderEntry.h
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/IO/HKL/CtfReader.h
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/IO/HKL/CtfPhase.h
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/IO/HKL/CtfFields.h
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/IO/HKL/DataParser.hpp
)

if(EbsdLib_ENABLE_HDF5)
  set(HKL_SRCS ${HKL_SRCS}
    ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/IO/HKL/H5CtfImporter.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/IO/HKL/H5CtfReader.cpp
    ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/IO/HKL/H5CtfVolumeReader.cpp
      ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/IO/HKL/H5OINAReader.cpp
  )
  set(HKL_HDRS ${HKL_HDRS}
    ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/IO/HKL/H5CtfImporter.h
    ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/IO/HKL/H5CtfReader.h
    ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/IO/HKL/H5CtfVolumeReader.h
      ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/IO/HKL/H5OINAReader.h
  )
endif()

#cmp_IDE_SOURCE_PROPERTIES("HKL" "${HKL_HDRS}" "${HKL_SRCS}" ${EbsdLib_INSTALL_FILES})

if(EbsdLib_INSTALL_FILES)
  install(FILES ${HKL_HDRS}
    DESTINATION include/EbsdLib/IO/HKL
    COMPONENT Headers
  )
endif()
