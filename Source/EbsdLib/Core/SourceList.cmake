
set(DIR_NAME Core )
set(EbsdLib_${DIR_NAME}_HDRS
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/AbstractEbsdFields.h 
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/DataArray.hpp
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/EbsdLibConstants.h
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/EbsdLibDLLExport.h
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/EbsdMacros.h         
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/EbsdSetGetMacros.h
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/EbsdTransform.h
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/QuaternionMath.hpp
  )

set(EbsdLib_${DIR_NAME}_SRCS
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/AbstractEbsdFields.cpp 
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/EbsdTransform.cpp
)


if( ${EbsdLib_INSTALL_FILES} EQUAL 1 )
    INSTALL (FILES ${EbsdLib_${DIR_NAME}_HDRS} ${EbsdLib_${DIR_NAME}_MOC_HDRS}
            DESTINATION include/EbsdLib/${DIR_NAME}
            COMPONENT Headers   )
endif()