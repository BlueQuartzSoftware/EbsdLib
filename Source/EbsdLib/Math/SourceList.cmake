set(DIR_NAME Math)

set(EbsdLib_${DIR_NAME}_HDRS
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/EbsdLibMath.h
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/GeometryMath.h
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/ArrayHelpers.hpp
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/EbsdMatrixMath.h
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/EbsdLibRandom.h
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/Matrix3X1.hpp
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/Matrix3X3.hpp  
)

set(EbsdLib_${DIR_NAME}_SRCS
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/EbsdLibMath.cpp
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/GeometryMath.cpp
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/EbsdMatrixMath.cpp
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/EbsdLibRandom.cpp
)

#cmp_IDE_SOURCE_PROPERTIES("LaueOps" "${EbsdLib${DIR_NAME}HDRS}" "${EbsdLib${DIR_NAME}SRCS}" "0")
if(EbsdLib_INSTALL_FILES)
  install(FILES ${EbsdLib_${DIR_NAME}_HDRS}
    DESTINATION include/EbsdLib/Math
    COMPONENT Headers
  )
endif()
