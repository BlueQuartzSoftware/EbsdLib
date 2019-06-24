
set(DIR_NAME LaueOps)

set(EbsdLib_${DIR_NAME}_HDRS
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/LaueOps.h
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/CubicOps.h
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/CubicLowOps.h
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/HexagonalOps.h
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/HexagonalLowOps.h
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/OrthoRhombicOps.h
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/TrigonalOps.h
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/TrigonalLowOps.h
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/TetragonalOps.h
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/TetragonalLowOps.h
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/TriclinicOps.h
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/MonoclinicOps.h
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/SO3Sampler.h
)
set(EbsdLib_${DIR_NAME}_SRCS
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/LaueOps.cpp
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/CubicOps.cpp
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/CubicLowOps.cpp
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/HexagonalOps.cpp
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/HexagonalLowOps.cpp
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/OrthoRhombicOps.cpp
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/TrigonalOps.cpp
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/TrigonalLowOps.cpp
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/TetragonalOps.cpp
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/TetragonalLowOps.cpp
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/TriclinicOps.cpp
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/MonoclinicOps.cpp
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/SO3Sampler.cpp
)


#cmp_IDE_SOURCE_PROPERTIES( "LaueOps" "${EbsdLib${DIR_NAME}HDRS}" "${EbsdLib${DIR_NAME}SRCS}" "0")
if( ${EbsdLib_INSTALL_FILES} EQUAL 1 )
    INSTALL (FILES ${EbsdLib_${DIR_NAME}_HDRS}
            DESTINATION include/EbsdLib/LaueOps
            COMPONENT Headers   )
endif()
