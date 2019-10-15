set(DIR_NAME OrientationMath)

set(EbsdLib_${DIR_NAME}_HDRS
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/OrientationConverter.hpp
)

set(EbsdLib_${DIR_NAME}_SRCS
)

#cmp_IDE_SOURCE_PROPERTIES( "OrientationMath" "${EbsdLib_OrientationMath_HDRS}" "${EbsdLib_OrientationMath_SRCS}" "0")

if( ${EbsdLib_INSTALL_FILES} EQUAL 1 )
    INSTALL (FILES ${EbsdLib_${DIR_NAME}_HDRS}
            DESTINATION include/EbsdLib/OrientationMath
            COMPONENT Headers   )
endif()
