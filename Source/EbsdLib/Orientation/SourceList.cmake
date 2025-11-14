set(DIR_NAME Orientation)

set(EbsdLib_${DIR_NAME}_HDRS
    ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/OrientationFwd.hpp
    ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/Euler.hpp
    ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/OrientationMatrix.hpp
    ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/AxisAngle.hpp
    ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/Rodrigues.hpp
    ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/Quaternion.hpp
    ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/Homochoric.hpp
    ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/Cubochoric.hpp
    ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/Stereographic.hpp
)

set(EbsdLib_${DIR_NAME}_SRCS

)

#cmp_IDE_SOURCE_PROPERTIES( "OrientationMath" "${EbsdLib_OrientationMath_HDRS}" "${EbsdLib_OrientationMath_SRCS}" "0")

if(EbsdLib_INSTALL_FILES)
    install(FILES ${EbsdLib_${DIR_NAME}_HDRS}
            DESTINATION include/EbsdLib/Orientation
            COMPONENT Headers
    )
endif()
