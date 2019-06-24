
set(DIR_NAME Texture)

set(EbsdLib_${DIR_NAME}_HDRS
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/TexturePreset.h
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/Texture.hpp
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/StatsGen.hpp
)

set(EbsdLib_${DIR_NAME}_SRCS
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/TexturePreset.cpp
)

#cmp_IDE_SOURCE_PROPERTIES( "Common" "${EbsdLib_Texture_HDRS}" "${EbsdLib_Texture_SRCS}" "0")


if( ${EbsdLib_INSTALL_FILES} EQUAL 1 )
    INSTALL (FILES ${EbsdLib_Texture_HDRS}
            DESTINATION include/EbsdLib/Texture
            COMPONENT Headers   )
endif()

