set(DIR_NAME Texture)

set(EbsdLib_${DIR_NAME}_HDRS
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/TexturePreset.h
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/Texture.hpp
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/StatsGen.hpp
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/SO3DeLaValleePoussinKernel.h
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/RandomAngleDistribution.h
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/MisorientationKDE.h
)

set(EbsdLib_${DIR_NAME}_SRCS
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/TexturePreset.cpp
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/SO3DeLaValleePoussinKernel.cpp
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/RandomAngleDistribution.cpp
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/MisorientationKDE.cpp
)

#cmp_IDE_SOURCE_PROPERTIES("Common" "${EbsdLib_Texture_HDRS}" "${EbsdLib_Texture_SRCS}" "0")

if(EbsdLib_INSTALL_FILES)
  install(FILES ${EbsdLib_Texture_HDRS}
    DESTINATION include/EbsdLib/Texture
    COMPONENT Headers
  )
endif()
