set(DIR_NAME Utilities)

set(EbsdLib_${DIR_NAME}_MOC_HDRS
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/PoleFigureData.h
)

set(EbsdLib_${DIR_NAME}_HDRS
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/PoleFigureUtilities.h
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/ModifiedLambertProjection.h
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/ModifiedLambertProjectionArray.h
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/ModifiedLambertProjection3D.hpp
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/ComputeStereographicProjection.h
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/LambertUtilities.h
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/ColorTable.h
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/ColorUtilities.h
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/EbsdStringUtils.hpp
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/ToolTipGenerator.h
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/TiffWriter.h
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/FiraSansRegular.hpp
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/Fonts.hpp
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/LatoBold.hpp
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/LatoRegular.hpp
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/CanvasUtilities.hpp
)

set(EbsdLib_${DIR_NAME}_SRCS
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/PoleFigureUtilities.cpp
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/ModifiedLambertProjection.cpp
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/ModifiedLambertProjectionArray.cpp
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/PoleFigureData.cpp
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/ComputeStereographicProjection.cpp
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/LambertUtilities.cpp
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/ColorTable.cpp
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/ColorUtilities.cpp
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/ToolTipGenerator.cpp
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/TiffWriter.cpp
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/CanvasUtilities.cpp
  ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/${DIR_NAME}/Fonts.cpp
)
# # QT5_WRAP_CPP( EbsdLib_Generated_MOC_SRCS ${EbsdLib_Utilities_MOC_HDRS} )
# set_source_files_properties( ${EbsdLib_Generated_MOC_SRCS} PROPERTIES HEADER_FILE_ONLY TRUE)
# set_source_files_properties( ${EbsdLib_Generated_MOC_SRCS} PROPERTIES GENERATED TRUE)

#cmp_IDE_SOURCE_PROPERTIES( "Utilities" "${EbsdLib_Utilities_HDRS}" "${EbsdLib_Utilities_SRCS}" "0")

if(EbsdLib_INSTALL_FILES)
  install(FILES ${EbsdLib_${DIR_NAME}_HDRS} ${EbsdLib_${DIR_NAME}_MOC_HDRS}
    DESTINATION include/EbsdLib/${DIR_NAME}
    COMPONENT Headers
  )
endif()

set(EbsdLib_Utilities_HDRS ${EbsdLib_Utilities_HDRS} ${EbsdLib_${DIR_NAME}_MOC_HDRS})
set(EbsdLib_Utilities_SRCS ${EbsdLib_Utilities_SRCS} ${EbsdLib_Generated_MOC_SRCS})
