set(BRUKER_NANO_SRCS
  )

set(BRUKER_NANO_HDRS
  )


if(EbsdLib_ENABLE_HDF5)
    set(BRUKER_NANO_SRCS ${BRUKER_NANO_SRCS}
      ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/IO/BrukerNano/EspritPhase.cpp
      ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/IO/BrukerNano/H5EspritReader.cpp
      ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/IO/BrukerNano/H5EspritFields.cpp
    )
    set(BRUKER_NANO_HDRS ${BRUKER_NANO_HDRS}
      ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/IO/BrukerNano/EspritPhase.h
      ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/IO/BrukerNano/H5EspritReader.h
      ${EbsdLibProj_SOURCE_DIR}/Source/EbsdLib/IO/BrukerNano/H5EspritFields.h
    )
endif()


# cmp_IDE_SOURCE_PROPERTIES( "BrukerNano" "${BRUKER_NANO_HDRS}" "${BRUKER_NANO_SRCS}" ${EbsdLib_INSTALL_FILES})

if( ${EbsdLib_INSTALL_FILES} EQUAL 1 )
    INSTALL (FILES ${BRUKER_NANO_HDRS}
            DESTINATION include/EbsdLib/IO/BrukerNano
            COMPONENT Headers   )
endif()


