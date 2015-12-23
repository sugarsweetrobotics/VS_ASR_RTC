set(hdrs VS_ASR_RTC.h
    ManipulatorCommonInterface_CommonSVC_impl.h
    ManipulatorCommonInterface_MiddleLevelSVC_impl.h
    )

install(FILES ${hdrs} DESTINATION ${INC_INSTALL_DIR}/${PROJECT_NAME_LOWER}
    COMPONENT library)

