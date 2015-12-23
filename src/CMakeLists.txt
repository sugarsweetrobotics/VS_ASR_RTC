set(comp_srcs VS_ASR_RTC.cpp ManipulatorCommonInterface_CommonSVC_impl.cpp ManipulatorCommonInterface_MiddleLevelSVC_impl.cpp useSilabs.cpp )
set(standalone_srcs ${comp_srcs} VS_ASR_RTCComp.cpp)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME})
include_directories(${PROJECT_BINARY_DIR})
include_directories(${PROJECT_BINARY_DIR}/idl)
include_directories(${OPENRTM_INCLUDE_DIRS})
include_directories(${OMNIORB_INCLUDE_DIRS})
include_directories(${CP2110_INCLUDE_DIR})
add_definitions(${OPENRTM_CFLAGS})
add_definitions(${OMNIORB_CFLAGS})

link_directories(${OPENRTM_LIBRARY_DIRS})
link_directories(${OMNIORB_LIBRARY_DIRS})
link_directories(${CP2110_LIBRARY_DIRS})

add_library(${PROJECT_NAME} ${LIB_TYPE} ${comp_srcs} ${ALL_IDL_SRCS})
set_source_files_properties(${ALL_IDL_SRCS} PROPERTIES GENERATED 1)
add_dependencies(${PROJECT_NAME} ALL_IDL_TGT)
target_link_libraries(${PROJECT_NAME} ${OPENRTM_LIBRARIES} ${CP2110_LIBRARY})

add_executable(${PROJECT_NAME}Comp ${standalone_srcs} ${ALL_IDL_SRCS})
# set_source_files_properties(${ALL_IDL_SRCS} PROPERTIES GENERATED 1)
add_dependencies(${PROJECT_NAME}Comp ALL_IDL_TGT)
target_link_libraries(${PROJECT_NAME}Comp ${OPENRTM_LIBRARIES} ${CP2110_LIBRARY})

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}Comp
    EXPORT ${PROJECT_NAME}
    RUNTIME DESTINATION ${BIN_INSTALL_DIR} COMPONENT component
    LIBRARY DESTINATION ${LIB_INSTALL_DIR} COMPONENT component
    ARCHIVE DESTINATION ${LIB_INSTALL_DIR} COMPONENT component)
install(EXPORT ${PROJECT_NAME}
    DESTINATION ${LIB_INSTALL_DIR}/${PROJECT_NAME_LOWER}
    FILE ${PROJECT_NAME}Depends.cmake)

