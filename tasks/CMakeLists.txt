# Generated from orogen/lib/orogen/templates/tasks/CMakeLists.txt

include(ctrl_libTaskLib)
ADD_LIBRARY(${CTRL_LIB_TASKLIB_NAME} SHARED 
    ${CTRL_LIB_TASKLIB_SOURCES})
add_dependencies(${CTRL_LIB_TASKLIB_NAME}
    regen-typekit)



TARGET_LINK_LIBRARIES(${CTRL_LIB_TASKLIB_NAME}
    ${OrocosRTT_LIBRARIES}
    ${QT_LIBRARIES}
    ${CTRL_LIB_TASKLIB_DEPENDENT_LIBRARIES})
SET_TARGET_PROPERTIES(${CTRL_LIB_TASKLIB_NAME}
    PROPERTIES LINK_INTERFACE_LIBRARIES "${CTRL_LIB_TASKLIB_INTERFACE_LIBRARIES}")
SET_TARGET_PROPERTIES(${CTRL_LIB_TASKLIB_NAME}
    PROPERTIES INTERFACE_LINK_LIBRARIES "${CTRL_LIB_TASKLIB_INTERFACE_LIBRARIES}")

INSTALL(TARGETS ${CTRL_LIB_TASKLIB_NAME}
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib/orocos)

INSTALL(FILES ${CTRL_LIB_TASKLIB_HEADERS}
    DESTINATION include/orocos/ctrl_lib)

