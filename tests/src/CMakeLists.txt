set(SOURCE_FILES 
	"animationPlugin.cpp"
)

if (APPLE)
    set(MAYA_LIBRARIES ${MAYA_LIBRARIES} ${CMAKE_BINARY_DIR}/src/libAnimX.dylib)
else()
    set(MAYA_LIBRARIES ${MAYA_LIBRARIES} AnimX)
endif()

find_package(Maya REQUIRED)
set(MAYA_INCLUDE_DIR ${MAYA_INCLUDE_DIR} ${MAYA_ANIMATION_INCLUDE})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${MAYA_CXX_FLAGS}")
include_directories(${MAYA_INCLUDE_DIR})
link_directories(${MAYA_LIBRARY_DIR})

include_directories(${LIB_ANIMATION_INCLUDE_DIR})
link_directories(${LIB_ANIMATION_LIBRARY_DIR})

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${MAYA_LIBRARIES})

MAYA_PLUGIN(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} ${MAYA_TARGET_TYPE} DESTINATION .)
install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/../scripts/ DESTINATION scripts)           
