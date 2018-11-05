set(SOURCE_FILES
	"animx.cpp"
	"animx.h"
	"internal/nurbs.h"
	"internal/interpolators.h"
	"internal/math.h"
	"internal/T4DblMatrix.h"
	"internal/T4DblVector.h"
	"internal/Tbezier.h"
	"internal/Tquaternion.h"
	"internal/TrotateXYZ.h"
)

if (APPLE AND ${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -stdlib=libstdc++")
endif()

set(MAYA_ANIMATION_INCLUDE ${PROJECT_SOURCE_DIR}/src CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE)
if (ANIMX_BUILD_SHARED)
    add_library(${PROJECT_NAME}_shared SHARED ${SOURCE_FILES})
    set_target_properties(${PROJECT_NAME}_shared PROPERTIES OUTPUT_NAME ${PROJECT_NAME})
    set_target_properties(${PROJECT_NAME}_shared PROPERTIES COMPILE_DEFINITIONS ANIMX_EXPORT)
endif()
if (ANIMX_BUILD_STATIC)
    add_library(${PROJECT_NAME}_static STATIC ${SOURCE_FILES})
    set_target_properties(${PROJECT_NAME}_static PROPERTIES OUTPUT_NAME ${PROJECT_NAME})
    if (MSVC)
        set_target_properties(${PROJECT_NAME}_static PROPERTIES PREFIX lib)
    endif()
    set_target_properties(${PROJECT_NAME}_static PROPERTIES COMPILE_DEFINITIONS ANIMX_STATIC)
endif()

install(FILES animx.h DESTINATION include)

if (ANIMX_BUILD_SHARED)
    install(TARGETS ${PROJECT_NAME}_shared
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION .
        RUNTIME DESTINATION bin)
endif()
if (ANIMX_BUILD_STATIC)
    install(TARGETS ${PROJECT_NAME}_static
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION .
        RUNTIME DESTINATION bin)
endif()
