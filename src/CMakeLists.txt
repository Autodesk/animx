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
add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})

install(TARGETS ${PROJECT_NAME}
	ARCHIVE DESTINATION .
	LIBRARY DESTINATION .
	RUNTIME DESTINATION .)