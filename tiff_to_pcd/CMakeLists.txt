cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(tiff_to_pcd)



find_package(Boost REQUIRED)
find_package( OpenCV REQUIRED )

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories("${PROJECT_SOURCE_DIR}/include")

add_executable (tiff_to_pcd 
	"${PROJECT_SOURCE_DIR}/source/tiff_to_pcd.cpp"
	
	)

target_link_libraries (tiff_to_pcd ${OpenCV_LIBS} ${PCL_LIBRARIES})
