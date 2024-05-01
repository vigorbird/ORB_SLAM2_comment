cmake_minimum_required(VERSION 2.8)
#这里的ORB_SLAM2就是${PROJECT_NAME}
project(ORB_SLAM2)

#设置生成的代码是发行版本的
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

#将变量 CMAKE_C_FLAGS 存放的内容传给 C 编译器，添加C编译选项
#-wall：打开警告开关。-O3：使用高级优化。-march=native：gcc-4.2以上版本支持 -march=native 参数，gcc会自动选择最适合cpu的优化
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
#使用变量CMAKE_CXX_FLAGS添加C++编译选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# Check C++11 or C++0x support
#让我们的cmake包含c++11 特性
include(CheckCXXCompilerFlag)
#检查编译器是否支持某个编译选项，并将结果输出给第二个参数
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   #向c++添加编译选项，-std=c++0x是原始的c++11的版本，-std=c++11是c++11发布之后的版本
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   #向C++在编译时添加宏定义
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

#添加${PROJECT_SOURCE_DIR}/cmake_modules元素到CMAKE_MODULE_PATH列表中
#一般来说，都是这样用 cmake ./这样当前目录和PROJECT_SOURCE_DIR是等价的。也就是当前源码的目录。
#如果执行cmake的时候，并不在源码的路径的话，比如cmake ../src。此时PROJECT_SOURCE_DIR目录就是../src
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

#首先在CMAKE_MODULE_PATH中寻找Find<name>.cmake这个文件然后再在其他的~/.cmake/packages/或/usr/local/share/中的各个包目录中查找。
#如果找到则执行这个文件，这个文件的任务就是寻找和<name>相关的库文件和头文件，更新如下的环境变量：
#<NAME>_INCLUDE_DIRS or <NAME>_INCLUDES   ;    <NAME>_LIBRARIES or <NAME>_LIBRARIES or <NAME>_LIBS
#并将是否找到这个结果赋值给变量<name>_FOUND
#此处是寻找opencv3.0模块或者opencv2.4.3模块
#QUIET选项表示如果包没有被发现不输出警告信息
find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

#REQUIRED选项表示如果报没有找到的话，cmake的过程会终止，并输出警告信息
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

#添加头文件，注意这里作者没有添加opencv3的头文件
include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
)

#CMAKE_LIBRARY_OUTPUT_DIRECTORY表示库文件的输出路径
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

#将源文件加入到lib${PROJECT_NAME}.so库中，此处加入的库就是libORB_SLAM2.so的链接库。这个库由如下的源文件组成
#SHARED表示动态链接库，STATIC表示静态库
add_library(${PROJECT_NAME} SHARED
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/PnPsolver.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
src/Viewer.cc
)

#将其他的库文件链接到这个目标库文件中，此处目标库文件的就是libORB_SLAM2.so
#注意此处的libDBoW2.so和libg2o.so都是其他的编译文件生成的库文件
target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
)

# Build examples

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)
#生成一个名为rgbd_tum的可执行文件，需要rgbd_tum.cc源文件。
add_executable(rgbd_tum
Examples/RGB-D/rgbd_tum.cc)
#生成一个名为rgbd_tum的可执行文件需要名为${PROJECT_NAME}的链接库
target_link_libraries(rgbd_tum ${PROJECT_NAME})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

add_executable(stereo_kitti
Examples/Stereo/stereo_kitti.cc)
target_link_libraries(stereo_kitti ${PROJECT_NAME})

add_executable(stereo_euroc
Examples/Stereo/stereo_euroc.cc)
target_link_libraries(stereo_euroc ${PROJECT_NAME})


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

add_executable(mono_tum
Examples/Monocular/mono_tum.cc)
target_link_libraries(mono_tum ${PROJECT_NAME})

add_executable(mono_kitti
Examples/Monocular/mono_kitti.cc)
target_link_libraries(mono_kitti ${PROJECT_NAME})

add_executable(mono_euroc
Examples/Monocular/mono_euroc.cc)
target_link_libraries(mono_euroc ${PROJECT_NAME})

