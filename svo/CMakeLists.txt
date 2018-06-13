################################################################################
# user build settings

SET(TRACE TRUE)
SET(HAVE_G2O FALSE) # G2O图优化
SET(USE_ROS TRUE)   # ros Set FALSE if you want to build this package without Catkin
SET(DEBUG_OUTPUT FALSE) # Only relevant if build without ROS

################################################################################

SET(PROJECT_NAME svo)# 项目名
PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED (VERSION 2.8.3)# cmake 版本
SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
SET(CMAKE_VERBOSE_MAKEFILE OFF)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")

# Set definitions
IF(TRACE)
  ADD_DEFINITIONS(-DSVO_TRACE)
ENDIF()
IF(HAVE_G2O)
  ADD_DEFINITIONS(-DUSE_BUNDLE_ADJUSTMENT)
ENDIF()
IF(USE_ROS)
  ADD_DEFINITIONS(-DSVO_USE_ROS)
ENDIF()
IF(DEBUG_OUTPUT)
  ADD_DEFINITIONS(-DSVO_DEBUG_OUTPUT)
ENDIF()

# Set build flags, set ARM_ARCHITECTURE environment variable on Odroid
# Set build flags. Set IS_ARM on odroid board as environment variable
SET(CMAKE_CXX_FLAGS "-Wall -D_LINUX -D_REENTRANT -march=native -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")
IF(DEFINED ENV{ARM_ARCHITECTURE})
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon -march=armv7-a")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3")
ENDIF()
IF(CMAKE_COMPILER_IS_GNUCC)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# 依赖 Add plain cmake packages 
FIND_PACKAGE(OpenCV REQUIRED)# opencv 库
FIND_PACKAGE(Eigen REQUIRED)# eigen库
FIND_PACKAGE(Sophus REQUIRED)# 李群李代数库 sophus
FIND_PACKAGE(fast REQUIRED)# fast角点检测库
FIND_PACKAGE(Boost REQUIRED COMPONENTS thread system)# 加速

IF(USE_ROS)
  FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp roslib cmake_modules vikit_common vikit_ros)
  catkin_package(
      DEPENDS Eigen OpenCV Sophus Boost fast
      CATKIN_DEPENDS roscpp roslib vikit_common vikit_ros
      INCLUDE_DIRS include
      LIBRARIES svo
  )
ELSE()
  FIND_PACKAGE(vikit_common REQUIRED)
  SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
ENDIF()

# 包含库文件 Include dirs
INCLUDE_DIRECTORIES(
  include
  ${Eigen_INCLUDE_DIRS}# eigen库
  ${OpenCV_INCLUDE_DIRS}# opencv 库
  ${Sophus_INCLUDE_DIRS}# 李群李代数库 sophus
  ${Boost_INCLUDE_DIRS}# boost 加速
  ${fast_INCLUDE_DIRS}# fast角点检测库
)

# 连接库 Set link libraries
LIST(APPEND LINK_LIBS 
  ${OpenCV_LIBS}
  ${Sophus_LIBRARIES}
  ${Boost_LIBRARIES}
  ${fast_LIBRARIES}
)

IF(USE_ROS)
  INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})
  LIST(APPEND LINK_LIBS ${catkin_LIBRARIES})
ELSE()
  INCLUDE_DIRECTORIES(${vikit_common_INCLUDE_DIRS})
  LIST(APPEND LINK_LIBS ${vikit_common_LIBRARIES})
ENDIF()

# 编译文件
# Set sourcefiles
LIST(APPEND SOURCEFILES
  src/frame_handler_mono.cpp
  src/frame_handler_base.cpp
  src/frame.cpp         # 帧
  src/point.cpp         # 点
  src/map.cpp           # 地图
  src/pose_optimizer.cpp# 位姿优化
  src/initialization.cpp# 初始化
  src/matcher.cpp       # 匹配
  src/reprojector.cpp   # 投影
  src/feature_alignment.cpp# 特征
  src/feature_detection.cpp# 特征检测
  src/depth_filter.cpp     # 深度滤波器
  src/config.cpp           # 配置
  src/sparse_img_align.cpp)# 稀疏图像配准

# Add g2o if available
IF(HAVE_G2O)
  FIND_PACKAGE(G2O REQUIRED)
  INCLUDE_DIRECTORIES(
    ${G2O_INCLUDE_DIR}
    /usr/include/suitesparse # for cholmod 稀疏矩阵求解器
  )
  LIST(APPEND LINK_LIBS
    ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} ${G2O_SOLVER_CHOLMOD} ${G2O_SOLVER_CSPARSE} ${G2O_SOLVER_DENSE}
    ${G2O_SOLVER_PCG} ${G2O_TYPES_SBA} cholmod cxsparse
  )
  LIST(APPEND SOURCEFILES src/bundle_adjustment.cpp)
ENDIF(HAVE_G2O)

# 创建SVO 共享库 Create svo library
ADD_LIBRARY(svo SHARED ${SOURCEFILES})
TARGET_LINK_LIBRARIES(svo ${LINK_LIBS})

################################################################################
# TESTS 测试

ADD_EXECUTABLE(test_feature_align test/test_feature_alignment.cpp)    # 特征配置
TARGET_LINK_LIBRARIES(test_feature_align svo)

ADD_EXECUTABLE(test_pipeline test/test_pipeline.cpp)                  # 测试流水线
TARGET_LINK_LIBRARIES(test_pipeline svo)

ADD_EXECUTABLE(test_matcher test/test_matcher.cpp)                    # 特征匹配
TARGET_LINK_LIBRARIES(test_matcher svo)

ADD_EXECUTABLE(test_feature_detection test/test_feature_detection.cpp)# 特征检测
TARGET_LINK_LIBRARIES(test_feature_detection svo)

ADD_EXECUTABLE(test_depth_filter test/test_depth_filter.cpp)          # 深度滤波
TARGET_LINK_LIBRARIES(test_depth_filter svo)

ADD_EXECUTABLE(test_sparse_img_align test/test_sparse_img_align.cpp)  # 稀疏图
TARGET_LINK_LIBRARIES(test_sparse_img_align svo)

ADD_EXECUTABLE(test_pose_optimizer test/test_pose_optimizer.cpp)      # 位姿优化
TARGET_LINK_LIBRARIES(test_pose_optimizer svo) 
