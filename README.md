### 1.1为什么叫半直接法？
    我们知道，VSLAM有直接法和特征点法两大类。直接法和特征点法，在帧间VO阶段的不同在于，

### 直接法：提取梯度纹理特征明显的像素，帧间VO是靠图像对齐，即通过

     最小化像素灰度差函数来优化帧间位姿。

### 特征点法：提取特征点（通常是角点），帧间VO靠PNP，即缩小在后帧图像上，

    重投影点与特征匹配点距离误差，来优化帧间位姿。

### 而SVO是这样干的：

    提取稀疏特征点（类似特征点法），帧间VO用图像对齐（类似于直接法），
    SVO结合了直接法和特征点法，因此，称它为半直接法。


### SVO主要干了两件事，

    <1>跟踪
    <2>深度滤波
    深度滤波是我们常说的建图（Mapping）部分。

### 1.2.1跟踪部分
    跟踪部分干的事情是：初始化位姿，估计和优化位姿(分别对应于帧间VO和局部地图优化)。

### 初始化位姿：

    用KLT光流法找匹配，然后恢复H矩阵。初始化思想是这样的，
    第一帧上提取的特征点，作为关键帧，后来的帧不断用KLT与第一帧匹配，
    直到匹配到的特征点平均视差比较大，就认为初始化成功，计算对应特征点的深度，
    与此对应的帧作为第二帧。之后进入正常工作模式，即估计和优化位姿。

### 正常工作模式：

    首先，通过和上一帧进行对齐，求取初始位姿；
    然后，建立局部地图，通过优化局部地图和当前帧的投影灰度块误差，来优化当前位姿；
    最后，判断此帧是否是关键帧，如果为关键帧就提取新的特征点。
    经过以上四个步骤，完成一帧的处理。如果在CMakeLists里打开了HAVE_G2O的选项，
    代码隔一段时间还会BA,不过非常慢。
    
### 1.2.2深度滤波部分
    深度滤波部分主要任务是完成估计特征点的深度信息。
    深度滤波和跟踪部分相互依赖，因为深度滤波是以相机位姿已知为前提进行的，
    而跟踪部分又依靠深度滤波的结果（较准确的三维点），完成位姿的估计。
    乍一看，这是个鸡生蛋，蛋生鸡的问题，既然两者循环迭代，总得有个起始点。
    其实，单目的slam在启动时需要初始化，而这个初始化就提供粗糙的帧间位姿，
    以便于深度滤波和跟踪部分的迭代。
    当深度可以用后（称之为收敛），把它放入地图里，用于跟踪。

### 1.2.3为什么叫VO
    这个得从SVO干的事来说，它既没有闭环检测，也没有重定位（SVO的重定位太。。。），
    它干的事只要是定位,比较符合视觉里程计(VO)的特点。
    ORBSLAM算是目前性能最好的开源算法，这些功能它都有，因此算一个比较完整的VSLAM算法。

### 1.2.4 svo怎么样
    优点：是比较快，如果你跑代码的时候发现很容易跟丢，可以修改这几个配置参数：
    quality_min_fts：匹配到的最小特征点。
    quality_max_drop_fts：容许相邻帧匹配到特征点的最大落差。 

### 缺点：缺点是比较明显的
    和ORBSLAM相比。
    <1>由于位姿的估计和优化全是靠灰度匹配，这就导致了系统对光照的鲁棒性不足。
    <2>对快速运动鲁棒性性不足。直接法都似这个样子。。可以加入IMU改善。
    <3>没有重定位和闭环检测功能。
    如果把此工程修改成RGBD的模式后，鲁棒性和精度明显提高，近似于ORBSLAM的RGBD模式。


# 结构 
    rpg_svo
    ├── rqt_svo       为与 显示界面 有关的功能插件
    ├── svo           主程序文件，编译 svo_ros 时需要
    │   ├── include
    │   │   └── svo
    │   │       ├── bundle_adjustment.h        光束法平差（图优化）
    │   │       ├── config.h                   SVO的全局配置
    │   │       ├── depth_filter.h             像素深度估计（基于概率） 高斯均值混合模型
    │   │       ├── feature_alignment.h        特征匹配
    │   │       ├── feature_detection.h        特征检测  faster角点
    │   │       ├── feature.h（无对应cpp）      特征定义
    │   │       ├── frame.h                    frame定义
    │   │       ├── frame_handler_base.h       视觉前端基础类
    │   │       ├── frame_handler_mono.h       单目视觉前端原理(较重要)==============================
    │   │       ├── global.h（无对应cpp）       有关全局的一些配置
    │   │       ├── initialization.h           单目初始化
    │   │       ├── map.h                      地图的生成与管理
    │   │       ├── matcher.h                  重投影匹配与极线搜索
    │   │       ├── point.h                    3D点的定义
    │   │       ├── pose_optimizer.h           图优化（光束法平差最优化重投影误差）
    │   │       ├── reprojector.h              重投影
    │   │       └── sparse_img_align.h         直接法优化位姿（最小化光度误差）
    │   ├── src
    │   │   ├── bundle_adjustment.cpp
    │   │   ├── config.cpp
    │   │   ├── depth_filter.cpp
    │   │   ├── feature_alignment.cpp
    │   │   ├── feature_detection.cpp
    │   │   ├── frame.cpp
    │   │   ├── frame_handler_base.cpp
    │   │   ├── frame_handler_mono.cpp
    │   │   ├── initialization.cpp
    │   │   ├── map.cpp
    │   │   ├── matcher.cpp
    │   │   ├── point.cpp
    │   │   ├── pose_optimizer.cpp
    │   │   ├── reprojector.cpp
    │   │   └── sparse_img_align.cpp
    ├── svo_analysis           未知
    ├── svo_msgs               一些配置文件，编译 svo_ros 时需要
    └── svo_ros                为与ros有关的程序，包括 launch 文件
         ├── CMakeLists.txt    定义ROS节点并指导rpg_svo的编译
    ├── include
    │   └── svo_ros
    │    └── visualizer.h                
    ├── launch
    │   └── test_rig3.launch   ROS启动文件
    ├── package.xml
    ├── param                   摄像头等一些配置文件
    ├── rviz_config.rviz        Rviz配置文件（启动Rviz时调用）
    └── src
             ├── benchmark_node.cpp
             ├── visualizer.cpp        地图可视化
             └── vo_node.cpp           VO主节点=======================================
             
    ==============frame_handler_mono.cpp  ============================================       
    processFirstFrame();// 作用是处理第1帧并将其设置为关键帧；
    processSecondFrame();// 作用是处理第1帧后面所有帧，直至找到一个新的关键帧；
    processFrame();// 作用是处理两个关键帧之后的所有帧；
    relocalizeFrame(SE3(Matrix3d::Identity(),Vector3d::Zero()),map_.getClosestKeyframe(last_frame_));
    //作用是在相关位置重定位帧以提供关键帧

    这4个函数被主函数文件vo_node.cpp中的addImage函数调用，他们与addImage一样都定义在frame_handler_mono.cpp中。

# ==============frame_handler_mono.cpp  ============================================       
    processFirstFrame(); // 作用是处理第1帧并将其设置为关键帧；
    processSecondFrame();// 作用是处理第1帧后面所有帧，直至找到一个新的关键帧；
    processFrame();      // 作用是处理两个关键帧之后的所有帧；
    relocalizeFrame(=);  //作用是在相关位置重定位帧以提供关键帧

    startFrameProcessingCommon(timestamp) 设置处理第一帧  stage_ = STAGE_FIRST_FRAME;

    processFirstFrame(); 处理第一帧，直到找打第一个关键帧==============================================
                    1. fast角点特征，单应变换求取位姿变换，特点点数超过100个点才设置第一帧为为关键帧，
                    2. 计算单应性矩阵（根据前两个关键帧）来初始化位姿，3d点
                    3. 且设置系统处理标志为第二帧 stage_ = STAGE_SECOND_FRAME

    processSecondFrame(); 处理第一帧关键帧到第二帧关键帧之间的所有帧(跟踪上一帧) ========================
                    1. 光流法 金字塔多尺度 跟踪关键点,根据跟踪的数量和阈值，做跟踪成功/失败的判断
                    2. 前后两帧计算单应性矩阵，根据重投影误差记录内点数量，根据阈值，判断跟踪 成功/失败,计算3d点
                    3. 集束调整优化, 非线性最小二乘优化, 
                    4. 计算场景深度均值和最小值
                    5. 深度滤波器对深度进行滤波，高斯均值混合模型进行更新深度值
                    6. 设置系统状态 stage_= STAGE_DEFAULT_FRAME；

    processFrame(); 处理前两个关键帧后的所有帧(跟踪上一帧) ============================================
                    1. 设置初始位姿, 上帧的位姿初始化为当前帧的位姿
                    2. 直接法 最小化 3d-2d 重投影 像素误差 求解位姿 LM算法优化位姿)
                    3. 最小化 3d-2d 重投影像素误差 优化特征块的预测位置 更新3d-2d特征块匹配关系
                    4. 使用优化后的 3d-2d特征块匹配关系， 类似直接法，最小化2d位置误差，来优化相机位姿(pose)
                    5. 使用优化后的 3d-2d特征块匹配关系， 类似直接法，最小化2d位置误差，来优化3D点(structure)
                    6. 深度值滤波

    relocalizeFrame(); 重定位模式（跟踪参考帧）=======================================================
                    1. 得到上一帧附近的关键帧作为参考帧 ref_keyframe
                    2. 进行直接法 最小化 3d-2d 重投影 像素误差 求解当前帧的位姿
                    3. 若跟踪质量较高(匹配特征点数大于30) 将参考帧设置为上一帧
                    4. 直接法跟踪上一帧参考帧进行processFrame()处理( 其实就是跟踪参考帧模式 )
                    5. 如果跟踪成功就设置为相应的跟踪后的位姿，否者设置为参考帧的位姿



SVO
===

This code implements a semi-direct monocular visual odometry pipeline.

Video: http://youtu.be/2YnIMfw6bJY

Paper: http://rpg.ifi.uzh.ch/docs/ICRA14_Forster.pdf

#### Disclaimer

SVO has been tested under ROS Groovy, Hydro and Indigo with Ubuntu 12.04, 13.04 and 14.04. This is research code, any fitness for a particular purpose is disclaimed.


#### Licence

The source code is released under a GPLv3 licence. A closed-source professional edition is available for commercial purposes. In this case, please contact the authors for further info.


#### Citing

If you use SVO in an academic context, please cite the following publication:

    @inproceedings{Forster2014ICRA,
      author = {Forster, Christian and Pizzoli, Matia and Scaramuzza, Davide},
      title = {{SVO}: Fast Semi-Direct Monocular Visual Odometry},
      booktitle = {IEEE International Conference on Robotics and Automation (ICRA)},
      year = {2014}
    }
    
    
#### Documentation

The API is documented here: http://uzh-rpg.github.io/rpg_svo/doc/

#### Instructions

See the Wiki for more instructions. https://github.com/uzh-rpg/rpg_svo/wiki

#### Contributing

You are very welcome to contribute to SVO by opening a pull request via Github.
I try to follow the ROS C++ style guide http://wiki.ros.org/CppStyleGuide

# 安装
    SVO需要建立两个工作空间workspace，
    一个用来放cmake工程，
    　　包括Sopuhus（一个李群库），
    　　Fast（特征点检测），还有可选的库g2o（图优化），
    另一个工作空间放ROS-Catkin工程
    　　rpg_vikit（vision kit）和
      　rpg_svo（svo工程文件）。
    保证clone到正确的目录里。

## 1. Sophus李群库
**Sophus实现李群代数，用来描述刚体运动变换的,安装步骤如下，一般不会出错。

    cd workspace
    git clone https://github.com/strasdat/Sophus.git
    cd Sophus
    git checkout a621ff
    mkdir build
    cd build
    cmake ..
    make

**make后不需要install，cmake ..会将包地址写到～/.cmake/packages/目录下，cmake可以找到这里

## 2. Fast 角点检测库

**用来检测角点的，基本不会错，步骤如下：

    cd workspace
    git clone https://github.com/uzh-rpg/fast.git
    cd fast
    mkdir build
    cd build
    cmake ..
    make
## 3. g2o - General Graph Optimization 图优化（可选）
    在需要运行bundle adjustment时要安装，
    在做visual odometry时不是必要安装的。
    作者在MAV上没有运行g2o。
    g2o的安装依赖包括：
    cmake, 
    libeigen3-dev, l
    ibsuitesparse-dev, 
    libqt4-dev,
    qt4-qmake, 
    libqglviewer-qt4-dev。
    可以用apt-get install分别安装。
    
    安装步骤：
        cd workspace
        git clone https://github.com/RainerKuemmerle/g2o.git
        cd g2o
        mkdir build
        cd build
        cmake ..
        make
        sudo make install
        
   　如果不想在系统安装的话，可以在编译时指定安装路径：
    　cmake .. -CMAKE_INSTALL_PREFIX:PATH=$HOME/installdir代替。   
        
## 4. vikit - Some useful tools that we need
    vikit库包含相机模型、一些svo用到的数学和插值函数，
    vikit是一个ROS下的catkin工程，
    所以需要下载到catkin工作空间的目录。
        cd catkin_ws/src
        git clone https://github.com/uzh-rpg/rpg_vikit.git
        catkin_make
## 5. ROS 依赖项
    有时候cmake-modules会丢失 (包含 Eigen库(矩阵运算) in ROS Indigo)，
    sudo apt-get install ros-indigo-cmake-modules
    indigo换成自己的ros版本。
## 6. SVO安装
    现在可以编译build svo了，把它clone到catkin工作空间。
    cd catkin_ws/src
    git clone https://github.com/uzh-rpg/rpg_svo.git
    catkin_make
    
    以下三句很重要:更新环境变量
        source devel/setup.bash
        echo “source ~/catkin_ws/devel/setup.bash” >> ~/.bashrc
        source ~/.bashrc
        
    如果安装了g2o的话，把svo/CmakeLists.txt里的
        HAVE_G20 = TRUE 
    如果把g2o安装在$HOME/installdir的话，
    需要设置环境变量G2O_ROOT告诉find_package.
        export G2O_ROOT=$HOME/installdir
## 7. 下载数据运行
    [下载数据](http://www.voidcn.com/link?url=http://rpg.ifi.uzh.ch/datasets/airground_rig_s3_2013-03-18_21-38-48.bag)
    比较大，有1.7g左右，下载完成后放在catkin_ws/src目录下（究竟放哪里比较好不清楚，这里反正是可以运行的）。  
    打开四个terminal终端运行：
        1.ros主节点 　roscore 
        2.svo主节点　 roslaunch svo_ros test_rig3.launch
        3.可视化rviz　
          rosrun rviz rviz -d /home/your_pc_name/catkin_ws/rpg_svo/svo_ros/rviz_config.rviz
        4.数据回放　rosbag
        　rosbag play airground_rig_s3_2013-03-18_21-38-48.bag
         
    现在应该就可以看到运行的效果了,包括跟踪的特征点和Rviz里面移动的相机，
    如果想看到跟踪到的特征数、fps和跟踪质量tracking quality的话，运行GUI。     
    
    若是需要用自己的摄像头运行程序，需要对相机进行标定，
    并修改/catkin_ws/src/rpg_svo/svo_ros/param目录下
    camera_atan.yaml, camera_pinhole.yaml的相机参数。
    或者将camera文件起一个不一样的名字，然后修改launchFile，
    目录为svo_ros/launch/live.launch，更改为新的标定yaml文件。
    最后运行roslaunch svo_ros live.launch。
    
    
        
        
    
