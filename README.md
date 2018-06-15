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
    
    
        
        
    
