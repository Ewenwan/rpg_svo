// This file is part of SVO - Semi-direct Visual Odometry.
//
/*
VoNode构造函数
    其中类VoNode中的构造函数完成了多种功能：
    1. 首先开辟了一个线程用于监听控制台输入（用到了boost库）：
           user_input_thread_ = boost::make_shared<vk::UserInputThread>();
    2. 然后加载摄像机参数（svo文件夹），用到了vikit工具库
    3. 初始化位姿，用到了Sophus、vikit
           其中，Sophus::SE3(R，t) 用于构建一个欧式群SE3，R，t为旋转矩阵和平移向量
           vk::rpy2dcm(const Vector3d &rpy) 可将欧拉角 rpy 转换为旋转矩阵
    4. 初始化视觉前端VO（通过定义变量vo_以及svo::FrameHandlerMono构造函数完成）
           svo::FrameHandlerMono定义在frame_handler_mono.cpp中。
           4.1 FrameHandlerMono函数初始化时，先调用了FrameHandlerBase（定义在frame_handler_base.cpp中）
                  FrameHandlerBase先完成了一些设置，如最近10帧中特征数量等。
                  然后初始化了一系列算法性能监视器
           4.2 然后进行重投影的初始化，由Reprojector（定义在reprojector.cpp中）构造函数完成（initializeGrid）：
                  initializeGrid ，
                  注：ceil为取整函数
                  grid_ 为Grid类变量，Grid中定义了CandidateGrid型变量cells，
                  而CandidateGrid是一个Candidate型的list（双向链表）组成的vector（向量）。
                  grid_.cells.resize是设置了cells（vector）的大小。即将图像划分成多少个格子。
                  然后通过for_each函数对cells每个链表（即图像每个格子）申请一块内存。
                  之后通过for函数给每个格子编号。
                  最后调用random_shuffle函数将格子的顺序打乱。
           4.3 通过DepthFilter（深度滤波器）构造函数完成初始化
                  设置了特征检测器指针、回调函数指针、随机种子、线程、新关键帧深度的初值。
           4.4 调用initialize初始化函数
                  设置了特征检测器指针类型为fast特征检测器，
                  设置了回调函数指针所指内容，即由bind函数生成的指针。
                  bind函数将newCandidatePoint型变量point_candidates_（map_的成员变量）
                  与输入参数绑定在一起构成函数指针depth_filter_cb。
                  最终depth_filter_cb有两个输入参数，这两个参数被传入point_candidates进行计算。
                  最后的最后，特征检测指针和回调函数指针在一起生成一个深度滤波器depth_filter_。
                  深度滤波器启动线程。
    5. 调用svo::FrameHandlerMono的start函数
           注：你可能在frame_handler_mono.cpp中找不到start函数，
           因为start原本是定义在frame_handler_base.h中，
           而FrameHandlerMono是继承自FrameHandlerBase，所以可以调用start函数。
           frame_handler_base.h中FrameHandlerBase::start()定义为：
                     void start() { set_start_ = true; }
           所以通过start函数将set_start_置为true。
    至此，VoNode的构造函数就介绍完了，知道了VoNode构造函数才能知道main函数中创建VoNode实例时发生了什么。

main()函数
  下面是main函数，也就是主函数：
  1. 首先调用ros::init完成了ros的初始化
  2.创建节点句柄NodeHandle ，名为nh（创建节点前必须要有NodeHandle）
  3.创建节点VoNode，名为vo_node，同时VoNode的构造函数完成一系列初始化操作。
  4.订阅cam消息：
      先定义topic的名称；
      创建图片发布/订阅器，名为it，使用了之前创建的节点句柄nh；
      调用image_transport::ImageTransport中的subscribe函数：
            it.subscribe(cam_topic, 5, &svo::VoNode::imgCb, &vo_node)
      意思是，对于节点vo_node，一旦有图像（5代表队列长度，应该是5张图片）发布到主题cam_topic时，就执行svo::VoNode::imgCb函数。
      然后it.subscribe返回值保存到image_transport::Subscriber型变量it_sub。
      其中svo::VoNode::imgCb工作如下：

      4.1 首先完成了图片的读取

             img = cv_bridge::toCvShare(msg, "mono8")->image

             这句将ROS数据转变为OpenCV中的图像数据。

      4.2 执行processUserActions()函数（定义在同文件中），开辟控制台输入线程，并根据输入的字母进行相应的操作。

      4.3 调用FrameHandlerMono：：addImage函数（定义在frame_handler_mono.cpp中）
             其中，msg->header.stamp.toSec()可获取系统时间（以秒为单位）
             获取的图片img和转换的系统时间被传入函数addImage，addImage过程：
       4.3.1 首先进行if判断，如果startFrameProcessingCommon返回false，则addImage结束，直接执行return。
              startFrameProcessingCommon函数过程：
              首先判断set_start_，值为true时，（在创建vo_node时就已通过VoNode构造函数将set_start_设置为true），
              然后执行resetAll（），resetAll函数定义在frame_handler_base.h中：virtual void resetAll(){resetCommon();}，
              且resetCommon()定义在同文件中，主要完成了Map型变量map_的初始化（包括关键帧和候选点的清空等），同时stage_
              被改为STAGE_PAUSED，set_reset和set_start都被设置为false，还有其它一些设置。执行resetAll后，
              
              设置=====stage_ = STAGE_FIRST_FRAME 处理第一帧===================。
              
              然后判断stage是否为STAGE_PAUSED，若是则结束startFrameProcessingCommon，并返回false。
              经过两个if后，将传进函数的系统时间和“New Frame”等信息记录至日志文件中。
              并启动vk::Timer型变量timer_（用于计量程序执行时间，可精确到毫秒级）。
              最后清空map_的垃圾箱trash，其实前面resetAll函数里已经完成这个功能了，不知道为什么还要再来一次。
              执行完这些后，startFrameProcessingCommon返回一个true。
        4.3.2 清空core_kfs_和overlap_kfs_，这两个都是同种指针的集合。core_kfs_是Frame类型的智能指针shared_ptr，
              用于表示一帧周围的关键帧。overlap_kfs_是一个向量，存储的是由一个指针和一个数值构成的组合变量，
              用于表示具有重叠视野的关键帧，数值代表了重叠视野中的地标数。
        4.3.3 创建新帧并记录至日志文件。
              新构造一个Frame对象，然后用Frame型智能指针变量new_frame指向它。
               .reset函数将智能指针原来所指对象销毁并指向新对象。
               创建Frame对象时，发生了一系列操作，通过其构造函数完成。
               Frame构造函数定义在frame.cpp中：
               首先完成了一些变量的初始化，如id、系统时间、相机模型、用于判两帧是否有重叠视野的特征数量、关键帧标志等。
               然后调用intFrame（img）函数。对传入的img创建图像金字塔img_pyr_（一个Mat型向量）。

        4.3.4 处理帧。首先设置UpdateResult型枚举变量res，值为RESULT_FAILURE。
               然后判断stage_：
               值为STAGE_FIRST_FRAME，执行 processFirstFrame(),如果成功(关键点数量>100),则设置为第二帧模式；
                                          faste角点提取匹配，
                                          单应矩阵求解变换，计算3d点
                                          设置系统状态 stage_ = STAGE_SECOND_FRAME
               值为STAGE_SECOND_FRAME，执行processSecondFrame()；
                                          金字塔多尺度光流跟踪
                                          计算前后两帧的单应变换矩阵，计算3d点
                                          集束调整优化, 非线性最小二乘优化
                                          计算场景深度均值和最小值，进行 高斯均值深度值滤波优化
                                          设置系统状态 stage_= STAGE_DEFAULT_FRAME；
               值为STAGE_DEFAULT_FRAME，执行processFrame()；
               
               值为STAGE_RELOCALIZING，执行relocalizeFrame。
             其中:
             processFirstFrame() 作用是处理第1帧并将其设置为关键帧；
             processSecondFrame()    作用是处理第1帧后面所有帧，直至找到一个新的关键帧；
             processFrame()          作用是处理两个关键帧之后的所有帧；
             relocalizeFrame()       作用是在相关位置重定位帧以提供关键帧（直译过来是这样，不太理解，
             
             由于这4个函数比较大，所以它们的解析放在最后附里面吧。
          4.3.5 将new_frame_赋给last_frame_，然后将new_frame_给清空，供下次使用。
          4.3.6 执行finishFrameProcessingCommon，传入的参数为last_frame_的id号和图像中的特征数nOb的值、res的值。
            finishFrameProcessingCommon工作如下：
            将last_frame_信息记录至日志。
            统计当前帧处理时间并压入acc_frame_timings_以捅进最近10帧的总处理时间。如果stage_值为STAGE_DEFAULT_FRAME，
            还将nOb传入的值压入acc_num_obs_以统计最近10帧的检测出的特征总数。
            然后是一个条件编译判断，如果定义了SVO_TRACE，就会调用PerformanceMonitor::writeToFile()
            （定义在performance_monitor.cpp中），writeToFile先调用trace()（定义在同文件中）
            将系统时间记录至文件（logs_是干什么的没看懂，也没注释）。
            然后用互斥锁对线程进行写保护，再将特征点数量记录至日志。
            将传入的参数res值赋给dropout，然后判断dropout：
            值为RESULT_FAILURE，同时stage_为STAGE_DEFAULT_FRAME或stage_ == STAGE_RELOCALIZING，就执行：
            stage_=STAGE_RELOCALIZING
            tracking_quality_ = TRACKING_INSUFFICIENT
            当只有dropout == RESULT_FAILURE时，就执行resetAll()：
            进行Map型变量map_的初始化（包括关键帧和候选点的清空等），
            同时stage_被改为STAGE_PAUSED，set_reset和set_start都被设置为false，还有其它一些设置。
            判断dropout后判断set_reset_，如果为真，同样执行resetAll()。
            最后返回0，结束finishFrameProcessingCommon。
          4.3.7 结束addImag函数。
      4.4 调用Visualizer类成员函数publishMinimal（进行ROS消息有关的设置）。
      4.5 调用Visualizer类成员函数visualizeMarkers
          （里面又调用了publishTfTransform、publishCameraMarke、publishPointMarker、publishMapRegion等函数），
          进行Marker和关键帧的显示。
      4.6调用Visualizer类成员函数exportToDense函数（稠密显示特征点）。
      4.7 判断stage_，若值为STAGE_PAUSED，则将线程挂起100000微秒（0.1秒）。
      4.8 结束imgCb函数。
  5. 订阅远程输入消息（应该指的就是键盘输入）
       nh.subscribe("svo/remote_key", 5, &svo::VoNode::remoteKeyCb, &vo_node)
      意思跟上面差不多，有消息发布到主题svo/remote_key
      （队列长度是5，如果输入6个数据，那么第6个就会被舍弃），就执行svo::VoNode::remoteKeyCb函数。
      返回值保存到vo_node.sub_remote_key_。
  6. 无图像信息输入或者键盘输入q，则停止程序，打印 SVO终止 信息。

*/
#include <ros/package.h>
#include <string>
#include <svo/frame_handler_mono.h>// 视觉前端原理 视觉前端基础类
#include <svo/map.h>// 地图管理 
#include <svo/config.h>//SVO的全局配置
#include <svo_ros/visualizer.h>// 可视化
#include <vikit/params_helper.h>
#include <sensor_msgs/Image.h>//ros的传感器消息 类 图像
#include <sensor_msgs/Imu.h>//imu惯性传感器
#include <std_msgs/String.h>
#include <message_filters/subscriber.h>// ros消息滤波器  订阅
#include <message_filters/synchronizer.h>// 消息同步
#include <message_filters/sync_policies/approximate_time.h>//时间戳
#include <image_transport/image_transport.h>// 图像传输
#include <boost/thread.hpp>// boost线程
#include <cv_bridge/cv_bridge.h>// opencv格式数据 转到
#include <Eigen/Core>// 矩阵运算
#include <vikit/abstract_camera.h>
#include <vikit/camera_loader.h>
#include <vikit/user_input_thread.h>

namespace svo {
  
// 类VoNode
/// SVO Interface
class VoNode
{
    public:
      svo::FrameHandlerMono* vo_;// 单目vo
      svo::Visualizer visualizer_;
      // 发布ar虚拟体
      bool publish_markers_;   //!< publish only the minimal amount of info (choice for embedded devices)
      bool publish_dense_input_;
        // 首先开辟了一个线程用于监听控制台输入（用到了boost库）：
      boost::shared_ptr<vk::UserInputThread> user_input_thread_;
        // 订阅节点
      ros::Subscriber sub_remote_key_;
      std::string remote_input_;
      vk::AbstractCamera* cam_;
      bool quit_;
      VoNode();//构造函数
      ~VoNode();//析构函数
      // 图像订阅 话题 回调函数 callback function
      void imgCb(const sensor_msgs::ImageConstPtr& msg);
      void processUserActions();
      void remoteKeyCb(const std_msgs::StringConstPtr& key_input);
    };

    //构造函数
    VoNode::VoNode() :
      vo_(NULL),// 初始化视觉前端VO（通过定义变量vo_以及svo::FrameHandlerMono构造函数完成）
      publish_markers_(vk::getParam<bool>("svo/publish_markers", true)),
      publish_dense_input_(vk::getParam<bool>("svo/publish_dense_input", false)),
      remote_input_(""),
      cam_(NULL),
      quit_(false)
    {
      // Start user input thread in parallel thread that listens to console keys
      if(vk::getParam<bool>("svo/accept_console_user_input", true))
    // 1. 首先开辟了一个线程用于监听控制台输入（用到了boost库）
        user_input_thread_ = boost::make_shared<vk::UserInputThread>();

    // 2. 然后加载摄像机参数（svo文件夹），用到了vikit工具库  Create Camera
      if(!vk::camera_loader::loadFromRosNs("svo", cam_))
        throw std::runtime_error("Camera model not correctly specified.");

    // 3. 初始化位姿，用到了Sophus、vikit  Get initial position and orientation
      visualizer_.T_world_from_vision_ = Sophus::SE3(
          // 其中，Sophus::SE3(R，t) 用于构建一个欧式群SE3，R，t为旋转矩阵和平移向量
          //  vk::rpy2dcm(const Vector3d &rpy) 可将欧拉角 rpy 转换为 旋转矩阵dcm
          vk::rpy2dcm(Vector3d(vk::getParam<double>("svo/init_rx", 0.0),
                               vk::getParam<double>("svo/init_ry", 0.0),
                               vk::getParam<double>("svo/init_rz", 0.0))),
          // 平移向量t Eigen::Vector3d
          Eigen::Vector3d(vk::getParam<double>("svo/init_tx", 0.0),
                          vk::getParam<double>("svo/init_ty", 0.0),
                          vk::getParam<double>("svo/init_tz", 0.0)));

    // 4. 初始化视觉前端VO（通过定义变量vo_以及svo::FrameHandlerMono构造函数完成）
      vo_ = new svo::FrameHandlerMono(cam_);// 定义在frame_handler_mono.cpp中
          // 4.1 先调用了FrameHandlerBase（定义在frame_handler_base.cpp中）
          // FrameHandlerBase先完成了一些设置，如最近10帧中特征数量等。
          // 然后初始化了一系列算法性能监视器
          // 4.2 然后进行重投影的初始化，
          // 由Reprojector（定义在reprojector.cpp中）构造函数完成（initializeGrid）：
          // 4.3 通过DepthFilter（深度滤波器）构造函数完成初始化
          // 4.4 调用initialize初始化函数 深度滤波器启动线程
    // 5. 调用svo::FrameHandlerMono的start函数
      // 你可能在frame_handler_mono.cpp中找不到start函数，
      // 因为start原本是定义在frame_handler_base.h中，
      //而FrameHandlerMono是继承自FrameHandlerBase，所以可以调用start函数。
      vo_->start();// 所以通过start函数将set_start_置为true
    }

    //析构函数
    VoNode::~VoNode()
    {
      delete vo_;
      delete cam_;
      if(user_input_thread_ != NULL)
        user_input_thread_->stop();
    }
    // 图像订阅回调函数
    void VoNode::imgCb(const sensor_msgs::ImageConstPtr& msg)
    {
      cv::Mat img;// opencv类型图像格式
      try {
    // 4.1 首先完成了图片的读取      
         // ros 图像消息转换成 opencv类型图像格式
        img = cv_bridge::toCvShare(msg, "mono8")->image;
      } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
      }
  // 4.2 处理用户行为, 开辟控制台输入线程，并根据输入的字母进行相应的操作。      
      processUserActions();// 处理用户行为  处理按键输入等中断
  // 4.3 vo视觉里程计添加一帧图像 附带时间戳
        // 调用FrameHandlerMono：：addImage函数（定义在frame_handler_mono.cpp中）
      vo_->addImage(img, msg->header.stamp.toSec());//其中，msg->header.stamp.toSec()可获取系统时间（以秒为单位）
        // 会完成一系列的初始化 Map型变量map_的初始化（包括关键帧和候选点的清空等）新构造一个Frame对象
        // 对传入的img创建图像金字塔img_pyr_（一个Mat型向量）。
        // startFrameProcessingCommon()设置 系统状态为处理第一帧 stage_ = STAGE_FIRST_FRAME
        //其中比较重要的函数:
        //processFirstFrame() 作用是处理第1帧并将其设置为关键帧；(如果特征点数量>100,则设置这一帧为第一个关键帧)
                             // 同时设置系统状态为 处理第二帧 
        //processSecondFrame()    作用是处理第1帧后面所有帧，直至找到一个新的关键帧；
        //processFrame()          作用是处理两个关键帧之后的所有帧；
        //relocalizeFrame()       作用是在相关位置重定位帧以提供关键帧（直译过来是这样，不太理解，
  // 4.4 调用Visualizer类成员函数publishMinimal（进行ROS消息有关的设置）。
      visualizer_.publishMinimal(img, vo_->lastFrame(), *vo_, msg->header.stamp.toSec());
  // 4.5 调用Visualizer类成员函数visualizeMarkers（里面又调用了
      // publishTfTransform、publishCameraMarke、publishPointMarker、publishMapRegion等函数），
      // 进行AR Marker和关键帧的显示。
      if(publish_markers_ && vo_->stage() != FrameHandlerBase::STAGE_PAUSED)
        visualizer_.visualizeMarkers(vo_->lastFrame(), vo_->coreKeyframes(), vo_->map());
  // 4.6调用Visualizer类成员函数exportToDense函数（稠密显示特征点）。
      if(publish_dense_input_)
        visualizer_.exportToDense(vo_->lastFrame());
  // 4.7 判断stage_，若值为STAGE_PAUSED，则将线程挂起100000微秒（0.1秒）。
      if(vo_->stage() == FrameHandlerMono::STAGE_PAUSED)
        usleep(100000);
    }

    // 处理用户行为  处理按键输入等中断
    void VoNode::processUserActions()
    {
      char input = remote_input_.c_str()[0];//c字符类型
      remote_input_ = "";

      if(user_input_thread_ != NULL)//用户输入线程
      {
        char console_input = user_input_thread_->getInput();
        if(console_input != 0)
          input = console_input;
      }

      switch(input)
      {
        case 'q':// quit 停止
          quit_ = true;
          printf("SVO user input: QUIT\n");
          break;
        case 'r':// reset 重置
          vo_->reset();
          printf("SVO user input: RESET\n");
          break;
        case 's':// start开始
          vo_->start();
          printf("SVO user input: START\n");
          break;
        default: ;
      }
    }
    // 按键响应话题回调函数
    void VoNode::remoteKeyCb(const std_msgs::StringConstPtr& key_input)
    {
      remote_input_ = key_input->data;
    }

} // namespace svo

// 主程序
int main(int argc, char **argv)
{
//  1.初始化节点 完成了ros的初始化
  ros::init(argc, argv, "svo");
// 2. 创建节点句柄NodeHandle ，名为nh（创建节点前必须要有NodeHandle）
  ros::NodeHandle nh;//节点句柄
  std::cout << "create vo_node" << std::endl;
//3. 创建voNode类 同时VoNode的构造函数完成一系列初始化操作。
  svo::VoNode vo_node;

// 4. 订阅相机图像话题 subscribe to cam msgs
  std::string cam_topic(vk::getParam<std::string>("svo/cam_topic", "camera/image_raw"));//获取参数配置文件内的配置参数
    // 参数名 "svo/cam_topic"  默认参数值 "camera/image_raw"
  image_transport::ImageTransport it(nh);//图像传输类
    // 创建图片发布/订阅器，名为it，使用了之前创建的节点句柄nh；
  image_transport::Subscriber it_sub = it.subscribe(cam_topic, 5, &svo::VoNode::imgCb, &vo_node);
    //  对于节点vo_node，一旦有图像（5代表队列长度，应该是5张图片）发布到主题cam_topic时，就执行svo::VoNode::imgCb函数
  // 订阅键盘输入话题 subscribe to remote input
// 5. 订阅远程输入消息（应该指的就是键盘输入）
  vo_node.sub_remote_key_ = nh.subscribe("svo/remote_key", 5, &svo::VoNode::remoteKeyCb, &vo_node);
    // 有消息发布到主题svo/remote_key（队列长度是5，如果输入6个数据，那么第6个就会被舍弃），
    // 就执行svo::VoNode::remoteKeyCb函数。
    // 返回值保存到vo_node.sub_remote_key_。
  // start processing callbacks
// 6. 无图像信息输入或者键盘输入q/CTRL+C等 ，则停止程序，打印 SVO终止 信息。
  while(ros::ok() && !vo_node.quit_)
  {
    ros::spinOnce();
    // TODO check when last image was processed. when too long ago. publish warning that no msgs are received!
  }

  printf("SVO terminated.\n");// 打印 SVO终止 信息。
  return 0;
}
