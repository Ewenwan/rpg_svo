// This file is part of SVO - Semi-direct Visual Odometry.
// 半直接视觉里程计svo 测试　　

#include <svo/config.h>
#include <svo/frame_handler_mono.h>
#include <svo/map.h>
#include <svo/frame.h>
#include <vector>
#include <string>
#include <vikit/math_utils.h>
#include <vikit/vision.h>
#include <vikit/abstract_camera.h>
#include <vikit/atan_camera.h>
#include <vikit/pinhole_camera.h>
#include <opencv2/opencv.hpp>
#include <sophus/se3.h>// 姿态李代数
#include <iostream>
#include "test_utils.h"
//　命名空间
namespace svo {
    // BenchmarkNode类
    class BenchmarkNode
    {
      vk::AbstractCamera* cam_;//针孔相机模型
      svo::FrameHandlerMono* vo_;//单目视觉里程计 跟踪

    public:
      BenchmarkNode();
      ~BenchmarkNode();
      void runFromFolder();//运行
    };
    // 类构造函数
    BenchmarkNode::BenchmarkNode()
    {
      cam_ = new vk::PinholeCamera(752, 480, 315.5, 315.5, 376.0, 240.0);
      vo_ = new svo::FrameHandlerMono(cam_);
      vo_->start();//启动视觉里程计 进行跟踪
    }
    // 类析构函数
    BenchmarkNode::~BenchmarkNode()
    {
      delete vo_;
      delete cam_;
    }
  
// 运行借口
    void BenchmarkNode::runFromFolder()
    {
      // 从第二帧开始跟踪
      for(int img_id = 2; img_id < 188; ++img_id)
      {
        // 载入图像
        std::stringstream ss;//图片文件地址字符串流
        ss << svo::test_utils::getDatasetDir() << "/sin2_tex2_h1_v8_d/img/frame_"
           << std::setw( 6 ) << std::setfill( '0' ) << img_id << "_0.png";//格式
        if(img_id == 2)
          std::cout << "reading image " << ss.str() << std::endl;
        // opencv读取图像
        cv::Mat img(cv::imread(ss.str().c_str(), 0));
        assert(!img.empty());//　图像为空时退出

        // 添加图像并处理　process frame
        vo_->addImage(img, 0.01*img_id);

        // 显示跟踪质量　display tracking quality
        if(vo_->lastFrame() != NULL)//上一帧
        {
          std::cout << "Frame-Id: " << vo_->lastFrame()->id_ << " \t"
                      << "#Features: " << vo_->lastNumObservations() << " \t"// 观测到的路标点数量
                      << "Proc. Time: " << vo_->lastProcessingTime()*1000 << "ms \n";

          // access the pose of the camera via vo_->lastFrame()->T_f_w_.
        }
      }
    }
  
} // namespace svo

int main(int argc, char** argv)
{
  {
    svo::BenchmarkNode benchmark;// 定义一个BenchmarkNode类的对象
    benchmark.runFromFolder();//运行　跟踪
  }
  printf("BenchmarkNode finished.\n");
  return 0;
}

