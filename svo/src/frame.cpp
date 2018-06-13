// This file is part of SVO - Semi-direct Visual Odometry.
// 帧 结构体
// 生成图像金字塔
// 获取帧场景深度均值 
// 世界坐标3d点投影到当前帧下，获取3d点的z轴值，记录最小值并计算均值
#include <stdexcept>
#include <svo/frame.h>
#include <svo/feature.h>
#include <svo/point.h>
#include <svo/config.h>
#include <boost/bind.hpp>
#include <vikit/math_utils.h>
#include <vikit/vision.h>
#include <vikit/performance_monitor.h>
#include <fast/fast.h>

namespace svo {

int Frame::frame_counter_ = 0;

Frame::Frame(vk::AbstractCamera* cam, const cv::Mat& img, double timestamp) :
    id_(frame_counter_++),//帧id
    timestamp_(timestamp),//时间戳
    cam_(cam),//相机参数
    key_pts_(5),//
    is_keyframe_(false),//关键帧标志
    v_kf_(NULL)
{
  initFrame(img);
}
// 析构函数
Frame::~Frame()
{
  std::for_each(fts_.begin(), fts_.end(), [&](Feature* i){delete i;});
}
// 初始化帧
void Frame::initFrame(const cv::Mat& img)
{
  // check image
  if(img.empty() || img.type() != CV_8UC1 || img.cols != cam_->width() || img.rows != cam_->height())
    throw std::runtime_error("Frame: provided image has not the same size as the camera model or image is not grayscale");

  // Set keypoints to NULL
  std::for_each(key_pts_.begin(), key_pts_.end(), [&](Feature* ftr){ ftr=NULL; });

  // Build Image Pyramid
  // 图像金字塔
  frame_utils::createImgPyramid(img, max(Config::nPyrLevels(), Config::kltMaxLevel()+1), img_pyr_);
}
//普通帧设置为关键帧
void Frame::setKeyframe()
{
  is_keyframe_ = true;
  setKeyPoints();
}
// 添加特征
void Frame::addFeature(Feature* ftr)
{
  fts_.push_back(ftr);
}
// 关键点
void Frame::setKeyPoints()
{
  for(size_t i = 0; i < 5; ++i)
    if(key_pts_[i] != NULL)
      if(key_pts_[i]->point == NULL)
        key_pts_[i] = NULL;

  std::for_each(fts_.begin(), fts_.end(), [&](Feature* ftr){ if(ftr->point != NULL) checkKeyPoints(ftr); });
}
// 检查关键点
void Frame::checkKeyPoints(Feature* ftr)
{
  const int cu = cam_->width()/2;//cx
  const int cv = cam_->height()/2;//cy

  // center pixel
  if(key_pts_[0] == NULL)
    key_pts_[0] = ftr;
  else if(std::max(std::fabs(ftr->px[0]-cu), std::fabs(ftr->px[1]-cv))
        < std::max(std::fabs(key_pts_[0]->px[0]-cu), std::fabs(key_pts_[0]->px[1]-cv)))
    key_pts_[0] = ftr;

  if(ftr->px[0] >= cu && ftr->px[1] >= cv)
  {
    if(key_pts_[1] == NULL)
      key_pts_[1] = ftr;
    else if((ftr->px[0]-cu) * (ftr->px[1]-cv)
          > (key_pts_[1]->px[0]-cu) * (key_pts_[1]->px[1]-cv))
      key_pts_[1] = ftr;
  }
  if(ftr->px[0] >= cu && ftr->px[1] < cv)
  {
    if(key_pts_[2] == NULL)
      key_pts_[2] = ftr;
    else if((ftr->px[0]-cu) * (ftr->px[1]-cv)
          > (key_pts_[2]->px[0]-cu) * (key_pts_[2]->px[1]-cv))
      key_pts_[2] = ftr;
  }
  if(ftr->px[0] < cv && ftr->px[1] < cv)
  {
    if(key_pts_[3] == NULL)
      key_pts_[3] = ftr;
    else if((ftr->px[0]-cu) * (ftr->px[1]-cv)
          > (key_pts_[3]->px[0]-cu) * (key_pts_[3]->px[1]-cv))
      key_pts_[3] = ftr;
  }
  if(ftr->px[0] < cv && ftr->px[1] >= cv)
  {
    if(key_pts_[4] == NULL)
      key_pts_[4] = ftr;
    else if((ftr->px[0]-cu) * (ftr->px[1]-cv)
          > (key_pts_[4]->px[0]-cu) * (key_pts_[4]->px[1]-cv))
      key_pts_[4] = ftr;
  }
}

void Frame::removeKeyPoint(Feature* ftr)
{
  bool found = false;
  std::for_each(key_pts_.begin(), key_pts_.end(), [&](Feature*& i){
    if(i == ftr) {
      i = NULL;
      found = true;
    }
  });
  if(found)
    setKeyPoints();
}
// 地图点投影到本帧图像坐标系下 z轴大于0 3d点在相机前方，再投影到像素平面上，在像素尺寸范围内，在视角范围内
bool Frame::isVisible(const Vector3d& xyz_w) const
{
  Vector3d xyz_f = T_f_w_*xyz_w;//投影到相机坐标系下
  if(xyz_f.z() < 0.0)//在轴为负，不可见
    return false; // point is behind the camera
  Vector2d px = f2c(xyz_f);//再投影到像素平面下
  // 如果也在像素平面内，则本帧可以观测到该3d点
  if(px[0] >= 0.0 && px[1] >= 0.0 && px[0] < cam_->width() && px[1] < cam_->height())
    return true;// 本帧可以观测到该3d点
  return false;
}


/// Utility functions for the Frame class
namespace frame_utils {
// 创建图像金字塔
void createImgPyramid(const cv::Mat& img_level_0, int n_levels, ImgPyr& pyr)
{
  pyr.resize(n_levels);//金字塔层数
  pyr[0] = img_level_0;//第0层
  for(int i=1; i<n_levels; ++i)
  {
    pyr[i] = cv::Mat(pyr[i-1].rows/2, pyr[i-1].cols/2, CV_8U);// 尺寸减半
    vk::halfSample(pyr[i-1], pyr[i]);//上层下采样生成后面的几层金字塔图像
  }
}
    
// 获取场景深度 世界坐标3d点投影到当前帧下，获取3d点的z轴值，记录最小值并计算均值
bool getSceneDepth(const Frame& frame, double& depth_mean, double& depth_min)
{
  vector<double> depth_vec;//深度 数组
  depth_vec.reserve(frame.fts_.size());
  depth_min = std::numeric_limits<double>::max();//最小值初始化为极大值
  for(auto it=frame.fts_.begin(), ite=frame.fts_.end(); it!=ite; ++it)//遍历帧的每一个3d点
  {
    if((*it)->point != NULL)// 3d点 不为空
    {
      const double z = frame.w2f((*it)->point->pos_).z();//世界坐标系转到相机坐标下下，并获取z轴值，即深度值
      depth_vec.push_back(z);// 存储深度
      depth_min = fmin(z, depth_min);//记录最小值
    }
  }
  if(depth_vec.empty())
  {
    SVO_WARN_STREAM("Cannot set scene depth. Frame has no point-observations!");
    return false;
  }
  depth_mean = vk::getMedian(depth_vec);//深度均值
  return true;
}

} // namespace frame_utils
} // namespace svo
