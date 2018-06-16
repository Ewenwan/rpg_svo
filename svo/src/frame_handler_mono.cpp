// This file is part of SVO - Semi-direct Visual Odometry.
// 视觉前端原理  深度滤波线程 基于 fast特征检测 和 地图点深度值滤波回调函数
/*
==============frame_handler_mono.cpp  ============================================       
processFirstFrame();// 作用是处理第1帧并将其设置为关键帧；
processSecondFrame();// 作用是处理第1帧后面所有帧，直至找到一个新的关键帧；
processFrame();// 作用是处理两个关键帧之后的所有帧；
relocalizeFrame(SE3(Matrix3d::Identity(),Vector3d::Zero()),map_.getClosestKeyframe(last_frame_));
//作用是在相关位置重定位帧以提供关键帧

*/
#include <svo/config.h>
#include <svo/frame_handler_mono.h>
#include <svo/map.h>
#include <svo/frame.h>
#include <svo/feature.h>
#include <svo/point.h>
#include <svo/pose_optimizer.h>
#include <svo/sparse_img_align.h>
#include <vikit/performance_monitor.h>
#include <svo/depth_filter.h>
#ifdef USE_BUNDLE_ADJUSTMENT
#include <svo/bundle_adjustment.h>
#endif

namespace svo {

// 视觉前端 单目帧处理类
// 类构造函数========================================================
    FrameHandlerMono::FrameHandlerMono(vk::AbstractCamera* cam) :
      FrameHandlerBase(),// 继承于(基于)视觉前端基础类
      cam_(cam),//继承于 相机类
      reprojector_(cam_, map_),//继承于 重投影类 相机类 地图的生成与管理类
      depth_filter_(NULL)// 深度值滤波器
    {
      initialize();
    }

// 视觉前端 单目帧处理类初始化函数==========================================================
    void FrameHandlerMono::initialize()
    {
   // 特征检测类(FastDetector) 指针
      feature_detection::DetectorPtr feature_detector(
          new feature_detection::FastDetector(
            cam_->width(), 
            cam_->height(), 
            Config::gridSize(), 
            Config::nPyrLevels() ) );

   // 地图点深度值滤波 回调函数 回调函数指针所指内容，即由bind函数生成的指针
      DepthFilter::callback_t depth_filter_cb = boost::bind(
          &MapPointCandidates::newCandidatePoint, &map_.point_candidates_, _1, _2);
    // 设置了特征检测器指针、回调函数指针、随机种子、线程、新关键帧深度的初值
    // 深度滤波 设置了特征检测器指针类型为fast特征检测器 
        // 设置了回调函数指针所指内容，即由bind函数生成的指针。
        // 特征检测指针和回调函数指针在一起生成一个深度滤波器depth_filter_。
      depth_filter_ = new DepthFilter(feature_detector, depth_filter_cb);
   // 启动线程   深度滤波器启动线程
      depth_filter_->startThread();
    }

// 类析构函数========================================
    FrameHandlerMono::~FrameHandlerMono()
    {
      delete depth_filter_;
    }

// 添加图像==================================================================================
    void FrameHandlerMono::addImage(const cv::Mat& img, const double timestamp)
    {
        
// 4.3.1 首先进行if判断，如果startFrameProcessingCommon返回false(暂停状态stage_ == STAGE_PAUSED )，
      // 最主要是进行 系统状态判断
      // 则addImage结束，直接执行return。
      // frame_handler_base.cpp 中
      // 会设置 stage_ = STAGE_FIRST_FRAME; 处理第一帧
      if(!startFrameProcessingCommon(timestamp))
        return;
      
// 4.3.2 清空上次迭代变量 core_kfs_和overlap_kfs_，这两个都是同种指针的集合。
      // some cleanup from last iteration, can't do before because of visualization
      core_kfs_.clear();// Frame类型的智能指针shared_ptr，用于表示一帧周围的关键帧。
      overlap_kfs_.clear();// 一个向量，存储一个指针和一个数值构成的组合变量
                           // 用于表示具有重叠视野的关键帧，数值代表了重叠视野中的地标数。
// 4.3.3 创建新帧并记录至日志文件。
   // 新构造一个Frame对象，然后用Frame型智能指针变量new_frame指向它。
   // .reset函数将智能指针原来所指对象销毁并指向新对象。
      // create new frame
      SVO_START_TIMER("pyramid_creation");
   // 对传入的img创建 图像金字塔img_pyr_ （一个Mat型向量）。
      new_frame_.reset(new Frame(cam_, img.clone(), timestamp));
      SVO_STOP_TIMER("pyramid_creation");
      
//  4.3.4 处理帧。首先设置UpdateResult型枚举变量res，值为RESULT_FAILURE。
      // process frame
      UpdateResult res = RESULT_FAILURE;
   // 然后判断stage_ 阶段标志：
      if(stage_ == STAGE_DEFAULT_FRAME)
        res = processFrame();// 作用是处理两个关键帧之后的所有帧； 
      
      else if(stage_ == STAGE_SECOND_FRAME)
        res = processSecondFrame();// 作用是处理第1帧后面所有帧，直至找到一个新的关键帧；
      
      else if(stage_ == STAGE_FIRST_FRAME)
        res = processFirstFrame();// 作用是处理第1帧并将其设置为关键帧( 特点点数超过100个点才设置第一帧为为关键帧)
                                  // 并且设置 
      
      else if(stage_ == STAGE_RELOCALIZING)
        // 作用是在相关位置重定位帧以提供关键帧 
        res = relocalizeFrame(SE3(Matrix3d::Identity(), Vector3d::Zero()),
                              map_.getClosestKeyframe(last_frame_));
// 4.3.5 迭代步骤，将new_frame_赋给last_frame_，然后将new_frame_给清空，供下次使用。
      // set last frame
      last_frame_ = new_frame_;
      new_frame_.reset();
// 4.3.6 完成帧处理， 执行finishFrameProcessingCommon，传入的参数为last_frame_的id号和图像中的特征数nOb的值、res的值。
      // finish processing
      // frame_handler_base.cpp 中
      finishFrameProcessingCommon(last_frame_->id_, res, last_frame_->nObs());
    }
  
  
// 作用是处理第1帧并将其设置为关键帧========================================================
//  特点点数超过100个点才设置第一帧为为关键帧，且设置系统处理标志为第二帧
    FrameHandlerMono::UpdateResult FrameHandlerMono::processFirstFrame()
    {
     // 初始化第一帧的位姿 用于表示从世界坐标到初始相机坐标的变换矩阵
      new_frame_->T_f_w_ = SE3(Matrix3d::Identity(), Vector3d::Zero());
     // 检测第一帧初始化情况
        // 其中klt_homography_init_是KltHomographyInit（FrameHandlerMono的友元类）类型的类，
        // 用于计算单应性矩阵（根据前两个关键帧）来初始化位姿。
      if(klt_homography_init_.addFirstFrame(new_frame_) == initialization::FAILURE)
        return RESULT_NO_KEYFRAME;// 第一帧初始化错误 没有关键帧
     //第一帧出海华成功 设置关键帧   
      new_frame_->setKeyframe();
     //    （定义在initialization.cpp中）
      map_.addKeyframe(new_frame_);
        // 初始化px_cur_（一个二维点向量，存储当前帧中被用于跟踪的关键点坐标）
        // 以及frame_ref_（一个frame型智能指针，设置参考帧）
        // 对new_frame进行Fast特征检测（调用FastDetector::detect函数）,得到二维点和三维点，分别赋给px_ref_和f_ref_
        // 判断特征数是否小于100，如果是，就结束addFirstFrame并返回FAILURE
        // 继续执行程序，将传入的new_frame_赋值给frame_ref_
        // 将new_frame_设置为关键帧后，通过addKeyFrame函数把它存入keyframes_中。
   // 切换系统状态，设置第二帧 标志
      stage_ = STAGE_SECOND_FRAME;
   // 将信息“Init: Selected first frame”记录至日志。
      SVO_INFO_STREAM("Init: Selected first frame.");
   // 返回处理标志   特点点数超过100个点才设置第一帧为为关键帧
      return RESULT_IS_KEYFRAME;
    }

  
// 作用是处理第1帧后面所有帧，直至找到一个新的关键帧=============================================
// 
    FrameHandlerBase::UpdateResult FrameHandlerMono::processSecondFrame()
    {
      initialization::InitResult res = klt_homography_init_.addSecondFrame(new_frame_);
      if(res == initialization::FAILURE)
        return RESULT_FAILURE;
      else if(res == initialization::NO_KEYFRAME)
        return RESULT_NO_KEYFRAME;

      // two-frame bundle adjustment
    #ifdef USE_BUNDLE_ADJUSTMENT
      ba::twoViewBA(new_frame_.get(), map_.lastKeyframe().get(), Config::lobaThresh(), &map_);
    #endif

      new_frame_->setKeyframe();
      double depth_mean, depth_min;
      frame_utils::getSceneDepth(*new_frame_, depth_mean, depth_min);
      depth_filter_->addKeyframe(new_frame_, depth_mean, 0.5*depth_min);

      // add frame to map
      map_.addKeyframe(new_frame_);
      stage_ = STAGE_DEFAULT_FRAME;
      klt_homography_init_.reset();
      SVO_INFO_STREAM("Init: Selected second frame, triangulated initial map.");
      return RESULT_IS_KEYFRAME;
    }
  
  
// 作用是处理两个关键帧之后的所有帧====================================================
    FrameHandlerBase::UpdateResult FrameHandlerMono::processFrame()
    {
      // Set initial pose TODO use prior
      new_frame_->T_f_w_ = last_frame_->T_f_w_;

      // sparse image align
      SVO_START_TIMER("sparse_img_align");
      SparseImgAlign img_align(Config::kltMaxLevel(), Config::kltMinLevel(),
                               30, SparseImgAlign::GaussNewton, false, false);
      size_t img_align_n_tracked = img_align.run(last_frame_, new_frame_);
      SVO_STOP_TIMER("sparse_img_align");
      SVO_LOG(img_align_n_tracked);
      SVO_DEBUG_STREAM("Img Align:\t Tracked = " << img_align_n_tracked);

      // map reprojection & feature alignment
      SVO_START_TIMER("reproject");
      reprojector_.reprojectMap(new_frame_, overlap_kfs_);
      SVO_STOP_TIMER("reproject");
      const size_t repr_n_new_references = reprojector_.n_matches_;
      const size_t repr_n_mps = reprojector_.n_trials_;
      SVO_LOG2(repr_n_mps, repr_n_new_references);
      SVO_DEBUG_STREAM("Reprojection:\t nPoints = "<<repr_n_mps<<"\t \t nMatches = "<<repr_n_new_references);
      if(repr_n_new_references < Config::qualityMinFts())
      {
        SVO_WARN_STREAM_THROTTLE(1.0, "Not enough matched features.");
        new_frame_->T_f_w_ = last_frame_->T_f_w_; // reset to avoid crazy pose jumps
        tracking_quality_ = TRACKING_INSUFFICIENT;
        return RESULT_FAILURE;
      }

      // pose optimization
      SVO_START_TIMER("pose_optimizer");
      size_t sfba_n_edges_final;
      double sfba_thresh, sfba_error_init, sfba_error_final;
      pose_optimizer::optimizeGaussNewton(
          Config::poseOptimThresh(), Config::poseOptimNumIter(), false,
          new_frame_, sfba_thresh, sfba_error_init, sfba_error_final, sfba_n_edges_final);
      SVO_STOP_TIMER("pose_optimizer");
      SVO_LOG4(sfba_thresh, sfba_error_init, sfba_error_final, sfba_n_edges_final);
      SVO_DEBUG_STREAM("PoseOptimizer:\t ErrInit = "<<sfba_error_init<<"px\t thresh = "<<sfba_thresh);
      SVO_DEBUG_STREAM("PoseOptimizer:\t ErrFin. = "<<sfba_error_final<<"px\t nObsFin. = "<<sfba_n_edges_final);
      if(sfba_n_edges_final < 20)
        return RESULT_FAILURE;

      // structure optimization
      SVO_START_TIMER("point_optimizer");
      optimizeStructure(new_frame_, Config::structureOptimMaxPts(), Config::structureOptimNumIter());
      SVO_STOP_TIMER("point_optimizer");

      // select keyframe
      core_kfs_.insert(new_frame_);
      setTrackingQuality(sfba_n_edges_final);
      if(tracking_quality_ == TRACKING_INSUFFICIENT)
      {
        new_frame_->T_f_w_ = last_frame_->T_f_w_; // reset to avoid crazy pose jumps
        return RESULT_FAILURE;
      }
      double depth_mean, depth_min;
      frame_utils::getSceneDepth(*new_frame_, depth_mean, depth_min);
      if(!needNewKf(depth_mean) || tracking_quality_ == TRACKING_BAD)
      {
        depth_filter_->addFrame(new_frame_);
        return RESULT_NO_KEYFRAME;
      }
      new_frame_->setKeyframe();
      SVO_DEBUG_STREAM("New keyframe selected.");

      // new keyframe selected
      for(Features::iterator it=new_frame_->fts_.begin(); it!=new_frame_->fts_.end(); ++it)
        if((*it)->point != NULL)
          (*it)->point->addFrameRef(*it);
      map_.point_candidates_.addCandidatePointToFrame(new_frame_);

      // optional bundle adjustment
    #ifdef USE_BUNDLE_ADJUSTMENT
      if(Config::lobaNumIter() > 0)
      {
        SVO_START_TIMER("local_ba");
        setCoreKfs(Config::coreNKfs());
        size_t loba_n_erredges_init, loba_n_erredges_fin;
        double loba_err_init, loba_err_fin;
        ba::localBA(new_frame_.get(), &core_kfs_, &map_,
                    loba_n_erredges_init, loba_n_erredges_fin,
                    loba_err_init, loba_err_fin);
        SVO_STOP_TIMER("local_ba");
        SVO_LOG4(loba_n_erredges_init, loba_n_erredges_fin, loba_err_init, loba_err_fin);
        SVO_DEBUG_STREAM("Local BA:\t RemovedEdges {"<<loba_n_erredges_init<<", "<<loba_n_erredges_fin<<"} \t "
                         "Error {"<<loba_err_init<<", "<<loba_err_fin<<"}");
      }
    #endif

      // init new depth-filters
      depth_filter_->addKeyframe(new_frame_, depth_mean, 0.5*depth_min);

      // if limited number of keyframes, remove the one furthest apart
      if(Config::maxNKfs() > 2 && map_.size() >= Config::maxNKfs())
      {
        FramePtr furthest_frame = map_.getFurthestKeyframe(new_frame_->pos());
        depth_filter_->removeKeyframe(furthest_frame); // TODO this interrupts the mapper thread, maybe we can solve this better
        map_.safeDeleteFrame(furthest_frame);
      }

      // add keyframe to map
      map_.addKeyframe(new_frame_);

      return RESULT_IS_KEYFRAME;
    }
  
// 作用是在相关位置重定位帧以提供关键帧 =================================================
    FrameHandlerMono::UpdateResult FrameHandlerMono::relocalizeFrame(
        const SE3& T_cur_ref,
        FramePtr ref_keyframe)
    {
      SVO_WARN_STREAM_THROTTLE(1.0, "Relocalizing frame");
      if(ref_keyframe == nullptr)
      {
        SVO_INFO_STREAM("No reference keyframe.");
        return RESULT_FAILURE;
      }
      SparseImgAlign img_align(Config::kltMaxLevel(), Config::kltMinLevel(),
                               30, SparseImgAlign::GaussNewton, false, false);
      size_t img_align_n_tracked = img_align.run(ref_keyframe, new_frame_);
      if(img_align_n_tracked > 30)
      {
        SE3 T_f_w_last = last_frame_->T_f_w_;
        last_frame_ = ref_keyframe;
        FrameHandlerMono::UpdateResult res = processFrame();
        if(res != RESULT_FAILURE)
        {
          stage_ = STAGE_DEFAULT_FRAME;
          SVO_INFO_STREAM("Relocalization successful.");
        }
        else
          new_frame_->T_f_w_ = T_f_w_last; // reset to last well localized pose
        return res;
      }
      return RESULT_FAILURE;
    }
  
  
// ========================================================
    bool FrameHandlerMono::relocalizeFrameAtPose(
        const int keyframe_id,
        const SE3& T_f_kf,
        const cv::Mat& img,
        const double timestamp)
    {
      FramePtr ref_keyframe;
      if(!map_.getKeyframeById(keyframe_id, ref_keyframe))
        return false;
      new_frame_.reset(new Frame(cam_, img.clone(), timestamp));
      UpdateResult res = relocalizeFrame(T_f_kf, ref_keyframe);
      if(res != RESULT_FAILURE) {
        last_frame_ = new_frame_;
        return true;
      }
      return false;
    }
  
// 重置 =============================================================
    void FrameHandlerMono::resetAll()
    {
      resetCommon();
      last_frame_.reset();
      new_frame_.reset();
      core_kfs_.clear();
      overlap_kfs_.clear();
      depth_filter_->reset();
    }
  
// 设置第一帧为关键帧 ==========================================================
    void FrameHandlerMono::setFirstFrame(const FramePtr& first_frame)
    {
      resetAll();
      last_frame_ = first_frame;
      last_frame_->setKeyframe();
      map_.addKeyframe(last_frame_);
      stage_ = STAGE_DEFAULT_FRAME;
    }
  
  
// 根据距离上次关键帧的位移量 判断是否需要新建关键帧 =============================================
    bool FrameHandlerMono::needNewKf(double scene_depth_mean)
    {
      for(auto it=overlap_kfs_.begin(), ite=overlap_kfs_.end(); it!=ite; ++it)
      {
        Vector3d relpos = new_frame_->w2f(it->first->pos());
        if(fabs(relpos.x())/scene_depth_mean < Config::kfSelectMinDist() &&
           fabs(relpos.y())/scene_depth_mean < Config::kfSelectMinDist()*0.8 &&
           fabs(relpos.z())/scene_depth_mean < Config::kfSelectMinDist()*1.3)
          return false;
      }
      return true;
    }
  
// CoreKfs 用于表示一帧周围的关键帧 =========================================
    void FrameHandlerMono::setCoreKfs(size_t n_closest)
    {
      size_t n = min(n_closest, overlap_kfs_.size()-1);
      std::partial_sort(overlap_kfs_.begin(), overlap_kfs_.begin()+n, overlap_kfs_.end(),
                        boost::bind(&pair<FramePtr, size_t>::second, _1) >
                        boost::bind(&pair<FramePtr, size_t>::second, _2));
      std::for_each(overlap_kfs_.begin(), overlap_kfs_.end(), [&](pair<FramePtr,size_t>& i){ core_kfs_.insert(i.first); });
    }

} // namespace svo
