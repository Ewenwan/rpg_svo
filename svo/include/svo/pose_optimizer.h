// This file is part of SVO - Semi-direct Visual Odometry.
// 图优化（光束法平差最优化重投影误差）
#ifndef SVO_POSE_OPTIMIZER_H_
#define SVO_POSE_OPTIMIZER_H_

#include <svo/global.h>

namespace svo {

using namespace Eigen;
using namespace Sophus;
using namespace std;

typedef Matrix<double,6,6> Matrix6d;
typedef Matrix<double,2,6> Matrix26d;
typedef Matrix<double,6,1> Vector6d;

class Point;

/// Motion-only bundle adjustment. Minimize the reprojection error of a single frame.
namespace pose_optimizer {

void optimizeGaussNewton(
    const double reproj_thresh,
    const size_t n_iter,
    const bool verbose,
    FramePtr& frame,
    double& estimated_scale,
    double& error_init,
    double& error_final,
    size_t& num_obs);

} // namespace pose_optimizer
} // namespace svo

#endif // SVO_POSE_OPTIMIZER_H_
