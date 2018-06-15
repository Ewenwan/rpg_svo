// This file is part of SVO - Semi-direct Visual Odometry.
// 特征匹配

#ifndef SVO_FEATURE_ALIGNMENT_H_
#define SVO_FEATURE_ALIGNMENT_H_

#include <svo/global.h>

namespace svo {

/// Subpixel refinement of a reference feature patch with the current image.
/// Implements the inverse-compositional approach (see "Lucas-Kanade 20 Years on"
/// paper by Baker.
namespace feature_alignment {

bool align1D(
    const cv::Mat& cur_img,
    const Vector2f& dir,                  // direction in which the patch is allowed to move
    uint8_t* ref_patch_with_border,
    uint8_t* ref_patch,
    const int n_iter,
    Vector2d& cur_px_estimate,
    double& h_inv);

bool align2D(
    const cv::Mat& cur_img,
    uint8_t* ref_patch_with_border,
    uint8_t* ref_patch,
    const int n_iter,
    Vector2d& cur_px_estimate,
    bool no_simd = false);

bool align2D_SSE2(
    const cv::Mat& cur_img,
    uint8_t* ref_patch_with_border,
    uint8_t* ref_patch,
    const int n_iter,
    Vector2d& cur_px_estimate);

bool align2D_NEON(
    const cv::Mat& cur_img,
    uint8_t* ref_patch_with_border,
    uint8_t* ref_patch,
    const int n_iter,
    Vector2d& cur_px_estimate);

} // namespace feature_alignment
} // namespace svo

#endif // SVO_FEATURE_ALIGNMENT_H_
