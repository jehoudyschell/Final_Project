// Copyright (C) 2016 West Virginia University.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials provided
//       with the distribution.
//
//     * Neither the name of West Virginia University nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Please contact the author of this library if you have any questions.
// Author: Victor Fragoso (victor.fragoso@mail.wvu.edu)

#include "camera.h"

#define _USE_MATH_DEFINES  // For using M_PI.
#include <math.h>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <GL/glew.h>
#include <glog/logging.h>

namespace wvu {
using Eigen::Matrix4f;
using Eigen::Vector3f;
namespace {
// Creates a 4x4 translation matrix from a translation vector.
// Parameters:
//   translation  The translation vector.
inline Matrix4f
CreateTranslationMatrix(const Vector3f& translation) {
  Matrix4f transformation = Matrix4f::Identity();
  transformation.block<3, 1>(0, 3) = translation;
  return transformation;
}

Matrix4f CreateLookAtMatrix(const Vector3f& position,
                            const Vector3f& view_direction,
                            const Vector3f& up_vector) {
  Matrix4f rotation = Matrix4f::Identity();
  rotation.block<1, 3>(2, 0) = -view_direction;
  rotation.block<1, 3>(0, 0) = view_direction.cross(up_vector);
  rotation.block<1, 3>(1, 0) =
      -view_direction.cross(rotation.block<1, 3>(0, 0));
  return (rotation * CreateTranslationMatrix(-position));
}

// Mathematical constants. The right way to get PI in C++ is to use the
// macro M_PI. To do so, we have to include math.h or cmath and define
// the _USE_MATH_DEFINES macro to enable these constants. See the header
// section.
constexpr GLfloat kHalfPi = 0.5f * static_cast<GLfloat>(M_PI);

// Compute cotangent. Since C++ does not provide cotangent, we implement it
// as follows. Recall that cotangent is essentially tangent flipped and
// translated 90 degrees (or PI / 2 radians). To do the flipping and translation
// we have to do PI / 2 - angle. Subtracting the angle flips the curve.
// See the plots for http://mathworld.wolfram.com/Cotangent.html and
// http://mathworld.wolfram.com/Tangent.html
inline GLfloat ComputeCotangent(const GLfloat angle) {
  return tan(kHalfPi - angle);
}

bool CreateProjectionMatrix(const GLfloat field_of_view,
                            const GLfloat aspect_ratio,
                            const GLfloat near_plane_distance,
                            const GLfloat far_plane_distance,
                            Matrix4f* projection_matrix) {
  // Create the projection matrix.
  const GLfloat y_scale = ComputeCotangent(0.5f * field_of_view);
  const GLfloat x_scale = y_scale / aspect_ratio;
  const GLfloat planes_distance = far_plane_distance - near_plane_distance;
  const GLfloat z_scale =
      -(near_plane_distance + far_plane_distance) / planes_distance;
  const GLfloat homogeneous_scale =
      -2 * near_plane_distance * far_plane_distance / planes_distance;
  *projection_matrix << x_scale, 0.0f, 0.0f, 0.0f,
      0.0f, y_scale, 0.0f, 0.0f,
      0.0f, 0.0f, z_scale, homogeneous_scale,
      0.0f, 0.0f, -1.0f, 0.0f;
  return true;
}

}  // namespace

CameraInitializationError Camera::Initialize() {
  // Initialize projection matrix.
  if (!CreateProjectionMatrix(field_of_view_,
                              aspect_ratio_,
                              near_plane_distance_,
                              far_plane_distance_,
                              &projection_)) {
    VLOG(1) << "Invalid proj. params";    
    return CameraInitializationError::INVALID_PROJECTION_PARAMETERS;
  }
  // Initialize camera pose.
  if (view_direction_.norm() == 0.0f) {
    VLOG(1) << "Invalid view direction";    
    return CameraInitializationError::INVALID_VIEW_DIRECTION;
  }
  if (up_vector_.norm() == 0.0f) {
    VLOG(1) << "Invalid up vector";    
    return CameraInitializationError::INVALID_UP_VECTOR;
  }
  // Normalize view direction and up_vector.
  view_direction_.normalize();
  up_vector_.normalize();
  // Verify that they are not colinear.
  if (fabs(view_direction_.dot(up_vector_)) == 1.0f) {
    VLOG(1) << "Invalid view direction and up vector";    
    return CameraInitializationError::INVALID_VIEW_DIRECTION_AND_UP_VECTOR;
  }
  look_at_ = CreateLookAtMatrix(position_, view_direction_, up_vector_);
  return CameraInitializationError::NO_ERROR;
}

const Matrix4f& Camera::ComputeProjectionMatrix() {
  CreateProjectionMatrix(field_of_view_,
                         aspect_ratio_,
                         near_plane_distance_,
                         far_plane_distance_,
                         &projection_);
  return projection_;
}

void Camera::UpdatePose() {
  view_direction_ = -look_at_.block<1, 3>(2, 0);
  position_ = -look_at_.block<3, 1>(0, 3);
}

const Matrix4f& Camera::ComputeLookAtMatrix() {
  look_at_ = CreateLookAtMatrix(position_, view_direction_, up_vector_);
  return look_at_;
}

const Matrix4f& Camera::Rotate(const AngleAxisf& rotation_offset) {
  Matrix4f rotation_matrix = Matrix4f::Identity();
  rotation_matrix.block<3, 3>(0, 0) = rotation_offset.matrix();
  look_at_ = rotation_matrix * look_at_;
  UpdatePose();
  return look_at_;
}

const Matrix4f& Camera::Move(const Vector3f& position_offset) {
  Matrix4f translation_matrix = Matrix4f::Identity();
  translation_matrix.block<3, 1>(0, 3) = -position_offset;
  look_at_ = translation_matrix * look_at_;
  UpdatePose();
  return look_at_;
}

const Matrix4f& Camera::RotateAndMove(const AngleAxisf& rotation_offset,
                                      const Vector3f& position_offset) {
  Matrix4f transformation = Matrix4f::Identity();
  transformation.block<3, 3>(0, 0) = rotation_offset.matrix();
  transformation.block<3, 1>(0, 3) = -position_offset;
  look_at_ = transformation * look_at_;
  UpdatePose();
  return look_at_;
}

const Matrix4f& Camera::SetViewDirectionAndMove(
    const Vector3f& new_view_direction,
    const Vector3f& position_offset) {
  position_ += position_offset;
  view_direction_ = new_view_direction;
  view_direction_.normalize();
  look_at_ = CreateLookAtMatrix(position_, view_direction_, up_vector_);
  return look_at_;
}

}  // namespace wvu
