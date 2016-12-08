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

#include "camera_controller.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <algorithm>
#include <Eigen/Core>
#include <GL/glew.h>
#include <glog/logging.h>

namespace wvu {
namespace {
// Mathematical constants.
constexpr GLfloat kPi = static_cast<GLfloat>(M_PI);
constexpr GLfloat kDegrees2RadiansRate = kPi / 180.0f;

inline GLfloat Degrees2Radians(const GLfloat degrees) {
  return kDegrees2RadiansRate * degrees;
}

}  // namespace

ControllerInitializationError CameraController::Initialize() {
  if (camera_speed_ < 0.0f) {
    VLOG(1) << "Invalid Camera Speed";
    return ControllerInitializationError::INVALID_CAMERA_SPEED;
  }
  if (rotation_sensitivity_ < 0.0f) {
    VLOG(1) << "Invalid Camera rot sens";
    return
        ControllerInitializationError::INVALID_CAMERA_ROTATION_SENSITIVITY;
  }
  if (num_keys_ < 1) {
    VLOG(1) << "Invalid Camera num keys";
    return ControllerInitializationError::INVALID_NUM_KEYS;
  }
  if (zoom_scale_factor_ <= 0.0f) {
    VLOG(1) << "Invalid Camera scale factor";
    return ControllerInitializationError::INVALID_ZOOM_SCALE_FACTOR;
  }
  // Resizing the movement vector with num_keys elements.
  movement_vector_.resize(num_keys_, false);
  // Initialize camera.
  return static_cast<ControllerInitializationError>(
      camera_.Initialize());
}

void CameraController::MoveFront() {
  position_offset_ += camera_speed_ * camera_.view_direction();
}

void CameraController::MoveBack() {
  position_offset_ -= camera_speed_ * camera_.view_direction();
}

void CameraController::MoveLeft() {
  position_offset_ -= camera_speed_ * camera_.GetXAxis();
}

void CameraController::MoveRight() {
  position_offset_ += camera_speed_ * camera_.GetXAxis();
}

void CameraController::AdjustZoom(
    const GLfloat field_of_view_offset) {
  // Clamp the field of view to be in the range of [1, 45].
  const GLfloat field_of_view = camera_.field_of_view() +
      zoom_scale_factor_ * field_of_view_offset;
  camera_.set_field_of_view(std::min(std::max(field_of_view, 44.0f), 45.0f));
  camera_.ComputeProjectionMatrix();
}

const Matrix4f& CameraController::UpdatePose() {
  // Compute new view direction.
  const Vector3f new_z_camera_axis(
      cos(Degrees2Radians(pitch_)) * cos(Degrees2Radians(yaw_)),
      sin(Degrees2Radians(pitch_)),
      cos(Degrees2Radians(pitch_)) * sin(Degrees2Radians(yaw_)));
  // Form a relative pose transformation.
  const Matrix4f& new_pose =
      camera_.SetViewDirectionAndMove(-new_z_camera_axis, position_offset_);
  // Reset offsets.
  position_offset_.setZero();
  return new_pose;
}

}  // namespace wvu
