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

#ifndef CAMERA_CONTROLLER_H_
#define CAMERA_CONTROLLER_H_

#include <vector>
#include <Eigen/Core>
#include <GL/glew.h>

#include "camera.h"

namespace wvu {
// First person camera controller initialization error.
enum struct ControllerInitializationError {
  NO_ERROR = 0,
  INVALID_VIEW_DIRECTION = 1,
  INVALID_UP_VECTOR = 2,
  INVALID_VIEW_DIRECTION_AND_UP_VECTOR = 3,
  INVALID_PROJECTION_PARAMETERS = 4,
  INVALID_CAMERA_SPEED = 5,
  INVALID_CAMERA_ROTATION_SENSITIVITY = 6,
  INVALID_NUM_KEYS = 7,
  INVALID_ZOOM_SCALE_FACTOR = 8
};

// Initial parameters of the camera controller.
struct CameraControllerParams {
  // The default parameters point the camera seeing towards the negative
  // z-axis direction.
  CameraControllerParams() :
      camera_speed(0.05f),
      rotation_sensitivity(0.33f),
      yaw(-90.0f),
      pitch(-180.0f),
      num_keys(1024),
      zoom_scale_factor(0.1f) {}
  virtual ~CameraControllerParams() {}

  // The speed at which the camera moves.
  GLfloat camera_speed;
  // The scale factor mapping the mouse position to the rotation parameters.
  GLfloat rotation_sensitivity;
  // The initial yaw angle in degrees.
  GLfloat yaw;
  // The initial pitch angle in degrees.
  GLfloat pitch;
  // The number of keys that the controller will handle.
  int num_keys;
  // Zoom scale factor mapping the scrolling offets.
  GLfloat zoom_scale_factor;
};

// This class aims at controlling the camera pose in the virtual scene. The
// class characterizes the camera pose using yaw and pitch as rotation
// parameters in the world coordinate system and a 3-vector describing the
// global position of the camera. The controller allows a first-person-like
// movement within the scene.
//
// The controller functions as follows:
//
// - The callbacks collecting parameters to update position and rotation
// update the internal camera parameters of the controller.
//
// - Once the function that renders the scene is called, the controller updates
// the state of the camera. The rendered must call the UpdatePose() function
// to apply the changes.
//
// Usage example:
//
// void KeyboardCallback() {
//   if Key1 Pressed then
//     controller.MoveFront()
//   if Key2 Pressed then
//     controller.MoveBack()
// }
// ...
// void MouseCallback() {
//     ...
//     yaw_offset = OffsetToAngle(x_offset)
//     pitch_offset = OffsetToAngle(y_offset)
//     controller.AddYawOffset(yaw_offset);
//     controller.AddPitchOffset(pitch_offset);
//     ...
// }
//
// void RenderScene() {
//   ...
//   const MatrixGl4f& new_pose = controller.UpdatePose();
//   ...
// }
class CameraController {
 public:
  // Params:
  //   controler_params  The camera controller initial parameters (see
  //      CameraControllerParams above).
  //   camera_params  The camera parameters (see camera class in camera.h)
  CameraController(
      const CameraControllerParams& controller_params,
      const CameraParameters& camera_params) :
      camera_speed_(controller_params.camera_speed),
      rotation_sensitivity_(controller_params.rotation_sensitivity),
      num_keys_(controller_params.num_keys),
      camera_(camera_params),
      position_offset_(Vector3f::Zero()),
      yaw_(controller_params.yaw),
      pitch_(controller_params.pitch),
      zoom_scale_factor_(controller_params.zoom_scale_factor) {}
  virtual ~CameraController() {}

  // Initializes the camera and returns an ControllerInitializationError in
  // case of an usuccessful call.
  ControllerInitializationError Initialize();

  // Moves the camera towards the viewing direction.
  void MoveFront();

  // Moves the camera towards the reversed viewing direction.
  void MoveBack();

  // Moves the camera towards the reversed direction of the x-axis of the
  // camera.
  void MoveLeft();

  // Moves the camera towards the direction of the x-axis of the camera.
  void MoveRight();

  // Adjusts the zoom of the camera by changing the field of view (fov). When
  // the fov increases the scene is zoomed-out, and zoomed-in otherwise.
  // Params:
  //   field_of_view_offset  The field of view offset for the new projection
  //     matrix.
  void AdjustZoom(const GLfloat field_of_view_offset);

  // This method updates all the state changes of the camera in a single call.
  // In other words, it generates a new camera pose using the current state of
  // the controller.
  const Matrix4f& UpdatePose();

  // Returns the projection matrix of the camera.
  const Matrix4f& GetProjectionMatrix() {
    return camera_.projection();
  }

  // Returns the camera speed parameter.
  GLfloat camera_speed() const {
    return camera_speed_;
  }

  // Returns the rotation sensitivity parameter.
  GLfloat rotation_sensitivity() const {
    return rotation_sensitivity_;
  }

  // Returns the viewing direction.
  const Vector3f& view_direction() const {
    return camera_.view_direction();
  }

  // Returns the yaw angle.
  GLfloat yaw() const {
    return yaw_;
  }

  // Returns the pitch angle.
  GLfloat pitch() const {
    return pitch_;
  }

  // Returns the zoom scale factor.
  GLfloat zoom_scale_factor() const {
    return zoom_scale_factor_;
  }

  // Returns a const reference to the movement vector.
  const std::vector<bool>& movement_vector() const {
    return movement_vector_;
  }

  // Adds a yaw offset.
  // Params:
  //   yaw_offset  The yaw offset.
  void AddYawOffset(const GLfloat yaw_offset) {
    yaw_ += yaw_offset;
  }

  // Adds a pitch offset.
  // Params:
  //   pitch_offset  The pitch offset.
  void AddPitchOffset(const GLfloat pitch_offset) {
    pitch_ += pitch_offset;
  }

  // Returns a pointer to the movement vector.
  std::vector<bool>* mutable_movement_vector() {
    return &movement_vector_;
  }

  // Accessor to the camera.
  const Camera& camera() const {
    return camera_;
  }

 private:
  // The camera speed that allows to control the speed at which we update the
  // position of the camera.
  const GLfloat camera_speed_;
  // The sensitivity at which we update the euler angles to define a rotation
  // transformation that will allow to move the camera and look around the
  // world.
  const GLfloat rotation_sensitivity_;
  // Number of keys to handle.
  const int num_keys_;
  // Camera instance.
  Camera camera_;
  // Camera position offset.
  Vector3f position_offset_;
  // Rotation parameters wrt to world coordinate system.
  // Yaw angle in degrees.
  GLfloat yaw_;
  // Pitch angle in degrees.
  GLfloat pitch_;
  // The scale factor mapping the scroll offset to field of view changes.
  GLfloat zoom_scale_factor_;
  // Movement/position vector.
  std::vector<bool> movement_vector_;
};

}  // namespace wvu

#endif  // CAMERA_CONTROLLER_H_
