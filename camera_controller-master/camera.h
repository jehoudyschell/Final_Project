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

#ifndef CAMERA_H_
#define CAMERA_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <GL/glew.h>

namespace wvu {
using Eigen::AngleAxisf;
using Eigen::Matrix4f;
using Eigen::Vector3f;

enum struct CameraInitializationError {
  NO_ERROR = 0,
  INVALID_VIEW_DIRECTION = 1,
  INVALID_UP_VECTOR = 2,
  INVALID_VIEW_DIRECTION_AND_UP_VECTOR = 3,
  INVALID_PROJECTION_PARAMETERS = 4
};

// TODO(vfragoso): Document structure.
struct CameraParameters {
  // Field of view angle.
  GLfloat field_of_view;
  // Aspect ratio.
  GLfloat aspect_ratio;
  // Near plane distance (this is the projection plane).
  GLfloat near_plane_distance;
  // Far plane distance.
  GLfloat far_plane_distance;
  // Position of the camera.
  Vector3f position;
  // World origin of the global coordinate camera frame.
  Vector3f view_direction;
  // Up vector in the world coordinate system.
  Vector3f up_vector;
};

// This class allows to keep track of the necessary elements to compute a LookAt
// or view matrix (which mats world to camera coordinates) and a projection
// matrix. Also, it allows to calculate the view/look-at transformation once
// these elements are set. This class is convinient to create camera controllers
// for games (e.g., first person camera controller).
//
// To construct a camera instance, the constructor requires initial camera
// parameters (see struct CameraParameters above). This structure is used as
// follows:
//
// 1. position. This is the position of the camera described by a 3D vector with
// respect to the world coordinate system.
//
// 2. view_direction. This is the viewing direction, i.e., the direction at
// which the camera is looking at. Remember that OpenGL assumes that the Z-axis
// of the camera points towards the camera. In other words, the -Z-axis points
// towards the scene.
//
// 3. up_vector.  This is the vector pointing upwards with respect to the world.
// This vector typically is the unit vector along the y-axis.
//
// How is the view/look-at matrix computed?
// Since the view matrix transforms coordinates from world to the coordinate
// system of the camera, then the matrix must be of the form:
//
// T = | R  t|
//     | 0  1|,
//
// where R is the rotation that aligns the world with the camera coordinate
// systems, and t is the negative of the position of the camera.
//
// Pose initialization.
// --------------------
//
// Calculating the rotation matrix. Since the coordinate frame of the camera is
// orthogonal, then we can build it from information about the world coordinate
// system and the camera.
// First, the view_direction vector points in the -z axis of the camera -- this
// is how OpenGL defines the camera. Let z_camera denote the unit vector
// pointing towards the z-axis of the camera coordinate system. Then, we can
// leverage this information to construct an orthogonal coordinate system using
// cross product and information of the world coordinate system.
// If we compute -z_camera x up_vector, we obtain the vector pointing towards
// the x-axis of the camera. Let x_camera = -z_camera x up_vector. Then, the
// final unit vector along the missing axis is y_camera = -z_camera x x_camera.
// This defines a rotation transformation that can be build as follows:
//
//     | x_camera^T  |
// R = | y_camera^T  |
//     | -z_camera^T |,
//
// where the super script T indicates transposition.
//
// To compute t, we simply set t = -position_camera since it is an offset that
// we need to remove because the transformation has as its origin the position
// of the camera.
//
// Projection matrix.
// ------------------
// TODO(vfragoso): Document projection parameters.
class Camera {
 public:
  Camera(const CameraParameters& camera_params) :
      field_of_view_(camera_params.field_of_view),
      aspect_ratio_(camera_params.aspect_ratio),
      near_plane_distance_(camera_params.near_plane_distance),
      far_plane_distance_(camera_params.far_plane_distance),
      position_(camera_params.position),
      view_direction_(camera_params.view_direction),
      up_vector_(camera_params.up_vector),
      look_at_(Matrix4f::Identity()),
      projection_(Matrix4f::Identity()) {}
  virtual ~Camera() {}

  // Initializes/creates the view transformation matrix given the input
  // parameters. Returns true upon success and false otherwise.
  CameraInitializationError Initialize();

  // Returns the position of the camera wrt the world.
  const Vector3f& position() const {
    return position_;
  }

  // Returns the look-at direction wrt the world.
  const Vector3f& view_direction() const {
    return view_direction_;
  }

  // Returns the up-vector of the world coordinate system known to the camera.
  const Vector3f& up_vector() const {
    return up_vector_;
  }

  // Returns the current look_at/view matrix.
  const Matrix4f& look_at() const {
    return look_at_;
  }

  // Returns the projection matrix.
  const Matrix4f& projection() const {
    return projection_;
  }

  // Returns the field of view.
  GLfloat field_of_view() const {
    return field_of_view_;
  }

  // Returns the aspect ratio.
  GLfloat aspect_ratio() const {
    return aspect_ratio_;
  }

  // Returns the plane distance.
  GLfloat near_plane_distance() const {
    return near_plane_distance_;
  }

  // Returns the far plane distance.
  GLfloat far_plane_distance() const {
    return far_plane_distance_;
  }

  // Sets a new far plane distance only if it is bigger than the near plane
  // distance.
  void set_far_plane_distance(const GLfloat far_plane_distance) {
    if (near_plane_distance_ < far_plane_distance) {
      far_plane_distance_ = far_plane_distance;
    }
  }

  // Sets a new near plane distance only if it is bigger than zero.
  void set_near_plane_distance(const GLfloat near_plane_distance) {
    if (near_plane_distance > 0.0f) {
      near_plane_distance_ = near_plane_distance;
    }
  }

  // Sets a new aspect ratio only if it is bigger than zero.
  void set_aspect_ratio(const GLfloat aspect_ratio) {
    if (aspect_ratio > 0.0f) {
      aspect_ratio_ = aspect_ratio;
    }
  }

  // Sets a new field of view for the camera. Only updates it if the field of
  // view parameter is bigger than zero. Call update projection matrix to apply
  // the changes.
  void set_field_of_view(const GLfloat field_of_view) {
    if (field_of_view > 1) {
      field_of_view_ = field_of_view;
    }
  }

  // Sets the position of the camera wrt the world.
  void set_position(const Vector3f& new_position) {
    position_ = new_position;
  }

  // Sets the look-at direction wrt the world.
  void set_view_direction(const Vector3f& view_direction) {
    view_direction_ = view_direction;
  }

  // Sets the up vector of the world coordinate system.
  void set_up_vector(const Vector3f& up_vector) {
    up_vector_ = up_vector;
  }

  // Gets the x-camera axis from the current camera pose.
  Vector3f GetXAxis() {
    return look_at_.block<1, 3>(0, 0);
  }

  // Gets the y-camera axis from the current camera pose.
  Vector3f GetYAxis() {
    return look_at_.block<1, 3>(1, 0);
  }

  // Computes the projection matrix using the current state variables. Returns
  // the generated projection matrix.
  const Matrix4f& ComputeProjectionMatrix();

  // Computes the look-at transformation using the state of position,
  // view_direction, and up_vector.
  const Matrix4f& ComputeLookAtMatrix();

  // Rotates camera by applying the rotation offset transformation. This
  // function modifies the look-at direction, computes the new look-at matrix,
  // and returns it.
  // Params:
  //   rotation_offset  The rotation transformation offset.
  const Matrix4f& Rotate(const AngleAxisf& rotation_offset);

  // Resets camera position by applying a translation transformation using the
  // position offset, computes the new look-at matrix, and returns it.
  // Params:
  //   position_offset  The position offset.
  const Matrix4f& Move(const Vector3f& position_offset);

  // Rotates and resets the camera position, computes the new look-at matrix,
  // and returns it.
  // Params:
  //   rotation_offset  The rotation transformation offset.
  //   position_offset  The position offset.
  const Matrix4f& RotateAndMove(const AngleAxisf& angle_axis,
                                const Vector3f& position_offset);

  // Rotates and resets the camera position, computes the new look-at matrix,
  // and returns it. For rotation it uses a new view direction to build the
  // look-at matrix.
  // Params:
  //   new_view_direction  The new look-at direction wrt to the world.
  //   position_offset  The position offset.
  const Matrix4f& SetViewDirectionAndMove(
      const Vector3f& new_view_direction,
      const Vector3f& position_offset);

 private:
  // Field of view angle of the camera.
  GLfloat field_of_view_;
  // The aspect ratio of the camera.
  GLfloat aspect_ratio_;
  // The near plane distance.
  GLfloat near_plane_distance_;
  // The far plane distance.
  GLfloat far_plane_distance_;
  // Position of the camera.
  Vector3f position_;
  // World origin of the global coordinate camera frame.
  Vector3f view_direction_;
  // Up vector in the world coordinate system.
  Vector3f up_vector_;
  // Look at transformation: world to camera transformation.
  Matrix4f look_at_;
  // Projection matrix.
  Matrix4f projection_;

  // Update members after an update in pose (Move or Rotate).
  void UpdatePose();
};

}  // namespace wvu

#endif  // CAMERA_H_
