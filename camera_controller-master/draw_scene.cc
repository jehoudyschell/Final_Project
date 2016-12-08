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

// Use the right namespace for google flags (gflags).
#ifdef GFLAGS_NAMESPACE_GOOGLE
#define GLUTILS_GFLAGS_NAMESPACE google
#else
#define GLUTILS_GFLAGS_NAMESPACE gflags
#endif

// Include first C-Headers.
#define _USE_MATH_DEFINES  // For using M_PI.
#include <cmath>
// Include second C++-Headers.
#include <iostream>
#include <string>
#include <vector>

// Include library headers.
// The macro below tells the linker to use the GLEW library in a static way.
// This is mainly for compatibility with Windows.
// Glew is a library that "scans" and knows what "extensions" (i.e.,
// non-standard algorithms) are available in the OpenGL implementation in the
// system. This library is crucial in determining if some features that our
// OpenGL implementation uses are not available.
#define GLEW_STATIC
#include <GL/glew.h>
// The header of GLFW. This library is a C-based and light-weight library for
// creating windows for OpenGL rendering.
// See http://www.glfw.org/ for more information.
#include <GLFW/glfw3.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gflags/gflags.h>
#include <glog/logging.h>

// Include system headers.
#include "shader_program.h"
#include "model_loader.h"
#include "camera.h"
#include "camera_controller.h"

// Google flags.
// (<name of the flag>, <default value>, <Brief description of flat>)
// These will define global variables w/ the following format
// FLAGS_vertex_shader_filepath and 
// FLAGS_fragment_shader_filepath.
DEFINE_string(vertex_shader_filepath, "", 
              "Filepath of the vertex shader.");
DEFINE_string(fragment_shader_filepath, "",
              "Filepath of the fragment shader.");
DEFINE_string(model_filepath, "", "Filepath of the model to load.");

// Annonymous namespace for constants and helper functions.
namespace {
// Window dimensions.
constexpr int kWindowWidth = 640;
constexpr int kWindowHeight = 480;

// Pointer to the camera controller.
static wvu::CameraController* camera_controller_ptr = nullptr;
// Pointer to the moving vector.
static std::vector<bool>* movement_vector_ptr = nullptr;

// ------------------------ User Input Callbacks -----------------------------
// Error callback function. This function follows the required signature of
// GLFW. See http://www.glfw.org/docs/3.0/group__error.html for more
// information.
static void ErrorCallback(int error, const char* description) {
  LOG(FATAL) << description;
}

void UpdateCameraPose() {
  // Camera position.
  if (movement_vector_ptr->at(GLFW_KEY_W)) {
    camera_controller_ptr->MoveFront();
  }
  if (movement_vector_ptr->at(GLFW_KEY_S)) {
    camera_controller_ptr->MoveBack();
  }
  if (movement_vector_ptr->at(GLFW_KEY_A)) {
    camera_controller_ptr->MoveLeft();
  }
  if (movement_vector_ptr->at(GLFW_KEY_D)) {
    camera_controller_ptr->MoveRight();
  }
}

// Keyboard event callback. See glfw documentation for information about
// parameters.
static void KeyCallback(GLFWwindow* window,
                        const int key,
                        const int scancode,
                        const int action,
                        const int mods) {
  if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
    glfwSetWindowShouldClose(window, GL_TRUE);
  }
  // Camera position.
  if (key >= 0 && key < 1024) {
    if (action == GLFW_PRESS) {
      movement_vector_ptr->at(key) = true;
    } else if (action == GLFW_RELEASE) {
      movement_vector_ptr->at(key) = false;
    }
  }
}

// Mouse events callback. See glfw documentation for information about
// parameters.
static void MouseCallback(GLFWwindow* window,
                          const double x_position,
                          const double y_position) {
  static int last_x_position;
  static int last_y_position;
  static bool first_call = true;
  // First call to this callback.
  if (first_call) {
    last_x_position = x_position;
    last_y_position = y_position;
    first_call = false;
  }
  // Offsets.
  camera_controller_ptr->AddYawOffset(
      camera_controller_ptr->rotation_sensitivity() *
      (x_position - last_x_position));
  camera_controller_ptr->AddPitchOffset(
      camera_controller_ptr->rotation_sensitivity() *
      (last_y_position - y_position));
  last_x_position = x_position;
  last_y_position = y_position;
}

static void ScrollCallback(GLFWwindow* window,
                           const double x_offset,
                           const double y_offset) {
  camera_controller_ptr->AdjustZoom(y_offset);
}

// ------------------------ End of User Input Callbacks ----------------------

// Class that will help us keep the state of any model more easily.
class Model {
public:
  // Constructor.
  // Params
  //  orientation  Axis of rotation whose norm is the angle
  //     (aka Rodrigues vector).
  //  position  The position of the object in the world.
  //  vertices  The vertices forming the object.
  //  indices  The sequence indicating how to use the vertices.
  Model(const Eigen::Vector3f& orientation,
        const Eigen::Vector3f& position,
        const Eigen::MatrixXf& vertices,
        const std::vector<GLuint>& indices) {
    orientation_ = orientation;
    position_ = position;
    vertices_ = vertices;
    indices_ = indices;
  }

  // Constructor.
  // Params
  //  orientation  Axis of rotation whose norm is the angle
  //     (aka Rodrigues vector).
  //  position  The position of the object in the world.
  //  vertices  The vertices forming the object.
  Model(const Eigen::Vector3f& orientation,
        const Eigen::Vector3f& position,
        const Eigen::MatrixXf& vertices) {
    orientation_ = orientation;
    position_ = position;
    vertices_ = vertices;
  }
  // Default destructor.
  ~Model() {}

  // Setters set members by *copying* input parameters.
  void SetOrientation(const Eigen::Vector3f& orientation) {
    orientation_ = orientation;
  }

  void SetPosition(const Eigen::Vector3f& position);

  // If we want to avoid copying, we can return a pointer to
  // the member. Note that making public the attributes work
  // if we want to modify directly the members. However, this
  // is a matter of design.
  Eigen::Vector3f* mutable_orientation() {
    return &orientation_;
  }

  Eigen::Vector3f* mutable_position() {
    return &position_;
  }

  // Getters, return a const reference to the member.
  const Eigen::Vector3f& GetOrientation() {
    return orientation_;
  }

  const Eigen::Vector3f& GetPosition() {
    return position_;
  }

  const Eigen::MatrixXf& vertices() const {
    return vertices_;
  }

  const std::vector<GLuint>& indices() const {
    return indices_;
  }

private:
  // Attributes.
  // The convention we will use is to define a '_' after the name
  // of the attribute.
  Eigen::Vector3f orientation_;
  Eigen::Vector3f position_;
  Eigen::MatrixXf vertices_;
  std::vector<GLuint> indices_;
};

// Implements the setter for position. Note that the class somehow defines
// a namespace.
void Model::SetPosition(const Eigen::Vector3f& position) {
  position_ = position;
}

// -------------------- Helper Functions ----------------------------------
Eigen::Matrix4f ComputeTranslation(
  const Eigen::Vector3f& offset) {
  Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
  transformation.col(3) = offset.homogeneous();
  return transformation;
}

Eigen::Matrix4f ComputeRotation(const Eigen::Vector3f& axis,
                                const GLfloat angle) {
  Eigen::AngleAxisf rotation(angle, axis);
  Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
  Eigen::Matrix3f rot3 = rotation.matrix();
  transformation.block(0, 0, 3, 3)  = rot3;
  return transformation;
}
// -------------------- End of Helper Functions --------------------------------

// Configures glfw.
void SetWindowHints() {
  // Sets properties of windows and have to be set before creation.
  // GLFW_CONTEXT_VERSION_{MAJOR|MINOR} sets the minimum OpenGL API version
  // that this program will use.
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
  // Sets the OpenGL profile.
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
  // Sets the property of resizability of a window.
  glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
}

// Configures the view port.
// Note: All the OpenGL functions begin with gl, and all the GLFW functions
// begin with glfw. This is because they are C-functions -- C does not have
// namespaces.
void ConfigureViewPort(GLFWwindow* window) {
  int width;
  int height;
  // We get the frame buffer dimensions and store them in width and height.
  glfwGetFramebufferSize(window, &width, &height);
  // Tells OpenGL the dimensions of the window and we specify the coordinates
  // of the lower left corner.
  glViewport(0, 0, width, height);
}

// Clears the frame buffer.
void ClearTheFrameBuffer() {
  // Sets the initial color of the framebuffer in the RGBA, R = Red, G = Green,
  // B = Blue, and A = alpha.
  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
  // Tells OpenGL to clear the Color buffer.
  glClear(GL_COLOR_BUFFER_BIT);
}

GLuint SetElementBufferObject(const Model& model) {
  // Creating element buffer object (EBO).
  GLuint element_buffer_object_id;
  glGenBuffers(1, &element_buffer_object_id);
  // Set the created EBO as current.
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, element_buffer_object_id);
  const std::vector<GLuint>& indices = model.indices();
  const int indices_size_in_bytes = indices.size() * sizeof(indices[0]);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER,
               indices_size_in_bytes,
               indices.data(),
               GL_STATIC_DRAW);
  // NOTE: Do not unbing EBO. It turns out that when we create a buffer of type
  // GL_ELEMENT_ARRAY_BUFFER, the VAO who contains the EBO remembers the
  // bindings we perform. Thus if we unbind it, we detach the created EBO and we
  // won't see results.
  return element_buffer_object_id;
}

// Creates and transfers the vertices into the GPU. Returns the vertex buffer
// object id.
GLuint SetVertexBufferObject(const Model& model) {
  // Create a vertex buffer object (VBO).
  GLuint vertex_buffer_object_id;
  glGenBuffers(1, &vertex_buffer_object_id);
  // Set the GL_ARRAY_BUFFER of OpenGL to the vbo we just created.
  glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_object_id);
  // Copy the vertices into the GL_ARRAY_BUFFER that currently 'points' to our
  // recently created vbo. In this case, sizeof(vertices) returns the size of
  // the array vertices (defined above) in bytes.
  // First parameter specifies the destination buffer.
  // Second parameter specifies the size of the buffer.
  // Third parameter specifies the pointer to the vertices buffer in RAM.
  // Fourth parameter specifies the way we want OpenGL to treat the buffer.
  // There are three different ways to treat this buffer:
  // 1. GL_STATIC_DRAW: the data will change very rarely.
  // 2. GL_DYNAMIC_DRAW: the data will likely change.
  // 3. GL_STREAM_DRAW: the data will change every time it is drawn.
  // See https://www.opengl.org/sdk/docs/man/html/glBufferData.xhtml.
  const Eigen::MatrixXf& vertices = model.vertices();
  const int vertices_size_in_bytes =
      vertices.rows() * vertices.cols() * sizeof(vertices(0, 0));
  glBufferData(GL_ARRAY_BUFFER,
               vertices_size_in_bytes,
               vertices.data(),
               GL_STATIC_DRAW);
  // Inform OpenGL how the vertex buffer is arranged.
  constexpr GLuint kIndex = 0;  // Index of the first buffer array.
  // A vertex right now contains 3 elements because we have x, y, z. But we can
  // add more information per vertex as we will see shortly.
  constexpr GLuint kNumElementsPerVertex = 3;
  constexpr GLuint kStride = kNumElementsPerVertex * sizeof(vertices(0, 0));
  const GLvoid* offset_ptr = nullptr;
  glVertexAttribPointer(kIndex, kNumElementsPerVertex, GL_FLOAT, GL_FALSE,
                        kStride, offset_ptr);
  // Set as active our newly generated VBO.
  glEnableVertexAttribArray(kIndex);
  // Unbind buffer so that later we can use it.
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  return vertex_buffer_object_id;
}

// Creates and sets the vertex array object (VAO) for our triangle. Returns the
// id of the created VAO.
void SetVertexArrayObject(const Model& model,
                          GLuint* vertex_buffer_object_id,
                          GLuint* vertex_array_object_id,
                          GLuint* element_buffer_object_id) {
  // Create the vertex array object (VAO).
  constexpr int kNumVertexArrays = 1;
  // This function creates kNumVertexArrays vaos and stores the ids in the
  // array pointed by the second argument.
  glGenVertexArrays(kNumVertexArrays, vertex_array_object_id);
  // Set the recently created vertex array object (VAO) current.
  glBindVertexArray(*vertex_array_object_id);
  // Create the Vertex Buffer Object (VBO).
  *vertex_buffer_object_id = SetVertexBufferObject(model);
  *element_buffer_object_id = SetElementBufferObject(model);
  // Disable our created VAO.
  glBindVertexArray(0);
}

// Renders the scene.
void RenderScene(const wvu::ShaderProgram& shader_program,
                 const GLuint vertex_array_object_id,
                 wvu::CameraController* camera_controller_ptr,
                 const GLfloat angle,
                 const int num_indices,
                 GLFWwindow* window) {
  // Clear the buffer.
  ClearTheFrameBuffer();
  // Let OpenGL know that we want to use our shader program.
  shader_program.Use();
  // Get the locations of the uniform variables.
  const GLint model_location = 
      glGetUniformLocation(shader_program.shader_program_id(), "model");
  const GLint view_location = 
      glGetUniformLocation(shader_program.shader_program_id(), "view");
  const GLint projection_location = 
      glGetUniformLocation(shader_program.shader_program_id(), "projection");
  const GLint vertex_color_location =
      glGetUniformLocation(shader_program.shader_program_id(), "vertex_color");
  Eigen::Matrix4f translation = 
    ComputeTranslation(Eigen::Vector3f(0.0f, -2.0f, -8.0f));
  Eigen::Matrix4f rotation = 
      ComputeRotation(Eigen::Vector3f(0.0, 1.0, 0.0f).normalized(), angle);
  Eigen::Matrix4f model = translation * rotation;
  const Eigen::Matrix4f& projection =
      camera_controller_ptr->GetProjectionMatrix();
  const Eigen::Matrix4f& view =
      camera_controller_ptr->UpdatePose();
  // We do not create the projection matrix here because the projection 
  // matrix does not change.
  glUniformMatrix4fv(model_location, 1, GL_FALSE, model.data());
  glUniformMatrix4fv(view_location, 1, GL_FALSE, view.data());
  glUniformMatrix4fv(projection_location, 1, GL_FALSE, projection.data());
  // Send color.
  Eigen::Vector4f color(0.5f, 0.5f, 0.5f, 1.0f);
  glUniform4fv(vertex_color_location, 1, color.data());
  // Draw the triangle.
  // Let OpenGL know what vertex array object we will use.
  glBindVertexArray(vertex_array_object_id);
  // Set to GL_LINE instead of GL_FILL to visualize the poligons as wireframes.
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  // Using EBOs.
  glDrawElements(GL_TRIANGLES, num_indices, GL_UNSIGNED_INT, 0);
  
  // Let OpenGL know that we are done with our vertex array object.
  glBindVertexArray(0);
}

}  // namespace

int main(int argc, char** argv) {
  GLUTILS_GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  // Initialize the GLFW library.
  if (!glfwInit()) {
    return -1;
  }

  // Setting the error callback.
  glfwSetErrorCallback(ErrorCallback);

  // Setting Window hints.
  SetWindowHints();

  // Create a window and its OpenGL context.
  const std::string window_name = "Hello Triangle";
  GLFWwindow* window = glfwCreateWindow(kWindowWidth,
                                        kWindowHeight,
                                        window_name.c_str(),
                                        nullptr,
                                        nullptr);
  if (!window) {
    glfwTerminate();
    return -1;
  }

  // Make the window's context current.
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);
  // Set up the callbacks for user input.
  glfwSetKeyCallback(window, KeyCallback);
  glfwSetCursorPosCallback(window, MouseCallback);  
  glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
  glfwSetScrollCallback(window, ScrollCallback); 

  // Initialize GLEW.
  glewExperimental = GL_TRUE;
  if (glewInit() != GLEW_OK) {
    LOG(FATAL) << "Glew did not initialize properly!";
    glfwTerminate();
    return -1;
  }

  // Configure View Port.
  ConfigureViewPort(window);

  // Compile shaders and create shader program.
  const std::string vertex_shader_filepath = FLAGS_vertex_shader_filepath;
  const std::string fragment_shader_filepath = FLAGS_fragment_shader_filepath;
  wvu::ShaderProgram shader_program;
  LOG(INFO) << "Attempting to load vertex shader from: "
            << vertex_shader_filepath;
  shader_program.LoadVertexShaderFromFile(vertex_shader_filepath);
  LOG(INFO) << "Attempting to load fragment shader from: "
            << fragment_shader_filepath;
  shader_program.LoadFragmentShaderFromFile(fragment_shader_filepath);
  std::string error_info_log;
  if (!shader_program.Create(&error_info_log)) {
    LOG(INFO) << "ERROR: " << error_info_log << "\n";
  }
  // TODO(vfragoso): Implement me!
  if (!shader_program.shader_program_id()) {
    LOG(FATAL) << "ERROR: Could not create a shader program.\n";
    return -1;
  }

  // Load model.
  std::vector<Eigen::Vector3f> model_vertices;
  std::vector<Eigen::Vector2f> model_texels;
  std::vector<Eigen::Vector3f> model_normals;
  std::vector<wvu::Face> model_faces;
  if (!wvu::LoadObjModel(FLAGS_model_filepath,
                         &model_vertices,
                         &model_texels,
                         &model_normals,
                         &model_faces)) {
    LOG(ERROR) << "Could not load model: " << FLAGS_model_filepath;
    return -1;
  }
  LOG(INFO) << "Model succesfully loaded! "
            << " Num. Vertices=" << model_vertices.size()
            << " Num. Triangles=" << model_faces.size();
  // Prepare buffers to hold the vertices in GPU.
  GLuint vertex_buffer_object_id;
  GLuint vertex_array_object_id;
  GLuint element_buffer_object_id;
  Eigen::MatrixXf vertices(3, model_vertices.size());
  for (int col = 0; col < model_vertices.size(); ++col) {
    vertices.col(col) = model_vertices[col];
  }
  std::vector<GLuint> indices;
  for (int face_id = 0; face_id < model_faces.size(); ++face_id) {
    const wvu::Face& face = model_faces[face_id];
    indices.push_back(face.vertex_indices[0]);
    indices.push_back(face.vertex_indices[1]);
    indices.push_back(face.vertex_indices[2]);
  }
  Model model(Eigen::Vector3f(0, 0, 0),  // Orientation of object.
              Eigen::Vector3f(0, 0, 0),  // Position of object.
              vertices,
              indices);
  SetVertexArrayObject(model,
                       &vertex_buffer_object_id,
                       &vertex_array_object_id,
                       &element_buffer_object_id);

  // Create projection matrix.
  wvu::CameraParameters camera_params;
  camera_params.field_of_view = M_PI * 45.0f / 180.0f;
  camera_params.aspect_ratio =
      static_cast<GLfloat>(kWindowWidth) / kWindowHeight;
  camera_params.near_plane_distance = 0.1f;
  camera_params.far_plane_distance = 20.0f;
  camera_params.position = Eigen::Vector3f(0, 0, 0);  // Origin of the world.
  camera_params.view_direction = -Eigen::Vector3f::UnitZ();  // Looking at -z.
  camera_params.up_vector = Eigen::Vector3f::UnitY();  // Up vector.

  // Creating the camera controller.
  wvu::CameraControllerParams camera_controller_params;
  wvu::CameraController camera_controller(camera_controller_params,
                                          camera_params);
  const wvu::ControllerInitializationError error =
      camera_controller.Initialize();
  if (error != wvu::ControllerInitializationError::NO_ERROR) {
    glfwTerminate();
    LOG(ERROR) << "Failed at initializing camera controller: "
               << static_cast<int>(error);
    return -1;
  }
  // Setting the camera controller pointer.
  camera_controller_ptr = &camera_controller;

  // Movement vector.
  movement_vector_ptr = camera_controller.mutable_movement_vector();

  // Angle of rotation for model.
  GLfloat angle = 0.0f;  // State of rotation.

  // Loop until the user closes the window.
  const GLfloat rotation_speed = 50.0f;
  while (!glfwWindowShouldClose(window)) {
    // Render the scene!
    UpdateCameraPose();
    // Casting using (<type>) -- which is the C way -- is not recommended.
    // Instead, use static_cast<type>(input argument).
    angle = rotation_speed * static_cast<GLfloat>(glfwGetTime()) * M_PI / 180.f;
    RenderScene(shader_program, vertex_array_object_id, 
                camera_controller_ptr, angle, indices.size(), window);

    // Swap front and back buffers.
    glfwSwapBuffers(window);

    // Poll for and process events.
    glfwPollEvents();
  }

  // Cleaning up tasks.
  glDeleteVertexArrays(1, &vertex_array_object_id);
  glDeleteBuffers(1, &vertex_buffer_object_id);
  // Destroy window.
  glfwDestroyWindow(window);
  // Tear down GLFW library.
  glfwTerminate();

  // Reseting camera controller pointer.
  camera_controller_ptr = nullptr;
  movement_vector_ptr = nullptr;

  return 0;
}
