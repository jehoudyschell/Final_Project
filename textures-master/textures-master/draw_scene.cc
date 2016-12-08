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
// Include CImg library to load textures.
// The macro below disables the capabilities of displaying images in CImg.
#define cimg_display 0
#include <CImg.h>

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

// Google flags.
// (<name of the flag>, <default value>, <Brief description of flat>)
// These will define global variables w/ the following format
// FLAGS_vertex_shader_filepath and 
// FLAGS_fragment_shader_filepath.
// DEFINE_<type>(name of flag, default value, brief description.)
// types: string, int32, bool.
DEFINE_string(vertex_shader_filepath, "", 
              "Filepath of the vertex shader.");
DEFINE_string(fragment_shader_filepath, "",
              "Filepath of the fragment shader.");
DEFINE_string(texture_filepath, "", 
              "Filepath of the texture.");

// Annonymous namespace for constants and helper functions.
namespace {
// Window dimensions.
constexpr int kWindowWidth = 640;
constexpr int kWindowHeight = 480;

// Error callback function. This function follows the required signature of
// GLFW. See http://www.glfw.org/docs/3.0/group__error.html for more
// information.
static void ErrorCallback(int error, const char* description) {
  std::cerr << "ERROR: " << description << std::endl;
}

// Key callback. This function follows the required signature of GLFW. See
// http://www.glfw.org/docs/latest/input_guide.html fore more information.
static void KeyCallback(GLFWwindow* window,
                        int key,
                        int scancode,
                        int action,
                        int mods) {
  if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
    glfwSetWindowShouldClose(window, GL_TRUE);
  }
}

// Class that will help us keep the state of any model more easily.
// TODO(vfragoso): Move it out of this file in the class so that students learn
// about the header guards.
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

// General form.
Eigen::Matrix4f ComputeProjectionMatrix(
  const GLfloat left, 
  const GLfloat right, 
  const GLfloat top, 
  const GLfloat bottom, 
  const GLfloat near, 
  const GLfloat far) {
  Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
  projection(0, 0) = 2.0 * near / (right - left);
  projection(1, 1) = 2.0 * near / (top - bottom);
  projection(2, 2) = -(far + near) / (far - near);
  projection(0, 2) = (right + left) / (right - left);
  projection(1, 2) = (top + bottom) / (top - bottom);
  projection(2, 3) = -2.0 * far * near / (far - near);
  projection(3, 3) = 0.0f;
  projection(3, 2) = -1.0f;
  return projection;
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

// Reparametrization of the ComputeProjectionMatrix. This function only
// requires 4 parameters rather than 6 parameters.
Eigen::Matrix4f ComputeProjectionMatrix(const GLfloat field_of_view,
                                        const GLfloat aspect_ratio,
                                        const GLfloat near,
                                        const GLfloat far) {
  // Create the projection matrix.
  const GLfloat y_scale = ComputeCotangent(0.5f * field_of_view);
  const GLfloat x_scale = y_scale / aspect_ratio;
  const GLfloat planes_distance = far - near;
  const GLfloat z_scale =
      -(near + far) / planes_distance;
  const GLfloat homogeneous_scale =
      -2 * near * far / planes_distance;
  Eigen::Matrix4f projection_matrix;
  projection_matrix << x_scale, 0.0f, 0.0f, 0.0f,
      0.0f, y_scale, 0.0f, 0.0f,
      0.0f, 0.0f, z_scale, homogeneous_scale,
      0.0f, 0.0f, -1.0f, 0.0f;
  return projection_matrix;
}

// -------------------- Texture helper functions -------------------------------
GLuint LoadTexture(const std::string& texture_filepath) {
  cimg_library::CImg<unsigned char> image;
  image.load(texture_filepath.c_str());
  const int width = image.width();
  const int height = image.height();
  // OpenGL expects to have the pixel values interleaved (e.g., RGBD, ...). CImg
  // flatens out the planes. To have them interleaved, CImg has to re-arrange
  // the values.
  // Also, OpenGL has the y-axis of the texture flipped.
  image.permute_axes("cxyz");
  GLuint texture_id;
  glGenTextures(1, &texture_id);
  glBindTexture(GL_TEXTURE_2D, texture_id);
  // We are configuring texture wrapper, each per dimension,s:x, t:y.
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  // Define the interpolation behavior for this texture.
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  /// Sending the texture information to the GPU.
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height,
               0, GL_RGB, GL_UNSIGNED_BYTE, image.data());
  // Generate a mipmap.
  glGenerateMipmap(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, 0);
  return texture_id;
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
  glEnable(GL_DEPTH_TEST);
  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
  // Tells OpenGL to clear the Color buffer.
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
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
  constexpr GLuint kStride = 8 * sizeof(vertices(0, 0));
  const GLvoid* offset_ptr = nullptr;
  glVertexAttribPointer(kIndex, kNumElementsPerVertex, 
                        GL_FLOAT, GL_FALSE,
                        kStride, offset_ptr);
  // Set as active our newly generated VBO.
  glEnableVertexAttribArray(kIndex);
  const GLvoid* offset_color = reinterpret_cast<GLvoid*>(3 * sizeof(vertices(0, 0)));
  glVertexAttribPointer(1, kNumElementsPerVertex, 
                        GL_FLOAT, GL_FALSE,
                        kStride, offset_color);
  glEnableVertexAttribArray(1);
  // Configure the texels.
  const GLvoid* offset_texel = 
    reinterpret_cast<GLvoid*>(6 * sizeof(vertices(0, 0)));
  glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE,
                        kStride, offset_texel);
  glEnableVertexAttribArray(2);
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
                 const Eigen::Matrix4f& projection,
                 const GLfloat angle,
                 const GLuint texture_id,
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
  // When variable is not found you get a - 1.
  Eigen::Matrix4f translation = 
    ComputeTranslation(Eigen::Vector3f(0.0f, 0.0f, -5.0f));
  Eigen::Matrix4f rotation = 
      ComputeRotation(Eigen::Vector3f(0.0, 1.0, 0.0f).normalized(), angle);
  Eigen::Matrix4f model = translation * rotation;
  std::cout << "Model: \n" << model << std::endl;
  Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
  // Bind texture.
  glBindTexture(GL_TEXTURE_2D, texture_id);
  // We do not create the projection matrix here because the projection 
  // matrix does not change.
  // Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
  glUniformMatrix4fv(model_location, 1, GL_FALSE, model.data());
  glUniformMatrix4fv(view_location, 1, GL_FALSE, view.data());
  glUniformMatrix4fv(projection_location, 1, GL_FALSE, projection.data());
  GLfloat color_scalar = static_cast<GLfloat>(glfwGetTime());
  // Draw the triangle.
  // Let OpenGL know what vertex array object we will use.
  glBindVertexArray(vertex_array_object_id);
    // Set to GL_LINE instead of GL_FILL to visualize the poligons as wireframes.
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  // First argument specifies the primitive to use.
  // Second argument specifies the starting index in the VAO.
  // Third argument specified the number of vertices to use.
  // glDrawArrays(GL_TRIANGLE_STRIP, 0, 10);

  // Using EBOs.
  glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
  
  // Let OpenGL know that we are done with our vertex array object.
  glBindVertexArray(0);
  glBindTexture(GL_TEXTURE_2D, 0);
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
  glfwSetKeyCallback(window, KeyCallback);

  // Initialize GLEW.
  glewExperimental = GL_TRUE;
  if (glewInit() != GLEW_OK) {
    std::cerr << "Glew did not initialize properly!" << std::endl;
    glfwTerminate();
    return -1;
  }

  // Configure View Port.
  ConfigureViewPort(window);

  // Compile shaders and create shader program.
  // This is how we access the flags.
  const std::string vertex_shader_filepath = 
    FLAGS_vertex_shader_filepath;
  const std::string fragment_shader_filepath =
    FLAGS_fragment_shader_filepath;
  wvu::ShaderProgram shader_program;
  std::cout << vertex_shader_filepath << std::endl;
  std::cout << fragment_shader_filepath << std::endl;
  shader_program.LoadVertexShaderFromFile(vertex_shader_filepath);
  shader_program.LoadFragmentShaderFromFile(fragment_shader_filepath);
  std::string error_info_log;
  if (!shader_program.Create(&error_info_log)) {
    std::cout << "ERROR: " << error_info_log << "\n";
  }
  // TODO(vfragoso): Implement me!
  if (!shader_program.shader_program_id()) {
    std::cerr << "ERROR: Could not create a shader program.\n";
    return -1;
  }

  // Prepare buffers to hold the vertices in GPU.
  GLuint vertex_buffer_object_id;
  GLuint vertex_array_object_id;
  GLuint element_buffer_object_id;
  Eigen::MatrixXf vertices(8, 4);
  // Vertex 0.
  vertices.block(0, 0, 3, 1) = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
  vertices.block(3, 0, 3, 1) = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
  vertices.block(6, 0, 2, 1) = Eigen::Vector2f(0, 0);
  // Vertex 1.
  vertices.block(0, 1, 3, 1) = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  vertices.block(3, 1, 3, 1) = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
  vertices.block(6, 1, 2, 1) = Eigen::Vector2f(0, 1);
  // Vertex 2.
  vertices.block(0, 2, 3, 1) = Eigen::Vector3f(1.0f, 1.0f, 0.0f);
  vertices.block(3, 2, 3, 1) = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
  vertices.block(6, 2, 2, 1) = Eigen::Vector2f(1, 0);
  // Vertex 3.
  vertices.block(0, 3, 3, 1) = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
  vertices.block(3, 3, 3, 1) = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
  vertices.block(6, 3, 2, 1) = Eigen::Vector2f(1, 1);
  std::vector<GLuint> indices = {
    0, 1, 3,  // First triangle.
    0, 3, 2,  // Second triangle.
  };
  Model model(Eigen::Vector3f(0, 0, 0),  // Orientation of object.
              Eigen::Vector3f(0, 0, 0),  // Position of object.
              vertices,
              indices);
  SetVertexArrayObject(model,
                       &vertex_buffer_object_id,
                       &vertex_array_object_id,
                       &element_buffer_object_id);
  const GLuint texture_id = LoadTexture(FLAGS_texture_filepath);

  // Create projection matrix.
  const GLfloat field_of_view = 45.0f;
  const GLfloat aspect_ratio = kWindowWidth / kWindowHeight;
  const Eigen::Matrix4f projection_matrix = 
      ComputeProjectionMatrix(field_of_view, aspect_ratio, 0.1, 10);
  std::cout << projection_matrix << std::endl;
  GLfloat angle = 0.0f;  // State of rotation.

  // Loop until the user closes the window.
  const GLfloat rotation_speed = 50.0f;
  while (!glfwWindowShouldClose(window)) {
    // Render the scene!
    // Casting using (<type>) -- which is the C way -- is not recommended.
    // Instead, use static_cast<type>(input argument).
    angle = rotation_speed * static_cast<GLfloat>(glfwGetTime()) * M_PI / 180.f;
    RenderScene(shader_program, vertex_array_object_id, 
                projection_matrix, angle, texture_id, window);

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

  return 0;
}
