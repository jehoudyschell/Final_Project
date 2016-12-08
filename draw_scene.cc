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
#include "camera.cc"
#include "camera_controller.cc"
#include "camera_utils.h"
#include "model.h"
#include "shader_program.h"
#include "transformations.h"

// Google flags.
// (<name of the flag>, <default value>, <Brief description of flat>)
// These will define global variables w/ the following format
// FLAGS_vertex_shader_filepath and
// FLAGS_fragment_shader_filepath.
// DEFINE_<type>(name of flag, default value, brief description.)
// types: string, int32, bool.
DEFINE_string(texture1_filepath, "texture1.bmp",
              "Filepath of the first texture.");
DEFINE_string(texture2_filepath, "texture2.bmp",
              "Filepath of the second texture.");

// Annonymous namespace for constants and helper functions.
namespace {
using wvu::Model;

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

// GLSL shaders.
// Every shader should declare its version.
// Vertex shader follows standard 3.3.0.
// This shader declares/expexts an input variable named position. This input
// should have been loaded into GPU memory for its processing. The shader
// essentially sets the gl_Position -- an already defined variable -- that
// determines the final position for a vertex.
// Note that the position variable is of type vec3, which is a 3D dimensional
// vector. The layout keyword determines the way the VAO buffer is arranged in
// memory. This way the shader can read the vertices correctly.
const std::string vertex_shader_src =
    "#version 330 core\n"
	"layout (location = 0) in vec3 position;\n"
	"layout (location = 1) in vec3 passed_color;\n"
	"layout (location = 2) in vec2 passed_texel;\n"
	"uniform mat4 model;\n"
	"uniform mat4 view;\n"
	"uniform mat4 projection;\n"
	"out vec4 vertex_color;\n"
	"out vec2 texel;\n"
  "void main() {\n"
  "gl_Position = projection * view * model * vec4(position, 1.0f);\n"
  "vertex_color = vec4(passed_color, 1.0f);\n"
  "texel = passed_texel;\n"
"}\n";


// Fragment shader follows standard 3.3.0. The goal of the fragment shader is to
// calculate the color of the pixel corresponding to a vertex. This is why we
// declare a variable named color of type vec4 (4D vector) as its output. This
// shader sets the output color to a (1.0, 0.5, 0.2, 1.0) using an RGBA format.
// shader sets the output color to a (1.0, 0.5, 0.2, 1.0) using an RGBA format.
const std::string fragment_shader_src =
    "#version 330 core\n"
	"in vec4 vertex_color;\n"
	"out vec4 color;\n"
	"in vec2 texel;\n"
	"uniform sampler2D texture_sampler;\n"
	"void main() {\n"
    "color = texture(texture_sampler, texel);\n"
  "}\n";

// Error callback function. This function follows the required signature of
// GLFW. See http://www.glfw.org/docs/3.0/group__error.html for more
// information.
static void ErrorCallback(int error, const char* description) {
  std::cerr << "ERROR: " << description << std::endl;
}

// Key callback. This function follows the required signature of GLFW. See
// http://www.glfw.org/docs/latest/input_guide.html fore more information.
static void KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
  if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
    glfwSetWindowShouldClose(window, GL_TRUE);
  }
}

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

bool CreateShaderProgram(wvu::ShaderProgram* shader_program) {
  if (shader_program == nullptr) return false;
  shader_program->LoadVertexShaderFromString(vertex_shader_src);
  shader_program->LoadFragmentShaderFromString(fragment_shader_src);
  std::string error_info_log;
  if (!shader_program->Create(&error_info_log)) {
    std::cout << "ERROR: " << error_info_log << "\n";
  }
  if (!shader_program->shader_program_id()) {
    std::cerr << "ERROR: Could not create a shader program.\n";
    return false;
  }
  return true;
}

// Renders the scene.
void RenderScene(const wvu::ShaderProgram& shader_program,
                  const Eigen::Matrix4f& projection,
        					const Eigen::Matrix4f& view,
        					std::vector<Model*>* models_to_draw,
        					GLFWwindow* window, GLuint texture_ids[]) {
  // Clear the buffer.
  ClearTheFrameBuffer();
  // Let OpenGL know that we want to use our shader program.
  shader_program.Use();
  // Render the models in a wireframe mode.
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  // Draw the models.
  // TODO: For every model in models_to_draw, call its Draw() method, passing
  // the view and projection matrices.
  for(int i = 0; i < models_to_draw->size(); i++){
    models_to_draw->at(i)->Draw(shader_program, projection, view, texture_ids[i]);
  }
  // Let OpenGL know that we are done with our vertex array object.
  glBindVertexArray(0);
}




void ConstructModels(std::vector<Model*>* models_to_draw) {
  //Square Pyramid
  std::vector<GLuint> indices = {
    0, 3, 2,
    0, 2, 1,
    0, 4, 1,
    0, 3, 4,
    3, 2, 4,
    2, 1, 4};
  Eigen::MatrixXf vertices(8, 5);
  // Pryamid Vertex 0.
  vertices.block(0, 0, 3, 1) = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  vertices.block(3, 0, 3, 1) = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
  vertices.block(6, 0, 2, 1) = Eigen::Vector2f(0, 0);
  // Pryamid Vertex 1.
  vertices.block(0, 1, 3, 1) = Eigen::Vector3f(2.0f, 0.0f, 0.0f);
  vertices.block(3, 1, 3, 1) = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
  vertices.block(6, 1, 2, 1) = Eigen::Vector2f(0, 1);
  // Pryamid Vertex 2.
  vertices.block(0, 2, 3, 1) = Eigen::Vector3f(2.0f, 0.0f, 2.0f);
  vertices.block(3, 2, 3, 1) = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
  vertices.block(6, 2, 2, 1) = Eigen::Vector2f(1, 0);
  // Pryamid Vertex 3.
  vertices.block(0, 3, 3, 1) = Eigen::Vector3f(0.0f, 0.0f, 2.0f);
  vertices.block(3, 3, 3, 1) = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
  vertices.block(6, 3, 2, 1) = Eigen::Vector2f(1, 1);
  // Pryamid Vertex 3.
  vertices.block(0, 4, 3, 1) = Eigen::Vector3f(1.0f, 2.0f, 1.0f);
  vertices.block(3, 4, 3, 1) = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
  vertices.block(6, 4, 2, 1) = Eigen::Vector2f(0, 0);

  Model* pyramid = new Model(Eigen::Vector3f(1.0f, 1.0f, 1.0f), Eigen::Vector3f(-3.0f, -1.0f, -15.0f), vertices, indices);
  pyramid->SetVerticesIntoGpu();
  models_to_draw->push_back(pyramid);

  //Cube
  std::vector<GLuint> indices2 = {
    0, 3, 2,
    0, 2, 1,
    0, 4, 1,
    1, 5, 2,
    2, 6, 3,
    3, 7, 0,
    4, 5, 1,
    5, 6, 2,
    6, 7, 3,
    7, 4, 0,
    4, 7, 5,
    5, 7, 6};
  Eigen::MatrixXf vertices2(8, 8);
  // Cube Vertex 0.
  vertices2.block(0, 0, 3, 1) = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  vertices2.block(3, 0, 3, 1) = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
  vertices2.block(6, 0, 2, 1) = Eigen::Vector2f(0, 0);
  // Cube Vertex 1.
  vertices2.block(0, 1, 3, 1) = Eigen::Vector3f(2.0f, 0.0f, 0.0f);
  vertices2.block(3, 1, 3, 1) = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
  vertices2.block(6, 1, 2, 1) = Eigen::Vector2f(0, 1);
  // Cube Vertex 2.
  vertices2.block(0, 2, 3, 1) = Eigen::Vector3f(2.0f, 0.0f, 2.0f);
  vertices2.block(3, 2, 3, 1) = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
  vertices2.block(6, 2, 2, 1) = Eigen::Vector2f(1, 0);
  // Cube Vertex 3.
  vertices2.block(0, 3, 3, 1) = Eigen::Vector3f(0.0f, 0.0f, 2.0f);
  vertices2.block(3, 3, 3, 1) = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
  vertices2.block(6, 3, 2, 1) = Eigen::Vector2f(1, 1);
  // Cube Vertex 4.
  vertices2.block(0, 0, 3, 1) = Eigen::Vector3f(0.0f, 2.0f, 0.0f);
  vertices2.block(3, 0, 3, 1) = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
  vertices2.block(6, 0, 2, 1) = Eigen::Vector2f(0, 0);
  // Cube Vertex 5.
  vertices2.block(0, 1, 3, 1) = Eigen::Vector3f(2.0f, 2.0f, 0.0f);
  vertices2.block(3, 1, 3, 1) = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
  vertices2.block(6, 1, 2, 1) = Eigen::Vector2f(0, 1);
  // Cube Vertex 6.
  vertices2.block(0, 2, 3, 1) = Eigen::Vector3f(2.0f, 2.0f, 2.0f);
  vertices2.block(3, 2, 3, 1) = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
  vertices2.block(6, 2, 2, 1) = Eigen::Vector2f(1, 0);
  // Cube Vertex 7.
  vertices2.block(0, 3, 3, 1) = Eigen::Vector3f(0.0f, 2.0f, 2.0f);
  vertices2.block(3, 3, 3, 1) = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
  vertices2.block(6, 3, 2, 1) = Eigen::Vector2f(1, 1);

  Model* cube = new Model(Eigen::Vector3f(1.0f, 1.0f, 1.0f), Eigen::Vector3f(1.0f, -1.0f, -15.0f), vertices2, indices2);
  cube->SetVerticesIntoGpu();
  models_to_draw->push_back(cube);
}

void DeleteModels(std::vector<Model*>* models_to_draw) {
  // TODO: Implement me!
  // Call delete on each models to draw.
  for(int i = 0; i < models_to_draw->size(); i++){
    delete models_to_draw->at(i);
  }
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
  const std::string window_name = "Assignment 4";
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
  wvu::ShaderProgram shader_program;
  if (!CreateShaderProgram(&shader_program)) {
    return -1;
  }

  // Construct the models to draw in the scene.
  std::vector<Model*> models_to_draw;
  ConstructModels(&models_to_draw);

  GLuint texture_ids[2];
  texture_ids[0] = LoadTexture(FLAGS_brick_filepath);
  texture_ids[1] = LoadTexture(FLAGS_stone_filepath);

  // Construct the camera projection matrix.
  const float field_of_view = wvu::ConvertDegreesToRadians(45.0f);
  const float aspect_ratio = static_cast<float>(kWindowWidth / kWindowHeight);
  const float near_plane = 0.1f;
  const float far_plane = 20.0f;
  const Eigen::Matrix4f& projection =
      wvu::ComputePerspectiveProjectionMatrix(field_of_view, aspect_ratio,
                                              near_plane, far_plane);
  const Eigen::Matrix4f view = Eigen::Matrix4f::Identity();


  // Loop until the user closes the window.
  while (!glfwWindowShouldClose(window)) {
    // Render the scene!
    RenderScene(shader_program, projection, view, &models_to_draw, window, texture_ids);

    // Swap front and back buffers.
    glfwSwapBuffers(window);

    // Poll for and process events.
    glfwPollEvents();
  }

  // Cleaning up tasks.
  DeleteModels(&models_to_draw);
  // Destroy window.
  glfwDestroyWindow(window);
  // Tear down GLFW library.
  glfwTerminate();

  return 0;
}
