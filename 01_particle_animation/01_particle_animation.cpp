// Math constants
#define _USE_MATH_DEFINES
#include <cmath>  
#include <random>

// Std. Includes
#include <filesystem>
#include <string>
#include <time.h>

// GLEW
//#define GLEW_STATIC
#include <GL/glew.h>

// GLFW
#include <GLFW/glfw3.h>

// GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/matrix_operation.hpp>
#include "glm/ext.hpp"


// project includes
#include "Shader.h"
#include "Camera.h"
#include "Mesh.h"


// Properties
const GLuint WIDTH = 800, HEIGHT = 600;
int SCREEN_WIDTH, SCREEN_HEIGHT;

// Function prototypes
void KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mode);
void ScrollCallback(GLFWwindow* window, double xOffset, double yOffset);
int main(int argc, const char** argv);
void MouseCallback(GLFWwindow* window, double xPos, double yPos);
void DoMovement();

// Camera
Camera  camera(glm::vec3(0.0f, 5.0f, 20.0f));
double lastX = WIDTH / 2.0;
double lastY = HEIGHT / 2.0;
bool keys[1024];
bool firstMouse = true;

// view and projection matrices
glm::mat4 view = glm::mat4(1.0f);
glm::mat4 projection = glm::mat4(1.0f);

// time
GLfloat deltaTime = 0.0f;


// window
GLFWwindow* window = NULL;

// Moves/alters the camera positions based on user input
void DoMovement()
{
	// Camera controls
	if (keys[GLFW_KEY_W] || keys[GLFW_KEY_UP])
	{
		camera.ProcessKeyboard(FORWARD, deltaTime);
	}

	if (keys[GLFW_KEY_S] || keys[GLFW_KEY_DOWN])
	{
		camera.ProcessKeyboard(BACKWARD, deltaTime);
	}

	if (keys[GLFW_KEY_A] || keys[GLFW_KEY_LEFT])
	{
		camera.ProcessKeyboard(LEFT, deltaTime);
	}

	if (keys[GLFW_KEY_D] || keys[GLFW_KEY_RIGHT])
	{
		camera.ProcessKeyboard(RIGHT, deltaTime);
	}
}

// Is called whenever a key is pressed/released via GLFW
void KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mode)
{
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
	{
		glfwSetWindowShouldClose(window, GL_TRUE);
	}

	if (key >= 0 && key < 1024)
	{
		if (action == GLFW_PRESS)
		{
			keys[key] = true;
		}
		else if (action == GLFW_RELEASE)
		{
			keys[key] = false;
		}
	}
}

void MouseCallback(GLFWwindow* window, double xPos, double yPos)
{
	if (firstMouse)
	{
		lastX = xPos;
		lastY = yPos;
		firstMouse = false;
	}

	double xOffset = xPos - lastX;
	double yOffset = lastY - yPos;

	lastX = xPos;
	lastY = yPos;

	camera.ProcessMouseMovement((GLfloat)xOffset, (GLfloat)yOffset);
}


void ScrollCallback(GLFWwindow* window, double xOffset, double yOffset)
{
	camera.ProcessMouseScroll((GLfloat)yOffset);
}


// Renderer initialisation
int initRender() {
	// Init GLFW
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

	window = glfwCreateWindow(WIDTH, HEIGHT, "Physics-Based Animation", nullptr, nullptr);

	if (nullptr == window)
	{
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();

		return EXIT_FAILURE;
	}

	glfwMakeContextCurrent(window);

	glfwGetFramebufferSize(window, &SCREEN_WIDTH, &SCREEN_HEIGHT);

	// Set the required callback functions
	glfwSetKeyCallback(window, KeyCallback);
	glfwSetCursorPosCallback(window, MouseCallback);
	glfwSetScrollCallback(window, ScrollCallback);

	// remove the mouse cursor
	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

	// moder GLEW approach
	glewExperimental = GL_TRUE;
	// Initialize GLEW to setup the OpenGL Function pointers
	if (GLEW_OK != glewInit())
	{
		std::cout << "Failed to initialize GLEW" << std::endl;
		return EXIT_FAILURE;
	}

	// Define the viewport dimensions
	glViewport(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT);

	// Setup some OpenGL options
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	return 1;
}

// main function
int main(int argc, const char** argv)
{
	// init renderer
	initRender();

	// Shader initialisation uses OpenGL, so needs to happen AFTER initRender
	auto defaultShader = CreateDefaultShader();

	// Mesh initialisation uses OpenGL, so needs to happen AFTER initRender
	Mesh particleMesh, groundMesh;

	particleMesh.Init(TetrahedronMeshData());
	groundMesh.Init(PlaneMeshData(glm::vec2(100.0f, 100.0f))); 


	Object ground;

	//particle.SetMesh(&particleMesh);
	//particle.SetShader(&defaultShader);

	ground.SetMesh(&groundMesh);
	ground.SetShader(&defaultShader);

	//particle.SetColor(glm::vec4(1, 0, 0, 1)); // set red color
	//particle.SetPosition(glm::vec3(0.0f, 5.0f, 0.0f));
	//particle.SetVelocity(glm::vec3(4.0f, 5.0f, 5.0f));
	// 
	//particle.Scale(glm::vec3(.1f, .1f, .1f)); // shrink it to 10% of original
	//particle.Rotate((GLfloat) M_PI_2, glm::vec3(1.0f, 0.0f, 0.0f)); // rotate by 90 degrees around X axis

	/*
	CREATE THE PARTICLE(S) YOU NEED TO COMPLETE THE TASKS HERE
	*/
	std::vector<Object> particles;

	srand(time(NULL));
	for (int i = 0; i < 100000; i++)
	{
		Object particle;
		particle.SetMesh(&particleMesh);
		particle.SetShader(&defaultShader);

		particle.SetColor(glm::vec4(((double)rand() / (RAND_MAX)), ((double)rand() / (RAND_MAX)), ((double)rand() / (RAND_MAX)), 1));
		particle.SetPosition(glm::vec3((rand() % 50) - 50, rand() % 2 + 10, (rand() % 50) - 50));

		particle.SetVelocity(glm::vec3(rand() % 20, rand() % 20, rand() % 20));
		//particle.SetVelocity(glm::vec3(10.0f, 10.0f, 10.0f));
		particles.push_back(particle);
	}

	GLfloat timeStart = (GLfloat)glfwGetTime();

	GLfloat lastFrameTimeSinceStart = timeStart;

	const float ANIMATION_SPEED = 1.0f; // increase this if you want time to move faster

	glm::vec3 acceleration = glm::vec3(0.0f, -9.8f, 0.0f);

	// Game loop
	while (!glfwWindowShouldClose(window))
	{
		projection = glm::perspective(camera.GetZoom(), (GLfloat)SCREEN_WIDTH / (GLfloat)SCREEN_HEIGHT, 0.1f, 1000.0f);
		view = camera.GetViewMatrix();

		GLfloat timeSinceStart = (GLfloat)glfwGetTime() - timeStart;
		// the animation can be sped up or slowed down by multiplying timeSinceStart by a factor.
		timeSinceStart *= ANIMATION_SPEED;

		// calculate the delta time
		deltaTime = timeSinceStart - lastFrameTimeSinceStart;

		// save the current time since start to the previous time since start, so that we can calculate the elapsed time between the different frames
		lastFrameTimeSinceStart = timeSinceStart;

		glfwPollEvents();
		DoMovement();

		projection = glm::perspective(camera.GetZoom(), (GLfloat)SCREEN_WIDTH / (GLfloat)SCREEN_HEIGHT, 0.1f, 1000.0f);
		view = camera.GetViewMatrix();

		for (int i = 0; i < particles.size(); i++)
		{
			particles[i].SetPosition(glm::vec3(particles[i].Position() + (particles[i].Velocity() * deltaTime) + (0.5f * acceleration * (deltaTime * deltaTime))));
			particles[i].SetVelocity(particles[i].Velocity() + (acceleration * deltaTime));


			if (particles[i].Position().y <= 0.5f)
			{
				particles[i].SetVelocity(
					(glm::vec3(particles[i].Velocity().x, -particles[i].Velocity().y, particles[i].Velocity().z)) + (acceleration * deltaTime) - 2.0f
				);
				
				if (particles[i].Velocity().y <= 0 && particles[i].Position().y <= 0.5f)
				{
					particles[i].SetVelocity(glm::vec3(0.0f, 0.0f, 0.0f));
					particles[i].SetPosition(glm::vec3(particles[i].Position().x, 0.5f, particles[i].Position().z));
				}
			}
		}

		glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		ground.Draw(view, projection);

		for (int j = 0; j < particles.size(); j++)
		{
			particles[j].Draw(view, projection);
		}

		//particle.Draw(view, projection);

		glfwSwapBuffers(window);

	}

	glfwTerminate();

	return EXIT_SUCCESS;
}

