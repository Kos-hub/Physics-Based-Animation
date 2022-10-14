#pragma once


#include <glm/glm.hpp>

#include "PhysicsObject.h"

// Fwd declaration
class MeshDb;
class ShaderDb;
class Camera;

class PhysicsEngine
{
public:
	void Init(Camera& camera, MeshDb& meshDb, ShaderDb& shaderDb);
	void Update(float deltaTime, float totalTime);
	void Display(const glm::mat4& viewMatrix, const glm::mat4& projMatrix);
	void HandleInputKey(int keyCode, bool pressed);

	void Task1Init(Camera& camera, MeshDb& meshDb, ShaderDb& shaderDb);
	void Task2Init(Camera& camera, MeshDb& meshDb, ShaderDb& shaderDb);
	void Task3Init(Camera& camera, MeshDb& meshDb, ShaderDb& shaderDb);
	void Task4Init(Camera& camera, MeshDb& meshDb, ShaderDb& shaderDb);

	void Task1Update(float deltaTime, float totalTime); // 5-particle chain
	void Task2Update(float deltaTime, float totalTime);
	void Task3Update(float deltaTime, float totalTime);
	void Task4Update(float deltaTime, float totalTime);

	// ... rest of the tasks here

private:

	PhysicsBody ground;
	// Task 1
	Particle sphere4;
	Particle sphere3;
	Particle sphere2;
	Particle sphere1;
	Particle anchor;

	// Task 2
	std::vector<Particle> task2Particles;

	// Task 3
	std::vector<Particle> task3Particles;

	// Task 4
	Particle task4Particles[10][10];
};