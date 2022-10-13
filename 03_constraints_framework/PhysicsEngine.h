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
	void Task1Update(float deltaTime, float totalTime); // 5-particle chain
	// ... rest of the tasks here

private:

	PhysicsBody ground;
	Particle sphere3;
	Particle sphere2;
	Particle sphere1;
	Particle anchor;
};