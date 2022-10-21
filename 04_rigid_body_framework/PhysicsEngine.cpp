#include "PhysicsEngine.h"

#include <map>
#include <numeric>

#include "Application.h"
#include "Camera.h"
#include "Force.h"

#include <glm/gtx/matrix_cross_product.hpp>
#include <glm/gtx/orthonormalize.hpp>

using namespace glm;

const glm::vec3 GRAVITY = glm::vec3(0, -9.81, 0);

void ExplicitEuler(vec3& pos, vec3& vel, float mass, const vec3& accel, const vec3& impulse, float dt)
{
	vec3 newVel = vel + (accel * dt + impulse / mass);

	pos += dt * vel;

	vel = newVel;
}

void SymplecticEuler(vec3& pos, vec3& vel, float mass, const vec3& accel, const vec3& impulse, float dt)
{
	vel += accel * dt + impulse / mass;
	pos += vel * dt;
}

void Integrate(RigidBody& rb, float dt)
{
	//vec3 dRot = rb.AngularVelocity() * dt;
	//if (glm::dot(dRot, dRot) > 0)
	//{
	//	rb.Rotate(glm::length(dRot), glm::normalize(dRot));
	//}
	rb.SetAngularVelocity(rb.AngularVelocity() + dt * rb.AngularAcceleration());
	mat3 anglVelSkew = matrixCross3(rb.AngularVelocity());
	mat3 R = glm::mat3(rb.Rotate());
}


void CollisionImpulse(RigidBody& rb,vec3 cubeCentre, float cubeHalfExtent, float coefficientOfRestitution)
{
	vec3 impulse{ 0.0f };
	vec3 surfaceNorm{ 0.0f };

	for (int i = 0; i < 3; i++)
	{
		if (rb.Position()[i] + rb.Scale()[i] >= (cubeCentre[i] + cubeHalfExtent))
		{
			surfaceNorm = vec3(0.0f);
			surfaceNorm[i] = -1.0f;

			if (i == 0)
			{
				rb.SetPosition(vec3(cubeCentre[i] + cubeHalfExtent - rb.Scale().x - 0.1f, rb.Position().y, rb.Position().z));
			}
			else if (i == 1)
			{
				rb.SetPosition(vec3(rb.Position().x, cubeCentre[i] + cubeHalfExtent - rb.Scale().y - 0.1f, rb.Position().z));
			}
			else if (i == 2)
			{
				rb.SetPosition(vec3(rb.Position().x, rb.Position().y, cubeCentre[i] + cubeHalfExtent - rb.Scale().z - 0.1f));
			}
		}
		else if (rb.Position()[i] - rb.Scale()[i] <= (cubeCentre[i] - cubeHalfExtent))
		{
			surfaceNorm = vec3(0.0f);
			surfaceNorm[i] = 1.0f;

			if (i == 0)
			{
				rb.SetPosition(vec3(cubeCentre[i] - cubeHalfExtent + rb.Scale().x + 0.1f, rb.Position().y, rb.Position().z));
			}
			else if (i == 1)
			{
				rb.SetPosition(vec3(rb.Position().x, cubeCentre[i] - cubeHalfExtent + rb.Scale().y + 0.1f, rb.Position().z));
			}
			else if (i == 2)
			{
				rb.SetPosition(vec3(rb.Position().x, rb.Position().y, cubeCentre[i] - cubeHalfExtent + rb.Scale().z + 0.1f));
			}
		}
	}

	impulse = -(1.0f + coefficientOfRestitution) * rb.Mass() * glm::dot(rb.Velocity(), surfaceNorm) * surfaceNorm;
	rb.ApplyImpulse(impulse);
}

// This is called once
void PhysicsEngine::Init(Camera& camera, MeshDb& meshDb, ShaderDb& shaderDb)
{
	// Get a few meshes/shaders from the databases
	auto defaultShader = shaderDb.Get("default");
	

	meshDb.Add("cube", Mesh(MeshDataFromWavefrontObj("resources/models/cube.obj")));
	meshDb.Add("sphere", Mesh(MeshDataFromWavefrontObj("resources/models/sphere.obj")));
	meshDb.Add("cone", Mesh(MeshDataFromWavefrontObj("resources/models/cone.obj")));

	auto rbMesh = meshDb.Get("cube");
	auto groundMesh = meshDb.Get("cube");
	// Initialise ground
	ground.SetMesh(groundMesh);
	ground.SetShader(defaultShader);
	ground.SetScale(vec3(10.0f));

	rbody1.SetMesh(rbMesh);
	rbody1.SetShader(defaultShader);
	rbody1.SetColor(vec4(1, 0, 0, 1));
	rbody1.SetPosition(vec3(0.0f, 5.0f, 0.0f));
	rbody1.SetScale(vec3(1,2,1));
	rbody1.SetAngularVelocity(vec3(0.0f, 1.0f, 0.0f));
	rbody1.SetAngularAcceleration(vec3(0.0f, 1.0f, 0.0f));
	camera = Camera(vec3(0, 5, 10));
}

void PhysicsEngine::Task1Init()
{
	// Initialise the rigid body
}

void PhysicsEngine::Task1Update(float deltaTime, float totalTime)
{

}


// This is called every frame
void PhysicsEngine::Update(float deltaTime, float totalTime)
{
	// Calculate forces, then acceleration, then integrate
	vec3 acceleration = rbody1.AccumulatedForce() / rbody1.Mass();

	vec3 p = rbody1.Position(), v = rbody1.Velocity();
	SymplecticEuler(p, v, rbody1.Mass(), acceleration, rbody1.AccumulatedImpulse(), deltaTime);
	rbody1.SetPosition(p);
	rbody1.SetVelocity(v);
	Integrate(rbody1, deltaTime);
}

// This is called every frame, after Update
void PhysicsEngine::Display(const mat4& viewMatrix, const mat4& projMatrix)
{
	ground.Draw(viewMatrix, projMatrix);
	rbody1.Draw(viewMatrix, projMatrix);
}

void PhysicsEngine::HandleInputKey(int keyCode, bool pressed)
{
	switch (keyCode)
	{
		//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		// TODO: Add any task swapping keys here
		//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	default:
		break;
	}
}