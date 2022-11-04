#include "PhysicsEngine.h"

#include <map>
#include <numeric>

#include "Application.h"
#include "Camera.h"
#include "Force.h"

#include <glm/gtx/matrix_cross_product.hpp>
#include <glm/gtx/orthonormalize.hpp>
#include "glm/ext.hpp"
#include "glm/gtx/string_cast.hpp"
using namespace glm;


const glm::vec3 GRAVITY = glm::vec3(0, -9.81, 0);

std::vector<Particle> vertices;

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
	// Integration calculation
	rb.SetAngularVelocity(rb.AngularVelocity() + dt * rb.AngularAcceleration());

	// Create Skew Symmetric Matrix for W (Omega)
	mat3 anglVelSkew = matrixCross3(rb.AngularVelocity());

	// Create 3x3 rotation matrix from rb rotation matrix
	mat3 R = glm::mat3(rb.Orientation());

	R += dt * anglVelSkew * R;
	R = glm::orthonormalize(R);
	rb.SetOrientation(glm::mat4(R));

	
}

float CalculateImpulseRigidBody(RigidBody& rb, vec3 r, vec3 surfNorm, float coefficientOfRestitution)
{
	vec3 relativeVelocity = rb.Velocity() + glm::cross(rb.AngularVelocity(), r);

	float numDotProd = glm::dot(relativeVelocity, surfNorm);
	float numerator = -(1.0f + coefficientOfRestitution) * numDotProd;

	vec3 rXn = glm::cross(r, surfNorm);
	vec3 iMultRXN = rb.InverseInertia() * rXn;
	vec3 iMultRXNXR = glm::cross(iMultRXN, r);
	float nDot = glm::dot(surfNorm, iMultRXNXR);
	float denominator = 1 / rb.Mass() + nDot;

	//const float denominator = (1 / rb.Mass() + glm::dot(surfNorm,
	//	glm::cross(rb.InverseInertia() * (glm::cross(r, surfNorm)), r)
	//)
	//	);

	return numerator / denominator;
}

vec3 GetLowestVector(vec3 firstVector, vec3 secondVector)
{
	if (firstVector.y < secondVector.y)
		return firstVector;
	else
		return secondVector;
	
}

vec3 GetPointOfContactForSide(std::vector<vec3> vertices)
{
	if (vertices[0].y == vertices[1].y == vertices[2].y == vertices[3].y)
	{
		return vec3((vertices[1].x + vertices[3].x) / 2,
			(vertices[1].y + vertices[3].y) / 2,
			(vertices[1].z + vertices[3].z) / 2);
	}
	else
	{
		vec3 lowestPoint = vec3(0.0f);
		for(int i = 0; i < vertices.size(); i++)
		{
			if(lowestPoint.y > vertices[i].y)
			{
				lowestPoint = vertices[i];
			}
		}
		return lowestPoint;
	}
}


void TranslateWhenPenetrating(RigidBody& rb, vec3 pointOfApplication)
{
	// How far did the object penetrate?
	const float vertexFallOff = glm::abs(pointOfApplication.y - 0.0f);

	// Translate the object up based on the falloff + an offset that will not make it collide with the plane.
	rb.Translate(vec3(0.0f, vertexFallOff, 0.0f));
}



void CollisionImpulse(RigidBody& rb,vec3 cubeCentre, float cubeHalfExtent, float coefficientOfRestitution)
{
	std::vector<vec3> touchingPositions;
	vec3 surfNorm = vec3(0.0f);
	surfNorm[1] = 1.0f;

	for(int i = 0; i < rb.GetVertices().size(); i++)
	{
		if(rb.GetVertices()[i].y <= 0.0f)
			touchingPositions.push_back(rb.GetVertices()[i]);
		
	}


	vec3 pointOfApplication = vec3(0.0f);
	switch (touchingPositions.size())
	{
	case 0:
		return;
		break;

	case 1:
		pointOfApplication = touchingPositions[0];
		break;

	case 2:
		if (touchingPositions[0].y == touchingPositions[1].y)
		{
			pointOfApplication = vec3((touchingPositions[0].x + touchingPositions[1].x) / 2,
				(touchingPositions[0].y + touchingPositions[1].y) / 2,
				(touchingPositions[0].z + touchingPositions[1].z) / 2);
		}
		else
		{
			pointOfApplication = GetLowestVector(touchingPositions[0], touchingPositions[1]);
		}
		break;

	case 3:

		for (int i = 0; i < touchingPositions.size(); i++)
		{
			if (pointOfApplication.y > touchingPositions[i].y)
			{
				pointOfApplication = touchingPositions[i];
			}
		}
		break;
	case 4:
		pointOfApplication = GetPointOfContactForSide(touchingPositions);
		break;

	}

	TranslateWhenPenetrating(rb, pointOfApplication);

	vec3 r = pointOfApplication - rb.Position();
	vec3 impulse = CalculateImpulseRigidBody(rb, r, surfNorm, coefficientOfRestitution) * surfNorm;

	vec3 angularVelocity = rb.AngularVelocity() + rb.InverseInertia() * glm::cross(r, impulse);
	rb.SetAngularVelocity(angularVelocity);
	rb.ApplyImpulse(impulse);

	std::cout << "AngularVelocity: " << glm::to_string(rb.AngularVelocity()) << std::endl;
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
	auto groundMesh = meshDb.Get("plane");
	auto sphereMesh = meshDb.Get("sphere");
	// Initialise ground
	ground.SetMesh(groundMesh);
	ground.SetShader(defaultShader);
	ground.SetScale(vec3(20.0f));


	rbody1.SetMesh(rbMesh);
	rbody1.SetShader(defaultShader);
	rbody1.SetColor(vec4(1, 0, 0, 1));
	rbody1.SetScale(vec3(2,6,2));
	rbody1.SetMass(1.0f);
	rbody1.SetVelocity(vec3(0.0f, 0.0f, 0.0f));
	rbody1.Rotate(0.8f, vec3(1.0f, 1.0f, 0.0f));
	rbody1.SetAngularVelocity(vec3(0.0f, 0.0f, 0.0f));
	rbody1.SetPosition(vec3(0.0f, 12.0f, 0.0f));

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
	rbody1.ClearForcesImpulses();

	CollisionImpulse(rbody1, vec3(0.0f), 0.0f, 0.0f);

	Force::Gravity(rbody1);
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

	for (auto p : vertices)
		p.Draw(viewMatrix, projMatrix);
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