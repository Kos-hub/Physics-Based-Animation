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


Camera tempCamera;
MeshDb tempMeshDb;
ShaderDb tempShaderDb;


const glm::vec3 GRAVITY = glm::vec3(0, -9.81, 0);
const float coeffOfFriction = 0.25f;


enum class TaskNo
{
	Task1,
	Task2,
	Task3,
	Task4
};
TaskNo currentTask = TaskNo::Task1;

#pragma region Integration

void ExplicitEuler(vec3& pos, vec3& vel, float mass, const vec3& accel, const vec3& impulse, float dt)
{
	vec3 newVel = vel + (accel * dt + impulse / mass);

	pos += dt * vel;

	vel = newVel;
}

void SymplecticEuler(vec3& pos, vec3& vel, float mass, const vec3& accel, const vec3& impulse, float dt)
{
	vel += accel * dt;
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

#pragma endregion


#pragma region Helper Functions for Collisions

float CalculateImpulseRigidBody(RigidBody& rb, const vec3 r, const vec3 surfNorm, const float coefficientOfRestitution)
{

	// Calculating relative velocity (v1 + w1 x r)
	vec3 relativeVelocity = rb.Velocity() + glm::cross(rb.AngularVelocity(), r);

	// Calculating the dot prod between relative velocity and surface normal (vr . n)
	float numDotProd = glm::dot(relativeVelocity, surfNorm);

	// Numerator of impulse equation -(1 + e) * (vr . n)
	float numerator = -(1.0f + coefficientOfRestitution) * numDotProd;

	// Cross product between r and n (R x N)
	vec3 rXn = glm::cross(r, surfNorm);

	// Product between inverse inertia and (R x N) => I^-1 * (R x N)
	vec3 iMultRXN = rb.InverseInertia() * rXn;

	// Cross product. (I ^ -1 * (R x N)) x R
	vec3 iMultRXNXR = glm::cross(iMultRXN, r);

	// Final Dot Product with normal. N * ((I ^ -1 * (R x N)) x R)
	float nDot = glm::dot(surfNorm, iMultRXNXR);

	// Getting the denominator by adding 1/mass
	float denominator = (1 / rb.Mass()) + nDot;

	return numerator / denominator;
}

vec3 CalculateFrictionalImpulse(RigidBody& rb, vec3 r, vec3 surfNorm, float magnitudeOfImpulse)
{
	vec3 relativeVelocity = rb.Velocity() + glm::cross(rb.AngularVelocity(), r);


	vec3 tangent = vec3(0.0f);

	if (glm::dot(relativeVelocity, surfNorm) != 0.0f)
	{
		if (glm::length(relativeVelocity - (glm::dot(relativeVelocity, surfNorm) * surfNorm)) != 0.0f)
			tangent = glm::normalize(relativeVelocity - (glm::dot(relativeVelocity, surfNorm) * surfNorm));
	}
	else if (glm::dot(relativeVelocity, surfNorm) == 0.0f && glm::dot(rb.AccumulatedForce(), surfNorm) != 0.0f)
	{
		if (glm::length(rb.AccumulatedForce() - (glm::dot(rb.AccumulatedForce(), surfNorm) * surfNorm)) != 0.0f)
			tangent = glm::normalize(rb.AccumulatedForce() - (glm::dot(rb.AccumulatedForce(), surfNorm) * surfNorm));
	}


	vec3 jt = vec3(0.0f);

	if (glm::dot(relativeVelocity, tangent) == 0.0f && glm::dot((rb.Mass() * relativeVelocity), tangent) <= coeffOfFriction * magnitudeOfImpulse)
	{
		jt = -(glm::dot(rb.Mass() * relativeVelocity, tangent)) * tangent;
	}
	else
	{
		jt = -(coeffOfFriction * magnitudeOfImpulse) * tangent;
	}

	return jt;
}
vec3 GetLowestPoint(const std::vector<vec3> points)
{
	// Simple function to get the lowest point in a set of points.
	vec3 lowestPoint = vec3(0.0f);

	for(int i = 0; i < points.size(); i++)
	{
		if (points[i].y < lowestPoint.y)
			lowestPoint = points[i];
	}

	return lowestPoint;
	
}

vec3 GetPointOfContactForSide(const std::vector<vec3>& vertices)
{
	// Function to determine the central point of 4 vertices
	float xComponent = 0.0f;
	float yComponent = 0.0f;
	float zComponent = 0.0f;

	for(auto& pos : vertices)
	{
		xComponent += pos.x;
		yComponent += pos.y;
		zComponent += pos.z;
	}

	return vec3(xComponent / 4.0f, yComponent / 4.0f, zComponent / 4.0f);


}

void TranslateWhenPenetrating(RigidBody& rb,const vec3 pointOfApplication)
{
	// How far did the object penetrate?
	const float vertexFallOff = glm::abs(0.0f - pointOfApplication.y);

	// Translate the object up based on the falloff + an offset that will not make it collide with the plane.
	rb.Translate(vec3(0.0f, vertexFallOff, 0.0f));
	rb.UpdateVertices();
}
#pragma endregion


void CollisionImpulse(RigidBody& rb, float coefficientOfRestitution)
{
	std::vector<vec3> touchingPositions;
	vec3 surfNorm = vec3(0.0f, 1.0f, 0.0f);


	// Checking the vertices that have almost completely penetrated the ground
	for (auto& vertex : rb.GetVertices())
	{
		if (vertex.y <= 0.01f)
			touchingPositions.push_back(vertex);
	}


	// This is just for the sake of task 2
	if(currentTask == TaskNo::Task2 && !touchingPositions.empty())
	{
		for (auto& pos : touchingPositions)
			std::cout << "Touching Position: " << glm::to_string(pos) << std::endl;

		std::cout << std::endl;
	}


	// Using a switch case to find two vectors: point of application and lowest point.
	// The point of application is either the lowest vertex or the average between two or more vertices. This is where the impulse will be applied
	// The lowest point will be the lowest vertex that has penetrated the ground and that will determine how much translation we need for the cuboid to be above the ground
	vec3 pointOfApplication = vec3(0.0f);
	vec3 lowestPoint = vec3(0.0f);

	switch (touchingPositions.size())
	{
	case 0:
		return;
		break;

	case 1:
		pointOfApplication = touchingPositions[0];
		lowestPoint = GetLowestPoint(touchingPositions);
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
			pointOfApplication = GetLowestPoint(touchingPositions);
		}

		lowestPoint = GetLowestPoint(touchingPositions);

		break;

	case 3:

		pointOfApplication = vec3((touchingPositions[0].x + touchingPositions[2].x) / 2,
			(touchingPositions[0].y + touchingPositions[2].y) / 2,
			(touchingPositions[0].z + touchingPositions[2].z) / 2);

		lowestPoint = GetLowestPoint(touchingPositions);
		break;

	case 4:
		pointOfApplication = GetPointOfContactForSide(touchingPositions);

		lowestPoint = GetLowestPoint(touchingPositions);
		break;
	}

	// Translate the rb upwards and calculate the point of application
	TranslateWhenPenetrating(rb, lowestPoint);
	vec3 r = pointOfApplication - rb.Position();


	// Normal Impulse
	float magnitudeOfImpulse = CalculateImpulseRigidBody(rb, r, surfNorm, coefficientOfRestitution);
	vec3 jr = magnitudeOfImpulse * surfNorm;


	// Frictional Impulse
	vec3 jt = CalculateFrictionalImpulse(rb, r, surfNorm, magnitudeOfImpulse);

	rb.SetVelocity(rb.Velocity() + (jr + jt) / rb.Mass());
	rb.SetAngularVelocity(rb.AngularVelocity() + rb.InverseInertia() * glm::cross(r, jr + jt));
}


void PhysicsEngine::Init(Camera& camera, MeshDb& meshDb, ShaderDb& shaderDb)
{

	tempCamera = camera;
	tempMeshDb = meshDb;
	tempShaderDb = shaderDb;

	
	meshDb.Add("cube", Mesh(MeshDataFromWavefrontObj("resources/models/cube.obj")));
	meshDb.Add("sphere", Mesh(MeshDataFromWavefrontObj("resources/models/sphere.obj")));
	meshDb.Add("cone", Mesh(MeshDataFromWavefrontObj("resources/models/cone.obj")));

	switch (currentTask)
	{
	case TaskNo::Task1:
		PhysicsEngine::Task1Init(camera, meshDb, shaderDb);
		break;
	case TaskNo::Task2:
		PhysicsEngine::Task2Init(camera, meshDb, shaderDb);
		break;
	case TaskNo::Task3:
		PhysicsEngine::Task3Init(camera, meshDb, shaderDb);
		break;
	case TaskNo::Task4:
		PhysicsEngine::Task4Init(camera, meshDb, shaderDb);
		break;
	//case TaskNo::Task5:
	//	PhysicsEngine::Task5Init(camera, meshDb, shaderDb);
	//	break;
	}



}
void PhysicsEngine::Update(float deltaTime, float totalTime)
{
	switch (currentTask)
	{
	case TaskNo::Task1:
		PhysicsEngine::Task1Update(deltaTime, totalTime);
		break;
	case TaskNo::Task2:
		PhysicsEngine::Task2Update(deltaTime, totalTime);
		break;
	case TaskNo::Task3:
		PhysicsEngine::Task3Update(deltaTime, totalTime);
		break;
	case TaskNo::Task4:
		PhysicsEngine::Task4Update(deltaTime, totalTime);
		break;
	}

}

#pragma region Tasks

void PhysicsEngine::Task1Init(Camera& camera, MeshDb& meshDb, ShaderDb& shaderDb)
{
	// Get a few meshes/shaders from the databases
	auto defaultShader = shaderDb.Get("default");

	auto rbMesh = meshDb.Get("cube");
	auto groundMesh = meshDb.Get("plane");
	auto sphereMesh = meshDb.Get("sphere");

	// Initialise ground
	ground.SetMesh(groundMesh);
	ground.SetShader(defaultShader);
	ground.SetScale(vec3(2000.0f));


	rbody1.SetMesh(rbMesh);
	rbody1.SetShader(defaultShader);
	rbody1.SetColor(vec4(1, 0, 0, 1));
	rbody1.SetScale(vec3(1, 3, 1));
	rbody1.SetMass(2.0f);
	rbody1.SetVelocity(vec3(2.0f, 0.0f, 0.0f));
	rbody1.SetAngularVelocity(vec3(0.0f, 0.0f, 0.0f));
	rbody1.SetPosition(vec3(0.0f, 12.0f, 0.0f));
	


	std::cout << "Inverse Inertia Tensor: " << std::endl;
	std::cout << glm::to_string(rbody1.InverseInertia()) << std::endl;
	camera = Camera(vec3(0, 5, 10));
}
void PhysicsEngine::Task1Update(float deltaTime, float totalTime)
{
	// Calculate forces, then acceleration, then integrate
	rbody1.ClearForcesImpulses();

	if (totalTime == 2.0f)
	{
		vec3 impulse = vec3(-4.0f, 0.0f, 0.0f);

		vec3 pointOfApplication = vec3(rbody1.Position().x, rbody1.Position().y - 1.0f, rbody1.Position().z);
		rbody1.SetVelocity(rbody1.Velocity() + impulse / rbody1.Mass());
		rbody1.SetAngularVelocity(rbody1.AngularVelocity() + rbody1.InverseInertia() * glm::cross(pointOfApplication - rbody1.Position(), impulse));


	}
	vec3 acceleration = rbody1.AccumulatedForce() / rbody1.Mass();

	vec3 p = rbody1.Position(), v = rbody1.Velocity();
	SymplecticEuler(p, v, rbody1.Mass(), acceleration, rbody1.AccumulatedImpulse(), deltaTime);
	rbody1.SetPosition(p);
	rbody1.SetVelocity(v);

	Integrate(rbody1, deltaTime);

}

void PhysicsEngine::Task2Init(Camera& camera, MeshDb& meshDb, ShaderDb& shaderDb)
{
	// Get a few meshes/shaders from the databases
	auto defaultShader = shaderDb.Get("default");

	auto rbMesh = meshDb.Get("cube");
	auto groundMesh = meshDb.Get("plane");
	auto sphereMesh = meshDb.Get("sphere");

	// Initialise ground
	ground.SetMesh(groundMesh);
	ground.SetShader(defaultShader);
	ground.SetScale(vec3(2000.0f));


	rbody1.SetMesh(rbMesh);
	rbody1.SetShader(defaultShader);
	rbody1.SetColor(vec4(1, 0, 0, 1));
	rbody1.SetScale(vec3(1, 3, 1));
	rbody1.SetMass(2.0f);
	rbody1.SetVelocity(vec3(0.0f, 0.0f, 0.0f));
	rbody1.SetAngularVelocity(vec3(0.0f, 0.0f, 0.0f));
	rbody1.SetPosition(vec3(0.0f, 12.0f, 0.0f));


	rbody1.UpdateVertices();
	camera = Camera(vec3(0, 5, 10));
}
void PhysicsEngine::Task2Update(float deltaTime, float totalTime)
{
	// Calculate forces, then acceleration, then integrate
	rbody1.ClearForcesImpulses();

	Force::Gravity(rbody1);
	vec3 acceleration = rbody1.AccumulatedForce() / rbody1.Mass();

	vec3 p = rbody1.Position(), v = rbody1.Velocity();
	SymplecticEuler(p, v, rbody1.Mass(), acceleration, rbody1.AccumulatedImpulse(), deltaTime);
	rbody1.SetPosition(p);
	rbody1.SetVelocity(v);

	Integrate(rbody1, deltaTime);
	rbody1.UpdateVertices();
	CollisionImpulse(rbody1, 0.7f);
}

void PhysicsEngine::Task3Init(Camera& camera, MeshDb& meshDb, ShaderDb& shaderDb)
{
	// Get a few meshes/shaders from the databases
	auto defaultShader = shaderDb.Get("default");

	auto rbMesh = meshDb.Get("cube");
	auto groundMesh = meshDb.Get("plane");
	auto sphereMesh = meshDb.Get("sphere");

	// Initialise ground
	ground.SetMesh(groundMesh);
	ground.SetShader(defaultShader);
	ground.SetScale(vec3(2000.0f));


	rbody1.SetMesh(rbMesh);
	rbody1.SetShader(defaultShader);
	rbody1.SetColor(vec4(1, 0, 0, 1));
	rbody1.SetScale(vec3(1, 3, 1));
	rbody1.SetMass(2.0f);
	rbody1.SetVelocity(vec3(0.0f, 0.0f, 0.0f));
	rbody1.SetAngularVelocity(vec3(0.1f, 0.1f, 0.1f));

	rbody2.SetMesh(rbMesh);
	rbody2.SetShader(defaultShader);
	rbody2.SetColor(vec4(0, 1, 0, 1));
	rbody2.SetScale(vec3(1, 3, 1));
	rbody2.SetMass(2.0f);
	rbody2.SetVelocity(vec3(0.0f, 0.0f, 0.0f));
	rbody2.SetAngularVelocity(vec3(0.0f, 0.0f, 0.5f));


	rbody1.SetPosition(vec3(0.0f, 12.0f, 0.0f));
	rbody2.SetPosition(vec3(6.0f, 12.0f, 0.0f));

	rbody1.UpdateVertices();
	rbody2.UpdateVertices();

	std::cout << "Inverse Inertia Tensor: " << std::endl;
	std::cout << glm::to_string(rbody1.InverseInertia()) << std::endl;



	camera = Camera(vec3(0, 5, 10));
}
void PhysicsEngine::Task3Update(float deltaTime, float totalTime)
{
	// Calculate forces, then acceleration, then integrate
	rbody1.ClearForcesImpulses();
	rbody2.ClearForcesImpulses();
	
	Force::Gravity(rbody1);
	Force::Gravity(rbody2);

	vec3 accelerationRb1 = rbody1.AccumulatedForce() / rbody1.Mass();
	vec3 accelerationRb2 = rbody2.AccumulatedForce() / rbody2.Mass();

	vec3 p1 = rbody1.Position(), v1 = rbody1.Velocity();
	SymplecticEuler(p1, v1, rbody1.Mass(), accelerationRb1, rbody1.AccumulatedImpulse(), deltaTime);
	rbody1.SetPosition(p1);
	rbody1.SetVelocity(v1);

	vec3 p2 = rbody2.Position(), v2 = rbody2.Velocity();
	SymplecticEuler(p2, v2, rbody2.Mass(), accelerationRb2, rbody2.AccumulatedImpulse(), deltaTime);
	rbody2.SetPosition(p2);
	rbody2.SetVelocity(v2);

	Integrate(rbody1, deltaTime);
	Integrate(rbody2, deltaTime);

	rbody1.UpdateVertices();
	rbody2.UpdateVertices();

	CollisionImpulse(rbody1, 0.7f);
	CollisionImpulse(rbody2, 1.0f);
}

void PhysicsEngine::Task4Init(Camera& camera, MeshDb& meshDb, ShaderDb& shaderDb) 
{
	// Get a few meshes/shaders from the databases
	auto defaultShader = shaderDb.Get("default");

	auto rbMesh = meshDb.Get("cube");
	auto groundMesh = meshDb.Get("plane");
	auto sphereMesh = meshDb.Get("sphere");

	// Initialise ground
	ground.SetMesh(groundMesh);
	ground.SetShader(defaultShader);
	ground.SetScale(vec3(50.0f));


	rbody1.SetMesh(rbMesh);
	rbody1.SetShader(defaultShader);
	rbody1.SetColor(vec4(1, 0, 0, 1));
	rbody1.SetScale(vec3(1, 3, 1));
	rbody1.SetMass(2.0f);
	rbody1.SetVelocity(vec3(1.0f, 0.0f, 0.0f));
	rbody1.SetPosition(vec3(0.0f, 6.0f, 0.0f));

	rbody1.UpdateVertices();

	camera = Camera(vec3(0, 5, 10));
}
void PhysicsEngine::Task4Update(float deltaTime, float totalTime)
{
	rbody1.ClearForcesImpulses();

	Force::Gravity(rbody1);
	vec3 acceleration = rbody1.AccumulatedForce() / rbody1.Mass();

	vec3 p = rbody1.Position(), v = rbody1.Velocity();
	SymplecticEuler(p, v, rbody1.Mass(), acceleration, rbody1.AccumulatedImpulse(), deltaTime);
	rbody1.SetPosition(p);
	rbody1.SetVelocity(v);

	Integrate(rbody1, deltaTime);

	rbody1.UpdateVertices();

	CollisionImpulse(rbody1, 0.6f);
	
}

#pragma endregion

// This is called every frame, after Update
void PhysicsEngine::Display(const mat4& viewMatrix, const mat4& projMatrix)
{
	ground.Draw(viewMatrix, projMatrix);
	if(currentTask == TaskNo::Task3)
	{
		rbody1.Draw(viewMatrix, projMatrix);
		rbody2.Draw(viewMatrix, projMatrix);
	}
	else
	{
		rbody1.Draw(viewMatrix, projMatrix);
	}
}

void PhysicsEngine::HandleInputKey(int keyCode, bool pressed)
{
	switch (keyCode)
	{
	case GLFW_KEY_1:
		if (currentTask != TaskNo::Task1)
		{
			currentTask = TaskNo::Task1;
			if (pressed)
				PhysicsEngine::Init(tempCamera, tempMeshDb, tempShaderDb);
		}
		break;
	case GLFW_KEY_2:
		if (currentTask != TaskNo::Task2)
		{
			currentTask = TaskNo::Task2;
			if (pressed)
				PhysicsEngine::Init(tempCamera, tempMeshDb, tempShaderDb);
		}
		break;
	case GLFW_KEY_3:
		if (currentTask != TaskNo::Task3)
		{
			currentTask = TaskNo::Task3;
			if (pressed)
				PhysicsEngine::Init(tempCamera, tempMeshDb, tempShaderDb);
		}
		break;
	case GLFW_KEY_4:
		if (currentTask != TaskNo::Task4)
		{
			currentTask = TaskNo::Task4;
			if (pressed)
				PhysicsEngine::Init(tempCamera, tempMeshDb, tempShaderDb);
		}
		break;
	default:
		break;
	}
}