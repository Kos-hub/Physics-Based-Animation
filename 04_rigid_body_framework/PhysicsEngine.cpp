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
const float coeffOfFriction = 2.0f;

std::vector<Particle> vertices;

enum class TaskNo
{
	Task1,
	Task2,
	Task3,
	Task4
};

TaskNo currentTask = TaskNo::Task1;

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
		std::cout << "Face!" << std::endl;
		pointOfApplication = GetPointOfContactForSide(touchingPositions);
		break;

	}

	TranslateWhenPenetrating(rb, pointOfApplication);

	vec3 r = pointOfApplication - rb.Position();
	std::cout << "Point of Application " << glm::to_string(pointOfApplication) << std::endl;
	float magnitudeOfImpulse = CalculateImpulseRigidBody(rb, r, surfNorm, coefficientOfRestitution);
	//vec3 impulse = magnitudeOfImpulse * surfNorm;
	

	vec3 relativeVelocity = rb.Velocity() + glm::cross(rb.AngularVelocity(), r);
	std::cout << "Relative velocity: " << glm::to_string(relativeVelocity) << std::endl;
	vec3 vt = relativeVelocity - (glm::dot(relativeVelocity, surfNorm)) * surfNorm;
	std::cout << "Vt is " << glm::to_string(glm::normalize(vt)) << std::endl;


	vec3 jt = -coeffOfFriction * magnitudeOfImpulse * glm::normalize(vt);

	std::cout << "JR = " << glm::to_string(magnitudeOfImpulse * surfNorm) << std::endl;
	std::cout << "JT = " << glm::to_string(jt) << std::endl;
	std::cout << std::endl;

	//vec3 newVel = rb.Velocity() + (magnitudeOfImpulse / rb.Mass()) * surfNorm;
	//rb.SetVelocity(newVel);

	//vec3 newAngVel = rb.AngularVelocity() + magnitudeOfImpulse * rb.InverseInertia() * (glm::cross(r, surfNorm));
	//rb.SetAngularVelocity(newAngVel);

	//vec3 angularVelocity = rb.AngularVelocity() + rb.InverseInertia() * glm::cross(r, impulse+jt);
	//rb.SetAngularVelocity(angularVelocity);
	//rb.ApplyImpulse(impulse+jt);



	vec3 jr = magnitudeOfImpulse * surfNorm;

	rb.SetVelocity(rb.Velocity() + (jr + jt) / rb.Mass());
	rb.SetAngularVelocity(rb.AngularVelocity() + rb.InverseInertia() * (glm::cross(r, jr + jt)));
	std::cout << "AngularVelocity: " << glm::to_string(rb.AngularVelocity()) << std::endl;
	std::cout << "Linear Velocity: " << glm::to_string(rb.Velocity()) << std::endl;
	std::cout << std::endl;
}


// This is called once
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
	//case TaskNo::Task2:
	//	PhysicsEngine::Task2Init(camera, meshDb, shaderDb);
	//	break;
	case TaskNo::Task3:
		PhysicsEngine::Task3Init(camera, meshDb, shaderDb);
		break;
	//case TaskNo::Task4:
	//	PhysicsEngine::Task4Init(camera, meshDb, shaderDb);
	//	break;
	//case TaskNo::Task5:
	//	PhysicsEngine::Task5Init(camera, meshDb, shaderDb);
	//	break;
	}



}

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
	ground.SetScale(vec3(20.0f));


	rbody1.SetMesh(rbMesh);
	rbody1.SetShader(defaultShader);
	rbody1.SetColor(vec4(1, 0, 0, 1));
	rbody1.SetScale(vec3(1, 3, 1));
	rbody1.SetMass(2.0f);
	rbody1.SetVelocity(vec3(2.0f, 0.0f, 0.0f));
	//rbody1.Rotate(0.8f, vec3(1.0f, 1.0f, 0.0f));
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
		rbody1.SetAngularVelocity(rbody1.AngularVelocity() + rbody1.InverseInertia() * glm::cross(pointOfApplication - rbody1.Position(), impulse));


		rbody1.ApplyImpulse(impulse);
	}
	vec3 acceleration = rbody1.AccumulatedForce() / rbody1.Mass();

	vec3 p = rbody1.Position(), v = rbody1.Velocity();
	SymplecticEuler(p, v, rbody1.Mass(), acceleration, rbody1.AccumulatedImpulse(), deltaTime);
	rbody1.SetPosition(p);
	rbody1.SetVelocity(v);

	Integrate(rbody1, deltaTime);

	std::cout << glm::to_string(rbody1.Position()) << std::endl;
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
	ground.SetScale(vec3(50.0f));


	rbody1.SetMesh(rbMesh);
	rbody1.SetShader(defaultShader);
	rbody1.SetColor(vec4(1, 0, 0, 1));
	rbody1.SetScale(vec3(1, 3, 1));
	rbody1.SetMass(200.0f);
	rbody1.SetVelocity(vec3(2.0f, 0.0f, 0.0f));
	//rbody1.Rotate(0.8f, vec3(1.0f, 1.0f, 0.0f));
	//rbody1.Rotate(0.8f, vec3(1.0f, 1.0f, 0.0f));
	//rbody1.SetAngularVelocity(vec3(0.0f, 0.0f, 0.5f));
	rbody1.SetPosition(vec3(0.0f, 3.0f, 0.0f));


	std::cout << "Inverse Inertia Tensor: " << std::endl;
	std::cout << glm::to_string(rbody1.InverseInertia()) << std::endl;
	camera = Camera(vec3(0, 5, 10));
}
void PhysicsEngine::Task3Update(float deltaTime, float totalTime)
{
	rbody1.ClearForcesImpulses();

	Force::Gravity(rbody1);
	vec3 acceleration = rbody1.AccumulatedForce() / rbody1.Mass();

	vec3 p = rbody1.Position(), v = rbody1.Velocity();
	SymplecticEuler(p, v, rbody1.Mass(), acceleration, rbody1.AccumulatedImpulse(), deltaTime);
	rbody1.SetPosition(p);
	rbody1.SetVelocity(v);

	Integrate(rbody1, deltaTime);

	CollisionImpulse(rbody1, vec3(0.0f), 0.0f, 1.0f);

	
}

// This is called every frame
void PhysicsEngine::Update(float deltaTime, float totalTime)
{
	switch (currentTask)
	{
	case TaskNo::Task1:
		PhysicsEngine::Task1Update(deltaTime, totalTime);
		break;
	//case TaskNo::Task2:
	//	PhysicsEngine::Task2Update(deltaTime, totalTime);
	//	break;
	case TaskNo::Task3:
		PhysicsEngine::Task3Update(deltaTime, totalTime);
		break;
	//case TaskNo::Task4:
	//	PhysicsEngine::Task4Update(deltaTime, totalTime);
	//	break;
	//case TaskNo::Task5:
	//	PhysicsEngine::Task5Update(deltaTime, totalTime);
	//	break;
	}

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