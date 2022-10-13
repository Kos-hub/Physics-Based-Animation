#include "PhysicsEngine.h"
#include "Application.h"
#include "Camera.h"
#include "Force.h"

using namespace glm;

const glm::vec3 GRAVITY = glm::vec3(0, -9.81, 0);


//Camera& camera, MeshDb& meshDb, ShaderDb& shaderDb
Camera tempCamera;
MeshDb tempMeshDb;
ShaderDb tempShaderDb;

std::vector<Particle> listOfParticles;

enum class TaskNo
{
	Task1,
	Task2,
	Task3
};

// Defaulting To Task 1
TaskNo currentTask = TaskNo::Task1;

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

vec3 CollisionImpulse(Particle& pobj, const glm::vec3& cubeCentre, float cubeHalfExtent, float coefficientOfRestitution = 0.9f)
{
	vec3 impulse{ 0.0f };
	vec3 surfaceNorm{ 0.0f };

	for (int i = 0; i < 3; i++)
	{
		if (pobj.Position()[i] + pobj.Scale()[i] >= (cubeCentre[i] + cubeHalfExtent))
		{
			surfaceNorm = vec3(0.0f);
			surfaceNorm[i] = -1.0f;

			if (i == 0)
			{
				pobj.SetPosition(vec3(cubeCentre[i] + cubeHalfExtent - pobj.Scale().x, pobj.Position().y, pobj.Position().z));
			}
			else if (i == 1)
			{
				pobj.SetPosition(vec3(pobj.Position().x, cubeCentre[i] + cubeHalfExtent - pobj.Scale().y, pobj.Position().z));
			}
			else if (i == 2)
			{
				pobj.SetPosition(vec3(pobj.Position().x, pobj.Position().y, cubeCentre[i] + cubeHalfExtent - pobj.Scale().z));
			}
		}
		else if (pobj.Position()[i] - pobj.Scale()[i] <= (cubeCentre[i] - cubeHalfExtent))
		{
			surfaceNorm = vec3(0.0f);
			surfaceNorm[i] = 1.0f;

			if (i == 0)
			{
				pobj.SetPosition(vec3(cubeCentre[i] - cubeHalfExtent + pobj.Scale().x, pobj.Position().y, pobj.Position().z));
			}
			else if (i == 1)
			{
				pobj.SetPosition(vec3(pobj.Position().x, cubeCentre[i] - cubeHalfExtent + pobj.Scale().y, pobj.Position().z));
			}
			else if (i == 2)
			{
				pobj.SetPosition(vec3(pobj.Position().x, pobj.Position().y, cubeCentre[i] - cubeHalfExtent + pobj.Scale().z));
			}
		}
	}

	impulse = -(1.0f + coefficientOfRestitution) * pobj.Mass() * glm::dot(pobj.Velocity(), surfaceNorm) * surfaceNorm;
	return impulse;
}

// This is called once
void PhysicsEngine::Init(Camera& camera, MeshDb& meshDb, ShaderDb& shaderDb)
{
	tempCamera = camera;
	tempMeshDb = meshDb;
	tempShaderDb = shaderDb;

	// Get a few meshes / shaders from the databases
	meshDb.Add("cube", Mesh(MeshDataFromWavefrontObj("resources/models/cube.obj")));
	meshDb.Add("sphere", Mesh(MeshDataFromWavefrontObj("resources/models/ball.obj")));
	meshDb.Add("cone", Mesh(MeshDataFromWavefrontObj("resources/models/cone.obj")));

	switch (currentTask)
	{
	case TaskNo::Task1:
		PhysicsEngine::Task1Init(camera, meshDb, shaderDb);
		break;
	case TaskNo::Task2:
		//PhysicsEngine::Task2Init(camera, meshDb, shaderDb);
		break;
	case TaskNo::Task3:
		//PhysicsEngine::Task3Init(camera, meshDb, shaderDb);
		break;
	}
}

void PhysicsEngine::Task1Init(Camera& camera, MeshDb& meshDb, ShaderDb& shaderDb)
{
	// Get a few meshes/shaders from the databases
	auto defaultShader = shaderDb.Get("default");

	meshDb.Add("cube", Mesh(MeshDataFromWavefrontObj("resources/models/cube.obj")));
	meshDb.Add("ball", Mesh(MeshDataFromWavefrontObj("resources/models/ball.obj")));
	meshDb.Add("cone", Mesh(MeshDataFromWavefrontObj("resources/models/cone.obj")));

	auto groundMesh = meshDb.Get("cube");
	auto sphereMesh = meshDb.Get("ball");
	
	// Initialise ground
	ground.SetMesh(groundMesh);
	ground.SetShader(defaultShader);
	ground.SetScale(vec3(2.0f));

	anchor.SetMesh(sphereMesh);
	anchor.SetShader(defaultShader);
	anchor.SetColor(vec4(0, 1, 0, 1));
	anchor.SetPosition(vec3(0.0f, 0.0f, 0.0f));
	anchor.SetScale(vec3(0.1f));
	anchor.SetMass(1.0f);
	anchor.SetVelocity(vec3(0.0f));
	anchor.SetFixed();
	listOfParticles.push_back(anchor);

	sphere1.SetMesh(sphereMesh);
	sphere1.SetShader(defaultShader);
	sphere1.SetColor(vec4(1, 0, 0, 1));
	sphere1.SetPosition(vec3(0.0f, -0.5f, 0.0f));
	sphere1.SetScale(vec3(0.1f));
	sphere1.SetMass(1.0f);
	sphere1.SetVelocity(vec3(0.0f));
	listOfParticles.push_back(sphere1);

	sphere2.SetMesh(sphereMesh);
	sphere2.SetShader(defaultShader);
	sphere2.SetColor(vec4(0, 0, 1, 1));
	sphere2.SetPosition(vec3(0.0f, -1.0f, 0.0f));
	sphere2.SetScale(vec3(0.1f));
	sphere2.SetMass(1.0f);
	sphere2.SetVelocity(vec3(0.0f));
	listOfParticles.push_back(sphere2);

	sphere3.SetMesh(sphereMesh);
	sphere3.SetShader(defaultShader);
	sphere3.SetColor(vec4(1, 1, 0, 1));
	sphere3.SetPosition(vec3(0.0f, -1.5f, 0.0f));
	sphere3.SetScale(vec3(0.1f));
	sphere3.SetMass(1.0f);
	sphere3.SetVelocity(vec3(0.0f));
	listOfParticles.push_back(sphere3);
	



	camera = Camera(vec3(0, 0, 1.8));
}

void PhysicsEngine::Task1Update(float deltaTime, float totalTime)
{
	// Calculate forces, then acceleration, then integrate
	//for (int i = 0; i < listOfParticles.size(); i++)
	//{
	//	listOfParticles[i].ClearForcesImpulses();
	//	if (!listOfParticles[i].IsFixed())
	//	{
	//		Force::Gravity(listOfParticles[i]);
	//		Force::Hooke(listOfParticles[i], listOfParticles[i-1], 0.5f, 2.0f, 0.0f);

	//		vec3 acceleration = listOfParticles[i].AccumulatedForce() / listOfParticles[i].Mass();

	//		vec3 p = listOfParticles[i].Position(), v = listOfParticles[i].Velocity();
	//		SymplecticEuler(p, v, listOfParticles[i].Mass(), acceleration, vec3(0.0f), deltaTime);
	//		listOfParticles[i].SetPosition(p);
	//		listOfParticles[i].SetVelocity(v);
	//	}


	//}
	sphere1.ClearForcesImpulses();
	sphere2.ClearForcesImpulses();

	Force::Gravity(sphere1);
	Force::Gravity(sphere2);

	Force::Hooke(sphere1, anchor, 0.5f, 2.0f, 0.0f);
	Force::Hooke(sphere2, anchor, 0.5f, 2.0f, 0.0f);

	vec3 acceleration1 = sphere1.AccumulatedForce() / sphere1.Mass();
	vec3 acceleration2 = sphere2.AccumulatedForce() / sphere2.Mass();

	vec3 p = sphere1.Position(), v = sphere1.Velocity();
	SymplecticEuler(p, v, sphere1.Mass(), acceleration1, vec3(0.0f), deltaTime);
	sphere1.SetPosition(p);
	sphere1.SetVelocity(v);

	p = sphere2.Position(), v = sphere2.Velocity();
	SymplecticEuler(p, v, sphere2.Mass(), acceleration2, vec3(0.0f), deltaTime);
	sphere2.SetPosition(p);
	sphere2.SetVelocity(v);
}


// This is called every frame
void PhysicsEngine::Update(float deltaTime, float totalTime)
{
	switch (currentTask)
	{
	case TaskNo::Task1:
		PhysicsEngine::Task1Update(deltaTime, totalTime);
		break;
	case TaskNo::Task2:
		//PhysicsEngine::Task2Update(deltaTime, totalTime);
		break;
	case TaskNo::Task3:
		//PhysicsEngine::Task3Update(deltaTime, totalTime);
		break;
	}
}

// This is called every frame, after Update
void PhysicsEngine::Display(const mat4& viewMatrix, const mat4& projMatrix)
{
	switch (currentTask)
	{
	case TaskNo::Task1:
		//for (Particle p : listOfParticles)
		//{
		//	p.Draw(viewMatrix, projMatrix);
		//}
		sphere1.Draw(viewMatrix, projMatrix);
		sphere2.Draw(viewMatrix, projMatrix);
		anchor.Draw(viewMatrix, projMatrix);
		break;
	}

	//ground.Draw(viewMatrix, projMatrix);
	
}

void PhysicsEngine::HandleInputKey(int keyCode, bool pressed)
{
	switch (keyCode)
	{
	case GLFW_KEY_1:
		currentTask = TaskNo::Task1;
		if (pressed)
			PhysicsEngine::Init(tempCamera, tempMeshDb, tempShaderDb);
		break;
	case GLFW_KEY_2:
		currentTask = TaskNo::Task2;
		if (pressed)
			PhysicsEngine::Init(tempCamera, tempMeshDb, tempShaderDb);
		break;
	case GLFW_KEY_3:
		currentTask = TaskNo::Task3;
		if (pressed)
			PhysicsEngine::Init(tempCamera, tempMeshDb, tempShaderDb);
		break;
	}
}