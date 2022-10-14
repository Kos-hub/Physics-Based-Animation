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
	Task3,
	Task4
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
		PhysicsEngine::Task2Init(camera, meshDb, shaderDb);
		break;
	case TaskNo::Task3:
		PhysicsEngine::Task3Init(camera, meshDb, shaderDb);
		break;
	case TaskNo::Task4:
		PhysicsEngine::Task4Init(camera, meshDb, shaderDb);
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
	ground.SetScale(vec3(15.0f));

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
	sphere1.SetPosition(vec3(0.0f, -0.1f, 0.0f));
	sphere1.SetScale(vec3(0.1f));
	sphere1.SetMass(1.0f);
	sphere1.SetVelocity(vec3(0.0f));
	listOfParticles.push_back(sphere1);

	sphere2.SetMesh(sphereMesh);
	sphere2.SetShader(defaultShader);
	sphere2.SetColor(vec4(0, 0, 1, 1));
	sphere2.SetPosition(vec3(0.0f, -0.2f, 0.0f));
	sphere2.SetScale(vec3(0.1f));
	sphere2.SetMass(1.0f);
	sphere2.SetVelocity(vec3(0.0f));
	listOfParticles.push_back(sphere2);

	sphere3.SetMesh(sphereMesh);
	sphere3.SetShader(defaultShader);
	sphere3.SetColor(vec4(1, 1, 0, 1));
	sphere3.SetPosition(vec3(0.0f, -0.3f, 0.0f));
	sphere3.SetScale(vec3(0.1f));
	sphere3.SetMass(1.0f);
	sphere3.SetVelocity(vec3(0.0f));
	listOfParticles.push_back(sphere3);
	

	sphere4.SetMesh(sphereMesh);
	sphere4.SetShader(defaultShader);
	sphere4.SetColor(vec4(1, 0, 1, 1));
	sphere4.SetPosition(vec3(0.0f, -0.4f, 0.0f));
	sphere4.SetScale(vec3(0.1f));
	sphere4.SetMass(1.0f);
	sphere4.SetVelocity(vec3(0.0f));
	listOfParticles.push_back(sphere4);

	camera = Camera(vec3(0, 0, 1.8));
}

void PhysicsEngine::Task1Update(float deltaTime, float totalTime)
{
	// Calculate forces, then acceleration, then integrate
	for (int i = 0; i < listOfParticles.size(); i++)
	{
		if (!listOfParticles[i].IsFixed())
		{
			listOfParticles[i].ClearForcesImpulses();
			Force::Gravity(listOfParticles[i]);

			for(int j = 1; j < listOfParticles.size(); j++)
				Force::Hooke(listOfParticles[j], listOfParticles[j-1], 0.5f, 20.0f, 0.2f);

			Force::Drag(listOfParticles[i]);
			vec3 acceleration = listOfParticles[i].AccumulatedForce() / listOfParticles[i].Mass();

			vec3 p = listOfParticles[i].Position(), v = listOfParticles[i].Velocity();
			SymplecticEuler(p, v, listOfParticles[i].Mass(), acceleration, vec3(0.0f), deltaTime);
			listOfParticles[i].SetPosition(p);
			listOfParticles[i].SetVelocity(v);
		}


	}
}


void PhysicsEngine::Task2Init(Camera& camera, MeshDb& meshDb, ShaderDb& shaderDb)
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
	ground.SetScale(vec3(15.0f));

	for (int i = 0; i < 10; i++)
	{
		Particle p;
		p.SetMesh(sphereMesh);
		p.SetShader(defaultShader);
		p.SetColor(vec4(rand() % 2, rand() % 2, rand() % 2, 1));
		p.SetPosition(vec3(10.0f - i, 0.0f, 0.0f));
		p.SetScale(vec3(0.1f));
		p.SetMass(1.0f);
		p.SetVelocity(vec3(0.0f));
		if (i == 0 || i == 9)
			p.SetFixed();
		task2Particles.push_back(p);

	}
	camera = Camera(vec3(0, 0, 1.8));
}

void PhysicsEngine::Task2Update(float deltaTime, float totalTime)
{
	for (int i = 0; i < task2Particles.size(); i++)
	{
		if (!task2Particles[i].IsFixed())
		{
			task2Particles[i].ClearForcesImpulses();
			Force::Gravity(task2Particles[i]);

			for (int j = 1; j < task2Particles.size(); j++)
				Force::Hooke(task2Particles[j], task2Particles[j - 1], 0.5f, 20.0f, 0.2f);

			Force::Drag(task2Particles[i]);
			vec3 acceleration = task2Particles[i].AccumulatedForce() / task2Particles[i].Mass();

			vec3 p = task2Particles[i].Position(), v = task2Particles[i].Velocity();
			SymplecticEuler(p, v, task2Particles[i].Mass(), acceleration, vec3(0.0f), deltaTime);
			task2Particles[i].SetPosition(p);
			task2Particles[i].SetVelocity(v);
		}
	}
}


void PhysicsEngine::Task3Init(Camera& camera, MeshDb& meshDb, ShaderDb& shaderDb)
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
	ground.SetScale(vec3(15.0f));

	for (int i = 0; i < 10; i++)
	{
		Particle p;
		p.SetMesh(sphereMesh);
		p.SetShader(defaultShader);
		p.SetColor(vec4(rand() % 2, rand() % 2, rand() % 2, 1));
		p.SetPosition(vec3(10.0f - i, -10.0f, 0.0f));
		p.SetScale(vec3(0.1f));
		p.SetMass(1.0f);
		p.SetVelocity(vec3(0.0f));
		if (i == 0 || i == 9)
			p.SetFixed();
		task3Particles.push_back(p);

	}
	camera = Camera(vec3(0, 0, 1.8));
}

void PhysicsEngine::Task3Update(float deltaTime, float totalTime)
{
	for (int i = 0; i < task3Particles.size(); i++)
	{
		if (!task3Particles[i].IsFixed())
		{			
			task3Particles[i].ClearForcesImpulses();

			auto impulse = CollisionImpulse(task3Particles[i], vec3(0.0f), 15.0f, 0.85f);

			Force::Gravity(task3Particles[i]);

			for (int j = 1; j < task3Particles.size(); j++)
				Force::Hooke(task3Particles[j], task3Particles[j - 1], 0.5f, 20.0f, 0.2f);

			Force::Drag(task3Particles[i]);
			vec3 acceleration = task3Particles[i].AccumulatedForce() / task3Particles[i].Mass();

			vec3 p = task3Particles[i].Position(), v = task3Particles[i].Velocity();
			SymplecticEuler(p, v, task3Particles[i].Mass(), acceleration, impulse, deltaTime);
			task3Particles[i].SetPosition(p);
			task3Particles[i].SetVelocity(v);
		}
	}
}


void PhysicsEngine::Task4Init(Camera& camera, MeshDb& meshDb, ShaderDb& shaderDb)
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
	ground.SetScale(vec3(15.0f));

	for (int i = 0; i < 10; i++)
	{
		for (int j = 0; j < 10; j++)
		{
			Particle p;
			p.SetMesh(sphereMesh);
			p.SetShader(defaultShader);
			p.SetColor(vec4(rand() % 2, rand() % 2, rand() % 2, 1));
			p.SetPosition(vec3(10.0f - i, 0.0f, 10.0f - j));
			p.SetScale(vec3(0.1f));
			p.SetMass(1.0f);
			p.SetVelocity(vec3(0.0f));
			if ((i == 9 && j == 9) || (i==0 && j ==9) || (i==0 && j==0) || (i==9 && j == 0))
				p.SetFixed();

			task4Particles[i][j] = p;
		}
	}
}

void PhysicsEngine::Task4Update(float deltaTime, float totalTime)
{
	float diagonalRest = glm::sqrt(pow(0.5, 2) + pow(0.5, 2));
	for (int i = 0; i < 10; i++)
	{
		for (int j = 0; j < 10; j++)
		{
			task4Particles[i][j].ClearForcesImpulses();
		}
	}

	// Horizontal hooke forces
	for (int i = 1; i < 10; i++)
	{
		for (int j = 0; j < 10; j++)
		{
			Force::Hooke(task4Particles[j][i], task4Particles[j][i - 1], 0.5f, 40.0f, 0.5f);

		}
	}

	// Vertical hooke forces
	for (int i = 1; i < 10; i++)
	{
		for (int j = 0; j < 10; j++)
		{
			Force::Hooke(task4Particles[i][j], task4Particles[i - 1][j], 0.5f, 40.0f, 0.5f);
		}
	}

	//// Positive diagonal hooke forces
	//for (int i = 1; i < 10; i++)
	//{
	//	for (int j = 0; j < 9; j++)
	//	{
	//		Force::Hooke(task4Particles[j + 1][i], task4Particles[j + 1][i - 1], diagonalRest, 40.0f, 0.5f);
	//	}
	//}

	//// Negative diagonal hooke forces
	//for (int i = 1; i < 10; i++)
	//{
	//	for (int j = 0; j < 9; j++)
	//	{
	//		Force::Hooke(task4Particles[i][j+1], task4Particles[i - 1][j+1], diagonalRest, 40.0f, 0.5f);
	//	}
	//}

	for (int i = 0; i < 10; i++)
	{
		for (int j = 0; j < 10; j++)
		{
			if (!task4Particles[i][j].IsFixed())
			{
				auto impulse = CollisionImpulse(task4Particles[i][j], vec3(0.0f), 15.0f, 0.85f);

				Force::Gravity(task4Particles[i][j]);
				Force::Drag(task4Particles[i][j]);
				
				vec3 acceleration = task4Particles[i][j].AccumulatedForce() / task4Particles[i][j].Mass();

				vec3 p = task4Particles[i][j].Position(), v = task4Particles[i][j].Velocity();
				SymplecticEuler(p, v, task4Particles[i][j].Mass(), acceleration, impulse, deltaTime);
				task4Particles[i][j].SetPosition(p);
				task4Particles[i][j].SetVelocity(v);
			}
		}
	}
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

// This is called every frame, after Update
void PhysicsEngine::Display(const mat4& viewMatrix, const mat4& projMatrix)
{
	switch (currentTask)
	{
	case TaskNo::Task1:
		for (Particle p : listOfParticles)
			p.Draw(viewMatrix, projMatrix);

		break;
	case TaskNo::Task2:
		for (Particle p : task2Particles)
			p.Draw(viewMatrix, projMatrix);
		break;
	case TaskNo::Task3:
		for (Particle p : task3Particles)
			p.Draw(viewMatrix, projMatrix);
		break;
	case TaskNo::Task4:
		for (int i = 0; i < 10; i++)
		{
			for (int j = 0; j < 10; j++)
			{
				task4Particles[i][j].Draw(viewMatrix, projMatrix);
			}
		}
		break;
	}

	ground.Draw(viewMatrix, projMatrix);
	
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
	case GLFW_KEY_4:
		currentTask = TaskNo::Task4;
		if (pressed)
			PhysicsEngine::Init(tempCamera, tempMeshDb, tempShaderDb);
		break;
	}
}