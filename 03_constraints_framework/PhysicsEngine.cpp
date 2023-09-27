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

const float SPRING_STIFFNESS = 40.0f;
const float SPRING_DAMP = 0.5;
const float REST_DISTANCE = 1.5;
enum class TaskNo
{
	Task1,
	Task2,
	Task3,
	Task4,
	Task5
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
	meshDb.Add("cone", Mesh(MeshDataFromWavefrontObj("C:/Users/giana/Documents/GitHub/Physics-Based-Animation/02_particles_framework/resources/models/cone.obj")));

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
	case TaskNo::Task5:
		PhysicsEngine::Task5Init(camera, meshDb, shaderDb);
		break;
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
	case TaskNo::Task5:
		PhysicsEngine::Task5Update(deltaTime, totalTime);
		break;
	}
}


void PhysicsEngine::Task1Init(Camera& camera, MeshDb& meshDb, ShaderDb& shaderDb)
{
	// Get a few meshes/shaders from the databases
	auto defaultShader = shaderDb.Get("default");

	auto groundMesh = meshDb.Get("cube");
	auto sphereMesh = meshDb.Get("sphere");
	
	// Initialise ground
	ground.SetMesh(groundMesh);
	ground.SetShader(defaultShader);
	ground.SetScale(vec3(15.0f));

	for (int i = 0; i < 5; i++)
	{
		Particle p;
		p.SetMesh(sphereMesh);
		p.SetShader(defaultShader);
		p.SetColor(vec4(1, 0, 0, 1));
		p.SetPosition(vec3(0.0f, i * -0.1, 0.0f));
		p.SetScale(vec3(0.1f));
		p.SetMass(1.0f);
		p.SetVelocity(vec3(0.0f));
		if (i == 0)
			p.SetFixed();
		task1Particles.push_back(p);
	}
	camera = Camera(vec3(0, 0, 10));
}

void PhysicsEngine::Task1Update(float deltaTime, float totalTime)
{
	// Calculate forces, then acceleration, then integrate
	for (int i = 0; i < task1Particles.size(); i++)
	{
		if (!task1Particles[i].IsFixed())
		{
			task1Particles[i].ClearForcesImpulses();
			Force::Gravity(task1Particles[i]);

			for(int j = 1; j < task1Particles.size(); j++)
				Force::Hooke(task1Particles[j], task1Particles[j-1], 0.5f, SPRING_STIFFNESS, SPRING_DAMP);

			Force::Drag(task1Particles[i]);
			vec3 acceleration = task1Particles[i].AccumulatedForce() / task1Particles[i].Mass();

			vec3 p = task1Particles[i].Position(), v = task1Particles[i].Velocity();
			SymplecticEuler(p, v, task1Particles[i].Mass(), acceleration, vec3(0.0f), deltaTime);
			task1Particles[i].SetPosition(p);
			task1Particles[i].SetVelocity(v);
		}


	}
}


void PhysicsEngine::Task2Init(Camera& camera, MeshDb& meshDb, ShaderDb& shaderDb)
{
	// Get a few meshes/shaders from the databases
	auto defaultShader = shaderDb.Get("default");

	auto groundMesh = meshDb.Get("cube");
	auto sphereMesh = meshDb.Get("sphere");

	// Initialise ground
	ground.SetMesh(groundMesh);
	ground.SetShader(defaultShader);
	ground.SetScale(vec3(15.0f));

	for (int i = 0; i < 10; i++)
	{
		Particle p;
		p.SetMesh(sphereMesh);
		p.SetShader(defaultShader);
		p.SetColor(vec4(0, 1, 0, 1));
		p.SetPosition(vec3(10.0f - i, 0.0f, 0.0f));
		p.SetScale(vec3(0.1f));
		p.SetMass(1.0f);
		p.SetVelocity(vec3(0.0f));
		if (i == 0 || i == 9)
			p.SetFixed();
		task2Particles.push_back(p);

	}
	camera = Camera(vec3(0, 0, 10));
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
				Force::Hooke(task2Particles[j], task2Particles[j - 1], 0.5f, SPRING_STIFFNESS, SPRING_DAMP);

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

	auto groundMesh = meshDb.Get("cube");
	auto sphereMesh = meshDb.Get("sphere");

	// Initialise ground
	ground.SetMesh(groundMesh);
	ground.SetShader(defaultShader);
	ground.SetScale(vec3(15.0f));

	for (int i = 0; i < 10; i++)
	{
		Particle p;
		p.SetMesh(sphereMesh);
		p.SetShader(defaultShader);
		p.SetColor(vec4(0, 0, 1, 1));
		p.SetPosition(vec3(10.0f - i, -10.0f, 0.0f));
		p.SetScale(vec3(0.1f));
		p.SetMass(1.0f);
		p.SetVelocity(vec3(0.0f));
		if (i == 0 || i == 9)
			p.SetFixed();
		task3Particles.push_back(p);

	}
	camera = Camera(vec3(0, 0, 10));
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
				Force::Hooke(task3Particles[j], task3Particles[j - 1], 0.5f, SPRING_STIFFNESS, SPRING_DAMP);

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

	auto groundMesh = meshDb.Get("cube");
	auto sphereMesh = meshDb.Get("sphere");

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
			p.SetColor(vec4(1, 1, 0, 1));
			p.SetPosition(vec3(10.0f - i, -10.0f, 10.0f - j));
			p.SetScale(vec3(0.1f));
			p.SetMass(1.0f);
			p.SetVelocity(vec3(0.0f));
			if ((i == 9 && j == 9) || (i==0 && j ==9) || (i==0 && j==0) || (i==9 && j == 0))
				p.SetFixed();
			task4Particles[i][j] = p;
		}
	}

	camera = Camera(vec3(0, 0, 10));
}

void PhysicsEngine::Task4Update(float deltaTime, float totalTime)
{
	float diagonalRest = glm::sqrt(pow(REST_DISTANCE, 2) + pow(REST_DISTANCE, 2));
	for (int i = 0; i < 10; i++)
	{
		for (int j = 0; j < 10; j++)
		{
			task4Particles[i][j].ClearForcesImpulses();
		}
	}

	// "Vertical" forces
	for (int i = 1; i < 10; i++)
	{
		for (int j = 0; j < 10; j++)
		{
			Force::Hooke(task4Particles[i][j], task4Particles[i-1][j], REST_DISTANCE, SPRING_STIFFNESS, SPRING_DAMP);
		}
	}

	// "Horizontal" Forces
	for (int i = 0; i < 10; i++)
	{
		for (int j = 1; j < 10; j++)
		{
			Force::Hooke(task4Particles[i][j], task4Particles[i][j - 1], REST_DISTANCE, SPRING_STIFFNESS, SPRING_DAMP);
		}
	}

	// Positive diagonal
	for (int i = 1; i < 10; i++)
	{
		for (int j = 1; j < 10; j++)
		{
			Force::Hooke(task4Particles[i][j], task4Particles[i-1][j-1], diagonalRest, SPRING_STIFFNESS, SPRING_DAMP);
		}
	}

	// Negative diagonal
	for (int i = 0; i < 9; i++)
	{
		for (int j = 1; j < 10; j++)
		{
			Force::Hooke(task4Particles[i+1][j-1], task4Particles[i][j], diagonalRest, SPRING_STIFFNESS, 0.5f);
		}
	}

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


void PhysicsEngine::Task5Init(Camera& camera, MeshDb& meshDb, ShaderDb& shaderDb)
{
	// Get a few meshes/shaders from the databases
	auto defaultShader = shaderDb.Get("default");

	auto groundMesh = meshDb.Get("cube");
	auto sphereMesh = meshDb.Get("sphere");
	auto blowDryerMesh = meshDb.Get("cone");

	// Initialise ground
	ground.SetMesh(groundMesh);
	ground.SetShader(defaultShader);
	ground.SetScale(vec3(15.0f));

	// Initialise blow dryer
	blowDryer.SetMesh(blowDryerMesh);
	blowDryer.SetShader(shaderDb.Get("default"));
	blowDryer.SetScale(vec3(10.0f));
	blowDryer.SetColor(vec4(0.5, 0, 0, 0.8f));
	blowDryer.SetPosition(vec3(0.0f, -0.5f, -15.0f));
	blowDryer.Rotate(glm::radians(-90.0f), vec3(1.0f, 0.0f, 0.0f));


	for (int i = 0; i < 10; i++)
	{
		for (int j = 0; j < 10; j++)
		{
			Particle p;
			p.SetMesh(sphereMesh);
			p.SetShader(defaultShader);
			p.SetColor(vec4(1, 0, 1, 1));
			p.SetPosition(vec3(5.0f - j, 6.0f - i, -10.0f));
			p.SetScale(vec3(0.1f));
			p.SetMass(1.0f);
			p.SetVelocity(vec3(0.0f));
			if (i==0)
				p.SetFixed();
			task5Particles[i][j] = p;
		}
	}

	camera = Camera(vec3(0, 0, 10));
}
void PhysicsEngine::Task5Update(float deltaTime, float totalTime) 
{
	float diagonalRest = glm::sqrt(pow(REST_DISTANCE, 2) + pow(REST_DISTANCE, 2));
	for (int i = 0; i < 10; i++)
	{
		for (int j = 0; j < 10; j++)
		{
			task5Particles[i][j].ClearForcesImpulses();
		}
	}

	// "Vertical" forces
	for (int i = 1; i < 10; i++)
	{
		for (int j = 0; j < 10; j++)
		{
			Force::Hooke(task5Particles[i][j], task5Particles[i - 1][j], REST_DISTANCE, SPRING_STIFFNESS, SPRING_DAMP);
		}
	}

	// "Horizontal" Forces
	for (int i = 0; i < 10; i++)
	{
		for (int j = 1; j < 10; j++)
		{
			Force::Hooke(task5Particles[i][j], task5Particles[i][j - 1], REST_DISTANCE, SPRING_STIFFNESS, SPRING_DAMP);
		}
	}

	// Positive diagonal
	for (int i = 1; i < 10; i++)
	{
		for (int j = 1; j < 10; j++)
		{
			Force::Hooke(task5Particles[i][j], task5Particles[i - 1][j - 1], diagonalRest, SPRING_STIFFNESS, SPRING_DAMP);
		}
	}

	// Negative diagonal
	for (int i = 0; i < 9; i++)
	{
		for (int j = 1; j < 10; j++)
		{
			Force::Hooke(task5Particles[i + 1][j - 1], task5Particles[i][j], diagonalRest, SPRING_STIFFNESS, 0.5f);
		}
	}

	for (int i = 0; i < 10; i++)
	{
		for (int j = 0; j < 10; j++)
		{
			if (!task5Particles[i][j].IsFixed())
			{
				auto impulse = CollisionImpulse(task5Particles[i][j], vec3(0.0f), 15.0f, 0.85f);

				Force::Gravity(task5Particles[i][j]);
				Force::Drag(task5Particles[i][j]);
				Force::BlowDryerForce(task5Particles[i][j], blowDryer.Position().z, blowDryer.Position().z - 10.0f, 5.0f, 100.0f);

				vec3 acceleration = task5Particles[i][j].AccumulatedForce() / task5Particles[i][j].Mass();

				vec3 p = task5Particles[i][j].Position(), v = task5Particles[i][j].Velocity();
				SymplecticEuler(p, v, task5Particles[i][j].Mass(), acceleration, impulse, deltaTime);
				task5Particles[i][j].SetPosition(p);
				task5Particles[i][j].SetVelocity(v);
			}
		}
	}
	
	if (blowDryer.Position().z >= 25.0f)
		blowDryer.SetPosition(vec3(0.0f, -0.5f, -15.0f));
	blowDryer.SetPosition(vec3(blowDryer.Position().x, blowDryer.Position().y, blowDryer.Position().z + 0.1f));
}


// This is called every frame, after Update
void PhysicsEngine::Display(const mat4& viewMatrix, const mat4& projMatrix)
{
	switch (currentTask)
	{
	case TaskNo::Task1:
		for (Particle p : task1Particles)
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
	case TaskNo::Task5:
		for (int i = 0; i < 10; i++)
		{
			for (int j = 0; j < 10; j++)
			{
				task5Particles[i][j].Draw(viewMatrix, projMatrix);
			}
		}
		blowDryer.Draw(viewMatrix, projMatrix);
		break;
	}

	ground.Draw(viewMatrix, projMatrix);
	
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
	case GLFW_KEY_5:

		if (currentTask != TaskNo::Task5)
		{
			currentTask = TaskNo::Task5;

			if (pressed)
				PhysicsEngine::Init(tempCamera, tempMeshDb, tempShaderDb);
		}
		break;
	}
}