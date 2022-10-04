#include "PhysicsEngine.h"
#include "Application.h"
#include "Camera.h"
using namespace glm;

const glm::vec3 GRAVITY = glm::vec3(0, -9.81, 0);

const float AIR_DENSITY = 1.225f;
const float DRAG_COEFF = 0.47f;

enum class TaskNo
{
	Task1,
	Task2,
	Task3
};

// Defaulting To Task 1
TaskNo currentTask = TaskNo::Task1;

//Camera& camera, MeshDb& meshDb, ShaderDb& shaderDb
Camera tempCamera;
MeshDb tempMeshDb;
ShaderDb tempShaderDb;

#pragma region PhysicsCalculations

vec3 BlowDryerForce(const vec3& particlePosition, float cone_y_base, float cone_y_tip, float cone_r_base, float max_force = 100)
{
	vec3 force = { 0,0,0 };

	// We assume that the blow dryer has the center EXACLY on (0.0f, y, 0.0f). The height of the blow dryer can change but the x and the z can't

	// Height to radius = r/h = 1/2

	// Find point on Vertical Axis of particle
	vec3 pointOnVertAxis = vec3(0.0f, particlePosition.y, 0.0f);

	// Calculate relative height between the point on the vertical axis and the tip 
	float relativeHeight = glm::distance(pointOnVertAxis, vec3(0.0f, -2.5f, 0.0f));

	// Calculate maximum radius r = h/2
	float maxRelRadius = relativeHeight / 2;

	// Calculate current radius of particle
	float currentRadius = glm::distance(particlePosition, pointOnVertAxis);

	// Comparing the height of the particle with the maximum relative radius of the particle position
	if ((particlePosition.y <= cone_y_base && particlePosition.y >= cone_y_tip) && currentRadius <= maxRelRadius)
	{

		// Calculation of the angle between the tip of the cone and the particle position
		float dotProd = dot(vec3(0.0f, -2.5f, 0.0f), particlePosition);
		float crossProd = length(vec3(0.0f, -2.5f, 0.0f)) * length(particlePosition);

		// We keep the cosine cause we do not need the radians for future calculations
		float cosAngle = dotProd/crossProd;

		// Height of cone
		float height = cone_y_base - cone_y_tip;

		// Scaling the force depending on how far we are on the Y axis
		float magnitudeVerticalForce = (1 - (relativeHeight / height)) * max_force;

		// Scaling the force depending on how far the radius is from the center
		float magnitudeHorizontalForce = (1 - (currentRadius / cone_r_base)) * max_force; 
		
		// Calculating the resultant force using the angle
		float magnitudeResultantForce = sqrt(pow(magnitudeVerticalForce, 2) + pow(magnitudeHorizontalForce, 2) - 
			2 * magnitudeHorizontalForce * magnitudeVerticalForce * cosAngle);

		// Direction of the force
		vec3 normalizedCenter = normalize(particlePosition - vec3(0.0f, -2.5f, 0.0f));

		force = normalizedCenter * magnitudeResultantForce;
	}
	return force;
}

vec3 Calculate_Forces(vec3 pos, vec3 vel, float mass, const vec3 accel)
{
	vec3 gravityForce = GRAVITY * mass;

	vec3 aeroForce = vec3(0.0f);

	if (glm::length(vel) != 0.0f)
	{
		aeroForce = 0.5f * AIR_DENSITY *
			(glm::length(vel) * glm::length(vel)) *
			DRAG_COEFF * (glm::pi<float>() * 0.1f * 0.1f) *
			(-glm::normalize(vel));
	}

	return gravityForce + aeroForce;
}

void ExplicitEuler(vec3& pos, vec3& vel, float mass, const vec3& accel, const vec3& impulse, float dt)
{
	vec3 newVel = vel + (accel * dt + impulse / mass);

	pos += dt * vel;

	vel = newVel;
}

void SymplecticEuler(vec3& pos, vec3& vel, float mass, const vec3& accel, const vec3& impulse, float dt)
{
	vel += accel * dt + impulse/mass;
	pos += vel * dt;
}

void VerletIntegration(vec3& pos, vec3& vel, float mass, vec3& accel, vec3& impulse, float dt)
{
	pos += vel * dt + (0.5f * accel * dt * dt);
	vec3 newAccel = Calculate_Forces(pos, vel, mass, accel) / mass;
	vel += 0.5f * (accel + newAccel) * dt + impulse / mass;

}

vec3 CollisionImpulse(Particle& pobj, const glm::vec3& cubeCentre, float cubeHalfExtent, float coefficientOfRestitution)
{
	vec3 impulse{ 0.0f };
	vec3 surfaceNorm{ 0.0f };

	for (int i = 0; i < 3; i++)
	{
		if (pobj.Position()[i] >= (cubeCentre[i] + cubeHalfExtent) && pobj.Velocity()[i] > 0.0f)
		{
			surfaceNorm = vec3(0.0f);
			surfaceNorm[i] = -1.0f;
			
			if (i == 0)
			{
				pobj.SetPosition(vec3(pobj.Position().x - pobj.Scale()[i], pobj.Position().y, pobj.Position().z));
			}
			else if (i == 1)
			{
				pobj.SetPosition(vec3(pobj.Position().x, pobj.Position().y - pobj.Scale()[i], pobj.Position().z));
			}
			else if (i == 2)
			{
				pobj.SetPosition(vec3(pobj.Position().x, pobj.Position().y, pobj.Position().z - pobj.Scale()[i]));
			}
		}
		else if (pobj.Position()[i] <= (cubeCentre[i] - cubeHalfExtent) && pobj.Velocity()[i] < 0.0f)
		{
			surfaceNorm = vec3(0.0f);
			surfaceNorm[i] = 1.0f;

			if (i == 0)
			{
				pobj.SetPosition(vec3(pobj.Position().x + pobj.Scale()[i], pobj.Position().y, pobj.Position().z));
			}
			else if (i == 1)
			{
				pobj.SetPosition(vec3(pobj.Position().x, pobj.Position().y + pobj.Scale()[i], pobj.Position().z));
			}
			else if (i == 2)
			{
				pobj.SetPosition(vec3(pobj.Position().x, pobj.Position().y, pobj.Position().z + pobj.Scale()[i]));
			}
		}
	}
	
	impulse = -(1.0f + coefficientOfRestitution) * pobj.Mass() * glm::dot(pobj.Velocity(), surfaceNorm) * surfaceNorm;
	return impulse;
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

#pragma endregion

#pragma region Init(s)

void PhysicsEngine::Task1Init(Camera& camera, MeshDb& meshDb, ShaderDb& shaderDb)
{
	// Get a few meshes / shaders from the databases
	auto defaultShader = shaderDb.Get("default");
	auto particleMesh = meshDb.Get("tetra");


	meshDb.Add("cube", Mesh(MeshDataFromWavefrontObj("resources/models/cube.obj")));
	meshDb.Add("sphere", Mesh(MeshDataFromWavefrontObj("resources/models/sphere.obj")));
	meshDb.Add("cone", Mesh(MeshDataFromWavefrontObj("resources/models/cone.obj")));

	auto groundMesh = meshDb.Get("cube");
	auto mesh = meshDb.Get("sphere");


	// Initialise ground
	ground.SetMesh(groundMesh);
	ground.SetShader(defaultShader);
	ground.SetScale(vec3(2.0f));

	// Initialise particle
	particleSymp.SetMesh(mesh);
	particleSymp.SetShader(defaultShader);
	particleSymp.SetColor(vec4(1, 0, 0, 1));
	particleSymp.SetPosition(vec3(0.0f, 0.0f, 0.0f));
	particleSymp.SetScale(vec3(0.1f));

	particleSymp.SetVelocity(vec3(1.0f, 2.0f, 2.0f));

	camera = Camera(vec3(0, 0, 1.8));

}

void PhysicsEngine::Task2Init(Camera& camera, MeshDb& meshDb, ShaderDb& shaderDb)
{
	// Get a few meshes / shaders from the databases
	auto defaultShader = shaderDb.Get("default");
	auto particleMesh = meshDb.Get("tetra");


	meshDb.Add("cube", Mesh(MeshDataFromWavefrontObj("resources/models/cube.obj")));
	meshDb.Add("sphere", Mesh(MeshDataFromWavefrontObj("resources/models/sphere.obj")));
	meshDb.Add("cone", Mesh(MeshDataFromWavefrontObj("resources/models/cone.obj")));

	auto groundMesh = meshDb.Get("cube");
	auto mesh = meshDb.Get("sphere");


	// Initialise ground
	ground.SetMesh(groundMesh);
	ground.SetShader(defaultShader);
	ground.SetScale(vec3(2.0f));

	// Initialise particles

	// Simple particle (stands still)
	simpleParticle.SetMesh(mesh);
	simpleParticle.SetShader(defaultShader);
	simpleParticle.SetColor(vec4(1, 1, 1, 1));
	simpleParticle.SetPosition(vec3(-0.5f, 0.0f, 0.0f));
	simpleParticle.SetScale(vec3(0.1f));

	simpleParticle.SetVelocity(vec3(0.0f));

	// Symplectic particle
	particleSymp.SetMesh(mesh);
	particleSymp.SetShader(defaultShader);
	particleSymp.SetColor(vec4(1, 0, 0, 1));
	particleSymp.SetPosition(vec3(0.0f, 0.0f, 0.0f));
	particleSymp.SetScale(vec3(0.1f));

	particleSymp.SetVelocity(vec3(0.0f));


	// Explicit particle
	particleFwd.SetMesh(mesh);
	particleFwd.SetShader(defaultShader);
	particleFwd.SetColor(vec4(0, 1, 0, 1));
	particleFwd.SetPosition(vec3(0.5f, 0.0f, 0.0f));
	particleFwd.SetScale(vec3(0.1f));

	particleFwd.SetVelocity(vec3(0.0f));


	// Verlet particle
	particleVerl.SetMesh(mesh);
	particleVerl.SetShader(defaultShader);
	particleVerl.SetColor(vec4(0, 0, 1, 1));
	particleVerl.SetPosition(vec3(1.0f, 0.0f, 0.0f));
	particleVerl.SetScale(vec3(0.1f));

	particleVerl.SetVelocity(vec3(0.0f));

	camera = Camera(vec3(0, 0, 1.8));
}

void PhysicsEngine::Task3Init(Camera& camera, MeshDb& meshDb, ShaderDb& shaderDb)
{
	// Get a few meshes / shaders from the databases
	auto defaultShader = shaderDb.Get("default");
	auto particleMesh = meshDb.Get("tetra");


	meshDb.Add("cube", Mesh(MeshDataFromWavefrontObj("resources/models/cube.obj")));
	meshDb.Add("sphere", Mesh(MeshDataFromWavefrontObj("resources/models/sphere.obj")));
	meshDb.Add("cone", Mesh(MeshDataFromWavefrontObj("resources/models/cone.obj")));

	auto groundMesh = meshDb.Get("cube");
	auto mesh = meshDb.Get("sphere");
	auto blowDryerMesh = meshDb.Get("cone");

	// Initialise ground
	ground.SetMesh(groundMesh);
	ground.SetShader(defaultShader);
	ground.SetScale(vec3(2.0f));

	blowDryer.SetMesh(blowDryerMesh);
	blowDryer.SetShader(defaultShader);
	blowDryer.SetScale(vec3(2.0f));
	blowDryer.SetColor(vec4(0.0f, 0.0f, 0.0f, 0.8f));
	blowDryer.SetPosition(vec3(0.0f, -0.5f, 0.0f));
	blowDryer.Rotate(glm::radians(180.0f), vec3(1.0f, 0.0f, 0.0f));

	// Initialise particle
	particleSymp.SetMesh(mesh);
	particleSymp.SetShader(defaultShader);
	particleSymp.SetColor(vec4(1, 0, 0, 1));
	particleSymp.SetPosition(vec3(0.5f, -1.0f, 0.0f));
	particleSymp.SetScale(vec3(0.1f));

	particleSymp.SetVelocity(vec3(0.0f));

	camera = Camera(vec3(0, 0, 1.8));
}

#pragma endregion

#pragma region Update(s)

void PhysicsEngine::Task1Update(float deltaTime, float totalTime)
{
	// Calculating impulse
	auto impulse = CollisionImpulse(particleSymp, glm::vec3(0.0f, 0.0f, 0.0f), 2.0f, 0.85f);

	// Each particle has the same area (using the Symp for getting the scale value)
	float area = glm::pi<float>() * particleSymp.Scale().x * particleSymp.Scale().x;

	// AeroForce calculation. Null checking the velocity so that It doesn't return NaN
	vec3 aeroForce = vec3(0.0f);
	if (glm::length(particleSymp.Velocity()) != 0.0f)
	{
		aeroForce = 0.5f * AIR_DENSITY *
			(glm::length(particleSymp.Velocity()) * glm::length(particleSymp.Velocity())) *
			DRAG_COEFF * area *
			(-glm::normalize(particleSymp.Velocity()));
	}

	// Calculating gravity force
	vec3 gravityForce = GRAVITY * particleSymp.Mass();
	// Calculating acceleration
	vec3 acceleration = (gravityForce + aeroForce) / particleSymp.Mass();

	// Symplectic Integration
	vec3 p = particleSymp.Position(), v = particleSymp.Velocity();
	SymplecticEuler(p, v, particleSymp.Mass(), acceleration, impulse, deltaTime);

	particleSymp.SetPosition(p);
	particleSymp.SetVelocity(v);

}

void PhysicsEngine::Task2Update(float deltaTime, float totalTime)
{

	// IMPULSE CALCULATION(S)

	// Symplectic
	auto impulseSymp = CollisionImpulse(particleSymp, glm::vec3(0.0f, 0.0f, 0.0f), 2.0f, 0.85f);

	// Explicit
	auto impulseFwd = CollisionImpulse(particleFwd, glm::vec3(0.0f, 0.0f, 0.0f), 2.0f, 0.85f);

	// Verlet
	auto impulseVerl = CollisionImpulse(particleVerl, glm::vec3(0.0f, 0.0f, 0.0f), 2.0f, 0.85f);

	// Assuming the area is the same for all particles
	float area = glm::pi<float>() * particleSymp.Scale().x * particleSymp.Scale().x;


	// AERODYNAMIC FORCE CALCULATION(S)

	// Init Aereodynamic Force for Symplectic
	vec3 aeroForceSymp = vec3(0.0f);

	// Init Aereodynamic Force for Explicit
	vec3 aeroForceFwd = vec3(0.0f);

	// Init Aereodynamic Force for Verlet
	vec3 aeroForceVerl = vec3(0.0f);

	// Calculating Aerodynamic Symplectic 
	if (glm::length(particleSymp.Velocity()) != 0.0f)
	{
		aeroForceSymp = 0.5f * AIR_DENSITY *
			(glm::length(particleSymp.Velocity()) * glm::length(particleSymp.Velocity())) *
			DRAG_COEFF * area *
			(-glm::normalize(particleSymp.Velocity()));
	}

	// Calculating Aerodynamic Explicit
	if (glm::length(particleFwd.Velocity()) != 0.0f)
	{
		aeroForceFwd = 0.5f * AIR_DENSITY *
			(glm::length(particleFwd.Velocity()) * glm::length(particleFwd.Velocity())) *
			DRAG_COEFF * area *
			(-glm::normalize(particleFwd.Velocity()));
	}

	// Calculating Aerodynamic Verlet
	if (glm::length(particleVerl.Velocity()) != 0.0f)
	{
		aeroForceVerl = 0.5f * AIR_DENSITY *
			(glm::length(particleVerl.Velocity()) * glm::length(particleVerl.Velocity())) *
			DRAG_COEFF * area *
			(-glm::normalize(particleVerl.Velocity()));
	}

	// GRAVITY FORCE CALCULATION(S)

	// Symplectic calculation
	vec3 gravityForceSymp = GRAVITY * particleSymp.Mass();

	// Explicit calculation
	vec3 gravityForceFwd = GRAVITY * particleFwd.Mass();

	// Verlet calculation
	vec3 gravityForceVerl = GRAVITY * particleVerl.Mass();

	// ACCELERATION CALCULATION(S)

	// Symplectic calculation
	vec3 accelerationSymp = (gravityForceSymp + aeroForceSymp) / particleSymp.Mass();

	// Explicit calculation
	vec3 accelerationFwd = (gravityForceFwd + aeroForceFwd) / particleFwd.Mass();

	// Verlet calculation
	vec3 accelerationVerl = (gravityForceVerl + aeroForceVerl) / particleVerl.Mass();

	// FINAL RESULTS
	
	// Symplectic final
	vec3 pSymp = particleSymp.Position(), vSymp = particleSymp.Velocity();
	SymplecticEuler(pSymp, vSymp, particleSymp.Mass(), accelerationSymp, impulseSymp, deltaTime);
	particleSymp.SetPosition(pSymp);
	particleSymp.SetVelocity(vSymp);

	// Explicit final
	vec3 pFwd = particleFwd.Position(), vFwd = particleFwd.Velocity(); 
	ExplicitEuler(pFwd, vFwd, particleFwd.Mass(), accelerationFwd, impulseFwd, deltaTime);
	particleFwd.SetPosition(pFwd);
	particleFwd.SetVelocity(vFwd);

	// Verlet final
	vec3 pVerl = particleVerl.Position(), vVerl = particleVerl.Velocity();
	VerletIntegration(pVerl, vVerl, particleVerl.Mass(), accelerationVerl, impulseVerl, deltaTime);
	particleVerl.SetPosition(pVerl);
	particleVerl.SetVelocity(vVerl);
}

void PhysicsEngine::Task3Update(float deltaTime, float totalTime)
{
	// ANIMATE PARTICLE WITH SYMP
	
	auto impulseSymp = CollisionImpulse(particleSymp, glm::vec3(0.0f, 0.0f, 0.0f), 2.0f, 0.85f);

	vec3 aeroForceSymp = vec3(0.0f);

	float area = glm::pi<float>() * particleSymp.Scale().x * particleSymp.Scale().x;

	
	if (glm::length(particleSymp.Velocity()) != 0.0f)
	{
		aeroForceSymp = 0.5f * AIR_DENSITY *
			(glm::length(particleSymp.Velocity()) * glm::length(particleSymp.Velocity())) *
			DRAG_COEFF * area *
			(-glm::normalize(particleSymp.Velocity()));
	}

	vec3 blowDryer = BlowDryerForce(particleSymp.Position(), -0.5f, -2.5f, 1.0f);
	vec3 gravityForceSymp = GRAVITY * particleSymp.Mass();
	vec3 accelerationSymp = (gravityForceSymp + aeroForceSymp + blowDryer) / particleSymp.Mass();

	// Symplectic integration
	vec3 pSymp = particleSymp.Position(), vSymp = particleSymp.Velocity();
	SymplecticEuler(pSymp, vSymp, particleVerl.Mass(), accelerationSymp, impulseSymp, deltaTime);
	particleSymp.SetPosition(pSymp);
	particleSymp.SetVelocity(vSymp);


}

#pragma endregion

// This is called once
void PhysicsEngine::Init(Camera& camera, MeshDb& meshDb, ShaderDb& shaderDb)
{
	tempCamera = camera;
	tempMeshDb = meshDb;
	tempShaderDb = shaderDb;

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
	}
}

// This is called every frame, after Update
void PhysicsEngine::Display(const mat4& viewMatrix, const mat4& projMatrix)
{
	if (currentTask == TaskNo::Task1)
	{
		particleSymp.Draw(viewMatrix, projMatrix);
	}
	
	if (currentTask == TaskNo::Task2)
	{
		simpleParticle.Draw(viewMatrix, projMatrix);
		particleSymp.Draw(viewMatrix, projMatrix);
		particleFwd.Draw(viewMatrix, projMatrix);
		particleVerl.Draw(viewMatrix, projMatrix);
	}

	if (currentTask == TaskNo::Task3)
	{
		particleSymp.Draw(viewMatrix, projMatrix);
		blowDryer.Draw(viewMatrix, projMatrix);
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
	}


}