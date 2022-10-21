#include "PhysicsEngine.h"
#include "Force.h"
#include "PhysicsObject.h"

using namespace glm;
const float AIR_DENSITY = 1.225f;
const float DRAG_COEFF = 0.47f;
const float kS = 1.0f;


void Force::Gravity(Particle& p)
{
	auto force = vec3(0, -9.81, 0) * p.Mass();
	p.ApplyForce(force);
}

void Force::Drag(Particle& p)
{
	// Each particle has the same area (using the Symp for getting the scale value)
	float area = glm::pi<float>() * p.Scale().x * p.Scale().x;

	// AeroForce calculation. Null checking the velocity so that It doesn't return NaN
	vec3 aeroForce = vec3(0.0f);

	if (glm::length(p.Velocity()) != 0.0f)
	{
		aeroForce = 0.5f * AIR_DENSITY *
			(glm::length(p.Velocity()) * glm::length(p.Velocity())) *
			DRAG_COEFF * area *
			(-glm::normalize(p.Velocity()));
	}

	p.ApplyForce(aeroForce);
}

void Force::Hooke(Particle& p1, Particle& p2, float restLength, float ks, float kd)
{
	// Compute distance between P1 and P2
	float distance = glm::distance(p2.Position(), p1.Position());

	// Compute the unit vector e from P1 and P2
	vec3 unitVector1 = glm::normalize(p2.Position() - p1.Position());
	vec3 unitVector2 = glm::normalize(p1.Position() - p2.Position());

	// Compute 1D velocities with dot prod
	float p1Vel = glm::dot(unitVector1, p1.Velocity());
	float p2Vel = glm::dot(unitVector2, p2.Velocity());

	// Back to 3D with Fsd force calculation
	float fsd1 = -ks * (restLength - distance) - kd * p1Vel;
	float fsd2 = -ks * (restLength - distance) - kd * p2Vel;

	p1.ApplyForce(fsd1 * unitVector1);
	p2.ApplyForce(fsd2 * unitVector2);
}

void Force::BlowDryerForce(Particle& p, float cone_z_base, float cone_z_tip, float cone_r_base, float max_force = 100)
{
	vec3 force = { 0,0,0 };

	// We assume that the blow dryer has the center EXACLY on (0.0f, y, 0.0f). The height of the blow dryer can change but the x and the z can't

	// Height to radius = r/h = 1/2

	// Find point on Vertical Axis of particle
	vec3 pointOnVertAxis = vec3(0.0f, -0.5f, p.Position().z);

	// Calculate relative height between the point on the vertical axis and the tip 
	float relativeHeight = glm::distance(pointOnVertAxis, vec3(0.0f, -0.5f, cone_z_tip));

	// Calculate maximum radius r = h/2
	float maxRelRadius = relativeHeight / 2;

	// Calculate current radius of particle
	float currentRadius = glm::distance(p.Position(), pointOnVertAxis);

	// Comparing the height of the particle with the maximum relative radius of the particle position
	if ((p.Position().z <= cone_z_base && p.Position().z >= cone_z_tip) && currentRadius <= maxRelRadius)
	{

		// Calculation of the angle between the tip of the cone and the particle position
		float dotProd = dot(vec3(0.0f, -0.5f, cone_z_tip), p.Position());
		float crossProd = length(vec3(0.0f, -0.5f, cone_z_tip)) * length(p.Position());

		// We keep the cosine cause we do not need the radians for future calculations
		float cosAngle = dotProd / crossProd;

		// Height of cone
		float height = cone_z_base - cone_z_tip;

		// Scaling the force depending on how far we are on the Y axis
		float magnitudeVerticalForce = (1 - (relativeHeight / height)) * max_force;

		// Scaling the force depending on how far the radius is from the center
		float magnitudeHorizontalForce = (1 - (currentRadius / cone_r_base)) * max_force;

		// Calculating the resultant force using the angle
		float magnitudeResultantForce = sqrt(pow(magnitudeVerticalForce, 2) + pow(magnitudeHorizontalForce, 2) -
			2 * magnitudeHorizontalForce * magnitudeVerticalForce * cosAngle);

		// Direction of the force
		vec3 normalizedCenter = normalize(p.Position() - vec3(0.0f, -0.5f, cone_z_tip));

		force = normalizedCenter * magnitudeResultantForce;
	}

	p.ApplyForce(force);
}