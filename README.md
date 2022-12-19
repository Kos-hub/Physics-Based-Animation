# Physics-Based-Animation
These are various courseworks I done for my Physics-Based Animation module.

## 01_particle_animation
Animating spheres by making them bounce on a floor.

## 02_particles_framework
There are 4 different tasks, that can be seen in different scenarios by pressing the 1,2, 3 and 4 buttons. <br />
Task 1 = Particle bouncing in a cube. I implemented the code for detecting collisions inside a cube and I have also implemented gravity and aerodynamic drag. <br />
Task 2 = 3 different particles bouncing with 3 different integration methods (Forward Euler, Semi-implicit Euler and Verlet Integration). <br />
Task 3 = Implementing a time step in Application.cpp in order to create a "physics tick" that would make the physic simulation not framerate dependent. <br />
Task 4 = Implementing a blow dryer force that would push a particle upwards, just like a blow dryer.

## 03_constraints_framework
There are 5 different tasks, that can be seen in different scenarios by pressing the 1,2, 3, 4 and 5 buttons. <br />
Task 1 = Chain of particles connected by springs. Hooke force is implemented. <br />
Task 2 = Chain of 10 particles connected to each other by a damped spring. The two extremities of the chain are fixed. <br />
Task 3 = Chain of 10 particles connected to each other that collide with a plane. <br />
Task 4 = Square piece of cloth (4 corners fixed). 10 x 10 Particle array located at the same height. All affected by gravity and collision. <br />
Task 5 = Square piece of cloth with a blow dryer that passes through the scene pushing the particles away (Wind Simulation).

## 04_rigid_body_framework
There are 4 different tasks, that can be seen in different scenarios by pressing the 1,2, 3 and 4 buttons. <br />
Task 1 = Rigid body that moves along the x axis with a velocity of 2 m/s. After 2 seconds, an impulse is applied that makes the body stop and spin clockwise. <br />
Task 2 = Detecting which vertex collides with plane. <br />
Task 3 = Collision response between the body and the plane using impulse-based response. <br />
Task 4 = Adding friction to the rigid body.
