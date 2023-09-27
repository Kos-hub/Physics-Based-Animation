# Physics-Based-Animation
These are various courseworks I done for my Physics-Based Animation module. You will need glew32.dll and glfw3.dll to run the code.

## 01_particle_animation
Animating particles by making them bounce on a floor.
![Alt Text](https://github.com/Kos-hub/Physics-Based-Animation/blob/master/videos/practical_1.gif)

## 02_particles_framework
There are 4 different tasks, that can be seen in different scenarios by pressing the 1, 2, 3 and 4 buttons. <br />
Task 1 = Particle bouncing in a cube. I implemented the code for detecting collisions inside a cube and I have also implemented gravity and aerodynamic drag. <br />
![Alt Text](https://github.com/Kos-hub/Physics-Based-Animation/blob/master/videos/practical_2_1.gif)
Task 2 = 3 different particles bouncing with 3 different integration methods (Forward Euler, Semi-implicit Euler and Verlet Integration). <br />
![Alt Text](https://github.com/Kos-hub/Physics-Based-Animation/blob/master/videos/practical_2_2.gif)
Task 3 = Implementing a time step in Application.cpp in order to create a "physics tick" that would make the physic simulation not framerate dependent. <br />
![Alt Text](https://github.com/Kos-hub/Physics-Based-Animation/blob/master/videos/practical_2_3.gif)
Task 4 = Implementing a blow dryer force that would push a particle upwards, just like a blow dryer.
![Alt Text](https://github.com/Kos-hub/Physics-Based-Animation/blob/master/videos/practical_2_4.gif)

## 03_constraints_framework
There are 5 different tasks, that can be seen in different scenarios by pressing the 1, 2, 3, 4 and 5 buttons. <br />
Task 1 = Chain of particles connected by springs. Hooke force is implemented. <br />
![Alt Text](https://github.com/Kos-hub/Physics-Based-Animation/blob/master/videos/practical_3_1.gif)
Task 2 = Chain of 10 particles connected to each other by a damped spring. The two extremities of the chain are fixed. <br />
![Alt Text](https://github.com/Kos-hub/Physics-Based-Animation/blob/master/videos/practical_3_2.gif)
Task 3 = Chain of 10 particles connected to each other that collide with a plane. <br />
![Alt Text](https://github.com/Kos-hub/Physics-Based-Animation/blob/master/videos/practical_3_3.gif)
Task 4 = Square piece of cloth (4 corners fixed). 10 x 10 Particle array located at the same height. All affected by gravity and collision. <br />
![Alt Text](https://github.com/Kos-hub/Physics-Based-Animation/blob/master/videos/practical_3_4.gif)
Task 5 = Square piece of cloth with a blow dryer that passes through the scene pushing the particles away (Wind Simulation).
![Alt Text](https://github.com/Kos-hub/Physics-Based-Animation/blob/master/videos/practical_3_5.gif)

## 04_rigid_body_framework
There are 4 different tasks, that can be seen in different scenarios by pressing the 1, 2, 3 and 4 buttons. <br />
Task 1 = Rigid body that moves along the x axis with a velocity of 2 m/s. After 2 seconds, an impulse is applied that makes the body stop and spin clockwise. <br />
![Alt Text](https://github.com/Kos-hub/Physics-Based-Animation/blob/master/videos/practical_4_1.gif)
Task 2 = Detecting which vertex collides with plane. <br />
![Alt Text](https://github.com/Kos-hub/Physics-Based-Animation/blob/master/videos/practical_4_2.gif)
Task 3 = Collision response between the body and the plane using impulse-based response. <br />
![Alt Text](https://github.com/Kos-hub/Physics-Based-Animation/blob/master/videos/practical_4_3.gif)
Task 4 = Adding friction to the rigid body.
![Alt Text](https://github.com/Kos-hub/Physics-Based-Animation/blob/master/videos/practical_4_4.gif)
