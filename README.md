# Agoraphilic* robot path planning algorithm
Agoraphilic* is an advanced robot path planning algorithm designed for 2.5D environments, extending the free space force concept of the traditional Agoraphilic algorithm to enable navigation in uneven terrain. It introduces a structured approach for free space identification and terrain-aware navigation.

# Simulation Environment  
This code was tested on Ubuntu 22.04 with ROS 2 Humble, Gazebo 11.10.2, and Python 3.10.12.


## Simulation Environment Models  
The simulation environment models and are located in the [gazebo models ](/gazebo%20models/) folder. Copy the models to the `.gazebo/models` directory. The skid robot platform model used for testing is located in the [gazebo models/robot](/gazebo%20models/robot/) folder. This robot platform includes a simulated depth camera.


<p align="center">
  <img src="media/image/Env_A_1.png" alt="Image 1" width="45%">
  <img src="media/image/terrain2_1.png" alt="Image 2" width="45%">
</p>

<p align="center">
  <img src="media/image/terrain4_1.png" alt="Image 3" width="45%">
  <img src="media/image/Env_B_1.png" alt="Image 4" width="45%">
</p>


## Agoraphilic* Algorithm Script  
The Agoraphilic* algorithm script is located in the [script](/scripts/) folder.  
Modify the initial parameters in the `initParameters()` function to match the environment and robot platform.

## Simulations

<p align="center">
  <img src="media/video/Env_A_T1.gif" alt="Image 1" width="45%">
  <img src="media/video/Env_A_T2.gif" alt="Image 2" width="45%">
</p>

<p align="center">
  <img src="media/video/Env_B_T3.gif" alt="Image 3" width="45%">
  <img src="media/video/Env_B_T6.gif" alt="Image 4" width="45%">
</p>

<p align="center">
  <img src="media/video/Env_A_T3.gif" alt="Image 3" width="45%">
  <img src="media/video/Terrain2_2.gif" alt="Image 4" width="45%">
</p>