# Robotic Arm sorting shapes ROS2

This project demonstrates the design, simulation, and real-life implementation of a robotic arm that sorts shapes into suitable bins using ROS2, Gazebo, and RViz. The arm was designed in SolidWorks, converted to URDF, simulated using Gazebo, and implemented with a 3D-printed structure powered by a Raspberry Pi and servo motors. Image processing is used to identify and classify the shapes for sorting.

<p align="center">
  <img src="https://github.com/user-attachments/assets/424a5f33-f143-4e4a-8472-dc625fa8d144" width="400" />
</p>

# Project overview
**1. Design and Modeling**  
   The robotic arm was designed in SolidWorks using accurate kinematic modeling.
   Exported from SolidWorks to URDF (Unified Robot Description Format) for ROS2 compatibility.
   
**2. Visualization and Simulation using RViz and Gazebo**  
   Joints and links were visualized and debugged using RViz.
   The URDF model was imported into Gazebo for physics-based simulation.
   Verified realistic joint movement and motor control in the simulated environment.

**3. 3D Printing and Real-World Implementation**  
   The robotic arm parts were 3D-printed using PLA material.
   Connected to a Raspberry Pi for control and processing.

**4. Shape Detection with Image Processing**  
   Camera attached to the robotic arm.
   Used OpenCV to process live images and identify shapes.
   
