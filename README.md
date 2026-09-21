# INDRA – Adaptive Autonomous Navigation for Unstructured Indian Roads

### SIH26037

> Adaptive Path Planning and Collision Avoidance for Autonomous Vehicles on Unstructured Indian Roads

## 🚗 Overview

INDRA is a software-in-the-loop autonomous driving prototype designed for challenging Indian road environments where lane markings may be missing or unreliable and traffic consists of heterogeneous road users such as cars, auto-rickshaws, pedestrians, two-wheelers and animals.

The system combines multi-sensor perception, sensor fusion, uncertainty-aware trajectory prediction, collision-risk assessment and adaptive path planning to continuously generate safe and vehicle-feasible trajectories.

## 🎯 Key Features

- Lane-independent navigation
- Camera, LiDAR and Radar sensor simulation
- Multi-sensor fusion
- Dynamic object trajectory prediction
- Uncertainty-aware risk assessment
- Time-to-Collision (TTC) analysis
- Behavior-based decision making
- Multiple candidate trajectory generation
- Safety-constrained path selection
- Continuous adaptive replanning
- Kinematic bicycle vehicle model

## 🧠 System Pipeline

Environment
↓
Sensor Simulation
↓
Sensor Fusion
↓
Trajectory Prediction
↓
TTC & Risk Assessment
↓
Behavior Decision
↓
Adaptive Path Planning
↓
Pure Pursuit Control
↓
Kinematic Bicycle Model
↓
Vehicle State Feedback

## 🛣️ Target Scenarios

1. Unmarked Village Road
2. Busy Unsignalized Urban Intersection
3. Highway Merge with Slow Vehicles
4. Dense Market Area
5. Sudden Cattle Crossing

## 🛠️ Technologies

- MATLAB
- Simulink
- Stateflow
- Automated Driving Toolbox
- Navigation Toolbox
- Vehicle Dynamics / Kinematic Bicycle Model

## 📊 Validation Metrics

The prototype is evaluated using:

- Collision-free performance
- Minimum safety distance
- Time-to-Collision
- Scenario completion
- Replanning behavior
- Average vehicle speed
- Path safety and smoothness

## 🎥 Project Demonstration

[▶ Watch the SIH26037 Project Demo](https://youtu.be/dJH7PX443VA?si=m1vJzlJiHVjaQJVL)

## 📁 Repository Structure

```text
MATLAB/          MATLAB simulation and algorithm files
Simulink/        Simulink system model
Demo/            Demonstration links
