# Butler Robot Navigation Using ROS 2 and Nav2

## Overview

This project implements an autonomous mobile robot acting as a café butler.  
The robot performs navigation tasks using ROS 2 Humble and the Nav2 stack.  
The solution focuses on task level behavior planning while delegating motion planning and control to Nav2.

The robot starts from a home position travels to the kitchen collects orders delivers food to customer tables and returns safely to home while handling failures timeouts and cancellations.

The implementation is generic data driven and extendable to additional scenarios.

---




## Implementation
![2026-02-1321-36-47-ezgif com-crop](https://github.com/user-attachments/assets/aa398f59-a93c-413c-b492-619b1876df55)
![2026-02-1321-39-06-ezgif com-crop](https://github.com/user-attachments/assets/7bb76dd9-648c-4d85-b2d4-114468e4cf64)


## Problem Scope

The system is designed to support a restaurant workflow with three tables.  
Orders are issued with a target table identifier.  
The robot autonomously manages navigation and task execution.

The following capabilities are implemented

1. Single table delivery  
2. Waiting behavior with timeout  
3. Conditional return logic based on confirmation  
4. Task cancellation handling  
5. Multiple table delivery in one run  
6. Skipping tables without confirmation  
7. Skipping canceled table orders  

All behaviors are handled at the task logic level without hardcoding motion paths.

---

## System Architecture

The system follows a layered robotics architecture.

### Navigation Layer

Handled entirely by Nav2

Responsibilities  
Global path planning  
Local obstacle avoidance  
Velocity control  
Goal execution  
Recovery behaviors  

Key components  
AMCL for localization  
Map server  
Planner server  
Controller server  
Behavior tree navigator  

The task manager never directly controls robot motion.

---

### Task Logic Layer

Implemented in a custom ROS 2 C plus plus node.

Responsibilities  
Order sequencing  
State transitions  
Timeout handling  
Failure detection  
Cancellation handling  

The task manager interacts with Nav2 exclusively through the NavigateToPose action interface.

---

### Data Layer

Semantic locations are defined externally.

Locations include  
Home  
Kitchen  
Table1  
Table2  
Table3  

All poses are captured from the AMCL pose topic after successful manual navigation.

This ensures only reachable poses are used.

---

## Node Description

### Butler Task Manager Node

Node name  
butler_task_manager  

Core behavior  
Initializes a map of named semantic locations  
Sends navigation goals sequentially using Nav2 actions  
Waits for goal result  
Implements decision logic based on result and timers  

The node is designed to be extended with additional states or input interfaces.

---

## Behavior Execution Flow

Single order flow  
Home to kitchen  
Kitchen to table  
Table to home  

Multiple order flow  
Home to kitchen  
Kitchen to table sequence  
Kitchen to home  

Conditional flows  
Timeout at kitchen returns home  
Timeout at table returns kitchen then home  
Cancellation during navigation handled based on current state  

---

## Confirmation and Timeout Handling

Confirmation is simulated using timers.  
This approach is intentional and allowed by the task description.

Timers represent human interaction latency.

Timeout duration is configurable and does not affect system stability.

---

## Robot Model and Simulation

The system uses a mobile robot description compatible with Nav2.

Key features  
Valid URDF  
Correct TF tree  
Nav2 compatible footprint  
Simulation ready  

All navigation behavior was verified in simulation using RViz and Gazebo.

---

## Development Environment

This project is developed and tested inside a Docker container running on Windows using WSLg.

### Base Environment

Operating system  
Ubuntu 22.04  

ROS distribution  
ROS 2 Humble  

CUDA base image  
nvidia cuda 12.2.0 devel ubuntu 22.04  

---

### Docker Setup

The container installs  
ROS 2 desktop full  
Nav2  
Gazebo ROS packages  
Colcon build tools  

A non root user is created for development.

The ROS environment is sourced automatically.

---

### VS Code Dev Container

The project is intended to be opened using VS Code Dev Containers.

Key features  
GPU access enabled  
Privileged mode enabled  
Desktop access via noVNC or VNC  
ROS extensions pre installed  

WSLg provides GUI support for RViz and Gazebo on Windows.

---

## Build Instructions

Open the repository in VS Code  
Reopen in Dev Container  
Build using colcon  

```bash
colcon build
source install setup bash
```

---

## Running the System

Start Nav2 bringup  
Launch the butler task manager node  


```bash
ros2 run butler_task_manager butler_task_manager_node
```

The robot will execute the configured task flow autonomously.

---

## Design Principles

Separation of concerns  
Navigation delegated to Nav2  
Behavior handled at task level  

Data driven configuration  
No hardcoded motion paths  

Failure aware design  
All failures are expected and handled  

Extendable architecture  
Additional tables or states can be added easily  

---

## Assumptions

Confirmation is simulated  
Single robot operation  
Static map environment  

These assumptions are documented and aligned with the assessment requirements.

---

## Conclusion

This project demonstrates a complete ROS 2 mobile robot solution with task level intelligence layered on top of a production grade navigation stack.

The implementation emphasizes correctness robustness extensibility and clarity.

It satisfies all functional requirements defined in the assessment and provides a strong foundation for future expansion.

