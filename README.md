# Obstacle-avoidance

## Introduction

The aim of this project is to implement obstacle avoidance for PX4 based drones. this repository contains a ROS implementation of an Obstacle avoidance for the drones. 

> **Note** Currently the implementation is limited to  ROS/Gazebo simulation and is yet to be flight tested.

The implementation uses the VFH algorithm to perform obstacle avoidance. the algorithm maps the local environment and is based on 2D occupancy grid.

#### simulation of avoidance drone

![](resources/avoidance_simulation.gif)

