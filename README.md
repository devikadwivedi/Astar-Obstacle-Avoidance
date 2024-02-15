# A* Obstacle Avoidance

## Introduction

This repository contains a Python implementation of an A* heuristic search algorithm, designed for solving a pathfinding problem involving a mobile agent navigating around obstacles. The problem scenario involves an agent positioned at a start point and tasked with reaching a goal point, while avoiding obstacles represented as rectangles in a 2D space. The A* algorithm efficiently computes the shortest path from the start point to the goal point while avoiding collision with obstacles.

## Problem Description

The agent, located at a starting point (point A), must navigate to a target point (point C) without intersecting any of the obstacles present in the environment. The obstacles are defined as rectangular regions in the 2D space, and the agent can only move between vertices of these rectangles or along their edges. The objective is to find a path that minimizes the total distance traveled from the start point to the goal point, while adhering to the constraints imposed by the obstacle layout.

## Implementation Details

The Python implementation utilizes the A* search algorithm to explore possible paths from the start point to the goal point. The state space consists of coordinates representing the agent's location, along with additional information such as the cost incurred so far (g-value), the estimated cost to reach the goal (h-value), and the combined cost (f-value). The algorithm maintains an open list and a closed list to efficiently explore and evaluate candidate states. Obstacle avoidance is enforced by checking for intersections between the agent's path and the obstacle boundaries.

## How to Run

To execute the algorithm, follow these steps:

1. Ensure you have Python 3 installed on your system.
2. Download the provided Python script (`main.py`) to your local machine.
3. Prepare an input file following the specified format:
   - The first line contains the coordinates of the start point.
   - The second line contains the coordinates of the goal point.
   - The third line contains the number of obstacles present.
   - Subsequent lines specify the vertices of each obstacle.
4. Run the script by executing the following command in your terminal or command prompt:
   ```
   python main.py input_file.txt
   ```
   Replace `input_file.txt` with the path to your input file.
5. The program will output the optimal path from the start point to the goal point, along with the cumulative cost of traversal for each point in the path.
