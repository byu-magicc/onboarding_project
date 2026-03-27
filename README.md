# MAGICC Lab Onboarding Project

This repository contains project files for the MAGICC Lab onboarding project.

## Purpose
The purpose of this project is to be a fun ~2-week project that prospective students and volunteers can do to understand more about the tools we use in the MAGICC lab.

> [!NOTE]
> The ~2-week statistic is our estimate of how long it would take someone who knew nothing about the lab and had some coding experience, if they were to spend 10hrs/week working on it.
> If you can't spend 10hrs/week on the project, expect that it will probably take a bit longer.

### Expected outcomes

* Gain exposure to 
  * [ROS 2](https://docs.ros.org/en/humble/index.html)
  * `git`
  * Python/C++
  * Linux
  * (optional) Docker
* Set up ROSflight on your computer
* Write your own ROS 2 node that controls a simulated multirotor
* Have fun!

> [!NOTE]
> [ROSflight](https://docs.rosflight.org) is an autopilot built from scratch here in the MAGICC lab.
> It is designed to be understandable, making it a good place for people to learn about drones, controls, and autopilots in general.
>
> Throughout this project, you will set up the ROSflight autopilot and a simulator on your computer.
> You'll then have the chance to write your own program that controls a multirotor in sim!

## Project Assignment

![Image of the maze to solve](docs/assets/maze.png)

Mr. Karl Maeser has unfortunately found himself stuck inside a chalk circle, and hasn't been able to find any BYU Creamery chocolate milk for days!
He is located at the end of a hallway, unable to move.

Luckily, you happen to have a quadrotor that just happens to have a case of BYU Creamery chocolate milk already loaded up.
Your assignment is to deliver the goods to Mr. Maeser as quick as you can.
But watch out!
Hit any walls and your quadrotor will crash (not shown)!

#### Your Task:
Design a controller that guides your quadrotor through a simulated hallway and arrives close to Mr. Maeser.
Avoid all walls.

#### Deliverables:
- A video link of your quadrotor completing the mission (screen recording, uploaded to YouTube or similar)
- The minimum distance to the walls from your run (produced by this package)
- A link to your forked repository with your solution code

## Instructions

1. Follow the [project instructions](docs/project-instructions.md) to complete your project.
2. After successfully getting Mr. Maeser his goods, [submit your results to the leaderboard](docs/leaderboard-instructions.md).

## Leaderboard
Ranked list of successful solutions.
Solutions are ranked by $\frac{\text{Minimum Distance From Walls}}{\text{Time}}$

This means that a large minimum distance and a small time will result in a higher score.

<!-- LEADERBOARD:START -->
<!-- The leaderboard below is automatically generated. Do not edit manually. -->
| Rank | Name | Minimum distance from walls (m) | Best time (s) | GitHub | Video Link |
|------|------|------------------|----------------|---------|----------|
| 1 | Cosmo | 2.5 | 44.5 | [repo](https://github.com/byu-magicc/onboarding_project) | [video](https://youtu.be/GJZMzQYB5zI) |

<!-- LEADERBOARD:END -->
