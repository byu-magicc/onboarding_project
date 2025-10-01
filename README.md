# MAGICC Lab Onboarding Project

This repository contains project files for the MAGICC Lab onboarding project.

## Purpose
The purpose of this project is to be a fun ~2-week project that prospective students and volunteers can do to understand more about the tools we use in the MAGICC lab.

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

## Project Assignment
Mr. Karl Maeser has unfortunately found himself stuck inside a chalk circle, and hasn't been able to find any BYU Creamery chocolate milk for days!
He is located at the end of a hallway, unable to move.

Luckily, you happen to have a quadrotor that just happens to have a case of BYU Creamery chocolate milk already loaded up!
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

## Project Instructions

> [!NOTE]
> These instructions are intentionally left vague in some places.
> This is because we want you to explore and learn, not just follow a tutorial.
> Often, there will be a hint or a note about places to look, but we encourage you to
>   * Read the documentation
>   * Search on Stack Overflow
>   * Ask ChatGPT (but beware)
>
> If you still are having issues, other students in the lab are willing to help!
> We have all been where you are now. Please reach out.
> Keep in mind that you'll have better outcomes if you do your homework first.

> [!WARNING]
> ROS 2 really only plays nicely with Linux, or Windows running WSL.
> If you are using a Windows machine, [install WSL and do all the following steps inside WSL](https://learn.microsoft.com/en-us/windows/wsl/install).
> Linux is an essential research tool. WSL will allow you to run Linux on Windows.
>
> MacOS does not play nicely with ROS 2, especially not with GUIs with ROS 2 (not even with Docker).
> Perhaps try to use a virtual machine, or get an account with a Linux-based lab computer.

To complete this project, do the following:

1. Install [ROS 2](https://docs.ros.org/en/jazzy/index.html) and set up [ROSflight](https://docs.rosflight.org/git-main/user-guide/installation/installation-sim/).
  > [!TIP]
  > Make sure to get familiar with what ROS 2 and ROSflight are, if you are unfamiliar with the projects. Information about both projects is found on the documentation links above.
  >
  > The [ROSflight docs](https://docs.rosflight.org/git-main/user-guide/overview) mention doing the ROS 2 tutorials.
  > We recommend that you do these tutorials **simultaneously** with the ROSflight tutorials.
  > This means you work on the ROSflight tutorials and do the ROS 2 tutorials for topics that you come across in the ROSflight tutorials.
  > 
  > Feel free to do as many ROS 2 tutorials as you want; they are all quite good.

2. Go through the [ROSflight tutorials](https://docs.rosflight.org/git-main/user-guide/tutorials/tutorial-overview/). You can skip the "Transitioning from Sim to Hardware" tutorial.

### Common issues:

<details><summary><strong>Not seeing correct documentation?</strong></summary>
Not seeing the correct documentation when you go to <href a=https://rosflight.org>https://rosflight.org</href>?
Make sure you select the "git-main" branch at the top near the logo (not "v1.3").
</details>
