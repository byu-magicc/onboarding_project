# Project Instructions

> [!NOTE]
> These instructions are not explicit and complete in all places.
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

## Installation and preliminaries

You will be using [ROS 2](https://docs.ros.org/en/jazzy/) and the ROSflight autopilot extensively in this project.
ROS 2 is a very popular set of software libraries developed by people outside the lab.
ROSflight is an autopilot that is built *on top of* ROS 2 to help researchers fly research code faster.

1. Read the [ROSflight overview page](https://docs.rosflight.org/latest). This will give you an idea of what ROSflight is.

1. Follow the [ROSflight installation instructions](https://docs.rosflight.org/git-main/user-guide/installation/installation-sim/) to install both [ROS 2](https://docs.ros.org/en/jazzy/index.html) and ROSflight to your computer.
  > [!TIP]
  > It's worth it to spend some time getting familiar with what ROS 2 and ROSflight are, if you are unfamiliar with the projects. Information about both projects is found on the documentation links above.

3. Go through the [ROSflight tutorials](https://docs.rosflight.org/git-main/user-guide/tutorials/tutorial-overview/). You can skip the "Transitioning from Sim to Hardware" tutorial section. We will be using ROScopter in this onboarding project, so make sure to do the ROScopter-specific tutorials.
  > [!TIP]
  > When you go read the [ROSflight tutorials](https://docs.rosflight.org/git-main/user-guide/overview), you'll see that the tutorials recommend also doing the [ROS 2](https://docs.ros.org/en/jazzy/Tutorials.html) tutorials.
  > We recommend that you do these tutorials **simultaneously** with the ROSflight tutorials.
  > This means you work on the ROSflight tutorials.
  > When you come across concepts you aren't familiar with, go find/do the ROS 2 tutorials for these topics.
  >
  > Feel free to do as many ROS 2 tutorials as you want; they are all quite good.

  Note that doing these tutorials may take some time. Its a lot to learn and get familiar with, so be patient and ask questions!

## Writing your code

Congrats on finishing the tutorials! This is a major milestone in the project. You should be a little more familiar with ROS 2 and ROSflight, which will help the rest of the project.

Once you've completed the ROSflight tutorials, you should be able to launch the simulator and the ROSflight autopilot.
You should have also flown some waypoint missions in simulation.

Remember that the purpose of this project is to control a quadrotor to get to the end of a maze-like hallway.
We have designed this project so that you will write a ROS 2 node that interfaces with ROScopter.
You can do this in either C++ or Python, whichever you prefer.
This is where you will start writing your code to do this!

> [!TIP]
> If you don't know how to write a ROS 2 node, remember to look into the ROS 2 tutorials (not the ROSflight tutorials!).
>
> Additionally, there is a Python and a C++ node in this repo that you can look at.

This repository contains some files that you will need in order to interface with ROScopter's simulation (e.g. load the scenario, compute your stats, etc.).
Feel free to write your node here in this repo (once you fork it).

### Understanding this repo

4. First, [fork](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/working-with-forks/fork-a-repo) and clone this repository (create a GitHub account if needed). We recommend cloning it into your `rosflight_ws/src` directory.

Let's take a look at the parts of the repo.

The `src` directory contains a file called `rviz_hallway_publisher.cpp`.
This file defines a ROS 2 node (in C++) that creates and publishes the hallway visualizations to [RViz](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html) (a ROS 2-based visualizer we use).
You should not have to do anything with this file or the associated `rviz_hallway_publisher.hpp` file.

The `resource` directory contains an STL file for Mr. Maeser and an RViz configuration file.
You will load this configuration file a little later (it defines some ROS 2 subscribers that you will need to visualize the hallways).

Everything else you need (besides your node) is in the `roscopter` and `rosflight_ros_pkgs` repos (already installed from steps 1-2).

### Writing your node

Your quadrotor has been outfitted with laser range sensors that tell you how close you are to a wall.
You will subscribe to these sensors on the `sensors/walls_sensor` topic.
This message is a `std_msgs/msg/Float32MultiArray` type, and contains 4 values for the north, east, south, and west distances (in meters) to walls.

> [!TIP]
> We made a simplification to the sensors so that these measurements are the **global** north, east, south, and west.
> In other words, it doesn't matter how you are oriented (e.g. if you are pointing east).
> The north sensor will always be the distance to the nearest wall in the **global** north direction.

5. Write a ROS 2 node (in either Python or C++) that subscribes to the `sensors/walls_sensor` and uses ROScopter to navigate down the hallway.

Remember, you are free to interface with ROScopter however you want!
Look back at the ROSflight tutorials if you need hints or help with what to publish or what to command.

<details><summary><strong>Hints for writing a node</strong></summary>
  Writing a ROS 2 node from scratch can be daunting at first.
  In this repository, you have 2 examples of ROS 2 nodes that you can look at and copy.

  One is the `walls_sensor` node (in Python), and the other is the `rviz_hallway_publisher` (in C++).
  Feel free to copy either one and work from there.

  We would recommend stripping out everything you think you don't need, and then try to build it (see next step for building instructions).
  Once your node builds, incrementally add functionality, then build and test.
  If you do it all at once, you might get lost!

  For example, first add a subscriber with a subscription callback (see ROS 2 tutorials for details on that).
  Build it, and test that you are successfully subscribing to the correct topics.
  Then move on to the next functionality.

  Also remember that AI can answer most ROS 2 questions.
</details>

<details><summary><strong>Hint for the approach</strong></summary>
  There are many ways to approach this problem.
  One way is to use the sensor measurements to compute a safe waypoint.
  "Safe" means that it isn't in a wall.

  After computing this waypoint, you could send it to ROScopter via the `/path_planner/add_waypoint` [ROS 2 service](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html).
  You might ask AI for information on how to write a service call from withing a node to another node.
</details>

### Building your code

If you put your ROS 2 node code in your forked version of this repository, building should be relatively straightforward.

6. You will need to modify the `CMakeLists.txt` in the main level of this repository to include your executable (if C++) or script (if Python).
Follow the examples of the other nodes there.

Once you add it to the `CMakeLists.txt` file, build it by navigating up to your `rosflight_ws`, and running `colcon build`.
This command (`colcon build`) takes care of compiling your executables, as well as a lot of other tasks.
Remember to source your `install/setup.bash` script **in every terminal** after you build!
Do this by running
```bash
# This works if you put your forked repo in your rosflight_ws/src dir.
# If you put the onboarding project repo somewhere differnt, cd to that location instead.
cd ~/rosflight_ws
colcon build
source install/setup.bash
```

Test that your node works by running this command):
```bash
ros2 run onboarding_project <name_of_your_executable_OR_python_script>
```

### Testing your code

Now that you have built your node, you can launch the ROSflight sim with the hallways and see it go!

> [!WARNING]
> Make sure to source before running! See [building your code](#building-your-code).

7. Launch the multirotor ROSflight sim in one terminal and the ROScopter autonomy stack in another.
Instructions on how to do both are found in the [multirotor ROSflight sim tutorial](https://docs.rosflight.org/git-main/user-guide/tutorials/setting-up-roscopter-in-sim/).

8. Launch the 2 nodes already included in this repo (the `rviz_hallway_publisher` and the `walls_sensor`) using
  ```bash
  ros2 launch onboarding_project onboarding_project.launch.py
  ```

  In the command we just ran, `onboarding_project` refers to the package name, while `onboarding_project.launch.py` refers to the file that is in the `launch` directory.

  Launch files are ways to run multiple ROS 2 nodes with one command. You should see both nodes show up if you use the `ros2 node list` command in a new terminal.

8. Load the RViz config file to the RViz GUI that was launched in step 6 by going to `File > Open Config`. Select the `onboarding_project_rviz_config.rviz` file in the `resource` directory of this repository. Once you do this, you should see the hallways appear in RViz.

9. Arm your aircraft.

10. Toggle RC override off. Your multirotor should now be flying around and (hopefully) doing what you want it to do!

You'll probably need to iteratively test your code and improve your solution.

### Submitting your code

Once you are happy with your performance, it's time to submit your scores!
Follow [these instructions](leaderboard-instructions.md) to submit your scores to the leaderboard.

## Common issues:

<details><summary><strong>Not seeing correct documentation?</strong></summary>
Not seeing the correct documentation when you go to <href a=https://rosflight.org>https://rosflight.org</href>?
Make sure you select the "git-main" branch at the top near the logo (not "v1.3").
</details>

If there is a common issue you feel should be listed here, please open a GitHub issue.
