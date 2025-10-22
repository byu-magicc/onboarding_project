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

1. Install [ROS 2](https://docs.ros.org/en/jazzy/index.html) and set up [ROSflight](https://docs.rosflight.org/git-main/user-guide/installation/installation-sim/).
  > [!TIP]
  > Make sure to get familiar with what ROS 2 and ROSflight are, if you are unfamiliar with the projects. Information about both projects is found on the documentation links above.
  >
  > The [ROSflight docs](https://docs.rosflight.org/git-main/user-guide/overview) mention doing the ROS 2 tutorials.
  > We recommend that you do these tutorials **simultaneously** with the ROSflight tutorials.
  > This means you work on the ROSflight tutorials and do the ROS 2 tutorials for topics that you come across in the ROSflight tutorials.
  > 
  > Feel free to do as many ROS 2 tutorials as you want; they are all quite good.

2. Go through the [ROSflight tutorials](https://docs.rosflight.org/git-main/user-guide/tutorials/tutorial-overview/). You can skip the "Transitioning from Sim to Hardware" tutorial section. We will be using ROScopter in this onboarding project.

## Writing your code

Remember that the purpose of this project is to control a quadrotor to get to the end of a maze-like hallway.
We have designed this project so that you will write a ROS 2 node that interfaces with ROScopter.
You can do this in either C++ or Python, whichever you prefer.
This is where you will start writing your code to do this!

This repository contains some files that you will need in order to interface with ROScopter's simulation (e.g. load the scenario, compute your stats, etc.).
Feel free to write your node here in this repo (once you fork it).

### Understanding this repo

3. First, [fork](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/working-with-forks/fork-a-repo) and clone this repository (create a GitHub account if needed). We recommend cloning it into your `rosflight_ws/src` directory.

Let's take a look at the parts of the repo.

The `src` directory contains a file called `rviz_hallway_publisher.cpp`.
This file defines a ROS 2 node (in C++) that creates and publishes the hallway visualizations to [RViz](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html) (a ROS 2-based visualizer we use).
You should not have to do anything with this file or the associated `rviz_hallway_publisher.hpp` file.

The `resource` directory contains an STL file for Mr. Maeser and an RViz configuration file.
You will load this configuration file a little later (it defines some ROS 2 subscribers that you will need to visualize the hallways).

Everything else you need is in the `roscopter` and `rosflight_ros_pkgs` repos (already installed from steps 1-2).

### Writing your node

Your quadrotor has been outfitted with laser range sensors that tell you how close you are to a wall.
You will subscribe to these sensors on the `sensors/walls_sensor` topic.
This message is a `std_msgs/msg/Float32MultiArray` type, and contains 4 values for the north, east, south, and west distances to walls.

> [!TIP]
> We made a simplification to the sensors so that these measurements are the **global** north, east, south, and west.
> In other words, it doesn't matter how you are oriented (e.g. if you are pointing east).
> The north sensor will always be the distance to the nearest wall in the **global** north direction.

4. Write a ROS 2 node (in either Python or C++) that subscribes to the `sensors/walls_sensor` and uses ROScopter to navigate down the hallway.

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
</details>

<details><summary><strong>Hint for the approach</strong></summary>
  There are many ways to approach this problem.
  One way is to use the sensor measurements to compute a safe waypoint.
  Safe here means that it isn't in a wall.

  After computing this waypoint, you could send it to ROScopter via the `/path_planner/add_waypoint` [ROS 2 service](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html).
  You might ask ChatGPT for information on how to write a service call from withing a node to another node.
</details>

### Building your code

If you put your ROS 2 node code in your forked version of this repository, building should be relatively straightforward.

5. You will need to modify the `CMakeLists.txt` in the main level of this repository to include your executable (if C++) or script (if Python).
Follow the examples of the other nodes there.

Once you add it to the `CMakeLists.txt` file, build it by navigating up to your `rosflight_ws`, and running `colcon build`.
Remember to source your `install/setup.bash` script **in every terminal** after you build!

Test that your node works by running this command):
```bash
ros2 run onboarding_project <name_of_your_executable_OR_python_script>
```

### Testing your code

Now that you have built your node, you can launch the ROSflight sim with the hallways and see it go!

6. Launch the multirotor ROSflight sim in one terminal and the ROScopter autonomy stack in another.
Instructions on how to do both are found in the [multirotor ROSflight sim tutorial](https://docs.rosflight.org/git-main/user-guide/tutorials/setting-up-roscopter-in-sim/).

7. Load the RViz config file to the RViz panel that just pulled up by going to `File > Open Config`. Select the `onboarding_project_rviz_config.rviz` file in the `resource` directory of this repository. Once you do this, you should see the hallways appear in RViz.

8. Arm your aircraft.

9. Toggle RC override off. Your multirotor should now be flying around and (hopefully) doing what it should be doing!

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
