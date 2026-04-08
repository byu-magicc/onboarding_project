# Project Instructions

> [!NOTE]
> **About These Instructions**
> 
> These instructions are intentionally not exhaustive step-by-step tutorials. Why? Because learning to explore documentation, troubleshoot problems, and find answers independently are crucial research skills!
> 
> You'll find hints and pointers to relevant resources, but you're encouraged to:
> - Read the linked documentation carefully
> - Search Stack Overflow for error messages or concepts
> - Ask AI tools like ChatGPT (but always verify the answers—they can be wrong or outdated!)
> - Use Google to find tutorials and examples
>
> **When you get stuck:** Other students in the lab are here to help! We've all been beginners at this. Feel free to reach out, but you'll get better help (and learn more) if you've done some investigation first—share what you've tried and what errors you're seeing.

> [!WARNING]
> **Operating System Requirements**
> 
> **Windows Users:** ROS 2 requires Linux. You must [install WSL (Windows Subsystem for Linux)](https://learn.microsoft.com/en-us/windows/wsl/install) and complete all project steps inside WSL. Linux is an essential tool in robotics research, and WSL provides a seamless way to run Linux on Windows.
>
> **Mac Users:** ROS 2 is not well-supported on macOS, especially for GUI applications like RViz (even with Docker). We recommend using a Linux virtual machine or accessing a lab computer that runs Linux.

Follow these steps to complete the project:

## Step 1: Installation and Preliminaries

In this project, you'll use two main software systems:
- **[ROS 2](https://docs.ros.org/en/jazzy/)** – An industry-standard robotics framework with tools for communication between different parts of a robot system
- **[ROSflight](https://docs.rosflight.org)** – An autopilot built on top of ROS 2 that handles low-level flight control, allowing you to focus on high-level navigation

### 1.1 Understand ROSflight

Read the [ROSflight overview page](https://docs.rosflight.org/latest) to understand what ROSflight is and how it fits into the robotics ecosystem.

  > [!TIP]
  > Don't rush this step! Understanding the architecture will help you later when writing your controller.

### 1.2 Install ROS 2 and ROSflight

Follow the [ROSflight installation instructions](https://docs.rosflight.org/git-main/user-guide/installation/installation-sim/) to install both ROS 2 and ROSflight on your computer. This will also set up the simulator you'll use to test your code.

  > [!TIP]
  > The installation process will take some time and requires downloading several gigabytes. Make sure you have a stable internet connection and at least 10 GB of free disk space.

### 1.3 Complete the Tutorials

Work through the [ROSflight tutorials](https://docs.rosflight.org/git-main/user-guide/tutorials/tutorial-overview/). These tutorials will teach you how to launch the simulator, control the quadrotor, and use ROScopter (the high-level autonomy package you'll interface with).

**Important:** You can **skip** the "Transitioning from Sim to Hardware" section—we're only working in simulation for this project. **Make sure** to complete the ROScopter-specific tutorials, as you'll be using ROScopter extensively.

  > [!TIP]
  > **Interleaving ROS 2 and ROSflight Tutorials**
  >
  > The ROSflight tutorials assume some familiarity with ROS 2 concepts (nodes, topics, services, etc.). If you encounter unfamiliar concepts, pause and complete the relevant [ROS 2 tutorials](https://docs.ros.org/en/jazzy/Tutorials.html) before continuing.
  >
  > We recommend this approach:
  > 1. Start a ROSflight tutorial
  > 2. When you see an unfamiliar ROS 2 concept (e.g., "topic", "service"), switch to the ROS 2 tutorials
  > 3. Learn the concept in the ROS 2 tutorial
  > 4. Return to the ROSflight tutorial with better understanding
  >
  > The ROS 2 tutorials are excellent and worth exploring beyond just what ROSflight requires!

**Patience is key!** These tutorials introduce a lot of new concepts. Take your time, experiment, and ask questions when you're confused. This foundation will make the rest of the project much smoother.

## Step 2: Writing Your Code

**Congratulations on completing the tutorials!** This is a major milestone. You should now be able to:
- Launch the simulator and ROSflight autopilot
- Fly basic waypoint missions in simulation
- Understand how ROS 2 nodes, topics, and services work

Now for the fun part: writing your own navigation controller!

### The Big Picture

Your goal is to write a **ROS 2 node** (a program that runs within the ROS 2 ecosystem) that:
1. Receives sensor data about nearby walls
2. Makes navigation decisions based on this data
3. Sends commands to ROScopter to fly the quadrotor

You can write this in **C++ or Python**—choose whichever you're more comfortable with.

> [!TIP]
> **New to writing ROS 2 nodes?**
>
> Review the ROS 2 tutorials (not ROSflight-specific) on creating nodes, subscribers, and publishers. 
>
> This repository also includes two example nodes you can study:
> - `scripts/walls_sensor.py` – A Python node
> - `src/rviz_hallway_publisher.cpp` – A C++ node
>
> Feel free to use these as templates for your own node!

This repository contains helper code to visualize the hallway and compute your statistics. You'll write your navigation node right here in this repo (after forking it to your own GitHub account).

### 2.1 Fork and Understand This Repository

**First:** [Fork this repository](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/working-with-forks/fork-a-repo) to your own GitHub account (create one if you don't have it), then clone your fork to your computer. We recommend cloning it into your `rosflight_ws/src` directory so it integrates with your ROS 2 workspace.

**Repository structure:**

- **`src/`** – Contains `rviz_hallway_publisher.cpp` and `rviz_hallway_publisher.hpp`, which create the 3D hallway visualization. You don't need to modify these files.

- **`scripts/`** – Contains `walls_sensor.py`, which simulates the wall distance sensors. You don't need to modify this file, but it's a good Python node example.

- **`resource/`** – Contains the Mr. Maeser 3D model and an RViz configuration file (`onboarding_project_rviz_config.rviz`) that you'll load later to see the hallway visualization.

- **`launch/`** – Contains launch files to start multiple nodes at once.

- **`CMakeLists.txt` and `package.xml`** – ROS 2 package configuration files. You'll edit `CMakeLists.txt` to add your own node.

Everything else you need (the simulator, ROSflight, ROScopter) was already installed in Step 1.

### 2.2 Write Your Navigation Node

> !TIP
> If this is your first time writing a ROS2 node, we **strongly recommend** the tutorials for [writing a Python ROS2 node]() or [writing a C++ ROS2 node](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html).
>
> Note that you may want to visit more of [the ROS2 tutorials](https://docs.ros.org/en/jazzy/Tutorials.html). They will come in handy in this phase of the project.

Your simulated quadrotor is equipped with four laser range sensors that measure distances to the nearest walls in each cardinal direction.

**Sensor Information:**
- **Topic:** `sensors/walls_sensor`
- **Message Type:** `std_msgs/msg/Float32MultiArray`
- **Data:** An array of 4 distance values (in meters): `[north, east, south, west]`

> [!TIP]
> **Important Simplification:** These sensors report distances in **global** cardinal directions (north, east, south, west), not relative to the quadrotor's orientation.
>
> For example, the "north" sensor always tells you the distance to the nearest wall in the global north direction, regardless of which way your quadrotor is facing. This makes navigation logic simpler.

**Your Task:** Write a ROS 2 node (C++ or Python) that:
1. Subscribes to `sensors/walls_sensor` to receive wall distance data
2. Uses this data to make navigation decisions
3. Sends commands to ROScopter to fly through the hallway

You have complete freedom in how you interface with ROScopter. Refer back to the ROSflight tutorials for examples of commands you can send (waypoints, velocity commands, etc.).

<details><summary><strong>Hints for Writing a Node</strong></summary>

Writing a ROS 2 node from scratch can feel overwhelming at first. Here's a strategy:

**Start with a template:** This repository includes two example nodes you can use as starting points:
- `scripts/walls_sensor.py` (Python)
- `src/rviz_hallway_publisher.cpp` (C++)

Pick one that matches your preferred language and copy it as a template.

**Build incrementally:**
1. Start by stripping out functionality you don't need until you have a minimal node
2. Try to build it (see next section for build instructions)
3. Once it builds successfully, add one small feature at a time
4. Build and test after each addition

**Example incremental approach:**
1. Create a node that just prints "Hello, world!" → Build and run
2. Add a subscriber to `sensors/walls_sensor` with a callback that prints the data → Build and test
3. Add logic to process the sensor data → Build and test
4. Add a publisher/service client to send commands to ROScopter → Build and test
5. Continue refining your navigation algorithm

**Remember:** AI assistants can help with ROS 2 syntax questions, but make sure you understand the code you're writing!

</details>

<details><summary><strong>Hint for the Navigation Approach</strong></summary>

There are many valid ways to solve this problem. Here's one approach to get you started:

**Waypoint-based navigation:**
1. Use the sensor measurements to compute a "safe" waypoint ahead of your current position
   - "Safe" means the waypoint isn't inside a wall
   - Consider using sensor data to steer toward open spaces
2. Send this waypoint to ROScopter using the `/path_planner/add_waypoint` [ROS 2 service](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)
3. Continuously update waypoints as you receive new sensor data

**Need help with service calls?** Ask an AI assistant how to call a ROS 2 service from within a node, or review the ROS 2 tutorials on services.

**Other approaches:** You could also use velocity commands, position hold, or a hybrid strategy. Be creative!

</details>

### 2.3 Build Your Code

Once you've written your node, you need to build it so ROS 2 can find and run it.

**Step 1: Update CMakeLists.txt**

Add your node to the `CMakeLists.txt` file in the root of this repository:
- **For C++ nodes:** Add an executable entry (follow the example of `rviz_hallway_publisher`)
- **For Python nodes:** Add an install script entry (follow the example of `walls_sensor.py`)

**Step 2: Build with colcon**

Navigate to your ROS 2 workspace and build:

```bash
# This works if you put your forked repo in your rosflight_ws/src dir.
# If you put the onboarding project repo somewhere different, cd to that location instead.
cd ~/rosflight_ws
colcon build
source install/setup.bash
```

The `colcon build` command compiles C++ code, installs Python scripts, and sets up your ROS 2 environment.

> [!IMPORTANT]
> **You must run `source install/setup.bash` in every new terminal** after building. This tells ROS 2 where to find your packages and nodes.
>
> **Pro tip:** Add `source ~/rosflight_ws/install/setup.bash` to your `~/.bashrc` file so it runs automatically when you open a terminal. You can do this with:
> ```bash
> echo "source ~/rosflight_ws/install/setup.bash" >> ~/.bashrc
> ```

**Step 3: Test your node**

Verify your node runs without errors:

```bash
ros2 run onboarding_project <your_node_name>
```

Replace `<your_node_name>` with the name you gave your executable (C++) or script (Python) in `CMakeLists.txt`.

**If you see errors, check:**
- Did you source `install/setup.bash` in your current terminal?
- Did `colcon build` complete without errors? Scroll up to check for compilation/build errors
- Did you add your node correctly to `CMakeLists.txt`?
- Are there syntax errors in your code?

## Step 3: Test Your Code

Time to see your controller in action! You'll launch several components and watch your quadrotor navigate the hallway.

> [!WARNING]
> **Remember to source!** Run `source ~/rosflight_ws/install/setup.bash` in every terminal before running ROS 2 commands. See [2.3 Build Your Code](#23-build-your-code) for details.

### 3.1 Launch the Simulator and ROScopter

Open two terminals and follow the [multirotor ROSflight sim tutorial](https://docs.rosflight.org/git-main/user-guide/tutorials/setting-up-roscopter-in-sim/):

**Terminal 1:** Launch the ROSflight simulator
**Terminal 2:** Launch the ROScopter autonomy stack

These provide the simulated quadrotor and the autopilot you'll command.

### 3.2 Launch the Hallway Visualization

In a **third terminal**, launch the helper nodes for this project:

```bash
ros2 launch onboarding_project onboarding_project.launch.py
```

This starts two nodes:
- `rviz_hallway_publisher` – Creates the 3D hallway visualization
- `walls_sensor` – Simulates the wall distance sensors

> [!TIP]
> **What is a launch file?** Launch files let you start multiple ROS 2 nodes with a single command. The `onboarding_project.launch.py` file is located in the `launch/` directory.
>
> You can verify the nodes are running by opening a new terminal and running: `ros2 node list`

### 3.3 Configure RViz Visualization

RViz (the 3D visualizer) should have opened when you launched the simulator. Now configure it to show the hallway:

1. In the RViz window, click **File → Open Config**
2. Navigate to the `resource/` directory of this repository
3. Select `onboarding_project_rviz_config.rviz`
4. The hallway and Mr. Maeser should now appear in the visualization!

### 3.4 Start Your Mission

Now run your navigation node:

```bash
ros2 run onboarding_project <your_node_name>
```

Then, in the simulator terminal:
1. **Arm** your aircraft (refer to the ROSflight tutorials if you forget how)
2. **Toggle RC override off** – This hands control to your autonomous controller

Your quadrotor should now start flying! Watch it navigate through the hallway (hopefully without crashing).

### 3.5 Iterate and Improve

Your first attempt probably won't be perfect. You'll likely need to:
- Debug issues with your navigation logic
- Tune parameters (speeds, safety margins, etc.)
- Try different control strategies
- Fix crashes and edge cases

## Step 4: Submit Your Results

Once your quadrotor successfully reaches Mr. Maeser, it's time to submit your results to the leaderboard!

Follow the [leaderboard submission instructions](leaderboard-instructions.md) to share your video, code, and performance metrics.

---

## Troubleshooting Common Issues

<details><summary><strong>❓ Wrong documentation version on rosflight.org</strong></summary>

If the documentation at [https://rosflight.org](https://rosflight.org) doesn't match these instructions, you may be viewing an outdated version.

**Solution:** Look for the version selector at the top of the page (near the logo) and select **"git-main"** instead of "v1.3" or other releases.

</details>

<details><summary><strong>❓ "Package not found" or "No executable found" errors</strong></summary>

**Likely cause:** You forgot to source your workspace.

**Solution:** Run `source ~/rosflight_ws/install/setup.bash` in the terminal where you're getting the error. Remember, you need to do this in **every new terminal**.

</details>

<details><summary><strong>❓ Build errors with colcon</strong></summary>

**Common causes:**
- Missing dependencies
- Syntax errors in your code
- Incorrect `CMakeLists.txt` configuration

**Solution:** Read the error messages carefully. They usually point to the problem. Search the error message online or ask for help with the specific error text.

</details>

<details><summary><strong>❓ RViz doesn't show the hallway</strong></summary>

**Solution:** Make sure you:
1. Launched the `onboarding_project.launch.py` file (which starts the hallway publisher)
2. Loaded the correct config file (`onboarding_project_rviz_config.rviz`) in RViz
3. Verified the nodes are running with `ros2 node list`

</details>

<details><summary><strong>❓ Quadrotor crashes immediately or behaves erratically</strong></summary>

**Common causes:**
- Navigation logic errors (sending commands that cause collisions)
- Not handling sensor data correctly
- Publishing to wrong topics or wrong message types

**Solution:** Add print statements to debug your logic. Check what commands you're sending using `ros2 topic echo`. Start with simple, conservative movements before optimizing for speed.

</details>

---

**Have a different issue?** Search Stack Overflow, ask lab members, or [open a GitHub issue](https://github.com/byu-magicc/onboarding_project/issues) so we can help and add it to this list!
