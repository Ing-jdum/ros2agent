
# ROS2-Agent

This project explores the integration of generative AI within the field of robotics, leveraging the ROS2 framework. Created as a complementary component of my Master’s Thesis in Computational Logic and Artificial Intelligence at the University of Seville, it aims to push the boundaries of how AI can enhance robotic systems. While not intended as a comprehensive framework and not slated for ongoing maintenance, this project demonstrates innovative approaches and serves as a valuable proof of concept.



## Installation

To deploy this project, ensure you have the following installed on your machine:

1. **ROS 2 Humble**: Follow the [ROS 2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation.html) to set up ROS 2.
2. **Plansys2**: Install Plansys2 by following the [Plansys2 Build Instructions](https://plansys2.github.io/build_instructions/index.html).
3. **Nav2 Stack**
4. **TurtleBot3**

### Important Note on Nav2 and TurtleBot3
Due to known issues with DDS and configuration settings for `robot_model_type` in Nav2, we recommend watching [this video tutorial](https://www.youtube.com/watch?v=idQb2pB-h2Q&ab_channel=RoboticsBack-End) by Robotics Back-End. This tutorial provides solutions and workarounds for setup bugs that may occur with Nav2 and TurtleBot3.

### Modifying Plansys2 for TCP Connections

1. To enable TCP connections, replace the `plansys2_terminal` package within the Plansys2 library.
   - Copy the file from the repository directory [`plansys2_packages_changes/plansys2_terminal`](path-to-your-repository/plansys2_packages_changes/plansys2_terminal) and paste it into your Plansys2 workspace, typically found at:
     ```plaintext
     plansys2ws/src/ros2_planning_system
     ```

2. Once the file is in place, build the modified package by running the following command in your Plansys2 workspace:
   ```bash
   colcon build --packages-select plansys2_terminal --symlink-install
## Deployment

To run this project, you will need to open four terminals. We recommend using [Terminator](https://gnome-terminator.org/) to manage multiple terminal windows efficiently.

### Terminal Setup

1. **Deploy the Simulation**
   - In the first terminal, navigate to your Plansys2 workspace and launch the simulation:
     ```bash
     ros2 launch workshop_plansys2 workshop.launch.py  
     ```

2. **Deploy the Nav2 Stack**
   - In the second terminal, launch the Nav2 stack, passing the paths to your `map.yaml` and `nav2_params.yaml` files as parameters:
     ```bash
     ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=<your clone location>/maps/workshop.yaml params_file:=<your plansys2 workspace>/src/workshop_plansys2/params/nav2_params.yaml  
     ```
   - This will open the RVIZ application. Once it loads, provide the initial pose 2D estimate for the robot.

3. **Launch the Plansys2 Controller**
   - In the third terminal, navigate to plansys2 workspace and run the Plansys2 controller:
     ```bash
     ros2 launch workshop_plansys2 plansys2_bt_example_launch.py  
     ```

4. **Run the Plansys2 Terminal**
   - In the fourth terminal, navigate to plansys2 workspace and start the Plansys2 terminal:
     ```bash 
     ros2 run plansys2_terminal plansys2_terminal
     ```

5. **Execute the Python Main Script**
   - Finally, in another terminal, or an ide, run your main Python script:
     ```bash
     python main.py
     ```

Once all terminals are set up, the project should be fully deployed and operational. You can communicate with the robot through the terminal with the python application.

## Environment Variables

To run this project, you will need to add the following environment variables to your .env file, you can rename the file '.env_template'

`OPENAI_API_KEY`


## Documentation

[Documentation](https://linktodocumentation)
