# RoboCup Autonomous Robot Manipulation Challenge
This repository contains the development of the RoboCup Autonomous Robot Manipulation(ARM) Challenge, which consist in the implementation of perception and control algorithms in MATLAB and Simulink to grasp and manipulate bottles and cans within a table to classify them into two bins.


## Set-up
To start with the challenge it is necessary to install the ROS packages of the robot and the world of gazebo, this can be done in two ways:

<details close>
<summary>Virtual machine</summary>
If the configuration is done from Windows or you don't want to install ROS on Ubuntu (or other Linux distros), you can use the virtual machine provided by Robocup and Mathworks.

* VMWare
    1. Download the [VMWare Workstation](https://www.vmware.com/co/products/workstation-player/workstation-player-evaluation.html) version for your OS and install it. *Don't forget to select the use non comercial version when installing*.
    1. Download the [file](https://ssd.mathworks.com/supportfiles/ros/virtual_machines/v2/ros_melodic_dashing_gazebov9_linux_win_v3.zip) with the virtual machine and unzip.
    1. Finally, open the virtual machine and start it. When a window appears asking if you copied or moved the virtual machine, select *I copied it*.

* VirtualBox
    1. Download [Virtual Box](https://www.virtualbox.org/wiki/Downloads) version for your OS and install it (to use virtual box in full screen, install [Guest additions](https://www.virtualbox.org/manual/ch04.html).
    1. Follow the same step for VMWare.
    1. Set the network as NAT.
    1. Open the virtual machine and start it.
    
> For a complete installation guide, see [Mathworks page](https://la.mathworks.com/support/product/robotics/ros2-vm-installation-instructions-v4.html).

<p align="center">
    <img src="https://user-images.githubusercontent.com/30636259/163751315-c7d1fa6f-35cc-41d8-9890-12e9e77a1084.png" alt="vm" width="65%">
</p>

When the virtual machine is started, you can start ROS open the file **Example World 1.desktop** (or **Example World 2.desktop** to run the second world) in the *RoboCup Challenge* folder located in the desktop, or launch the nodes with the following commands:
```bash
cd ~
./start-robocup-example-world-1.sh # or ...world-2.sh
```
</details>


<details close>
<summary>Native installation</summary>
In order to use the host computer resources in a better way, it is possible to install ROS(melodic or noetic) and the necessary packages to run the challenge simulation.

1. Create a new ROS workspace or use a previous.
1. Clone [ros kortex](https://github.com/Kinovarobotics/ros_kortex) packages in the *src* folder. Don't forget select the intended branch (melodic or noetic, use `git clone <url> -b <branch_name>`).
1. Get the robocup packages...
1. Install all the necessary dependencies with `rosdep install --from-paths src --ignore-src -y` command in the *ROS workspace* folder.
    > For a complete installation guide, see [ros kortex readme file](https://github.com/Kinovarobotics/ros_kortex).
1. Build the packages (`catkin build` if use *catkin tools*).
</details>



## References 
1. [Robocup github Template](https://github.com/mathworks-robotics/templates-robocup-robot-manipulation-challenge)

2. [Matlab pick and place tutorial](https://www.mathworks.com/help/robotics/ug/pick-and-place-workflow-in-gazebo-using-ros.html)

<!-- * kortex_control
* ros-controllers
* kortex_description
* kortex_gazebo
* kortex_gazebo_camera -->