# RoboCup Autonomous Robot Manipulation Challenge
This repository contains the development of the RoboCup Autonomous Robot Manipulation(ARM) Challenge, which consist in the implementation of perception and control algorithms in MATLAB and Simulink to grasp and manipulate bottles and cans within a table to classify them into two bins. [^template]


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
    <img src="https://user-images.githubusercontent.com/30636259/163751315-c7d1fa6f-35cc-41d8-9890-12e9e77a1084.png" alt="vm" width="500px">
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
1. Clone [ros kortex](https://github.com/Kinovarobotics/ros_kortex) packages in the *src* folder:
    ```bash
    cd src
    git clone https://github.com/Kinovarobotics/ros_kortex -b kinetic-devel
    git reset --hard 6970f5b
    ```
    > It is necessary to clone the repository in the branch **kinetic-devel** and this version because the Robocup's packages are not compatible with the latest versions of this packages.
1. Remove packages that are not needed and can cause conflicts:
    ```bash
    rm -rfv !("kortex_control"|"kortex_description"|"kortex_gazebo") # Remove all packages except kortex_control, kortex_description and kortex_gazebo
    ```
1. Get the robocup packages from [virtual machine file](https://ssd.mathworks.com/supportfiles/ros/virtual_machines/v2/ros_melodic_dashing_gazebov9_linux_win_v3.zip). 
    1. Extract the file.
    1. Download and install [VMWare Workstation](https://www.vmware.com/co/products/workstation-player/workstation-player-evaluation.html).
    1. Open the virtual machine and sharing the files between host and guess, or use `vmware-mount`
        ```bash
        sudo mkdir /mnt/vmdkfile
        sudo vmware-mount <vm_folder_path>/ros_melodic_dashing_gazebov9.vmdk 1 /mnt/vmdkfile/
        ```
        > Don't forget shutdown the virtual machine (do not suspend it) and have `libaio` or `libaio1` installed.
    1. Copy the following folders from `mnt/vmdkfile/home/user/catkin_ws/src` to the *src* folder in your *ROS workspace*.
        * kortex_gazebo_camera
        * **kortex_gazebo_depth**
    1. Unmount the *.vmdk file*
        ```bash
        sudo vmware-mount -X
        sudo rm -rf /mnt/vmdkfile
        ```
1. Install all the necessary dependencies with the next command:
    ```bash
    rosdep install --from-paths src --ignore-src -y
    ```
1. Build the packages (`catkin build` if use *catkin tools*).  
    > Don't forget source the *devel/setup.bash* file in the *ROS workspace*.
1. Run the launch file with the following command:
    ```bash
    roslaunch kortex_gazebo_depth pickplace.launch world:=RoboCup_1.world # or RoboCup_2.world
    ```
    > If you have ROS noetic, you probably have problems with `--in-order` option in xacro when run the launch file, to fix it remove `--in-order` option in the line 40 of pickplace.launch in the *kortex_gazebo_depth* package.

<p align="center">
    <img src="https://user-images.githubusercontent.com/30636259/163854442-007a01d1-0c22-4884-a281-2e49dac8aa2f.png" alt="native roslaunch" width="700px">
</p>
</details>



To launch ROS inside matlab  inside you can use 'system' to run shell commands but ROS requieres some libraries so you need to link them as shown here `system(['export LD_LIBRARY_PATH="LD_path";' 'roslaunch xyz.launch &']);` where the *LD_path* is a place holder for the path of the library that can be found by running in shell `echo $LD_LIBRARY_PATH` [^rosMatlab]

first aproach consist in [^pick-place]

## References 

[^rosMatlab]: [roslaunch-on-matlab](https://answers.ros.org/question/255008/roslaunch-on-matlab/)

[^template]: [Robocup github Template](https://github.com/mathworks-robotics/templates-robocup-robot-manipulation-challenge)

[^pick-place]: [Matlab pick and place tutorial](https://www.mathworks.com/help/robotics/ug/pick-and-place-workflow-in-gazebo-using-ros.html)