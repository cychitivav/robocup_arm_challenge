# RoboCup Autonomous Robot Manipulation Challenge
This repository contains the development of the RoboCup Autonomous Robot Manipulation(ARM) Challenge, which consist in the implementation of perception and control algorithms in MATLAB and Simulink to grasp and manipulate bottles and cans within a table to classify them into two bins.


## Set-up
To start with the challenge it is necessary to install the ROS packages of the robot and the world of gazebo, this can be done in two ways:

<details close>
    <summary>Virtual machine</summary>
    If the configuration is done from Windows or you don't want to install ROS on Ubuntu (or other Linux distros), you can use the virtual machine provided by Robocup and Mathworks.
    <ul>
        <li>VMWare
            <ol>
                <li type="1">Download the <a href="https://www.vmware.com/co/products/workstation-player/workstation-player-evaluation.html">VMWare Workstation</a> version for your OS and install it. <em>Don't forget to select the use non comercial version when installing.</em> </li>
                <li type="1">Download the <a href="https://ssd.mathworks.com/supportfiles/ros/virtual_machines/v2/ros_melodic_dashing_gazebov9_linux_win_v3.zip">archive</a> with the virtual machine and unzip. </li>
                <li type="1"> Finally, open the virtual machine and start it. When a window appears asking if you copied or moved the virtual machine, select <em>I copied it.</em> </li>
            </ol>
        </li>
        <li>VirtualBox
            <ol>
                <li type="1">Download <a href="https://www.virtualbox.org/wiki/Downloads">Virtual Box</a> version for your OS and install it (to use virtual box in full screen, install <a href=https://www.virtualbox.org/manual/ch04.html">Guest additions</a>).</li>
                <li type="1">Follow the same step for <a href="#VMware_install">VMWare</a>.</li>
                <li type="1">Set the network as NAT.</li>
                <li type="1"> Open the virtual machine and start it.</li>
            </ol>
        </li>
    </ul>
    <blockquote>For a complete installation guide, see <a href="https://la.mathworks.com/support/product/robotics/ros2-vm-installation-instructions-v4.html">Mathworks page</a>.</blockquote>
    <p align="center">
        <img src="https://user-images.githubusercontent.com/30636259/163751315-c7d1fa6f-35cc-41d8-9890-12e9e77a1084.png" alt="vm" width="50%">
    </p>
    When the virtual machine is started, you can start ROS open the file <b>Example World 1.desktop</b> (or <b>Example World 2.desktop</b> to run the second world) in the <em>RoboCup Challenge</em> folder located in the desktop, or launch the nodes with the following commands:
    <pre><code class="language-bash">cd ~
./start-robocup-example-world-1.sh # or *-2.sh
</code></pre>
</details>

<details close>
    <summary>Native installation</summary>
    In order to use the host computer resources in a better way, it is possible to install ROS(melodic or noetic) and the necessary packages to run the challenge simulation.
    <ol>
        <li>Create a new ROS workspace or use a previous. </li>
        <li>Clone <a href="https://github.com/Kinovarobotics/ros_kortex">ros kortex</a> packages in the <em>src</em> folder. Don't forget select the intended branch (melodic or noetic, use <code>git clone &lt;url&gt; -b &lt;name_branch&gt;</code>).</li>
        <li>Get the robocup packages...</li>
        <li>Install all the necessary dependencies with <code>rosdep install --from-paths src --ignore-src -y</code> command in the <em>ROS workspace</em> folder.
        <blockquote>For a complete installation guide, see <a href="https://github.com/Kinovarobotics/ros_kortex">ros kortex readme file</a></blockquote></li>
        <li>Build the packages (<code>catkin build</code> if use <em>catkin tools</em>).</li>
    </ol>
</details>

## Reference 
1. [Robocup github Template](https://github.com/mathworks-robotics/templates-robocup-robot-manipulation-challenge)

2. [Matlab pick and place tutorial](https://www.mathworks.com/help/robotics/ug/pick-and-place-workflow-in-gazebo-using-ros.html)

<!-- * kortex_control
* ros-controllers
* kortex_description
* kortex_gazebo
* kortex_gazebo_camera -->