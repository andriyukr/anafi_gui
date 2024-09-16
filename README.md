# ROS2 Package for Parrot Anafi Graphic User Interface
This ROS2 package contains the Graphic User Interface (GUI) for the Parrot Anafi family (4K, Thermal, USA, Ai, Sphinx).

## Overview

**Author:** Andriy Sarabakha<br />
**Affiliation:** [Nanyang Technological University (NTU)](https://www.ntu.edu.sg), Singapore<br />
**Maintainer:** Andriy Sarabakha, andriy.sarabakha@ntu.edu.sg

**Keywords:** Parrot, UAV, controller

This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

This package has been tested with **python3** in **ROS2 Humble**/**Ubuntu 22.04**.

### Dependencies

- [anafi_ros](https://github.com/andriyukr/anafi_ros) - ROS bridge for Parrot Olympe SDK

- [anafi_autonomy](https://github.com/andriyukr/anafi_autonomy) - ROS package for Parrot ANAFI drone navigation

### Build

To build from source, clone the latest version from this repository into your ROS2 workspace and build the package using:

    cd ~/ros2_ws/src
    git clone https://github.com/andriyukr/anafi_gui.git
    sudo chmod -R 777 anafi_gui/
    cd ..
    colcon build
    source install/setup.bash

## Usage

To control the drone,

1. To connect to the drone, in the terminal 1, run:

       ros2 launch anafi_autonomy anafi_autonomy_launch.py

1. To launch the GUI, in the terminal 2, run:

       ros2 launch anafi_gui anafi_gui_launch.py

**Troubleshooting**

<details> 
    <summary>ModuleNotFoundError: No module named 'PyQt6'</summary>

Install `PyQt6`:

    pip install PyQt6
</details>

<details> 
    <summary>qt.qpa.plugin: Could not load the Qt platform plugin "xcb" in "" even though it was found.</summary>

Install `libxcb-cursor0`:

    sudo apt install libxcb-cursor0
</details>
