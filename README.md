<!-- # grvc_bt_ros
Behavior Tree wrapper for UAVs in ROS -->
<a name="readme-top"></a>

<!-- PROJECT LOGO -->
<br />
<div align="center">
  <!--<a href="https://github.com/miggilcas/wirispro_manager_panel/tree/ethGui-WIRIS">
    <img src="docs/logo_workswell_wwp.png" alt="Logo" width="" height="64">
  </a>
  <a href="https://github.com/miggilcas/wirispro_manager_panel/tree/ethGui-WIRIS">
    <img src="docs/wirisPro.png" alt="Logo" width="" height="150">
  </a>-->

  <h3 align="center">GRVC Behavior Trees ROS</h3>

  <p align="center">
    Behavior Tree wrapper for UAVs in ROS 
    <br />
    <!--<a href="https://github.com/miggilcas/wirispro_manager_panel/tree/ethGui-WIRIS"><strong>Explore the docs »</strong></a>
    <br />
    <br />
    <a href="https://github.com/miggilcas/wirispro_manager_panel/issues/new?labels=bug&template=bug-report---.md">Report Bug</a>
    ·
    <a href="https://github.com/miggilcas/wirispro_manager_panel/issues/new?labels=enhancement&template=feature-request---.md">Request Feature</a> -->
  </p>
</div>


<!-- TABLE OF CONTENTS 
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
  </ol>
</details>
-->

## Overview


This package provides a wrapper for ROS and UAVs.

## Support ROS version
- melodic
- noetic
## Build up environment
1. git clone the repo.
```
mkdir -p ~/bt_ros1_ws/src
cd ~/bt_ros1_ws/src
git clone https://github.com/miggilcas/grvc_bt_ros.git
```

2. Install dependencies
```
cd ~/bt_ros1_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Build
```
catkin build
```


## Help / Contribution

* Contact: **Miguel Gil Castilla** (mgil8@us.es)


* Found a bug? Create an ISSUE!

* Do you want to contribute? Create a PULL-REQUEST!

<br />
<div align="center">
  <a href="https://github.com/miggilcas/grvc_bt_ros">
    <img src="docs/logo_grvc.png" alt="Logo" width="" height="100">
  </a>

  
</div>

<!-- ACKNOWLEDGMENTS -->
## Acknowledgments

Finally I want to thank the following packages or URLs that inspired us to create the plugin.

* [BT_ros1](https://github.com/Adlink-ROS/BT_ros1)


---
---

