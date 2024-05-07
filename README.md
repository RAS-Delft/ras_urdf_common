# ras_urdf_common
This repository is intended to contain urdf and related files for the Researchlab Autonomous Shipping Delft. 

![image](https://github.com/RAS-Delft/ras_urdf_common/assets/5917472/407cb5bd-b051-4199-a1d8-c2ab7487da76)
This image shows an rviz2 representation of two Tito Neri ships, using the urdf description of this model in this repo. 

## Use
Related urdf files can be found in the /urdf folder. 
The rviz folder has rviz2 settings that we found nice. To use Rviz2 the tf's and robot description need to be streamed on the appropriate topics.

A transformation tree for the ship links commonly looks like this:
![image](https://github.com/RAS-Delft/ras_urdf_common/assets/5917472/f838ca41-b631-4983-8ea1-21f436cbda98)


Rviz will display ships if:
- Its transformation (world->body) is published on /tf (default bodyframe is named [vesselID](https://github.com/RAS-Delft/ras-documentation-overview/tree/main/module_overviews#namespaces)/base_link)
- A robot description (URDF) is published on /vesselID/robot_description

(optional) joints will be displayed if their respective transforms are published on /tf


You can launch Rviz with default fleet settings , after building the package and sourcing the workspace:
```
ros2 launch ras_urdf_common rviz_bringup.launch.py 
```