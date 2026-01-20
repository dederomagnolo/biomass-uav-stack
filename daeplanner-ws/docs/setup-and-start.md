# Setup & Start Experiment

#### Aeplanner Params

```
rosparam set /aeplanner/boundary/min "[-8, -8, 0.2]"
rosparam set /aeplanner/boundary/max "[ 8,  8, 3.0]"
rosparam set /aeplanner/visualize_tree true
rosparam set /aeplanner/visualize_exploration_area true
rosparam set /aeplanner/visualize_rays false
```

#### Aeplanner node

`roslaunch aeplanner aeplanner_standalone.launch`


#### Bridge

For now, the bridge is mapping mrs topics to aeplanner topics

`roslaunch mrs_daep_supervisor mrs_daep_bridge.launch uav_ns:=/uav1`

(manual command, just in case: rosrun topic_tools relay /uav1/octomap_server/octomap_local_full /aeplanner/octomap_full)

this node launches:

- octomap_relay.py: `/uav1/octomap_server/octomap_local_full` -> `/aeplanner/octomap_full`
- pose_relay.py:  `/uav1/odometry/odom_main` -> `/pose`

#### Axclient panel to trigger goals manually

`rosrun actionlib_tools axclient.py /aeplanner/make_plan`