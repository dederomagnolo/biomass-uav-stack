
# Research Notes

Action é acionada pelo axclient

```
rostopic info /aeplanner/make_plan/goal
Type: aeplanner/aeplannerActionGoal

Publishers:
 * /axclient_66307_1768930834379 (http://3e995a1e51bf:37841/)

Subscribers:
 * /aeplanner/AEPlannerNodelet (http://3e995a1e51bf:34027/)
 ```

Abrir axclient panel:
 `rosrun actionlib_tools axclient.py /aeplanner/make_plan`


STATUS

rostopic echo /aeplanner/make_plan/status

RESULT

rostopic echo /aeplanner/make_plan/result


### Pose

Aeplanner está usando `/pose` sem namespace. 

*problemático nesse estágio para multi-uavs. Precisaria colocar por uav name space & aeplanner 

```
root@3e995a1e51bf:~/biomass-uav-ws# rostopic info /pose
Type: geometry_msgs/PoseStamped

Publishers: None

Subscribers:
 * /aeplanner/AEPlannerNodelet (http://3e995a1e51bf:34027/)
 ```