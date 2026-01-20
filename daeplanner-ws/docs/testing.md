
#### Check bridge results

```
rostopic echo -n 1 /pose/header
rostopic echo -n 1 /aeplanner/octomap_full/header
```

#### Reference manual testing

This command published a reference. The drone should move.

```
rostopic pub -r 10 /uav1/control_manager/reference mrs_msgs/ReferenceStamped "header:
>   stamp: now
>   frame_id: 'uav1/gps_baro_origin'
> reference:
>   position:
>     x: -2.50
>     y: 0.37
>     z: 2.09
>   heading: 0.0"
```

