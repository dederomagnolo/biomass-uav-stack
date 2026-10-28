# Buildar container

docker compose -f compose.ros2.yaml up -d --build




#Iniciar o Livox


ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args -p xfer_format:=1 -p multi_topic:=0 -p data_src:=0 -p publish_freq:=10.0 -p output_data_type:=0 -p frame_id:=livox_frame -p use>



# Iniciar o FastLio


ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml use_sim_time:=false rviz:=true


# Inicar Zed M

ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedm param_overrides:="depth.depth_mode:=PERFORMANCE" 
