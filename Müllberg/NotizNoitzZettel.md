
ros2 launch match_launch rtabmap_slam.launch.py 
ros2 launch match_launch match_drohne_alles.launch.py 
ros2 launch match_launch world2.launch.py 
ros2 launch match_launch rtabmap_icp_odom.launch.py 


/home/luca/match_drone/src/RGLGazeboPlugin/install/RGLServerPlugin


export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/luca/match_drone/src/RGLGazeboPlugin/install/RGLServerPlugin:$GZ_SIM_SYSTEM_PLUGIN_PATH




/home/luca/match_drone/src/RGLGazeboPlugin/install/RGLVisualize

export GZ_GUI_PLUGIN_PATH=/home/luca/match_drone/src/RGLGazeboPlugin/install/RGLVisualize:$GZ_GUI_PLUGIN_PATH



https://github.com/RobotecAI/RGLGazeboPlugin/issues/53sdsd