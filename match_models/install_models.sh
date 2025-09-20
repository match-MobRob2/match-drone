cp misc/* ../../PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/

cp -r sdf/* ../../PX4-Autopilot/Tools/simulation/gz/models/

sed -i '/# \[22000, 22999\] Reserve for custom models/q' ../../PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt

cat <<EOL >> ../../PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt
    40011_gz_match_drohne
   	40012_gz_match_drohne_lidar
    40013_gz_match_drohne_fr_camera
)
EOL