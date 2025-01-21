source /home/dinhnambkhn/UGV_Autonomy_ws/devel/setup.bash;
roslaunch faster_lio mapping_mid360_indoor.launch & sleep 2;

source /home/dinhnambkhn/UGV_Autonomy_ws/devel/setup.bash;
roslaunch terrain_analysis terrain_analysis_run.launch & sleep 2;

source /home/dinhnambkhn/UGV_Autonomy_ws/devel/setup.bash;
roslaunch local_planner local_planner_run.launch & sleep 2;
