from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("omtp_factory", package_name="omtp_factory_moveit_config").parameter("use_sim_time",True).to_moveit_configs()
    return LaunchDescription(
        [   
            
            generate_demo_launch(moveit_config)
        ]
    )
    
