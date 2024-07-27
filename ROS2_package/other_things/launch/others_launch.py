import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='haptic',
            executable='hardware_interface',
            name='hardware_interface_ros'),
        # launch_ros.actions.Node(
        #     package='other_things',
        #     executable='ui_send_target_for_pathway',
        #     name='ui_send_target_for_pathway'),
        # launch_ros.actions.Node(
        #     package='other_things',
        #     executable='command_pathway_generation',
        #     name='command_pathway_generation'),
        # launch_ros.actions.Node(
        #     package='other_things',
        #     executable='speed_control_with_jacobian_own_micro',
        #     name='speed_control_with_jacobian_own_micro'),
        # launch_ros.actions.Node(
        #     package='other_things',
        #     executable='micro_pid_velocity_control_3_motors',
        #     name='micro_pid_velocity_control_3_motors'),
  ])