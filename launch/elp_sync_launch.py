from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'CAMERA_NAME',
            default_value='elp',
            description='Camera name for the calibration'
        ),

        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='sync',
            namespace='elp',
            parameters=[{
                'video_device': '/dev/video4',
                'framerate': 30.0,
                'pixel_format': 'mjpeg2rgb',
                'image_width': 3200,
                'image_height': 1200,
                'camera_name': 'elp_sync',
                'brightness': -1,
                'contrast': -1,
                'saturation': -1,
                'sharpness': -1,
                'gain': -1,
                'auto_white_balance': True,
                'white_balance': 4000,
                'autoexposure': True,
                'exposure': 100,
                'autofocus': False,
                'focus': -1,
                'camera_frame_id': 'elp_sync_optical_frame'
            }]
        ),

        Node(
            package='elp_stereo',
            executable='split_sync_image_node',
            name='split_sync_image_node',
            namespace='elp',
            output='screen',
            parameters=[{
                'left_cam_info': PathJoinSubstitution([
                    FindPackageShare('elp_stereo'), 'calibration', 'elp_left.yaml'
                ]),
                'right_cam_info': PathJoinSubstitution([
                    FindPackageShare('elp_stereo'), 'calibration', 'elp_right.yaml'
                ])
            }]
        )
    ])
