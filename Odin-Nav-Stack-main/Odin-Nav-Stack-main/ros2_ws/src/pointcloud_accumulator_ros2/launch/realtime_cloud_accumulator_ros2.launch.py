import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    ROS2 Humble Launch file for Realtime Point Cloud Accumulator
    """

    # 输入点云话题 (Odin1 ROS2 默认发布 /odin1/cloud_slam)
    cloud_topic_arg = DeclareLaunchArgument(
        "cloud_topic",
        default_value="/odin1/cloud_slam",
        description="Input point cloud topic from Odin1"
    )

    # 目标坐标系 (通常为 map 或 odom)
    target_frame_arg = DeclareLaunchArgument(
        "target_frame",
        default_value="map",
        description="Target frame for accumulated point cloud"
    )

    # 保存目录
    save_dir_arg = DeclareLaunchArgument(
        "save_dir",
        default_value=os.path.join(
            get_package_share_directory("pointcloud_accumulator_ros2"),
            "..", "..", "..", "..", "maps"  # 指向 ros2_ws/maps
        ),
        description="Directory to save PCD files"
    )

    # 体素下采样大小
    voxel_size_arg = DeclareLaunchArgument(
        "voxel_size",
        default_value="0.05",
        description="Voxel size in meters for online downsampling"
    )

    # 最大累积点数
    max_points_arg = DeclareLaunchArgument(
        "max_points",
        default_value="10000000",
        description="Maximum number of voxels to accumulate"
    )

    # 是否保存 RGB 颜色
    save_rgb_arg = DeclareLaunchArgument(
        "save_rgb",
        default_value="true",
        description="Preserve RGB color from point cloud"
    )

    # PCD 保存格式
    pcd_binary_arg = DeclareLaunchArgument(
        "pcd_binary",
        default_value="true",
        description="Use binary PCD format (true) or ASCII (false)"
    )

    # 发布频率
    publish_rate_arg = DeclareLaunchArgument(
        "publish_rate",
        default_value="2.0",
        description="Publishing rate for /accumulated_cloud (Hz)"
    )

    # 是否自动开始累积
    auto_start_arg = DeclareLaunchArgument(
        "auto_start",
        default_value="true",
        description="Start accumulating immediately on launch"
    )

    # 最大接收帧率
    max_fps_arg = DeclareLaunchArgument(
        "max_frames_per_sec",
        default_value="10.0",
        description="Maximum point cloud frames to process per second"
    )

    # 创建节点
    accumulator_node = Node(
        package="pointcloud_accumulator_ros2",
        executable="realtime_cloud_accumulator_ros2",
        name="realtime_cloud_accumulator",
        output="screen",
        parameters=[{
            "cloud_topic": LaunchConfiguration("cloud_topic"),
            "target_frame": LaunchConfiguration("target_frame"),
            "save_dir": LaunchConfiguration("save_dir"),
            "voxel_size": LaunchConfiguration("voxel_size"),
            "max_points": LaunchConfiguration("max_points"),
            "save_rgb": LaunchConfiguration("save_rgb"),
            "pcd_binary": LaunchConfiguration("pcd_binary"),
            "publish_rate": LaunchConfiguration("publish_rate"),
            "auto_start": LaunchConfiguration("auto_start"),
            "max_frames_per_sec": LaunchConfiguration("max_frames_per_sec"),
        }]
    )

    ld = LaunchDescription()
    ld.add_action(cloud_topic_arg)
    ld.add_action(target_frame_arg)
    ld.add_action(save_dir_arg)
    ld.add_action(voxel_size_arg)
    ld.add_action(max_points_arg)
    ld.add_action(save_rgb_arg)
    ld.add_action(pcd_binary_arg)
    ld.add_action(publish_rate_arg)
    ld.add_action(auto_start_arg)
    ld.add_action(max_fps_arg)
    ld.add_action(accumulator_node)

    return ld
