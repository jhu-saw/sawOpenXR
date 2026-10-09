"""Start the Isaac cart and sawOpenXR using shared session supervision."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from dvrk_simulator_base.launch import SystemProfile, start_session, default_config


def _launch_setup(context):
    share = Path(get_package_share_directory("saw_openxr"))
    selected = Path(LaunchConfiguration("system_config").perform(context)).expanduser()
    system = selected if selected.is_absolute() else share / "config" / selected
    if not system.is_file():
        raise ValueError(f"dVRK system configuration does not exist: {system}")
    profile = SystemProfile(
        config=Path(LaunchConfiguration("config").perform(context)),
        cart_scene=LaunchConfiguration("isaac_scene").perform(context),
        system_config=system, cwd=share,
    )
    # OpenXR retries its video source; the ROS arm adapter can wait for the
    # simulator. Both processes are supervised without a guessed startup delay.
    return start_session(context, "dvrk_isaac_sim", profile)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("system_config", default_value="system-MTML-MTMR-OpenXR-patient-cart-ROS.json"),
        DeclareLaunchArgument("isaac_config", default_value=str(default_config("dvrk_isaac_sim"))),
        DeclareLaunchArgument("isaac_scene", default_value="ECM_PSM1_PSM2_PSM3_stereo_rtsp.yaml"),
        DeclareLaunchArgument("isaac_headless", default_value="true"),
        DeclareLaunchArgument("config", default_value=LaunchConfiguration("isaac_config")),
        DeclareLaunchArgument("headless", default_value=LaunchConfiguration("isaac_headless")),
        DeclareLaunchArgument("scene", default_value=""),
        DeclareLaunchArgument("rqt", default_value="false"),
        DeclareLaunchArgument("rqt_console", default_value="false"),
        DeclareLaunchArgument("console", default_value="console"),
        OpaqueFunction(function=_launch_setup),
    ])
