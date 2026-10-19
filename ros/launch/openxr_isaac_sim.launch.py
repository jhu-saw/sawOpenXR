"""Run the OpenXR console with the Isaac Sim patient cart."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from dvrk_simulator_base.launch import SystemProfile, simulator_launch


def generate_launch_description():
    directory = None
    try:
        cand = Path(get_package_share_directory("saw_openxr")) / "dvrk_isaac_sim"
        if cand.is_dir():
            directory = cand
    except Exception:
        pass

    if directory is None:
        directory = Path(__file__).resolve().parent.parent.parent / "share" / "dvrk_isaac_sim"

    config = directory / "isaac_sim.yaml"

    return simulator_launch("dvrk_isaac_sim", profile=SystemProfile(
        config=config,
        cart_scene="ECM_PSM1_PSM2_PSM3_stereo_rtsp.yaml",
        system_config=directory / "system-OpenXR-IsaacSim.json",
        cwd=directory,
        display_config=None,
    ), exercise_argument="exercise")
