# Isaac Sim patient cart with OpenXR

This configuration starts the three-PSM/ECM Isaac Sim cart, the dVRK OpenXR
console, and RTSP stereo video. The scene is fixed to
`ECM_PSM1_PSM2_PSM3_stereo_rtsp.yaml`; its camera is consumed directly by
`sawOpenXR`, so no local video overlay process is required.

Build and source `dvrk_isaac_sim`, `sawOpenXR`, and the dVRK ROS packages, then
run:

```bash
ros2 launch dvrk_isaac_sim open_xr.launch.py
```

The launch uses the portable `share/open-xr/isaac_sim.yaml` runtime profile
and defaults to headless mode. Select Isaac Python with `DVRK_ISAAC_SIM_PYTHON`
or `ISAAC_SIM_DIR`; interpreter selection happens when the worker starts.
Override runtime settings with `config:=/path/to/isaac_sim.yaml` and select an
exercise with `scene:=tray_cubes.yaml`. Use `console:=...` to select the dVRK
console namespace and `rqt:=true` to launch the Console, tabbed Arms, and
Diagnostics panels.
