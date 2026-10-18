# NVIDIA Newton patient cart with Quest/OpenXR console

This directory contains optional integration files connecting the three-PSM
NVIDIA Newton patient cart to a Quest surgeon console. The selected scene renders
left and right ECM views into a side-by-side stream:

- `ECM_PSM1_PSM2_PSM3.yaml` provides PSM1, PSM2, PSM3, and ECM through ROS;
- `system-MTML-MTMR-OpenXR-patient-cart-ROS.json` imports those four ROS arms
  into `dvrk_system` and obtains MTML, MTMR, and console inputs from OpenXR;
- `dvrk-console-overlay.json` consumes the raw ECM image from
  `@dvrk:simulator:stereo_source` via Unix-FD / memfd zero-copy transport, adds the standard dVRK console overlay, and
  publishes `@dvrk:console:stereo_overlay`;
- `sawOpenXR-newton-unixfd.json` receives that overlayed stream for the HMD.

These integration files do not make the external packages runtime dependencies
of `dvrk_newton`. Install and source them separately when using this example:

- [`sawOpenXR`](https://github.com/adeguet1/sawOpenXR) is the authoritative
  source for headset setup, video behavior, and controller mappings.
- [`sawIntuitiveResearchKit`](https://github.com/jhu-dvrk/sawIntuitiveResearchKit)
  provides `dvrk_system` and the ROS arm adapters used by the system JSON.

The local system JSON is the authoritative record of which MTM is paired with
which PSM and of the teleoperation parameters for this example.

## Run

Follow the `sawOpenXR` documentation to prepare and connect the headset, then
launch the patient cart and console together:

```bash
source ~/wss/dvrk/.venv-newton/bin/activate
source ~/wss/dvrk/install/setup.bash
ros2 launch dvrk_newton open_xr.launch.py
```

The launch file starts the simulator, dVRK console video overlay, and optional
`dvrk_system` together. `sawOpenXR` retries its video source until the overlay
socket is available.

Override headless mode or select an exercise scene (`tray_cubes.yaml` by default, `peg_board_ring.yaml`, or `peg_board_CUHK.yaml`) when needed:

```bash
ros2 launch dvrk_newton open_xr.launch.py \
  scene:=peg_board_ring.yaml \
  headless:=false
```

Add `rqt:=true` to start the dVRK Console widget, CRTK Arm panels, and the
diagnostics panel.
