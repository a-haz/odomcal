# odomcal

[![CI](https://github.com/a-haz/odomcal/actions/workflows/ci.yml/badge.svg)](https://github.com/a-haz/odomcal/actions/workflows/ci.yml)
[![ROS 2 Jazzy](https://github.com/a-haz/odomcal/actions/workflows/ros2-jazzy.yml/badge.svg)](https://github.com/a-haz/odomcal/actions/workflows/ros2-jazzy.yml)

odomcal measures and corrects the two main systematic odometry errors of a differential-drive robot: unequal wheel diameters and a wrong track width.

A seeded unicycle simulator drives the UMBmark square in both directions. The closed-form method of Borenstein and Feng recovers the diameter ratio and the track-width ratio. A linear least-squares fit on the same runs recovers the two wheel radii and the track width directly. Held-out squares then give the return-position and heading error before and after each correction.

## Method

The plant is a differential-drive unicycle. Wheel rotations `dphi_L` and `dphi_R` (radians) produce a body displacement and a yaw increment

```
ds    = (r_R * dphi_R + r_L * dphi_L) / 2
dth   = (r_R * dphi_R - r_L * dphi_L) / b
```

which are integrated as a circular arc. The two systematic parameters follow Borenstein and Feng. `Ed = r_R / r_L` is the wheel-diameter ratio and `Eb = b_actual / b_nominal` is the track-width ratio. Injected errors keep the arithmetic mean of the two radii at the nominal radius, so a pure diameter mismatch is not also a scale error. Zero-mean Gaussian slip is added to the executed wheel rotations and is not seen by the encoders.

UMBmark drives a square of side `L` in both directions, stops after each leg, and turns in place through a nominal right angle, including the fourth turn that restores the initial heading. At least five runs are taken clockwise and five counter-clockwise. The return-position error on a run is the ground-truth pose minus the odometry pose. The centers of gravity of those errors are

```
x_cg,cw  = (1/n) sum_i x_i,cw      y_cg,cw  = (1/n) sum_i y_i,cw
x_cg,ccw = (1/n) sum_i x_i,ccw     y_cg,ccw = (1/n) sum_i y_i,ccw
r_cg     = sqrt(x_cg^2 + y_cg^2)
E_max,syst = max(r_cg,cw, r_cg,ccw)
```

Type A (baseline) and Type B (diameter) orientation errors, in radians, are separated on the longitudinal centers of gravity:

```
alpha = (x_cg,cw + x_cg,ccw) / (-4 L)
beta  = (x_cg,cw - x_cg,ccw) / (-4 L)
```

The track-width ratio and the diameter ratio then follow from the square geometry:

```
Eb = 90 deg / (90 deg - alpha_deg)
R  = (L / 2) / sin(beta / 2)
Ed = (R + b/2) / (R - b/2)
```

The odometry correction leaves the average wheel diameter unchanged:

```
c_L = 2 / (Ed + 1)
c_R = 2 / ((1/Ed) + 1)
r_L = c_L * r_nominal
r_R = c_R * r_nominal
b   = Eb * b_nominal
```

The least-squares estimator uses the same encoder logs and the externally measured pose at every integration step. Arc length `ds` is reconstructed from the chord of the step, and each step contributes the linear pair

```
 r_L * dphi_L + r_R * dphi_R         = 2 ds
-r_L * dphi_L + r_R * dphi_R - b dth = 0
```

The stacked system is solved by a rank-revealing QR factorization. `Ed` and `Eb` are then `r_R / r_L` and `b / b_nominal`. Calibration runs and the runs that are scored are drawn from different seeds, so the reported before/after errors are held out from the fit. Corrected odometry is replayed on the held-out encoder logs; the plant is unchanged.

The results below come from `docs/benchmark.csv`, written by `odomcal_bench`. The setup is: square side `4.00000000` m, nominal wheel radius `0.05000000` m, nominal track width `0.30000000` m, `5` runs in each direction, `8` seeds, wheel-slip standard deviation `0.00050000` rad per step, and `16` steps on each straight leg and each turn.

## Results

These are simulation results, not a physical robot. `Ed` and `Eb` are injected into the simulated plant, so the true values are known. Real hardware adds floor contact, varying slip and encoder quantization, which the simulator does not model.

Mean over the `8` seeds (`row_type=mean`). Return error is `E_max,syst` in metres. Heading error is the mean absolute yaw of the return-position error, in degrees.

| Ed | Eb | E_max before | heading before | E_max UMBmark | heading UMBmark | E_max least squares | heading least squares |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 1.00000000 | 1.00000000 | 0.00221049 | 0.06153584 | 0.00395952 | 0.07571997 | 0.00238390 | 0.06379108 |
| 1.00100000 | 1.00000000 | 0.15190557 | 3.05354391 | 0.00410838 | 0.08330204 | 0.00238096 | 0.06379299 |
| 1.00000000 | 1.01000000 | 0.17760878 | 3.55548231 | 0.00396776 | 0.08086354 | 0.00239221 | 0.06315948 |
| 1.00200000 | 1.01000000 | 0.48345050 | 6.04429644 | 0.03153240 | 0.33588174 | 0.00238652 | 0.06316326 |
| 0.99800000 | 1.00500000 | 0.39455269 | 6.08792844 | 0.02077996 | 0.26656089 | 0.00239442 | 0.06346990 |
| 1.00500000 | 0.99000000 | 0.89934299 | 15.39400443 | 0.09115732 | 1.45105474 | 0.00236227 | 0.06444504 |

Recovered ratios on the same mean rows, next to the injected values:

| Ed true | Eb true | Ed UMBmark | Eb UMBmark | Ed least squares | Eb least squares |
| --- | --- | --- | --- | --- | --- |
| 1.00000000 | 1.00000000 | 1.00000013 | 1.00001064 | 0.99999951 | 0.99999144 |
| 1.00100000 | 1.00000000 | 1.00099973 | 0.99984071 | 1.00099950 | 0.99999144 |
| 1.00000000 | 1.01000000 | 1.00000012 | 1.00985012 | 0.99999951 | 1.00999136 |
| 1.00200000 | 1.01000000 | 1.00189538 | 1.00911159 | 1.00199949 | 1.00999136 |
| 0.99800000 | 1.00500000 | 0.99805417 | 1.00426024 | 0.99799953 | 1.00499140 |
| 1.00500000 | 0.99000000 | 1.00516543 | 0.98604066 | 1.00499947 | 0.98999153 |

At `Ed = 1.002`, `Eb = 1.01` the mean return error drops from 0.483 m to 0.032 m with UMBmark and 0.0024 m with least squares, and the heading error from 6.04 deg to 0.34 deg and 0.063 deg. The least-squares fit on that row gives `r_L = 0.04994999` m, `r_R = 0.05004987` m, and `b = 0.30299741` m, against the plant values `0.04995005` m, `0.05004995` m, and `0.30300000` m.

With no injected error, both corrections stay at the slip floor (about 0.002 to 0.004 m). The closed-form fit cannot shrink a cloud that has no systematic offset.

![Mean return-position error before and after calibration](docs/emax_before_after.png)

The animation shows the first held-out clockwise and counter-clockwise squares for seed 0 at `Ed = 1.002`, `Eb = 1.01` (`docs/trajectories.csv`). Solid lines are ground truth and dashed lines are odometry. Before calibration, odometry traces the commanded square while the plant drifts. After calibration the least-squares odometry sits on the ground truth, and UMBmark is close behind.

![Before and after square trajectories](docs/trajectories.gif)

## Build and run

The library is C++17. CMake fetches Eigen 3.4.0 and GoogleTest 1.15.2. The plot script needs Python 3 with `matplotlib` and `pillow` (`python3 -m pip install -r scripts/requirements.txt`).

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=g++
cmake --build build -j
ctest --test-dir build --output-on-failure
./build/odomcal_bench docs/benchmark.csv docs/trajectories.csv
python3 scripts/plot_benchmark.py
```

`odomcal_bench` rewrites the CSV files above. The plot script reads them and writes `docs/emax_before_after.png` and `docs/trajectories.gif`.

## ROS 2 node

`ros2/odomcal_ros` is an ament_cmake package for ROS 2 Jazzy. It pulls in this repository with `add_subdirectory` and links the `odomcal` target.

`odomcal_node` subscribes to `sensor_msgs/JointState`. `left_wheel_joint` and `right_wheel_joint` select the wheels. With `use_velocities` false, the default, the node integrates position deltas. With `use_velocities` true it integrates `velocity * dt` from successive header stamps. Each step calls `odomcal::integrate` with the left radius `r_l`, the right radius `r_r`, and the track width `track_width`, then publishes `nav_msgs/Odometry` and, unless `publish_tf` is false, the `odom` to `base_link` transform. All three are in metres and must be positive.

`config/default.yaml` is the parameter file loaded by `launch/odomcal.launch.py`. `config/calibration.yaml` is a flat file with `r_l`, `r_r`, and `track_width` (the key `b` is accepted as an alias for the track width). The launch file points `calibration_file` at it and the node reads it at startup. Reloading later keeps the pose integrated so far.

```bash
ros2 service call /odom_calibrator/set_calibration odomcal_ros/srv/SetCalibration "{r_l: 0.05, r_r: 0.05, track_width: 0.30}"
ros2 service call /odom_calibrator/reload_calibration std_srvs/srv/Trigger
```

`set_calibration` applies the three values in the request. `reload_calibration` reads `calibration_file` again.

Build and test from the repository root in a Jazzy environment. `--base-paths ros2` is needed because the root `CMakeLists.txt` is the standalone library and colcon would not find the package otherwise.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths ros2 --packages-select odomcal_ros --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=g++
colcon test --base-paths ros2 --packages-select odomcal_ros --event-handlers console_direct+
colcon test-result --verbose
source install/setup.bash
ros2 launch odomcal_ros odomcal.launch.py
```

The node listens on `joint_states` and publishes `odom`. Pass another calibration file with:

```bash
ros2 launch odomcal_ros odomcal.launch.py calibration_file:=/path/to/calibration.yaml
```

CI for this package is `.github/workflows/ros2-jazzy.yml`, which runs `colcon build` and `colcon test` in the `ros:jazzy` image. `.github/workflows/ci.yml` builds and tests the standalone library.

### Simulated square replay

`replay_square` is a simulated replay, not a recording from a real robot. It drives one counter-clockwise square in `odomcal::simulate`, publishes the encoder rotations as `sensor_msgs/JointState`, and records `nav_msgs/Odometry` from two `odomcal_node` instances. The raw node uses the nominal geometry. The corrected node loads the simulated plant's true radii and track width from YAML. Endpoint error is the planar distance between the plant and the published odometry at the end of the square, and heading error is the yaw difference there.

```bash
source install/setup.bash
ros2 run odomcal_ros replay_square --output ros2/odomcal_ros/demo/square_replay.csv
```

The numbers come from `ros2/odomcal_ros/demo/square_replay.csv`. The run uses a 4 m square, 16 steps per segment, no slip noise, `Ed = 1.002` and `Eb = 1.01`. Nominal geometry is `r = 0.05` m and `b = 0.3` m; the plant has `r_L = 0.04995`, `r_R = 0.05005` and `b = 0.303` m. Each node published 129 odometry messages from 129 joint messages.

| | raw | corrected |
| --- | --- | --- |
| x (m) | 1.165963023405459e-13 | 0.08861923920535794 |
| y (m) | -1.1368683772161603e-13 | -0.08354551951860723 |
| theta (rad) | -9.439671266875393e-14 | 0.04329530048338545 |
| endpoint error (m) | 0.121791721347939 | 1.5113053650432164e-13 |
| heading error (rad) | 0.043295300483575355 | 9.550693569337909e-14 |
| heading error (deg) | 2.48063799045958 | 5.47214432945798e-12 |

## References

Borenstein, J. and Feng, L. (1996). Measurement and Correction of Systematic Odometry Errors in Mobile Robots. IEEE Transactions on Robotics and Automation, 12(6), 869-880.

Borenstein, J. and Feng, L. (1995). UMBmark: A Benchmark Test for Measuring Odometry Errors in Mobile Robots. Proceedings of the SPIE Conference on Mobile Robots, Philadelphia, 22-26 October 1995.
