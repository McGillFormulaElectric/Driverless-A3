# MFE Driverless — Assignment 3: TF2 and coordinate frames

Every node in a driverless stack lives inside a tree of coordinate frames — the LiDAR sees things in `lidar`, the camera in `camera`, the ego pose is tracked in `base_link`, and the map is anchored to `map`. Getting a cone from a pixel to a map-frame waypoint is the exact skill this assignment drills.

We use ROS 2's [tf2](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html) library — the standard way to publish, listen to, and chain transforms — on a scenario that mirrors the Formula Student **skidpad**: a car driving a circle of radius ~9.125 m at ~4.6 m/s while a front-mounted camera reports cone positions.

It is split into two parts, [Advent-of-Code style](https://adventofcode.com/):

- **A3.1** — transform cones from the camera frame into `base_link` using a **static** transform.
- **A3.2** — transform cones all the way into `map`, through a **dynamic** `map -> base_link` published at 50 Hz published by Neil. Because the car is moving, the same cone constellation traces circles in the map frame — that is the visual proof your TF chain works.

Everything runs inside a Docker container so your local OS and Python version don't matter.

---

## 1. Getting started

### 1.1 GitHub
1. Go to this repo on GitHub.
2. Create a new branch named after you, e.g. `NeilJoeGeorge`.
3. Clone and check out your branch:

```bash
git clone <repo-url>
cd Driverless-A3
git checkout <FirstNameLastName>
```

### 1.2 Tailscale (class VPN)
Neil runs the TF scenario + grader on the class Tailscale network. Every student joins the same tailnet so DDS discovery works between machines.

1. Install Tailscale: <https://tailscale.com/download>.
2. `sudo tailscale up` and sign in with the invite Neil sent.
3. Verify: `tailscale ping neil` (hostname will be shared in class).
4. Note your own Tailscale hostname/IP — you'll set it via env var below if auto-detection fails.

### 1.3 Docker
Linux host with Docker + Docker Compose is the supported path (host networking + Tailscale interface work cleanly).

```bash
cd docker
export GITHUB_USER=<your-github-handle>            # required
export A3_NEIL_HOST=<professor tailnet host>  # e.g. neil.tail1234.ts.net
docker compose build
docker compose run --rm student
```

Inside the container you'll have `/workspace` mounted to `ros2_ws/`. Build and source:

```bash
cd /workspace
colcon build --symlink-install
source install/setup.bash
```

> **macOS/Windows caveat:** Docker Desktop's `network_mode: host` is limited. If you're not on Linux, run the container with `--network host` on a Linux VM, or use Tailscale's [userspace networking mode](https://tailscale.com/kb/1112/userspace-networking) inside the container. Ask Neil for the current recommendation.

---

## 2. The scenario

Neil's node (`a3_neil tf_scenario_node`) does three things:

1. Publishes the **static** transform `base_link -> camera` at translation `(0.5, 0.0, 0.2)`, no rotation. The camera sits 0.5 m in front of the rear axle and 0.2 m up.
2. Publishes the **dynamic** transform `map -> base_link` at 50 Hz. The car follows a skidpad circle:

   ```
   x(t) = R * cos(omega * t)
   y(t) = R * sin(omega * t)
   yaw(t) = omega * t + pi/2       (tangent to the circle)
   R = 9.125 m,  omega = 0.5 rad/s
   ```

3. Publishes `/neil/sensor_cones` (`geometry_msgs/PoseArray`, `frame_id = "camera"`) at 20 Hz. Eight cones in a fixed constellation 5–15 m in front of the sensor.

That is the whole scene. Your job is to move those cones into the frame the rest of the driverless stack cares about.

TF theory refs:
- ROS 2 [tf2 tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html).
- REP 105 — [Coordinate Frames for Mobile Platforms](https://www.ros.org/reps/rep-0105.html).
- REP 103 — [Standard Units of Measure and Coordinate Conventions](https://www.ros.org/reps/rep-0103.html).

---

## 3. A3.1 — Static transform (warm-up)

**Goal:** subscribe to `/neil/sensor_cones`, transform every pose into `base_link` using the static camera mount, and publish the result on `/${GITHUB_USER}/cones_base_link` (`geometry_msgs/PoseArray`, `frame_id = "base_link"`).

Template: `ros2_ws/src/a3_new_member/a3_new_member/static_tf_node.py`. There's a `TODO` block inside `_on_cones`. Use `tf2_geometry_msgs.do_transform_pose(pose, tf)` — the `tf` is already looked up for you.

Because this transform is static, the expected result is trivially each camera-frame cone shifted by `(0.5, 0.0, 0.2)`. That is exactly what the grader checks (mean per-cone error <= **0.05 m** over the last 50 messages).

Copy the professor's `header.stamp` into your output PoseArray header — the grader time-matches on it for A3.2, and you should get in the habit now.

Run it:
```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch a3_new_member static.launch.py github_user:=$GITHUB_USER
```

Watch the feedback:
```bash
ros2 topic echo /neil/feedback
```

You should see:
```
Congrats <your-github-user>, the answer is correct
```

**Deliverable for A3.1:** screenshot of `/neil/feedback`, committed to your branch as `submissions/a3_1_feedback.png`, plus your finished `static_tf_node.py`.

---

## 4. A3.2 — Dynamic transform on the skidpad

Same input, harder target frame: publish the cones in `map` on `/${GITHUB_USER}/cones_map` (`geometry_msgs/PoseArray`, `frame_id = "map"`).

Template: `ros2_ws/src/a3_new_member/a3_new_member/dynamic_tf_node.py`. The code you write is *almost identical* to A3.1 — just ask tf2 for `map <- camera` at the message stamp. tf2 walks the tree `map <- base_link <- camera` for you.

The magic here: the car is **driving in a circle**, so the exact same cone in the camera frame is at a different point in `map` every single message. Plot the published PoseArray in Foxglove with the `map` frame fixed and every cone traces its own circle. That is your visual proof the dynamic TF chain works.

Grader tolerance: mean per-cone error <= **0.15 m** (looser than A3.1 because the dynamic transform is time-sensitive and small stamp-vs-lookup slop is expected).

Run it:
```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch a3_new_member dynamic.launch.py github_user:=$GITHUB_USER
```

**Deliverable for A3.2:** screenshot of `/neil/feedback`, plus a Foxglove screenshot showing the cones tracing circles in `map`, committed as `submissions/a3_2_feedback.png` and `submissions/a3_2_map.png`, plus your finished `dynamic_tf_node.py`.

---

## 5. Visualizing with Foxglove Studio (strongly recommended)

Seeing the frames move in 3D is the fastest way to spot a bug (wrong frame, missing stamp, swapped source/target).

### 5.1 Start the Foxglove bridge inside the container
Already available in the image:
```bash
ros2 run foxglove_bridge foxglove_bridge port:=8765
```
Leave that terminal running.

### 5.2 Install Foxglove Studio on your host
Download from <https://foxglove.dev/download> (free, Linux/macOS/Windows).

### 5.3 Connect and build the layout
1. Open Foxglove Studio → **Open connection…** → **Foxglove WebSocket**.
2. URL: `ws://localhost:8765` (or `ws://<your-tailscale-host>:8765` from another machine on the tailnet).
3. Add a **3D** panel.
   - Set **Fixed Frame** to `map`.
   - Enable topics `/tf` and `/tf_static` so you see the frames.
   - Add `/neil/sensor_cones` (drawn in the `camera` frame — will fly around with the car).
   - Add `/${GITHUB_USER}/cones_base_link` (drawn in `base_link` — will orbit the origin with the car).
   - Add `/${GITHUB_USER}/cones_map` (drawn in `map` — should trace circles as time passes).
4. Add a **Raw Messages** panel on `/neil/feedback` to watch verdicts.
5. (Debug) In a terminal, `ros2 run tf2_tools view_frames` prints the current frame tree to a PDF; useful when a transform is missing.

> Tip: save your Foxglove layout to `submissions/a3_layout.json` (**Layout → Export**) so future assignments can reuse it.

---

## 6. Topic contract (summary)

| Topic                          | Type                        | Frame        | Owner    | Purpose                                    |
| ------------------------------ | --------------------------- | ------------ | -------- | ------------------------------------------ |
| `/tf`, `/tf_static`            | `tf2_msgs/TFMessage`        | —            | Neil | `base_link->camera` (static), `map->base_link` (50 Hz) |
| `/neil/sensor_cones`      | `geometry_msgs/PoseArray`   | `camera`     | Neil | 8 cones, 20 Hz                             |
| `/neil/feedback`          | `std_msgs/String`           | —            | Neil | Per-student grading verdict                |
| `/<user>/cones_base_link`      | `geometry_msgs/PoseArray`   | `base_link`  | Student  | A3.1 output                                |
| `/<user>/cones_map`            | `geometry_msgs/PoseArray`   | `map`        | Student  | A3.2 output                                |

All pubs/subs use `RELIABLE`, `KEEP_LAST`, depth 10 QoS. The TF broadcasters/listeners use tf2's own baked-in QoS — leave those alone.

---

## 7. Layout

```
Driverless-A3/
├── docker/                    # Dockerfile, compose, CycloneDDS config, entrypoint
├── ros2_ws/
│   └── src/
│       ├── a3_new_member/        # your template — this is where you write code
│       └── a3_neil/      # for reference; not run by students
└── README.md
```

## 8. Troubleshooting

- **`ros2 topic list` doesn't show `/neil/sensor_cones` or `/tf`.** DDS discovery isn't reaching Neil's node. Confirm `tailscale ping <professor-host>` works, that `A3_NEIL_HOST` is set, and that `ROS_DOMAIN_ID` matches (`42`).
- **`TransformException: "camera" passed to lookupTransform argument source_frame does not exist`.** The static transform hasn't arrived yet, or your listener started before `/tf_static` was published. Wait a second and try again, or add a `TransformListener` earlier in your node's construction (the template already does this).
- **`Lookup would require extrapolation into the past/future`.** You're calling `lookup_transform` with a stamp the buffer no longer has (past) or hasn't received yet (future). The templates ask for the message stamp with a small timeout — keep it that way.
- **Grader keeps saying incorrect.** Check `header.frame_id` on your output is exactly `base_link` / `map`, that you copy `msg.header.stamp` into your output, that you're publishing 8 poses, and that `tf2_geometry_msgs` is imported (needed for the side-effect Pose registration even if you only call `do_transform_pose`).
- **A3.2 works for a few seconds then goes wrong.** You're probably transforming with `Time()` (latest) instead of `msg.header.stamp`. The car has moved between the message being generated and your callback firing; use the stamp.

---

## 9. Submission
1. Commit your changes to your `FirstNameLastName` branch.
2. Include the feedback + Foxglove screenshots in `submissions/`.
3. Open a pull request against `main` when done.
