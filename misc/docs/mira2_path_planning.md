# mira2_path_planning

Autonomous navigation nodes for the Mira2 AUV. Provides task-specific navigation for gate traversal, docking, and bucket approach. Each node implements a closed-loop PID controller driven by perception data and vehicle telemetry.

## Architecture

```mermaid
graph TD
    subgraph Perception
        OBJ["/mira/perception/front/objects<br/>(vision_msgs/Detection2DArray)"]
        GP["/mira/perception/front/gate_pose<br/>(geometry_msgs/PoseStamped)"]
        DP["/mira/perception/bottom/dock_pose<br/>(geometry_msgs/PoseStamped)"]
        TP["/mira/perception/bottom/target_pose<br/>(geometry_msgs/PoseStamped)"]
        TC["/mira/perception/bottom/target_color<br/>(std_msgs/String)"]
        PO["/mira/perception/bottom/pipeline/{offset,angle}"]
    end

    subgraph Vehicle State
        TEL["/mira/state/telemetry<br/>(custom_msgs/Telemetry)"]
    end

    subgraph mira2_path_planning
        GN["gate_navigator_exe<br/>(C++)"]
        DC["phase2_dock_controls.py"]
        P2["phase2.py (bucket task)"]
        PC["pipeline_controls.py"]
    end

    subgraph Output
        CMD["/mira/control/command<br/>(custom_msgs/CommandsNormalized)"]
    end

    OBJ --> GN
    GP  --> GN
    TEL --> GN

    DP  --> DC
    TEL --> DC

    TP  --> P2
    TC  --> P2
    TEL --> P2

    PO  --> PC
    TEL --> PC

    GN --> CMD
    DC --> CMD
    P2 --> CMD
    PC --> CMD
```

## Nodes

### `gate_navigator_exe` (C++)

Vision-guided gate approach node running at 10 Hz. Uses four PID controllers to simultaneously maintain depth, heading, lateral centering on the detected gate, and forward progress.

**PID configuration (defaults):**

| Controller | kp | ki | kd | base_offset | setpoint |
|---|---|---|---|---|---|
| Depth | 5.5 | 0.03 | 31.5 | 1580 | 1069 |
| Yaw | 3.18 | 0.01 | 7.2 | 1500 | 280° |
| Lateral | −0.5 | −0.05 | −2.0 | 1500 | center |
| Forward | −0.8 | −0.1 | −4.0 | 1500 | — |

### `dock_controller` (`dock_controller.py`)

State machine docking controller running at 20 Hz. Drives the AUV to an ArUco-tagged dock through a sequence of alignment and approach phases.

**Docking state machine:**

```mermaid
stateDiagram-v2
    [*] --> SEARCH
    SEARCH --> ALIGN_XY : dock_pose received
    ALIGN_XY --> ALIGN_YAW : lateral + depth error < threshold
    ALIGN_YAW --> APPROACH : yaw error < threshold
    APPROACH --> BLIND_LATCH : distance < latch threshold
    BLIND_LATCH --> DOCKED : timeout elapsed
    ALIGN_XY --> RECOVERY : detection lost
    ALIGN_YAW --> RECOVERY : detection lost
    APPROACH --> RECOVERY : detection lost
    RECOVERY --> SEARCH : recovery timeout
    DOCKED --> [*]
```

### `phase2` (`phase2.py`) — Bucket Task

Vision-based bucket approach for Phase 2 of the competition task. Aligns with a colored ellipsoid marker, then approaches and locks onto the bucket.

**States:** `SEARCH → ALIGN_XY → APPROACH → LOCK → SEARCH2`

## Topics

Names come from `utils.topics` / `utils/topics.hpp` (see `misc/docs/system_topics.md`).

| Topic | Type | Direction | Node |
|---|---|---|---|
| `/mira/state/telemetry` | `custom_msgs/Telemetry` | Subscribed | all |
| `/mira/perception/front/objects` | `vision_msgs/Detection2DArray` | Subscribed | gate_navigator |
| `/mira/perception/front/gate_pose` | `geometry_msgs/PoseStamped` | Subscribed | gate_navigator (no publisher yet) |
| `/mira/perception/bottom/dock_pose` | `geometry_msgs/PoseStamped` | Subscribed | phase2_dock_controls |
| `/mira/perception/bottom/target_pose` | `geometry_msgs/PoseStamped` | Subscribed | phase2 |
| `/mira/perception/bottom/target_color` | `std_msgs/String` | Subscribed | phase2 |
| `/mira/perception/bottom/pipeline/offset` | `geometry_msgs/Point` | Subscribed | pipeline_controls |
| `/mira/perception/bottom/pipeline/angle` | `std_msgs/Float32` | Subscribed | pipeline_controls |
| `/mira/control/command` | `custom_msgs/CommandsNormalized` | Published | all |
| `/mira/control/pipeline/enable` | `std_srvs/SetBool` | Service | pipeline_controls |

## Usage

```bash
# Gate navigation
ros2 launch mira2_path_planning qualification.launch

# Docking controller
ros2 run mira2_path_planning dock_controller

# Bucket phase 2
ros2 run mira2_path_planning phase2
```

> These are task-specific routines. PID gains and timing parameters are tuned for competition hardware and will need adjustment for different vehicles.

## External Resources

- [PID Controller Theory](https://en.wikipedia.org/wiki/PID_controller)
- [ArUco Marker Detection](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)
