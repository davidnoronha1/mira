# mira2_actions

Behavior Tree-based autonomous task execution for the Mira2 AUV. Each mission (gate pass, docking, bucket task) is described as an XML behavior tree that composes reusable motion and detection leaf nodes.

## Architecture

```mermaid
graph TD
    subgraph Perception
        DET["/detectnet/detections<br/>(vision_msgs/Detection2DArray)"]
        GP["gate_pose<br/>(geometry_msgs/PoseStamped)"]
        DP["dock_pose<br/>(geometry_msgs/PoseStamped)"]
        HDG["/master/heading<br/>(std_msgs/Float32)"]
    end

    subgraph Vehicle State
        TEL["/master/telemetry<br/>(custom_msgs/Telemetry)"]
    end

    subgraph mira2_actions_exe
        BT["BehaviorTree Factory<br/>(behaviortree_cpp)"]
        subgraph Behaviour Leaves
            DOB["DetectObjectBoundingBox"]
            DTP["DetectTargetPoint"]
        end
        subgraph Motion Leaves
            ABB["ApproachBoundingBox"]
            AXY["AlignXY"]
            AWD["ApproachWithDepth"]
            HP["HoldPosition"]
            LE["LateralEvasion"]
            RTH["RotateToTargetHeading"]
            YS["YawSweep"]
        end
    end

    subgraph Output
        CMD["/master/commands<br/>(custom_msgs/Commands)"]
        G2["Groot2 Publisher<br/>(ZMQ port 1667)"]
    end

    DET --> DOB
    GP  --> DTP
    DP  --> ABB
    HDG --> RTH
    TEL --> ABB
    TEL --> AXY
    TEL --> AWD
    TEL --> HP
    TEL --> RTH
    TEL --> YS

    BT --> CMD
    BT --> G2
```

## Behavior Tree Nodes

### Behaviour (Condition / Action) Nodes
| Node | Description |
|---|---|
| `DetectObjectBoundingBox` | Waits for a stable YOLO detection from `/detectnet/detections` with consistency checking |
| `DetectTargetPoint` | Detects a target point from a configurable pose topic with optional color filtering |

### Motion (Action) Nodes
| Node | Description |
|---|---|
| `ApproachBoundingBox` | PID-controlled forward approach to a detected bounding box |
| `AlignXY` | PD lateral/depth alignment to center a detection in frame |
| `ApproachWithDepth` | Descends to a depth setpoint while maintaining heading |
| `HoldPosition` | Holds the current position for a configurable duration |
| `TimedForward` | Drives forward at fixed thrust for a fixed duration |
| `LateralEvasion` | Moves laterally to avoid an obstacle |
| `RotateToTargetHeading` | Rotates to an absolute heading via yaw PID |
| `YawSweep` | Oscillates yaw to search for a target |

## Mission XML Files

| File | Description |
|---|---|
| `config/default.xml` | Simple single-gate approach |
| `config/docking.xml` | 4-phase docking: detect → align XY → align yaw → approach |
| `config/bucket_task.xml` | Bucket detection and approach task |
| `config/test_detect_adarsh.xml` | Detection smoke-test tree |

## Topics

| Topic | Type | Direction |
|---|---|---|
| `/master/telemetry` | `custom_msgs/Telemetry` | Subscribed |
| `/master/heading` | `std_msgs/Float32` | Subscribed |
| `detectnet/detections` | `vision_msgs/Detection2DArray` | Subscribed |
| `gate_pose` | `geometry_msgs/PoseStamped` | Subscribed |
| `dock_pose` | `geometry_msgs/PoseStamped` | Subscribed |
| `/master/commands` | `custom_msgs/Commands` | Published |
| `<name>_pid_error` | `std_msgs/Float32` | Published (debug) |
| `<name>_pid_output` | `std_msgs/Float32` | Published (debug) |

## Docking Sequence

```mermaid
sequenceDiagram
    participant BT as BehaviorTree
    participant DET as DetectObjectBoundingBox
    participant AXY as AlignXY
    participant RTH as RotateToTargetHeading
    participant ABB as ApproachBoundingBox
    participant CMD as /master/commands

    BT->>DET: tick — await stable dock detection
    DET-->>BT: SUCCESS
    BT->>AXY: tick — center dock in frame (lateral + depth PIDs)
    AXY-->>CMD: PWM commands
    AXY-->>BT: SUCCESS
    BT->>RTH: tick — rotate to dock heading (yaw PID)
    RTH-->>CMD: PWM commands
    RTH-->>BT: SUCCESS
    BT->>ABB: tick — close range approach (forward + lateral PIDs)
    ABB-->>CMD: PWM commands
    ABB-->>BT: SUCCESS
```

## Usage

```bash
ros2 launch mira2_actions mira2_actions_launch.py
```

To visualize the running behavior tree, connect Groot2 to `localhost:1667`.

To change the active mission, edit `mira2_actions_launch.py` and point the `tree_xml` parameter at the desired config file.
