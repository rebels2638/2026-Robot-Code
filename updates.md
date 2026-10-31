# Codebase Updates & Log Analysis Report

## Overview of Log Analysis

An analysis was conducted across all **240 `.wpilog` files** in [`FRC Robot Logs/`](file:///c:/Users/sleepy/Documents/GitHub/2026-Robot-Code/FRC%20Robot%20Logs), with deep cycle-by-cycle telemetry inspection of the **11 Houston Championship (Johnson Division)** match logs (`P002`, `Q007`, `Q017`, `Q029`, `Q047`, `Q058`, `Q072`, `Q080`, `Q091`, `Q110`, `Q122`) as well as the **NYLI2 Regional** and shop logs.

### Key Telemetry Observations Across Houston Championship Logs
| Metric | Observed Telemetry in `.wpilog` Files |
| :--- | :--- |
| **Loop Period Overruns (`> 20ms`)** | **147,597** total loop overrun warnings across all 240 logs; **9,787** overruns across the 11 Houston Championship matches (`610`–`1,073` per match). |
| **Average / P95 User Code Time** | Mean `UserCodeMS`: **15.3–17.7 ms** (leaving only `2.3–4.7 ms` of headroom per `20 ms` cycle); P95 `UserCodeMS`: **21.2–27.1 ms**; max spikes up to **1,161 ms**. |
| **Overrun Subsystem Breakdown (`Q122`)** | During overrun cycles: `Vision.periodic()` averaged **13.97 ms** (P95 **36.23 ms**, max **379.16 ms**), `SwerveDrive.periodic()` averaged **10.09 ms** (P95 **18.29 ms**), `Superstructure.periodic()` averaged **8.35 ms** (P95 **23.67 ms**), `Shooter.periodic()` peaked at **129.07 ms**, and `Intake.periodic()` peaked at **75.92 ms**. |
| **Disabled Vision Gyro Reset Spikes** | `/RealOutputs/RobotState/vision/gyroReset` fired **283–506 times per match** (`446` times in `Q122`, `506` in `Q058`). On cycles where `gyroReset` executed (`Q122`), `UserCodeMS` averaged **30.67 ms** (P95 **66.58 ms**, max **1,161.16 ms**), and **33.2% (`79 / 238`)** of all rejected vision observations occurred within `50 ms` of a `gyroReset`. |
| **Uncached TalonFX Closed-Loop Signals** | In `Q122` (`6,593` loop cycles), `/Inputs/Shooter/turretClosedLoopReferenceRotations` updated only **607 times** (`9.2%`) and `/Inputs/Intake/pivotClosedLoopReferenceRotations` updated only **50 times** (`0.76%`) because neither signal was registered with `PhoenixUtil` or configured before `optimizeBusUtilization()`. |
| **Intake Pivot Direction Chatter** | While enabled with a stationary pivot setpoint, `Intake` flipped its `movingUp` profile selection **56–401 times per match** (`401` flips in `Q072`, `299` in `Q122`, `228` in `Q058`), repeatedly calling blocking CAN configuration updates (`getConfigurator().apply(...)`). |
| **DriverStation Console Warning Spam** | `Joystick Button 5 on port 3 not available` was logged **108 times** in Houston Championship matches (**9,532 times** across all 240 logs). |

---

## Detailed Changes, Purpose, and Reasoning

### 1. Non-Blocking Gyro Resets, Pose Buffer Seeding, and Redundant Disabled Reset Suppression
* **Files Modified**:
  * [`src/main/java/frc/robot/subsystems/swerve/gyro/GyroIOPigeon2.java`](file:///c:/Users/sleepy/Documents/GitHub/2026-Robot-Code/src/main/java/frc/robot/subsystems/swerve/gyro/GyroIOPigeon2.java)
  * [`src/main/java/frc/robot/RobotState.java`](file:///c:/Users/sleepy/Documents/GitHub/2026-Robot-Code/src/main/java/frc/robot/RobotState.java)
  * [`src/test/java/frc/robot/RobotStateTest.java`](file:///c:/Users/sleepy/Documents/GitHub/2026-Robot-Code/src/test/java/frc/robot/RobotStateTest.java)
* **Problem Identified in Logs**:
  1. While disabled before a match, `RobotState.addVisionObservation()` triggered `resetPose()` every `1.0 s` whenever a multi-tag or single-tag MegaTag1 observation arrived (`446` times in `Q122`).
  2. `GyroIOPigeon2.resetGyro()` called `gyro.setYaw(yaw.getDegrees())` using Phoenix 6's default **50 ms blocking timeout** on the main robot thread, causing `UserCodeMS` on `gyroReset` cycles to average **30.67 ms** (P95 **66.58 ms**).
  3. `RobotState.resetPose()` called `poseBuffer.clear()` without seeding the initial pose sample. Any subsequent camera observations processed in the same `Vision.periodic()` pass (or before the next odometry sample) threw `NoSuchElementException` on `poseBuffer.getInternalBuffer().lastKey()` inside `addVisionObservation()`, accounting for **33.2% (`79 / 238`)** of all rejected vision measurements in `Q122`.
  4. `RobotState.addVisionObservation()` only checked `visionRotationStdDev > 0.0`, which evaluates to `true` for `Double.POSITIVE_INFINITY` (used by MegaTag2 observations to reject yaw updates).
  5. Telemetry output had a typo `Logger.recordOutput("RobotState/vision/gyroRestRotation", rotation)`.
* **Changes Made**:
  * Updated [`GyroIOPigeon2.resetGyro()`](file:///c:/Users/sleepy/Documents/GitHub/2026-Robot-Code/src/main/java/frc/robot/subsystems/swerve/gyro/GyroIOPigeon2.java#L125) to call `gyro.setYaw(yaw.getDegrees(), 0.0)` so the CAN frame is transmitted asynchronously without blocking the `20 ms` robot loop.
  * Updated [`RobotState.resetPose()`](file:///c:/Users/sleepy/Documents/GitHub/2026-Robot-Code/src/main/java/frc/robot/RobotState.java#L256-L259) to immediately seed `poseBuffer.addSample(lastEstimatedPoseUpdateTime, initialPose)` after `poseBuffer.clear()`.
  * Added [`RobotState.shouldSkipRedundantDisabledGyroReset()`](file:///c:/Users/sleepy/Documents/GitHub/2026-Robot-Code/src/main/java/frc/robot/RobotState.java#L288-L299) so automatic disabled vision gyro resets are skipped when the robot's estimated heading already agrees with the vision heading within `1.0°` (whilst still updating the pose estimator's translation and rotation normally).
  * Guarded rotational vision updates in [`RobotState.addVisionObservation()`](file:///c:/Users/sleepy/Documents/GitHub/2026-Robot-Code/src/main/java/frc/robot/RobotState.java#L221) with `Double.isFinite(visionRotationStdDev)`.
  * Corrected telemetry key to `RobotState/vision/gyroResetRotation`.

---

### 2. Consolidate Native RoboRIO CAN Bus Signal Refreshing (`""` vs `"rio"`)
* **Files Modified**:
  * [`src/main/java/frc/robot/lib/util/PhoenixUtil.java`](file:///c:/Users/sleepy/Documents/GitHub/2026-Robot-Code/src/main/java/frc/robot/lib/util/PhoenixUtil.java)
  * [`src/main/deploy/configs/hopper/comp.json`](file:///c:/Users/sleepy/Documents/GitHub/2026-Robot-Code/src/main/deploy/configs/hopper/comp.json)
  * [`src/test/java/frc/robot/lib/util/PhoenixUtilTest.java`](file:///c:/Users/sleepy/Documents/GitHub/2026-Robot-Code/src/test/java/frc/robot/lib/util/PhoenixUtilTest.java)
* **Problem Identified in Logs**:
  * `hopper/comp.json` configured `"canBusName": ""` while `intake/comp.json`, `shooter/comp.json`, and `climber/comp.json` configured `"canBusName": "rio"`.
  * Both `""` and `"rio"` refer to the native RoboRIO CAN bus in Phoenix 6, but `PhoenixUtil.registerSignals()` keyed its signal map by the raw string, causing `PhoenixUtil.refreshAll()` to perform two separate `BaseStatusSignal.refreshAll()` JNI calls for the same CAN bus every `20 ms` cycle.
* **Changes Made**:
  * Added [`PhoenixUtil.normalizeNetworkName()`](file:///c:/Users/sleepy/Documents/GitHub/2026-Robot-Code/src/main/java/frc/robot/lib/util/PhoenixUtil.java#L29-L38) to normalize `null`, `""`, and `"rio"` to `"rio"`, and updated [`hopper/comp.json`](file:///c:/Users/sleepy/Documents/GitHub/2026-Robot-Code/src/main/deploy/configs/hopper/comp.json#L2) to `"rio"`.

---

### 3. Cache Unregistered TalonFX Closed-Loop Reference Signals & Eliminate Redundant Control Allocations
* **Files Modified**:
  * [`src/main/java/frc/robot/subsystems/shooter/ShooterIOTalonFX.java`](file:///c:/Users/sleepy/Documents/GitHub/2026-Robot-Code/src/main/java/frc/robot/subsystems/shooter/ShooterIOTalonFX.java)
  * [`src/main/java/frc/robot/subsystems/intake/IntakeIOTalonFX.java`](file:///c:/Users/sleepy/Documents/GitHub/2026-Robot-Code/src/main/java/frc/robot/subsystems/intake/IntakeIOTalonFX.java)
  * [`src/main/java/frc/robot/subsystems/climber/ClimberIOTalonFX.java`](file:///c:/Users/sleepy/Documents/GitHub/2026-Robot-Code/src/main/java/frc/robot/subsystems/climber/ClimberIOTalonFX.java)
* **Problem Identified in Logs**:
  * `ShooterIOTalonFX.updateInputs()` called `turretMotor.getClosedLoopReference().getValueAsDouble()` and `IntakeIOTalonFX.updateInputs()` called `pivotMotor.getClosedLoopReference().getValueAsDouble()` directly without caching the `StatusSignal`, configuring its frequency before `optimizeBusUtilization()`, or registering it with `PhoenixUtil.registerSignals()`.
  * Because `optimizeBusUtilization()` disables all unconfigured status signals (`0 Hz`), calling `.getClosedLoopReference().getValueAsDouble()` every loop triggered synchronous on-demand CAN queries on a disabled signal (`607` and `50` updates across `6,593` cycles in `Q122`) and contributed to `Shooter.periodic()` / `Intake.periodic()` latency spikes.
  * Additionally, `ShooterIOTalonFX.setShotVelocity()` and `setFlywheelVoltage()` allocated a `new Follower(...)` and sent a duplicate `flywheelFollowerMotor.setControl(...)` CAN frame every `20 ms` cycle even when the follower was already following the leader.
* **Changes Made**:
  * Cached `turretClosedLoopReference` in [`ShooterIOTalonFX`](file:///c:/Users/sleepy/Documents/GitHub/2026-Robot-Code/src/main/java/frc/robot/subsystems/shooter/ShooterIOTalonFX.java#L60) and `pivotClosedLoopReference` in [`IntakeIOTalonFX`](file:///c:/Users/sleepy/Documents/GitHub/2026-Robot-Code/src/main/java/frc/robot/subsystems/intake/IntakeIOTalonFX.java#L48), configured both at `50 Hz`, and registered them with `PhoenixUtil.registerSignals()`.
  * Pre-allocated `Follower`, `VoltageOut`, and `TorqueCurrentFOC` control requests in [`ShooterIOTalonFX`](file:///c:/Users/sleepy/Documents/GitHub/2026-Robot-Code/src/main/java/frc/robot/subsystems/shooter/ShooterIOTalonFX.java#L86-L90) and [`ClimberIOTalonFX`](file:///c:/Users/sleepy/Documents/GitHub/2026-Robot-Code/src/main/java/frc/robot/subsystems/climber/ClimberIOTalonFX.java#L40), and tracked `isFollowerFollowingLeader` so the follower control frame is only sent when transitioning into follower mode.

---

### 4. Eliminate Blocking Intake Pivot Configurator Calls and Direction Chatter
* **Files Modified**:
  * [`src/main/java/frc/robot/subsystems/intake/IntakeIOTalonFX.java`](file:///c:/Users/sleepy/Documents/GitHub/2026-Robot-Code/src/main/java/frc/robot/subsystems/intake/IntakeIOTalonFX.java)
  * [`src/main/java/frc/robot/subsystems/intake/Intake.java`](file:///c:/Users/sleepy/Documents/GitHub/2026-Robot-Code/src/main/java/frc/robot/subsystems/intake/Intake.java)
  * [`src/test/java/frc/robot/subsystems/intake/IntakeTest.java`](file:///c:/Users/sleepy/Documents/GitHub/2026-Robot-Code/src/test/java/frc/robot/subsystems/intake/IntakeTest.java)
* **Problem Identified in Logs**:
  * In `Intake.setPivotAngle()`, `boolean movingUp = angle.getRotations() > inputs.pivotPositionRotations` had zero hysteresis around the target angle. When holding a constant target angle (`0.0` deployed or `0.328` stowed), vibration and encoder noise caused `movingUp` to flip **56–401 times per match** (`401` times in `Q072`, `299` in `Q122`).
  * Whenever `movingUp` flipped, `IntakeIOTalonFX.setPivotAngle()` invoked `pivotMotor.getConfigurator().apply(pivotConfig.MotionMagic, 0.01)`—a blocking CAN configuration write inside the periodic control loop, causing `Intake.periodic()` spikes up to **75.9 ms**.
* **Changes Made**:
  * Replaced `MotionMagicVoltage` + runtime `getConfigurator().apply(...)` in [`IntakeIOTalonFX`](file:///c:/Users/sleepy/Documents/GitHub/2026-Robot-Code/src/main/java/frc/robot/subsystems/intake/IntakeIOTalonFX.java#L51-L53) with `DynamicMotionMagicVoltage`, which transmits velocity, acceleration, and jerk constraints directly inside the `setControl()` frame with zero blocking configuration overhead.
  * Added [`Intake.resolvePivotMovingUp()`](file:///c:/Users/sleepy/Documents/GitHub/2026-Robot-Code/src/main/java/frc/robot/subsystems/intake/Intake.java#L115-L126) with a hysteresis band of `pivotAngleToleranceRotations` so the motion profile direction does not chatter when the pivot is already within tolerance of its target angle.

---

### 5. Vision Periodic NetworkTables Caching, Fast-Path Empty Queues, and Interface Method Integration
* **Files Modified**:
  * [`src/main/java/frc/robot/subsystems/vision/VisionIO.java`](file:///c:/Users/sleepy/Documents/GitHub/2026-Robot-Code/src/main/java/frc/robot/subsystems/vision/VisionIO.java)
  * [`src/main/java/frc/robot/subsystems/vision/VisionIOLimelight.java`](file:///c:/Users/sleepy/Documents/GitHub/2026-Robot-Code/src/main/java/frc/robot/subsystems/vision/VisionIOLimelight.java)
  * [`src/main/java/frc/robot/subsystems/vision/Vision.java`](file:///c:/Users/sleepy/Documents/GitHub/2026-Robot-Code/src/main/java/frc/robot/subsystems/vision/Vision.java)
* **Problem Identified in Logs**:
  * `Vision.periodic()` was the single largest contributor to loop overruns, averaging **13.97 ms** (P95 **36.23 ms**, max **379.16 ms**) during overrun cycles in `Q122`.
  * `VisionIOLimelight` looked up NetworkTables entries by string (`getLimelightNTTableEntry(name, "robot_orientation_set").setDoubleArray(...)` and `"imumode_set"`) and allocated new `double[6]` arrays, `HashSet`, and `ArrayList` instances for all 4 Limelights every `20 ms` cycle, even when `readQueue()` returned zero frames.
  * `Vision.ENABLE_DETAILED_POSE_LOGGING` had been hardcoded to `true` (with a comment noting it was temporary), serializing 6 `Pose3d[]` arrays per camera (`24` `Pose3d[]` arrays per cycle) even when `Constants.VERBOSE_LOGGING_ENABLED` was `false`.
* **Changes Made**:
  * Added `public default void setImuMode(int imuMode) {}` to [`VisionIO`](file:///c:/Users/sleepy/Documents/GitHub/2026-Robot-Code/src/main/java/frc/robot/subsystems/vision/VisionIO.java#L47) and implemented it in [`VisionIOLimelight`](file:///c:/Users/sleepy/Documents/GitHub/2026-Robot-Code/src/main/java/frc/robot/subsystems/vision/VisionIOLimelight.java#L74).
  * Cached `DoubleArrayPublisher robotOrientationPublisher`, `DoublePublisher imuModePublisher`, and a reusable `double[6]` orientation buffer in [`VisionIOLimelight`](file:///c:/Users/sleepy/Documents/GitHub/2026-Robot-Code/src/main/java/frc/robot/subsystems/vision/VisionIOLimelight.java#L37-L39).
  * Added a fast path in [`VisionIOLimelight.updateInputs()`](file:///c:/Users/sleepy/Documents/GitHub/2026-Robot-Code/src/main/java/frc/robot/subsystems/vision/VisionIOLimelight.java#L100-L108) that reuses static empty arrays (`EMPTY_POSE_OBSERVATIONS`, `EMPTY_TAG_IDS`, `EMPTY_TAG_POSES`) without allocating collections when no new frames arrived on the NT queue.
  * Updated [`Vision.periodic()`](file:///c:/Users/sleepy/Documents/GitHub/2026-Robot-Code/src/main/java/frc/robot/subsystems/vision/Vision.java#L149-L179) to only publish `imuMode` when `imuMode != lastImuMode` or when a camera reconnects (`sawReconnectThisCycle`), and gated `ENABLE_DETAILED_POSE_LOGGING` on `Constants.currentMode == Constants.Mode.REPLAY || Constants.VERBOSE_LOGGING_ENABLED`.

---

### 6. Swerve Odometry Sample Bounds Safety, Observation Array Isolation, and Drive Acceleration Guards
* **Files Modified**:
  * [`src/main/java/frc/robot/subsystems/swerve/SwerveDrive.java`](file:///c:/Users/sleepy/Documents/GitHub/2026-Robot-Code/src/main/java/frc/robot/subsystems/swerve/SwerveDrive.java)
  * [`src/main/java/frc/robot/subsystems/swerve/module/ModuleIOTalonFX.java`](file:///c:/Users/sleepy/Documents/GitHub/2026-Robot-Code/src/main/java/frc/robot/subsystems/swerve/module/ModuleIOTalonFX.java)
  * [`src/test/java/frc/robot/subsystems/swerve/SwerveDriveRangedRotationTest.java`](file:///c:/Users/sleepy/Documents/GitHub/2026-Robot-Code/src/test/java/frc/robot/subsystems/swerve/SwerveDriveRangedRotationTest.java)
  * [`src/test/java/frc/robot/subsystems/swerve/module/ModuleIOTalonFXTest.java`](file:///c:/Users/sleepy/Documents/GitHub/2026-Robot-Code/src/test/java/frc/robot/subsystems/swerve/module/ModuleIOTalonFXTest.java)
* **Problem Identified**:
  * `SwerveDrive.periodic()` iterated over `moduleInputs[0].odometryTimestampsSeconds.length` without bounding against the queue lengths of `moduleInputs[1..3]`, risking `ArrayIndexOutOfBoundsException` if the `250 Hz` odometry thread added a sample between per-module `updateInputs()` lock acquisitions.
  * `SwerveDrive.periodic()` passed the mutable `filteredOdometryModulePositions` and `filteredOdometryModuleStates` arrays directly into `OdometryObservation`, where `RobotState` stored the array reference as `lastWheelPositions` across iterations.
  * `ModuleIOTalonFX.setState()` divided `(state.speedMetersPerSecond - lastSpeedMetersPerSecond) / (currentTime - lastTime)` without guarding against `dt <= 1e-4`, large `dt` gaps on mode transitions, or non-finite values, and `setSteerTorqueCurrentFOC()` did not clear `.withAcceleration(0.0)` on the shared `steerPositionControl` request.
* **Changes Made**:
  * Added [`SwerveDrive.getSafeOdometrySampleCount()`](file:///c:/Users/sleepy/Documents/GitHub/2026-Robot-Code/src/main/java/frc/robot/subsystems/swerve/SwerveDrive.java#L565-L582) and cloned `filteredOdometryModulePositions` and `filteredOdometryModuleStates` when constructing each `OdometryObservation`.
  * Added [`ModuleIOTalonFX.calculateDriveAccelerationMetersPerSec2()`](file:///c:/Users/sleepy/Documents/GitHub/2026-Robot-Code/src/main/java/frc/robot/subsystems/swerve/module/ModuleIOTalonFX.java#L316-L328) and cleared `.withAcceleration(0.0)` in [`ModuleIOTalonFX.setSteerTorqueCurrentFOC()`](file:///c:/Users/sleepy/Documents/GitHub/2026-Robot-Code/src/main/java/frc/robot/subsystems/swerve/module/ModuleIOTalonFX.java#L371).

---

### 7. Silence Unplugged Joystick Console Spam & Fix Intake Bump Degree Conversion
* **Files Modified**:
  * [`src/main/java/frc/robot/Robot.java`](file:///c:/Users/sleepy/Documents/GitHub/2026-Robot-Code/src/main/java/frc/robot/Robot.java)
  * [`src/main/java/frc/robot/RobotContainer.java`](file:///c:/Users/sleepy/Documents/GitHub/2026-Robot-Code/src/main/java/frc/robot/RobotContainer.java)
* **Problem Identified in Logs & Codebase**:
  * Optional test/operator controller bindings on port 3 generated `Warning at edu.wpi.first.wpilibj.DriverStation.reportJoystickUnpluggedWarning(DriverStation.java:1345): Joystick Button 5 on port 3 not available` **108 times** during Houston Championship matches (**9,532 times** across all 240 logs), adding DriverStation console and file I/O overhead during matches.
  * In `RobotContainer.buildIntakeBumpCommand()`, `intake.getPivotAngleRotations() / 360 > 10` mistakenly divided rotations by 360 to compare against 10 degrees.
* **Changes Made**:
  * Added `DriverStation.silenceJoystickConnectionWarning(true);` in [`Robot.robotInit()`](file:///c:/Users/sleepy/Documents/GitHub/2026-Robot-Code/src/main/java/frc/robot/Robot.java#L61).
  * Fixed degree conversion in [`RobotContainer.buildIntakeBumpCommand()`](file:///c:/Users/sleepy/Documents/GitHub/2026-Robot-Code/src/main/java/frc/robot/RobotContainer.java#L242) to `intake.getPivotAngleRotations() * 360.0 > 10.0`.

---

## Verification

All code changes and unit tests were compiled and executed using the WPILib 2026 JDK:
```powershell
.\gradlew build '-Dorg.gradle.java.home=C:\Users\Public\wpilib\2026\jdk'
```
* **Compilation (`:compileJava`, `:compileTestJava`)**: Passed with 0 errors and 0 warnings.
* **Unit Test Suite (`:test`)**: Passed 100% (including all regression tests in [`RobotStateTest`](file:///c:/Users/sleepy/Documents/GitHub/2026-Robot-Code/src/test/java/frc/robot/RobotStateTest.java), [`IntakeTest`](file:///c:/Users/sleepy/Documents/GitHub/2026-Robot-Code/src/test/java/frc/robot/subsystems/intake/IntakeTest.java), [`ModuleIOTalonFXTest`](file:///c:/Users/sleepy/Documents/GitHub/2026-Robot-Code/src/test/java/frc/robot/subsystems/swerve/module/ModuleIOTalonFXTest.java), [`PhoenixUtilTest`](file:///c:/Users/sleepy/Documents/GitHub/2026-Robot-Code/src/test/java/frc/robot/lib/util/PhoenixUtilTest.java), and [`SwerveDriveRangedRotationTest`](file:///c:/Users/sleepy/Documents/GitHub/2026-Robot-Code/src/test/java/frc/robot/subsystems/swerve/SwerveDriveRangedRotationTest.java)).

