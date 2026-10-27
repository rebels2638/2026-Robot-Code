# Optimum demo: shop test checklist

Branch `optimum-demo` (COMP mode). Deploy from the main checkout.

**Demo vs full field:** the *Demo mode (after restart)* toggle (Demo Setup and Autonomous tabs,
NT `/Preferences/DemoMode`, defaults on) is read once at startup. Flip it, then **Restart Robot Code**
from the Driver Station; *Demo mode active* shows what is running. Off = comp bindings, full speed,
vision pose correction and the BLine auto chooser.

**Partial field:** *Ignore zone/line-of-sight checks* (Autonomous tab, `/Preferences/IgnoreFieldChecks`,
defaults off) applies immediately: the hub can be shot from any zone and B passes ignore blockers.

**On-the-fly BLine autos:** draw `optimum-demo-path` in BLine Web Beta (project `src/main/deploy/autos`),
redeploy, then pick *optimum-demo-path* (starts from the current pose) or *optimum-demo-path-with-reset*
(resets pose to the path start) in the Auto Chooser. Both flip for red, never mirror, and work in demo mode.
The *Auto path* field on the Autonomous tab shows the selected path as it will run. The path must start
and end on a point; an empty path only reports a warning. Speed is the comp default (4.5 m/s) unless
you set a max velocity on the path.

**Before any restart or redeploy:** the hood (70 deg), turret (180 deg), intake pivot (stowed) and climber
(down) encoders are reset to those positions when robot code starts. Re-enable TeleOp (goes HOME), press
RB to stow the intake, disable, then restart. Otherwise the mechanisms come up mis-zeroed.

## Setup
1. `./gradlew deploy`, then set the Driver Station team station to **Blue** and select **TeleOperated**.
   In demo mode only the two demo path autos run; comp autos do nothing on purpose.
2. Elastic: load the layout from the robot (`elastic-layout.json` in deploy). Tabs: **Demo**, **Demo Setup**, **Autonomous**.
3. Print AprilTag **36h11 ID 26**, black square exactly **6.5 in (165.1 mm)**, mounted flat on stiff board.
4. Controller on port 3. Map: sticks drive/turn, RT fire, LT/LB/RB intake, X/Y reverse,
   **A = Manual/Tag**, **B = save tag target**. Zero gyro / clear target are dashboard buttons.

## Tests, in order
1. **Drive.** Robot facing away from you, press *Zero gyro*. Forward stick drives away. Speed is ~0.25x of comp.
   Check the *Drive speed* slider can't go past 0.5. *Robot relative* toggle works.
2. **Manual fire.** Defaults: 40 deg hood, 22 RPS, turret 180. Hold RT: status shows SPINNING UP then FIRING
   (READY / TIMEOUT / FORCED). Confirm which way turret 180 actually points. Try a couple of RPS/hood values.
3. **Tag geometry (no shooting).** Tag 2 m straight in front of the robot center:
   *Tag distance* ~2.0, *bearing* ~0, *height* = real tag height. Move it left: bearing goes positive.
   Tilt the tag ~30 deg: distance/height barely change. Note which *Camera* sees it.
4. **Camera coverage.** Walk the tag around at floor, ~0.6 m, ~1.2 m, overhead, at 2 m and 4 m.
   Note where *Tag locked* drops out.
5. **Tag mode.** Press A until *Fire mode* = TAG. Turret should follow the tag with RT released.
   Hold RT: READY then FIRING. Read *Status* if it won't fire (NO TARGET / OUT OF RANGE / CEILING /
   SHOOTER NOT READY). *Ceiling m* defaults to 3.66 (12 ft); change it if the room differs (0 turns that limit off).
   *Speed at tag* is shown for information only; it never blocks a shot.
6. **Saved target.** Show tag, press B (one rumble = saved, three = no tag). Remove the tag: still aims there.
   *Zero gyro* must not move the saved spot. *Clear saved* returns to live tag.
7. **Calibration** (Demo Setup tab), tag at chest height, then overhead:
   turn on *Calibration mode* (fields pre-fill), shoot, adjust *Cal RPS* (and hood if needed) until it hits,
   *Save point*. Do 2, 3, 4 m at each height (6 points).
   Then copy the file into the repo and commit it:
   `scp lvuser@10.26.38.2:/home/lvuser/demo-calibration.json src/main/deploy/`

## Before the trip
- Build once with internet (BLine-Lib v0.9.2 must be cached), USB stick in the roboRIO for logs.
- Logs: everything new is under `Demo/` in the .wpilog (AdvantageScope), including 3D poses under `Demo/Poses/`.
