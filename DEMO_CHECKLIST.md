# Optimum demo: shop test checklist

Branch `optimum-demo` (COMP mode, `Constants.DEMO_MODE = true`). Deploy from the main checkout.

## Setup
1. `./gradlew deploy`, then set the Driver Station team station to **Blue** and select **TeleOperated**.
   Autonomous does nothing in demo mode on purpose.
2. Elastic: load the layout from the robot (`elastic-layout.json` in deploy). Tabs: **Demo**, **Demo Setup**.
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
   Hold RT: READY then FIRING. Read *Status* if it won't fire (NO TARGET / OUT OF RANGE / TOO FAST AT TAG /
   CEILING / SHOOTER NOT READY). Set *Ceiling m* to the real ceiling. *Max speed at tag* default 5.5 m/s
   (~3.5 m at chest height); 0 turns it off.
6. **Saved target.** Show tag, press B (one rumble = saved, three = no tag). Remove the tag: still aims there.
   *Zero gyro* must not move the saved spot. *Clear saved* returns to live tag.
7. **Calibration** (Demo Setup tab), tag at chest height, then overhead:
   turn on *Calibration mode* (fields pre-fill), shoot, adjust *Cal RPS* (and hood if needed) until it hits,
   *Save point*. Do 2, 3, 4 m at each height (6 points).
   Then copy the file into the repo and commit it:
   `scp lvuser@10.26.38.2:/home/lvuser/demo-calibration.json src/main/deploy/`

## Before the trip
- Build once with internet (BLine-Lib v0.8.4 must be cached), USB stick in the roboRIO for logs.
- Logs: everything new is under `Demo/` in the .wpilog (AdvantageScope), including 3D poses under `Demo/Poses/`.
