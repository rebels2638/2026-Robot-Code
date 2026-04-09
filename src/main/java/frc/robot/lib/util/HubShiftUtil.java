// package frc.robot.lib.util;

// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.Timer;
// import org.littletonrobotics.junction.Logger;
// import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
// import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

// public class HubShiftUtil {
//     private static HubShiftUtil instance;

//     public static HubShiftUtil getInstance() {
//         if (instance == null) {
//             instance = new HubShiftUtil();
//         }
//         return instance;
//     }

//     public enum ShiftState {
//         TRANSITION,
//         SHIFT1,
//         SHIFT2,
//         SHIFT3,
//         SHIFT4,
//         ENDGAME,
//         AUTO,
//         DISABLED
//     }

//     public record ShiftInfo(ShiftState gameState, double elapsedTime, double remainingTime, boolean active) {}

//     private static final double AUTO_END_TIME = 20.0;
//     private static final double TELEOP_DURATION = 140.0;
//     private static final double TIME_RESET_THRESHOLD = 3.0;

//     private static final double[] shiftStartTimes = {0.0, 10.0, 35.0, 60.0, 85.0, 110.0};
//     private static final double[] shiftEndTimes = {10.0, 35.0, 60.0, 85.0, 110.0, 140.0};
//     private static final ShiftState[] shiftStates = {
//         ShiftState.TRANSITION, ShiftState.SHIFT1, ShiftState.SHIFT2,
//         ShiftState.SHIFT3, ShiftState.SHIFT4, ShiftState.ENDGAME
//     };

//     // activeSchedule: for the alliance that is active first (active in shifts 1 & 3)
//     // inactiveSchedule: for the alliance that is inactive first (inactive in shifts 1 & 3)
//     private static final boolean[] activeSchedule = {true, true, false, true, false, true};
//     private static final boolean[] inactiveSchedule = {true, false, true, false, true, true};

//     private final Timer shiftTimer = new Timer();
//     private double shiftTimerOffset = 0.0;

//     // Output NT values
//     private final LoggedNetworkNumber matchTimeSeconds =
//         new LoggedNetworkNumber("HubShift/matchTimeSeconds", -1);
//     private final LoggedNetworkNumber remainingShiftTimeSeconds =
//         new LoggedNetworkNumber("HubShift/remainingShiftTimeSeconds", -1);
//     private final LoggedNetworkBoolean shiftActive =
//         new LoggedNetworkBoolean("HubShift/shiftActive", false);
//     private final LoggedNetworkBoolean activeFirst =
//         new LoggedNetworkBoolean("HubShift/activeFirst", false);

//     // Manual override inputs
//     private final LoggedNetworkBoolean overrideEnabled =
//         new LoggedNetworkBoolean("HubShift/overrideEnabled", false);
//     private final LoggedNetworkBoolean overrideWeAreActiveFirst =
//         new LoggedNetworkBoolean("HubShift/overrideWeAreActiveFirst", false);

//     private HubShiftUtil() {}

//     /** Call at the start of auto to start the shift timer for auto tracking. */
//     public void initializeAuto() {
//         shiftTimerOffset = 0.0;
//         shiftTimer.restart();
//     }

//     /** Call at the start of teleop to reset the shift timer for shift tracking. */
//     public void initialize() {
//         shiftTimerOffset = 0.0;
//         shiftTimer.restart();
//     }

//     /**
//      * Determines which alliance is active first (active in shifts 1 and 3).
//      * Checks manual override first, then FMS game-specific message.
//      * Game message "R" means red goes inactive first → blue is active first.
//      * Game message "B" means blue goes inactive first → red is active first.
//      */
//     private Alliance getFirstActiveAlliance() {
//         Alliance ourAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);

//         // Check manual override
//         if (overrideEnabled.get()) {
//             if (overrideWeAreActiveFirst.get()) {
//                 return ourAlliance;
//             } else {
//                 return ourAlliance == Alliance.Blue ? Alliance.Red : Alliance.Blue;
//             }
//         }

//         // Check FMS game-specific message
//         String message = DriverStation.getGameSpecificMessage();
//         if (message != null && message.length() > 0) {
//             char character = message.charAt(0);
//             if (character == 'R') {
//                 return Alliance.Blue;
//             } else if (character == 'B') {
//                 return Alliance.Red;
//             }
//         }

//         // Default: opposite alliance (same as 6328)
//         return ourAlliance == Alliance.Blue ? Alliance.Red : Alliance.Blue;
//     }

//     private boolean[] getSchedule() {
//         Alliance firstActive = getFirstActiveAlliance();
//         Alliance ourAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);
//         return firstActive == ourAlliance ? activeSchedule : inactiveSchedule;
//     }

//     public ShiftInfo getShiftInfo() {
//         boolean[] schedule = getSchedule();

//         if (DriverStation.isAutonomousEnabled()) {
//             double currentTime = Math.max(0, shiftTimer.get() - shiftTimerOffset);
//             return new ShiftInfo(ShiftState.AUTO, currentTime, Math.max(0, AUTO_END_TIME - currentTime), true);
//         }

//         if (!DriverStation.isEnabled()) {
//             return new ShiftInfo(ShiftState.DISABLED, 0, -1, false);
//         }

//         // Teleop
//         double timerValue = shiftTimer.get();
//         double currentTime = timerValue - shiftTimerOffset;
//         double fieldTeleopTime = TELEOP_DURATION - DriverStation.getMatchTime();

//         // Sync with FMS time if drift exceeds threshold
//         if (Math.abs(fieldTeleopTime - currentTime) >= TIME_RESET_THRESHOLD
//                 && fieldTeleopTime <= 135
//                 && DriverStation.isFMSAttached()) {
//             shiftTimerOffset += currentTime - fieldTeleopTime;
//             currentTime = timerValue - shiftTimerOffset;
//         }

//         // Clamp to valid teleop range
//         currentTime = Math.max(0, Math.min(currentTime, TELEOP_DURATION));

//         // Find current shift index
//         int currentShiftIndex = -1;
//         for (int i = 0; i < shiftStartTimes.length; i++) {
//             if (currentTime >= shiftStartTimes[i] && currentTime < shiftEndTimes[i]) {
//                 currentShiftIndex = i;
//                 break;
//             }
//         }
//         if (currentShiftIndex < 0) {
//             // Past last shift boundary, assume endgame
//             currentShiftIndex = shiftStartTimes.length - 1;
//         }

//         double elapsedTime = currentTime - shiftStartTimes[currentShiftIndex];
//         double remainingTime = shiftEndTimes[currentShiftIndex] - currentTime;

//         // Combine adjacent shifts with same activity status
//         if (currentShiftIndex > 0
//                 && schedule[currentShiftIndex] == schedule[currentShiftIndex - 1]) {
//             elapsedTime = currentTime - shiftStartTimes[currentShiftIndex - 1];
//         }
//         if (currentShiftIndex < shiftEndTimes.length - 1
//                 && schedule[currentShiftIndex] == schedule[currentShiftIndex + 1]) {
//             remainingTime = shiftEndTimes[currentShiftIndex + 1] - currentTime;
//         }

//         boolean active = schedule[currentShiftIndex];
//         ShiftState gameState = shiftStates[currentShiftIndex];

//         return new ShiftInfo(gameState, Math.max(0, elapsedTime), Math.max(0, remainingTime), active);
//     }

//     /** Returns whether our alliance is the one that is active first (active in shifts 1 and 3). */
//     public boolean isOurAllianceActiveFirst() {
//         Alliance firstActive = getFirstActiveAlliance();
//         Alliance ourAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);
//         return firstActive == ourAlliance;
//     }

//     /** Call every robot periodic cycle to update dashboard NT values. */
//     public void periodic() {
//         double matchTime = DriverStation.getMatchTime();
//         matchTimeSeconds.set(matchTime);

//         boolean weAreActiveFirst = isOurAllianceActiveFirst();
//         activeFirst.set(weAreActiveFirst);

//         ShiftInfo info = getShiftInfo();
//         remainingShiftTimeSeconds.set(info.remainingTime());
//         shiftActive.set(info.active());
//         Logger.recordOutput("HubShift/gameState", info.gameState().name());
//     }
// }
