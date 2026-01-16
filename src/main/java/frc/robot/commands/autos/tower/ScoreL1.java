// package frc.robot.commands.autos.tower;

// import java.util.Optional;

// import org.littletonrobotics.junction.Logger;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.RobotState;
// import frc.robot.constants.MechAElementConstants;
// import frc.robot.constants.vision.VisionConstants;
// import frc.robot.lib.BLine.Path;
// import frc.robot.lib.util.LimelightHelpers;
// import frc.robot.subsystems.Superstructure;
// import frc.robot.subsystems.swerve.SwerveDrive;
// import frc.robot.subsystems.swerve.SwerveDrive.DesiredSystemState;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;

// public class ScoreL1 extends Command {
//     private final RobotState robotState;
//     private final Superstructure superstructure;
//     private final SwerveDrive swerveDrive;

//     private enum State {
//         WAITING_FOR_SHOT,
//         MOVING_BACK,
//         NAVIGATING_TO_TOWER,
//         FINISHED
//     }

//     private Optional<Alliance> alliance = Optional.empty();
//     private State currentState = State.WAITING_FOR_SHOT;
//     private Path backwardPath;
//     private double stateStartTime;
//     private Translation2d towerPosition;
//     private Pose2d backwardTargetPose;
//     private boolean moveBackStarted;
//     private boolean navigateStarted;

//     public ScoreL1() {
//         robotState = RobotState.getInstance();
//         superstructure = Superstructure.getInstance();
//         swerveDrive = SwerveDrive.getInstance();
//     }

//     @Override
//     public void initialize() {
//         alliance = DriverStation.getAlliance();

//         stateStartTime = Timer.getFPGATimestamp();
//         backwardPath = null;
//         towerPosition = null;
//         moveBackStarted = false;
//         navigateStarted = false;

//         Logger.recordOutput("ScoreL1/Initialize", "Command initialized");
//         Logger.recordOutput("ScoreL1/Alliance", alliance.isPresent() ? alliance.get().toString() : "None");
//         Logger.recordOutput("ScoreL1/SuperstructureStateAtInit", superstructure.getCurrentState().toString());
//         Logger.recordOutput("ScoreL1/SwerveStateAtInit", swerveDrive.getCurrentSystemState().toString());

//         if (superstructure.getCurrentState() == Superstructure.CurrentState.SHOOTING) {
//             currentState = State.WAITING_FOR_SHOT;
//             Logger.recordOutput("ScoreL1/InitialState", "WAITING_FOR_SHOT");
//         } else {
//             currentState = State.MOVING_BACK;
//             Logger.recordOutput("ScoreL1/InitialState", "MOVING_BACK");
//             transitionToMovingBack();
//         }

//         Logger.recordOutput("ScoreL1/State", currentState.toString());
//     }

//     private boolean isRedAlliance() {
//         return alliance.isPresent() && alliance.get() == Alliance.Red;
//     }

//     private Translation2d applyAllianceTranslation(Translation2d blueTranslation) {
//         if (isRedAlliance()) {
//             return new Translation2d(
//                     MechAElementConstants.fieldLength - blueTranslation.getX(),
//                     blueTranslation.getY());
//         }

//         return blueTranslation;
//     }

//     private boolean isWithinAllianceXZone(double xMeters) {
//         if (isRedAlliance()) {
//             return xMeters >= MechAElementConstants.AllianceBounds.blueZoneLineX;
//         } else {
//             return xMeters <= MechAElementConstants.AllianceBounds.redZoneLineX;
//         }
//     }

//     private boolean isWithinHubYBounds(double yMeters) {
//         double hubCenterY = MechAElementConstants.fieldWidth / 2.0;
//         return Math.abs(yMeters - hubCenterY) <= 1.5;
//     }

//     private boolean ensureWithinBounds(double xMeters, double yMeters, String context) {
//         boolean inAllianceXZone = isWithinAllianceXZone(xMeters);
//         boolean inHubYBounds = isWithinHubYBounds(yMeters);
//         Logger.recordOutput("ScoreL1/InAllianceXZone", inAllianceXZone);
//         Logger.recordOutput("ScoreL1/InHubYBounds", inHubYBounds);
//         if (!inAllianceXZone || !inHubYBounds) {
//             Logger.recordOutput("ScoreL1/OutOfBoundsContext", context);
//             Logger.recordOutput("ScoreL1/OutOfBoundsX", xMeters);
//             Logger.recordOutput("ScoreL1/OutOfBoundsY", yMeters);
//             currentState = State.FINISHED;
//             swerveDrive.setDesiredSystemState(DesiredSystemState.TELEOP);
//             return false;
//         }
//         return true;
//     }

//     private Translation2d getClosestTowerPosition(Pose2d currentPose) {
//         Translation2d[] towers = new Translation2d[] {
//             applyAllianceTranslation(MechAElementConstants.Tower.leftTower.getTranslation()),
//             applyAllianceTranslation(MechAElementConstants.Tower.middleTower.getTranslation()),
//             applyAllianceTranslation(MechAElementConstants.Tower.rightTower.getTranslation())
//         };

//         Translation2d closest = towers[0];
//         double closestDistance = currentPose.getTranslation().getDistance(closest);
//         for (int i = 1; i < towers.length; i++) {
//             double distance = currentPose.getTranslation().getDistance(towers[i]);
//             if (distance < closestDistance) {
//                 closest = towers[i];
//                 closestDistance = distance;
//             }
//         }

//         Logger.recordOutput("ScoreL1/ClosestTowerDistance", closestDistance);
//         return closest;
//     }

//     @Override
//     public void execute() {
//         Logger.recordOutput("ScoreL1/State", currentState.toString());
//         Logger.recordOutput("ScoreL1/ExecuteCalled", Timer.getFPGATimestamp());
//         Logger.recordOutput("ScoreL1/BackwardPathExists", backwardPath != null);

//         switch (currentState) {
//             case WAITING_FOR_SHOT:
//                 handleWaitingForShot();
//                 break;

//             case MOVING_BACK:
//                 handleMovingBack();
//                 break;

//             case NAVIGATING_TO_TOWER:
//                 handleNavigatingToTower();
//                 break;

//             case FINISHED:
//                 break;
//         }
//     }

//     private void handleWaitingForShot() {
//         Logger.recordOutput("ScoreL1/SuperstructureState", superstructure.getCurrentState().toString());

//         if (superstructure.getCurrentState() != Superstructure.CurrentState.SHOOTING) {
//             currentState = State.MOVING_BACK;
//             transitionToMovingBack();
//         }
//     }

//     private void transitionToMovingBack() {
//         Pose2d currentPose = robotState.getEstimatedPose();
//         Rotation2d currentHeading = currentPose.getRotation();

//         Logger.recordOutput("ScoreL1/CurrentPose", currentPose);
//         Logger.recordOutput("ScoreL1/CurrentPoseX", currentPose.getX());
//         Logger.recordOutput("ScoreL1/CurrentPoseY", currentPose.getY());
//         Logger.recordOutput("ScoreL1/CurrentPoseHeading", currentHeading.getDegrees());

//         if (!ensureWithinBounds(currentPose.getX(), currentPose.getY(), "MoveBackStart")) {
//             return;
//         }

//         double backwardDistance = isRedAlliance() ? 0.9 : -0.9;

//         backwardTargetPose = new Pose2d(
//                 currentPose.getTranslation().plus(new Translation2d(backwardDistance, 0.0)),
//                 new Rotation2d(Math.PI));

//         Logger.recordOutput("ScoreL1/BackwardTargetX", backwardTargetPose.getX());
//         Logger.recordOutput("ScoreL1/BackwardTargetY", backwardTargetPose.getY());
//         Logger.recordOutput("ScoreL1/BackwardDistance", backwardDistance);

//         if (!ensureWithinBounds(backwardTargetPose.getX(), backwardTargetPose.getY(), "MoveBackTarget")) {
//             return;
//         }

//         Path.PathConstraints constraints = new Path.PathConstraints()
//                 .setMaxVelocityMetersPerSec(1.2);

//         backwardPath = new Path(
//                 constraints,
//                 new Path.Waypoint(
//                         currentPose.getTranslation(),
//                         0.85,
//                         currentHeading),
//                 new Path.Waypoint(
//                         backwardTargetPose.getTranslation(),
//                         0.2,
//                         backwardTargetPose.getRotation()));

//         Logger.recordOutput("ScoreL1/BackwardPathNotNull", backwardPath != null);
//         Logger.recordOutput("ScoreL1/ShouldPoseReset", false);

//         swerveDrive.setPathSupplier(() -> backwardPath, () -> false, () -> false);

//         swerveDrive.setDesiredSystemState(SwerveDrive.DesiredSystemState.FOLLOW_PATH);

//         Logger.recordOutput("ScoreL1/SwerveDesiredState", "FOLLOW_PATH");
//         Logger.recordOutput("ScoreL1/SwerveDesiredStateEnum", swerveDrive.getDesiredSystemState().toString());

//         stateStartTime = Timer.getFPGATimestamp();
//         Logger.recordOutput("ScoreL1/BackwardTarget", backwardTargetPose);
//         Logger.recordOutput("ScoreL1/StateStartTime", stateStartTime);
//     }

//     private void handleMovingBack() {
//         if (!moveBackStarted
//                 && swerveDrive.getCurrentSystemState() == SwerveDrive.CurrentSystemState.FOLLOW_PATH) {
//             moveBackStarted = true;
//             Logger.recordOutput("ScoreL1/MoveBackStarted", true);
//         }

//         boolean idle = swerveDrive.getCurrentSystemState() == SwerveDrive.CurrentSystemState.IDLE;
//         boolean teleop = swerveDrive.getDesiredSystemState() == SwerveDrive.DesiredSystemState.TELEOP;

//         Logger.recordOutput("ScoreL1/MoveBackStartedFlag", moveBackStarted);
//         Logger.recordOutput("ScoreL1/SwerveCurrentState", swerveDrive.getCurrentSystemState().toString());

//         if (moveBackStarted && (idle || teleop)) {
//             Logger.recordOutput("ScoreL1/TransitionReason", "Idle");
//             currentState = State.NAVIGATING_TO_TOWER;
//             transitionToNavigatingToTower();
//         }
//     }

//     // TODO: make a proper mechA constants file
//     private void transitionToNavigatingToTower() {
//         Pose2d currentPose = robotState.getEstimatedPose();
//         if (!ensureWithinBounds(currentPose.getX(), currentPose.getY(), "NavigateStart")) {
//             return;
//         }

//         towerPosition = getClosestTowerPosition(currentPose);
//         Logger.recordOutput("ScoreL1/TowerSelection", "Closest tower selected");
//         Logger.recordOutput("ScoreL1/TowerPosition", new Pose2d(towerPosition, new Rotation2d()));

//         if (!ensureWithinBounds(towerPosition.getX(), towerPosition.getY(), "NavigateTarget")) {
//             return;
//         }

//         Rotation2d headingToTower = new Rotation2d(Math.PI);

//         Translation2d approachOffset =
//             new Translation2d(isRedAlliance() ? -0.2 : 0.2, 0)
//             .rotateBy(headingToTower);
//         Translation2d approachPosition = towerPosition.minus(approachOffset);

//         Path.PathConstraints towerConstraints = new Path.PathConstraints()
//                 .setMaxVelocityMetersPerSec(
//                     new Path.RangedConstraint(2.5, 0, 1),
//                     new Path.RangedConstraint(1.5, 1, 2)
//                 );

//         Path towerPath = new Path(
//                 towerConstraints,
//                 new Path.Waypoint(
//                         currentPose.getTranslation(),
//                         0.3,
//                         currentPose.getRotation()),
//                 new Path.Waypoint(
//                         approachPosition,
//                         0.2,
//                         headingToTower),
//                 new Path.Waypoint(
//                         towerPosition,
//                         0.1,
//                         headingToTower));

//         swerveDrive.setPathSupplier(() -> towerPath, () -> false, () -> false);
//         swerveDrive.setDesiredSystemState(SwerveDrive.DesiredSystemState.FOLLOW_PATH);

//         currentState = State.NAVIGATING_TO_TOWER;
//         stateStartTime = Timer.getFPGATimestamp();

//         Logger.recordOutput("ScoreL1/ApproachPosition", new Pose2d(approachPosition, headingToTower));
//         Logger.recordOutput("ScoreL1/FinalPosition", new Pose2d(towerPosition, headingToTower));
//     }

//     private void handleNavigatingToTower() {
//         if (!navigateStarted
//                 && swerveDrive.getCurrentSystemState() == SwerveDrive.CurrentSystemState.FOLLOW_PATH) {
//             navigateStarted = true;
//             Logger.recordOutput("ScoreL1/NavigateStarted", true);
//         }

//         boolean idle = swerveDrive.getCurrentSystemState() == SwerveDrive.CurrentSystemState.IDLE;

//         if (navigateStarted && idle) {
//             Logger.recordOutput("ScoreL1/TransitionReason", "Idle");
//             currentState = State.FINISHED;
//         }

//         Logger.recordOutput("ScoreL1/NavigateStartedFlag", navigateStarted);
//         Logger.recordOutput("ScoreL1/NavigateIdleStatus", idle);
//         Logger.recordOutput("ScoreL1/NavigateSwerveState", swerveDrive.getCurrentSystemState().toString());
//     }

//     private Translation2d detectTowerPosition() {
//         int towerId = isRedAlliance()
//                 ? VisionConstants.TagIDs.RED_TOWER_APRIL_TAG_ID.getId()
//                 : VisionConstants.TagIDs.BLUE_TOWER_APRIL_TAG_ID.getId();

//         LimelightHelpers.RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(VisionConstants.camera0Name);

//         if (fiducials.length == 0) {
//             return null;
//         }

//         for (LimelightHelpers.RawFiducial fiducial : fiducials) {
//             var tagPose = VisionConstants.aprilTagLayout.getTagPose(fiducial.id);

//             if (tagPose.isPresent() && fiducial.id == towerId) {
//                 return tagPose.get().getTranslation().toTranslation2d().plus(
//                         MechAElementConstants.Tower.middleTower.getTranslation());
//             }
//         }

//         return null;
//     }

//     @Override
//     public void end(boolean interrupted) {
//         Logger.recordOutput("ScoreL1/EndInterrupted", interrupted);
//         Logger.recordOutput("ScoreL1/EndState", currentState.toString());
//         Logger.recordOutput("ScoreL1/EndSwerveState", swerveDrive.getCurrentSystemState().toString());
//         Logger.recordOutput("ScoreL1/EndSwerveDesiredState", swerveDrive.getDesiredSystemState().toString());

//         swerveDrive.setDesiredSystemState(DesiredSystemState.TELEOP);
//     }

//     @Override
//     public boolean isFinished() {
//         return currentState == State.FINISHED;
//     }
// }
