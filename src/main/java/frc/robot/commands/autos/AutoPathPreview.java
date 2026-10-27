package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.constants.Constants;
import frc.robot.lib.BLine.BLineField;
import frc.robot.lib.BLine.Path;
import java.util.List;
import java.util.Objects;
import java.util.Optional;

/**
 * Elastic "Auto Path" field: the robot pose plus the selected auto's BLine path, flipped for the
 * current alliance the same way the auto will flip it. Autos without a preview path show none.
 */
public final class AutoPathPreview extends SubsystemBase {
    private static final String PATH_OBJECT_NAME = "SelectedAutoTrajectory";

    private final SendableChooser<Command> autoChooser;
    private final Field2d field = new Field2d();
    private Command lastAuto = null;
    private Boolean lastFlipped = null;

    public AutoPathPreview(SendableChooser<Command> autoChooser) {
        this.autoChooser = autoChooser;
        SmartDashboard.putData("Auto Path", field);
    }

    @Override
    public void periodic() {
        field.setRobotPose(RobotState.getInstance().getEstimatedPose());

        Command selectedAuto = autoChooser.getSelected();
        boolean flipped = Constants.shouldFlipPath();
        if (selectedAuto == lastAuto && Objects.equals(lastFlipped, flipped)) {
            return;
        }
        lastAuto = selectedAuto;
        lastFlipped = flipped;

        Optional<Path> previewPath = selectedAuto == null ? Optional.empty() : Autos.getPreviewPath(selectedAuto);
        if (previewPath.isEmpty()) {
            field.getObject(PATH_OBJECT_NAME).setPoses(List.of());
            return;
        }
        Path path = previewPath.get();
        if (flipped) {
            path.flip();
        }
        BLineField.drawPath(field, PATH_OBJECT_NAME, path);
    }
}
