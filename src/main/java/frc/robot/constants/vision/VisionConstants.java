package frc.robot.constants.vision;

import java.io.File;
import java.io.IOException;
import java.util.logging.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.Filesystem;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout;
  private static final Logger logger = Logger.getLogger(VisionConstants.class.getName());

  static {
    try {
      File layoutFile = new File(Filesystem.getDeployDirectory(), "custom_apriltag_field_layout.fmap");
      aprilTagLayout = new AprilTagFieldLayout(layoutFile.getAbsolutePath());
      logger.info("Loaded custom AprilTag field layout from deploy directory");
    } catch (IOException e) {
      // Fallback to default field layout if custom field fails to load
      aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
      logger.severe("AprilTagFieldLayout failed to load: " + e.getMessage());
      e.printStackTrace();
    }
  }

}