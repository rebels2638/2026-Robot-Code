package frc.robot.constants.robotState;

public class RobotStateConfigProto extends RobotStateConfigBase {
    private static RobotStateConfigProto instance;
    public static RobotStateConfigProto getInstance() {
        if (instance == null) {
            instance = new RobotStateConfigProto();
        }

        return instance;
    }
    
    private RobotStateConfigProto() {}

    @Override
    public double getVisionTranslationDevBase() {
        return 0.02;
    }

    @Override
    public double getVisionRotationDevBase() {
        return 0.02;
    }

    @Override
    public double getOdomTranslationDevBase() {
        return 0.005;
    }

    @Override
    public double getOdomRotationDevBase() {
        return 0.005;
    }
}
