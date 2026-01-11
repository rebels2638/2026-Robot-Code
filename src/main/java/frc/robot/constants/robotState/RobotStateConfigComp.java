package frc.robot.constants.robotState;


public class RobotStateConfigComp extends RobotStateConfigBase {
    private static RobotStateConfigComp instance;
    public static RobotStateConfigComp getInstance() {
        if (instance == null) {
            instance = new RobotStateConfigComp();
        }

        return instance;
    }
    
    private RobotStateConfigComp() {}

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
