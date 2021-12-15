package frc.robot;

import frc.robot.functions.io.FileLogger;
import frc.robot.library.Constants;
import frc.robot.library.OpMode;

public class Disabled implements OpMode {

    private FileLogger logger;
    private final int mintDebug = 0;

    @Override
    public void init() {
//        this.logger = new FileLogger(mintDebug, Constants.RobotState.DISABLED);
    }

    @Override
    public void periodic() {

    }

    @Override
    public void end() {
//        logger.writeEvent(0, FileLogger.EventType.Status, "DISABLED Ending");
//        logger.close();
    }
}
