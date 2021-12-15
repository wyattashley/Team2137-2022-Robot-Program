package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.functions.io.FileLogger;
import frc.robot.functions.io.xmlreader.XMLSettingReader;
import frc.robot.library.*;
import frc.robot.library.hardware.DriveTrain;
import frc.robot.library.hardware.Gamepad;
import frc.robot.library.hardware.swerve.SwerveDrivetrain;
import frc.robot.library.hardware.swerve.SwerveModuleState;
import frc.robot.library.hardware.swerve.SwerveNEODriveModule;

public class Teleop implements OpMode {

    private FileLogger logger;
    private final int mintDebug = 0;

    private CANSubsystem mRobotSubsystem;
    private XMLSettingReader mSettingReader;
    private DriveTrain mDrivetrain;

    private Runnable mCurrentDrivetrainPeriodRunnable;

    private final Gamepad mDriverController = new Gamepad(0);
    private final Gamepad mOperatorController = new Gamepad(1);

    @Override
    public void init() {
        this.logger = new FileLogger(mintDebug, Constants.RobotState.TELEOP);

        this.mSettingReader = new XMLSettingReader("/home/lvuser/XML Files/", "Swerve-2021.xml", logger);
        this.mRobotSubsystem = this.mSettingReader.getRobotCANSubsystem();

        this.mSettingReader.getRobotCANSubsystem().printHardwareMap(logger);

        switch(mRobotSubsystem.getSubsystemByType("DriveTrain").get(0).getName()) {
            case "Swerve Falcon":
            case "Swerve NEO":
                logger.writeEvent(0, mRobotSubsystem.getSubsystemByType("DriveTrain").get(0).getName());
                mCurrentDrivetrainPeriodRunnable = this::SwerveDrivetrainPeriodic;
                this.mDrivetrain = new SwerveDrivetrain(mRobotSubsystem.getSubsystemByType("DriveTrain").get(0), mSettingReader, logger);
                break;
        }

        SwerveDrivetrain drive = (SwerveDrivetrain) (this.mDrivetrain);
        drive.leftFrontModule.setModuleAngle(Rotation2d.fromDegrees(90));
    }

    @Override
    public void periodic() {
        SwerveDrivetrain drive = (SwerveDrivetrain) (this.mDrivetrain);
        SmartDashboard.putNumber("Left Front Module Actual Angle", drive.leftFrontModule.getModuleAngle().getDegrees());
//        mCurrentDrivetrainPeriodRunnable.run();
    }

    @Override
    public void end() {
        logger.writeEvent(0, FileLogger.EventType.Status, "TELEOP Ending");
        logger.close();
    }

    private void SwerveDrivetrainPeriodic() {
        logger.setTag("SwerveDrivetrainPeriodic()");
        double xMag = mDriverController.getX(GenericHID.Hand.kLeft, 0.1);
        double yMag = mDriverController.getY(GenericHID.Hand.kLeft, 0.1);
        double rMag = mDriverController.getX(GenericHID.Hand.kRight, 0.1); //TODO must fix TrackWidth
        SwerveModuleState[] state = ((SwerveDrivetrain) mDrivetrain).calculateSwerveMotorSpeeds(xMag, yMag, rMag, 1, 1, Constants.DriveControlType.RAW);

        SmartDashboard.putNumber(logger.getTag() + "-RightBackPower", state[0].getRawPowerValue());
        SmartDashboard.putNumber(logger.getTag() + "-RightBackAngle", state[0].getRotation2d().getDegrees());
        SmartDashboard.putNumber(logger.getTag() + "-LeftBackPower", state[1].getRawPowerValue());
        SmartDashboard.putNumber(logger.getTag() + "-LeftBackAngle", state[1].getRotation2d().getDegrees());
        SmartDashboard.putNumber(logger.getTag() + "-RightFrontPower", state[2].getRawPowerValue());
        SmartDashboard.putNumber(logger.getTag() + "-RightFrontAngle", state[2].getRotation2d().getDegrees());
        SmartDashboard.putNumber(logger.getTag() + "-LeftFrontPower", state[3].getRawPowerValue());
        SmartDashboard.putNumber(logger.getTag() + "-LeftFrontAngle", state[3].getRotation2d().getDegrees());

        ((SwerveDrivetrain) mDrivetrain).setSwerveModuleStates(state);
    }
}
