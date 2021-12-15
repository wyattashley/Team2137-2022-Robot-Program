package frc.robot.library.hardware.swerve;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.robot.library.Constants;
import frc.robot.library.Distance2d;
import frc.robot.library.Speed2d;

public interface SwerveModule {

    default void setSwerveVelocityModuleState(SwerveModuleState swerveModuleState) {
        setModuleAngle(swerveModuleState.getRotation2d());
        setVelocityDriveSpeed(swerveModuleState.getSpeed2d());
    }

    default void setSwerveDistanceModuleState(SwerveModuleState swerveModuleState) {
        setModuleAngle(swerveModuleState.getRotation2d());
        setDriveDistanceTarget(swerveModuleState.getDistance2d());
    }

    default void setSwerveRawModuleState(SwerveModuleState swerveModuleState) {
        setModuleAngle(swerveModuleState.getRotation2d());
        setRawDriveSpeed(swerveModuleState.getRawPowerValue());
    }

    default void setSwerveModuleState(SwerveModuleState swerveModuleState) {
        setModuleAngle(swerveModuleState.getRotation2d());
        switch(swerveModuleState.getControlType()) {
            case VELOCITY:
                setVelocityDriveSpeed(swerveModuleState.getSpeed2d());
                break;
            case DISTANCE:
                setDriveDistanceTarget(swerveModuleState.getDistance2d());
                break;
            case RAW:
                setRawDriveSpeed(swerveModuleState.getRawPowerValue());
                break;
        }
    }

    void setModuleAngle(Rotation2d angle);
    Rotation2d getModuleAngle();

    void setRawDriveSpeed(double speed);
    void setVelocityDriveSpeed(Speed2d speed);

    Speed2d getDriveVelocity();
    Speed2d getDriveVelocityGoal();

    void setDriveDistanceTarget(Distance2d distance2d);
    Distance2d getDriveDistanceTarget();
    Distance2d getCurrentDrivePosition();

    void setDriveCoastMode(boolean brake);

    void configDrivetrainControlType(Constants.DriveControlType control);
}
