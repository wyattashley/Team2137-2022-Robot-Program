package frc.robot.library.hardware;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.library.Constants;
import frc.robot.library.Distance2d;
import org.ejml.simple.SimpleMatrix;

public interface DriveTrain {

    default void setSpeed(double speed) {
        setLeftSpeed(speed);
        setRightSpeed(speed);
    }

    void setLeftSpeed(double speed);
    void setRightSpeed(double speed);

    Rotation2d getAngle();

    void setAngleOffset(Rotation2d offset);

    void configDrivetrainControlType(Constants.DriveControlType control);
}
