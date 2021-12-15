package frc.robot.library.hardware.swerve;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.robot.library.Constants;
import frc.robot.library.Distance2d;
import frc.robot.library.Speed2d;

public class SwerveModuleState {
    public enum SwerveModulePositions {
        LEFT_FRONT,
        LEFT_BACK,
        RIGHT_FRONT,
        RIGHT_BACK
    }

    private final Rotation2d rotation2d;

    private Speed2d speed2d;
    private Distance2d distance2d;
    private Double rawPowerValue;

    private final Constants.DriveControlType controlType;
    private final SwerveModulePositions position;

    public SwerveModuleState(Speed2d _speed, Rotation2d _rotation2d, SwerveModulePositions pos) {
        rotation2d = _rotation2d;
        speed2d = _speed;
        controlType = Constants.DriveControlType.VELOCITY;
        position = pos;
    }

    public SwerveModuleState(Distance2d _distance, Rotation2d _rotation2d, SwerveModulePositions pos) {
        rotation2d = _rotation2d;
        distance2d = _distance;
        controlType = Constants.DriveControlType.DISTANCE;
        position = pos;
    }

    public SwerveModuleState(double _rawPowerValue, Rotation2d _rotation2d, SwerveModulePositions pos) {
        rotation2d = _rotation2d;
        rawPowerValue = _rawPowerValue;
        controlType = Constants.DriveControlType.RAW;
        position = pos;
    }

    public Rotation2d getRotation2d() {
        return rotation2d;
    }

    public Speed2d getSpeed2d() {
        if(speed2d != null)
            return speed2d;
        else
            throw new IllegalStateException();
    }

    public Distance2d getDistance2d() {
        if (distance2d != null)
            return distance2d;
        else
            throw new IllegalStateException();
    }

    public double getRawPowerValue() {
        if(rawPowerValue != null)
            return rawPowerValue;
        else
            throw new IllegalStateException();
    }

    public Constants.DriveControlType getControlType() {
        return controlType;
    }

    public SwerveModulePositions getPosition() { return position; }
}
