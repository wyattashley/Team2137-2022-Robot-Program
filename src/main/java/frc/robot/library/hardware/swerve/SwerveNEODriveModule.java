package frc.robot.library.hardware.swerve;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.functions.io.xmlreader.Encoder;
import frc.robot.functions.io.xmlreader.Motor;
import frc.robot.functions.io.xmlreader.XMLSettingReader;
import frc.robot.library.*;
import frc.robot.library.Constants.DriveControlType;

@SuppressWarnings("All")
public class SwerveNEODriveModule extends SubsystemBase implements SwerveModule {

    private static final int intDriveVelocityPIDSlotID = 0;
    private static final int intDriveDistancePIDSlotID = 1;
    public CANSparkMax mDriveMotor;
    public CANSparkMax mTurnMotor;
    private CANEncoder mTurnMotorEncoder;
    private CANEncoder mDriveMotorEncoder;
    private CANCoder mTurnEncoder;
    private CANPIDController mDrivePIDController;
    private CANPIDController mTurnPIDController;
    private Speed2d mDriveVelocityGoal = new Speed2d(0);
    private Distance2d mDriveDistanceGoal = Distance2d.fromFeet(0);
    private String name = "";
    private Rotation2d turningSetPoint;
    private Motor mDriveMotorObj;
    private DriveControlType mDriveControlType = DriveControlType.RAW;

    public SwerveNEODriveModule(String name, Motor turn, Motor drive, Encoder encoder, XMLSettingReader settings) {
        this.name = name;
        this.mDriveMotorObj = drive;

        this.mDriveMotor = new CANSparkMax(drive.getCANID(), drive.getMotorType().getREVType());
        this.mDriveMotor.restoreFactoryDefaults();
        this.mDriveMotor.setInverted(drive.inverted());
        this.mDriveMotor.setSmartCurrentLimit(drive.getCurrentLimit());
        this.mDriveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        this.mDriveMotorEncoder = this.mDriveMotor.getEncoder();

        //Inches per Rotation.
        //For MK3 Swerve (3.1415 * 4")/8.16
        this.mDriveMotorEncoder.setPositionConversionFactor(settings.getSetting("DriveTrain-ConversionFactor", (2.1415 * 4)/8.16));

        this.mDrivePIDController = this.mDriveMotor.getPIDController();

        this.mDrivePIDController.setSmartMotionAllowedClosedLoopError(1, 0);
        this.mDrivePIDController.setSmartMotionMaxVelocity(2000, 0);
        this.mDrivePIDController.setSmartMotionMaxAccel(1500, 0);

        this.mTurnMotor = new CANSparkMax(turn.getCANID(), turn.getMotorType().getREVType());
        this.mTurnMotor.restoreFactoryDefaults();
        this.mTurnMotor.setInverted(turn.inverted());
        this.mTurnMotor.setSmartCurrentLimit(turn.getCurrentLimit());
        this.mTurnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        this.mTurnEncoder = new CANCoder(encoder.getCANID());
        this.mTurnMotorEncoder = this.mTurnMotor.getEncoder();
        this.mTurnMotorEncoder.setPositionConversionFactor((1 / turn.getGearRatio()) / 360);

        this.mTurnPIDController = mTurnMotor.getPIDController();
        this.mTurnPIDController.setP(turn.getPID(0).getP());
        this.mTurnPIDController.setP(turn.getPID(0).getI());
        this.mTurnPIDController.setP(turn.getPID(0).getD());
        this.mTurnPIDController.setSmartMotionAllowedClosedLoopError(1, 0);
//        this.mTurnPIDController.setSmartMotionMaxVelocity(2000, 0);
//        this.mTurnPIDController.setSmartMotionMaxAccel(1500, 0);

        this.mTurnEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180); // -180 to 180
        this.mTurnEncoder.configMagnetOffset(encoder.getOffset());

        this.mTurnMotorEncoder.setPosition(this.mTurnEncoder.getAbsolutePosition());
    }

    public SwerveNEODriveModule(CANSubsystem subsystem, XMLSettingReader settingReader) {
        this(subsystem.getName(), (Motor) subsystem.getDevice("Turn Motor"), (Motor) subsystem.getDevice("Drive Motor"), (Encoder) subsystem.getDevice("Turn Encoder"), settingReader);
    }

    /**
     * Periodic for NEO Type Module does not need anything in the periodic
     */
    @Override
    public void periodic() {
    }

    /**
     * Returns the current wheel angle on the swereve module using the motor encoder.
     * @return - current wheel angle
     */
    @Override
    public Rotation2d getModuleAngle() {
        return Rotation2d.fromDegrees(this.mTurnMotorEncoder.getPosition());
    }

    /**
     * Sets the target wheel angle to feed the pid controller
     * @param angle - desired wheel angle (Degrees 0-360)
     */
    @Override
    public void setModuleAngle(Rotation2d angle) {
        turningSetPoint = angle;
        this.mTurnPIDController.setReference(turningSetPoint.getDegrees(), ControlType.kPosition);
    }

    /**
     * Sets the raw wheel speed -1~1
     * @param speed - desired raw power
     */
    @Override
    public void setRawDriveSpeed(double speed) {
        this.mDriveMotor.set(speed);
    }

    //Velocity getters and setters

    @Override
    public void setVelocityDriveSpeed(Speed2d speed) {
        mDriveVelocityGoal = speed;

        if (mDriveControlType != DriveControlType.VELOCITY) {
            this.mDriveMotorObj.getPID(intDriveVelocityPIDSlotID).setREVPIDValues(this.mDrivePIDController);
            mDriveControlType = DriveControlType.VELOCITY;
        }

        this.mDrivePIDController.setReference(mDriveVelocityGoal.getValue(Distance2d.DistanceUnits.METER, Time2d.TimeUnits.SECONDS), ControlType.kVelocity);
    }

    @Override
    public Speed2d getDriveVelocity() {
        return new Speed2d(Distance2d.DistanceUnits.INCH, Time2d.TimeUnits.MINUTES, this.mDriveMotorEncoder.getVelocity());
    }

    @Override
    public Speed2d getDriveVelocityGoal() {
        return mDriveVelocityGoal;
    }

    //Distance getters and setters

    /**
     * Sets the distance target of the drive wheel and automatically sets the PID values
     * @param distance2d - Target distance for the wheel.
     */
    @Override
    public void setDriveDistanceTarget(Distance2d distance2d) {
        mDriveDistanceGoal = distance2d;

        if (mDriveControlType != DriveControlType.DISTANCE) {
            this.mDriveMotorObj.getPID(intDriveDistancePIDSlotID).setREVPIDValues(this.mDrivePIDController);
            mDriveControlType = DriveControlType.DISTANCE;
        }

        this.mDriveMotorEncoder.setPosition(0);
        this.mDrivePIDController.setReference(mDriveDistanceGoal.getValue(Distance2d.DistanceUnits.INCH), ControlType.kPosition);
    }

    @Override
    public Distance2d getCurrentDrivePosition() {
        return Distance2d.fromUnit(Distance2d.DistanceUnits.INCH, this.mDriveMotorEncoder.getPosition());
    }

    @Override
    public Distance2d getDriveDistanceTarget() {
        return mDriveDistanceGoal;
    }

    @Override
    public void setDriveCoastMode(boolean brake) {
        this.mDriveMotor.setIdleMode(brake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    }

    /**
     * Usage is ONLY for time saving pre enable (Note change is automatic when set velocity or distance is called)
     *
     * @param control
     */
    @Override
    public void configDrivetrainControlType(DriveControlType control) {
        switch (control) {
            case VELOCITY:
                this.mDriveMotorObj.getPID(intDriveVelocityPIDSlotID).setREVPIDValues(this.mDrivePIDController);
                mDriveControlType = DriveControlType.VELOCITY;
                break;
            case DISTANCE:
                this.mDriveMotorObj.getPID(intDriveDistancePIDSlotID).setREVPIDValues(this.mDrivePIDController);
                mDriveControlType = DriveControlType.DISTANCE;
                break;
        }
    }
}
