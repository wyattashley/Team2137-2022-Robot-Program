package frc.robot.library.hardware.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.functions.io.xmlreader.Device;
import frc.robot.functions.io.xmlreader.Encoder;
import frc.robot.functions.io.xmlreader.Motor;
import frc.robot.functions.io.xmlreader.XMLSettingReader;
import frc.robot.library.*;

import static frc.robot.library.Constants.deadband;

public class SwerveFALCONDriveModule extends Device implements SwerveModule {

    private static final int intDriveVelocityPIDSlotID = 0;
    private static final int intDriveDistancePIDSlotID = 1;
    private final TalonFX driveMotor;
    private final TalonFX turningMotor;
    private final CANCoder encoder;
    private final PIDController turningPID;
    private PIDController drivePID;
    private SimpleMotorFeedforward driveFeedForward;
    private Rotation2d turnMotorSetpoint = Rotation2d.fromDegrees(0);
    private double dblWheelConversionValue = 0;
    private Speed2d mDriveVelocityGoal = new Speed2d(0);
    private Distance2d mDriveDistanceGoal = Distance2d.fromFeet(0);
    private final Motor mDriveMotorObj;
    private Constants.DriveControlType mDriveControlType = Constants.DriveControlType.RAW;

    public SwerveFALCONDriveModule(String _name, Motor turn, Motor drive, Encoder encoder, XMLSettingReader settingReader) {
        super(_name);
        this.mDriveMotorObj = drive;

        this.driveMotor = new TalonFX(drive.getCANID());
        this.driveMotor.configFactoryDefault();
        this.driveMotor.setInverted(drive.inverted());
        this.driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, drive.getCurrentLimit(), drive.getCurrentLimit(), 1));
        this.driveMotor.setNeutralMode(NeutralMode.Brake);
        this.driveMotor.configOpenloopRamp(drive.getRampRate());

        // Turning motor setup
        this.turningMotor = new TalonFX(turn.getCANID());
        this.turningMotor.configFactoryDefault();
        this.turningMotor.setInverted(turn.inverted());
        this.turningMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, drive.getCurrentLimit(), drive.getCurrentLimit(), 1));
        this.turningMotor.setNeutralMode(NeutralMode.Brake);

        // Encoder setup
        this.encoder = new CANCoder(encoder.getCANID());
        this.encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180); // -180 to 180
        this.encoder.configMagnetOffset(encoder.getOffset());

        this.turningPID = turn.getPID(0).getWPIPIDController();
        this.turningPID.enableContinuousInput(-180, 180); // allows module to wrap properly

        this.drivePID = drive.getPID(intDriveVelocityPIDSlotID).getWPIPIDController();
        this.driveFeedForward = drive.getPID(intDriveVelocityPIDSlotID).getWPIFeedForwardController();

        this.dblWheelConversionValue = settingReader.getSetting("DriveTrain-ConversionFactor", ((2.1415 * 4)/8.16));
    }

    public SwerveFALCONDriveModule(CANSubsystem subsystem, XMLSettingReader settingReader) {
        this(subsystem.getName(),
                (Motor) subsystem.getDevice("Turn Motor"),
                (Motor) subsystem.getDevice("Drive Motor"),
                (Encoder) subsystem.getDevice("Turn Encoder"),
                settingReader);
    }

    @Override
    public void periodic() {
        Rotation2d wheelAngle;
        // prevents the module from doing a >90 degree flip to get to a target, instead reverse wheel direction
        //optimization now done in SwerveDrivetrain's methods, so this is obsolete
        if (Math.abs(turnMotorSetpoint.minus(getModuleAngle()).getDegrees()) > 90) {
            wheelAngle = turnMotorSetpoint.plus(Rotation2d.fromDegrees(180));
        } else {
            wheelAngle = turnMotorSetpoint;
        }

        double pidEffort = turningPID.calculate(getModuleAngle().getDegrees(), wheelAngle.getDegrees());
//        double output = Math.signum(pidEffort) * Constants.Drivetrain.turningFeedForward * Math.abs(Math.signum(Util.deadband(turningPID.getPositionError(), 0.5))) + pidEffort;
//        double output = Math.signum(pidEffort) * Math.abs(Math.signum(turningPID.getPositionError())) + pidEffort;
        double output = pidEffort;

        turningMotor.set(ControlMode.PercentOutput, output / 12);

        switch(mDriveControlType) {
            case VELOCITY:
                output = driveFeedForward.calculate(getDriveVelocity().getValue(Distance2d.DistanceUnits.METER, Time2d.TimeUnits.SECONDS)) +
                        drivePID.calculate(getDriveVelocity().getValue(Distance2d.DistanceUnits.METER, Time2d.TimeUnits.SECONDS));
                driveMotor.set(ControlMode.PercentOutput, output / 12);
                break;
            case DISTANCE:
                output = driveFeedForward.calculate(getCurrentDrivePosition().getValue(Distance2d.DistanceUnits.METER)) +
                        drivePID.calculate(getCurrentDrivePosition().getValue(Distance2d.DistanceUnits.METER));
                driveMotor.set(ControlMode.PercentOutput, output / 12);
                break;
        }
    }

    /**
     * getModuleAngle() takes the value from the CANCoder
     * @return Wheel angle on module
     */
    @Override
    public Rotation2d getModuleAngle() {
        return Rotation2d.fromDegrees(encoder.getAbsolutePosition());
    }

    /**
     * Assigns a new module goal angle to the PID
     * @param angle - new angle goal for the wheel
     */
    @Override
    public void setModuleAngle(Rotation2d angle) {
        turnMotorSetpoint = angle;
        turningPID.calculate(angle.getDegrees());
    }


    /**
     * Sets the raw speed -1~1 to motor and disables Velocity and Distance
     * @param speed - raw speed value to set
     */
    @Override
    public void setRawDriveSpeed(double speed) {
        if(this.mDriveControlType != Constants.DriveControlType.RAW)
            this.mDriveControlType = Constants.DriveControlType.RAW;
        driveMotor.set(TalonFXControlMode.PercentOutput, speed);
    }

    /**
     * Set the goal velocity value to the PID Controller
     * @param speed - Speed to set to the drive train
     */
    @Override
    public void setVelocityDriveSpeed(Speed2d speed) {
        this.mDriveVelocityGoal = speed;

        if(this.mDriveControlType != Constants.DriveControlType.VELOCITY) {
            configDrivetrainControlType(Constants.DriveControlType.VELOCITY);
        }

        drivePID.setSetpoint(speed.getValue(Distance2d.DistanceUnits.METER, Time2d.TimeUnits.SECONDS));
    }

    /**
     * Returns the instantaneous velocity of the wheel using integrated motor encoder.
     * @return - a Speed2d is returned
     */
    @Override
    public Speed2d getDriveVelocity() {
        return new Speed2d(Distance2d.DistanceUnits.METER, Time2d.TimeUnits.SECONDS, (driveMotor.getSensorCollection().getIntegratedSensorVelocity() * 10 / 2048) * dblWheelConversionValue);
    }

    /**
     * Returns the current velocity goal of the drive train
     * @return current velocity goal
     */
    @Override
    public Speed2d getDriveVelocityGoal() {
        return this.mDriveVelocityGoal;
    }

    /**
     * Return the current distance goal of the drive train
     * @return current distance goal
     */
    @Override
    public Distance2d getDriveDistanceTarget() {
        return this.mDriveDistanceGoal;
    }

    /**
     * Returns the current wheel position on the robot.
     * @return Current drive wheel encoder position
     */
    @Override
    public Distance2d getCurrentDrivePosition() {
        return Distance2d.fromUnit(Distance2d.DistanceUnits.INCH, (this.encoder.getPosition() / 2048) * dblWheelConversionValue);
    }

    /**
     * Sets drive train distance goal (Only drive wheel)
     * @param distance2d - Distance goal for robot drive wheel
     */
    @Override
    public void setDriveDistanceTarget(Distance2d distance2d) {
        this.mDriveDistanceGoal = distance2d;

        if(this.mDriveControlType != Constants.DriveControlType.DISTANCE) {
            configDrivetrainControlType(Constants.DriveControlType.DISTANCE);
        }

        drivePID.setSetpoint(distance2d.getValue(Distance2d.DistanceUnits.INCH));
    }

    /**
     * Dictates weather the drive motor is to power stop
     * @param brake - True if braking is enabled
     */
    @Override
    public void setDriveCoastMode(boolean brake) {
        driveMotor.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
    }

    /**
     * Usage is ONLY for time saving pre enable (Note change is automatic when set velocity or distance is called)
     * @param control - Desired control type to be used
     */
    @Override
    public void configDrivetrainControlType(Constants.DriveControlType control) {
        switch(control) {
            case VELOCITY:
                this.drivePID = this.mDriveMotorObj.getPID(intDriveVelocityPIDSlotID).getWPIPIDController();
                this.driveFeedForward = this.mDriveMotorObj.getPID(intDriveVelocityPIDSlotID).getWPIFeedForwardController();
                mDriveControlType = Constants.DriveControlType.VELOCITY;
                break;
            case DISTANCE:
                this.drivePID = this.mDriveMotorObj.getPID(intDriveDistancePIDSlotID).getWPIPIDController();
                this.driveFeedForward = this.mDriveMotorObj.getPID(intDriveDistancePIDSlotID).getWPIFeedForwardController();
                mDriveControlType = Constants.DriveControlType.DISTANCE;
                break;
            case RAW:
                mDriveControlType = Constants.DriveControlType.RAW;
                break;
        }
    }
}
