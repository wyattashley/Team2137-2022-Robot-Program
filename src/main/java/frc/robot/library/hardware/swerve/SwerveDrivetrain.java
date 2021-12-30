package frc.robot.library.hardware.swerve;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.functions.io.FileLogger;
import frc.robot.functions.io.xmlreader.Device;
import frc.robot.functions.io.xmlreader.Motor;
import frc.robot.functions.io.xmlreader.XMLSettingReader;
import frc.robot.library.CANSubsystem;
import frc.robot.library.Constants;
import frc.robot.library.Distance2d;
import frc.robot.library.Speed2d;
import frc.robot.library.hardware.DriveTrain;
import org.ejml.simple.SimpleMatrix;

import static frc.robot.functions.io.xmlreader.Motor.MotorTypes.FALCON;
import static frc.robot.functions.io.xmlreader.Motor.MotorTypes.NEO;

public class SwerveDrivetrain extends Device implements DriveTrain {

    public SwerveModule leftFrontModule;
    public SwerveModule leftBackModule;
    public SwerveModule rightFrontModule;
    public SwerveModule rightBackModule;

    public Motor.MotorTypes driveTrainType;

    private PigeonIMU pigeonIMU;

    public SwerveDrivetrain(CANSubsystem canSubsystem, XMLSettingReader settings, FileLogger fileLogger) {
        super("DriveTrain");
        try {
            if (canSubsystem.getName().equalsIgnoreCase("Swerve NEO")) {
                leftFrontModule = new SwerveNEODriveModule(canSubsystem.getRequiredSubsystem("LeftFront", "SwerveModule"), settings);
                leftBackModule = new SwerveNEODriveModule(canSubsystem.getRequiredSubsystem("LeftBack", "SwerveModule"), settings);
                rightFrontModule = new SwerveNEODriveModule(canSubsystem.getRequiredSubsystem("RightFront", "SwerveModule"), settings);
                rightBackModule = new SwerveNEODriveModule(canSubsystem.getRequiredSubsystem("RightBack", "SwerveModule"), settings);
                driveTrainType = NEO;
            } else if (canSubsystem.getName().equalsIgnoreCase("Swerve Falcon")){
                leftFrontModule = new SwerveFALCONDriveModule(canSubsystem.getRequiredSubsystem("LeftFront", "SwerveModule"), settings);
                leftBackModule = new SwerveFALCONDriveModule(canSubsystem.getRequiredSubsystem("LeftBack", "SwerveModule"), settings);
                rightFrontModule = new SwerveFALCONDriveModule(canSubsystem.getRequiredSubsystem("RightFront", "SwerveModule"), settings);
                rightBackModule = new SwerveFALCONDriveModule(canSubsystem.getRequiredSubsystem("RightBack", "SwerveModule"), settings);
                driveTrainType = FALCON;
            } else {
                fileLogger.writeEvent(0, FileLogger.EventType.Error, "DriveTrain name not compatible with swerve");
            }
        } catch (Constants.MissingDeviceException e) {
            fileLogger.writeEvent(0, FileLogger.EventType.Error, "Looks like there is a missing device in XML File SwerveDrive train disabled");
            e.printStackTrace();
            return;
        }

//        pigeonIMU = new PigeonIMU(canSubsystem.getCANDevice("IMU").getCANID());
//        pigeonIMU.configFactoryDefault();
//        pigeonIMU.setFusedHeading(0);
    }

    @Override
    public void periodic() {}

    public SwerveModuleState[] calculateSwerveMotorSpeedsFieldCentric(double xMag, double yMag, double rMag, double trackWidth, double wheelBase, Constants.DriveControlType controlType) {
        return calculateSwerveMotorSpeeds(Constants.convertFrame(getAngle().getRadians(), Constants.createFrameMatrix(xMag, yMag, rMag)), trackWidth, wheelBase, controlType);
    }

    /**
     * input matrix ->
     * [X1]
     * [Y1]
     * [R1]
     *
     * Resultant ->
     * [Left Front Speed]   [Left Front Angle]
     * [Left Back Speed]    [Left Back Angle]
     * [Right Front Speed]  [Right Front Angle]
     * [Right Back Speed]   [Right Back Angle]
     */
    public SwerveModuleState[] calculateSwerveMotorSpeeds(SimpleMatrix directions, double trackWidth, double wheelBase, Constants.DriveControlType controlType) {
        return calculateSwerveMotorSpeeds(directions.get(0, 0), directions.get(1, 0), directions.get(2, 0), trackWidth, wheelBase, controlType);
    }

    /**
     * Returns the speeds of the Swerve Drive Train when given the Controller Values
     *
     * Inputs can be velocities for -1 to 1 from the joy stick
     * xD - The left joystick -1 ~ 1
     * yD - The left joystick -1 ~ 1
     * tD - The rightjoy stick or turning button -1 ~ 1
     * @return An Array of Points with x being drive speed and y wheel angle in degree
     */
    public SwerveModuleState[] calculateSwerveMotorSpeeds(double xMag, double yMag, double rMag, double axelDistance, double wheelBase, Constants.DriveControlType controlType) {
        double r = Math.sqrt((axelDistance * axelDistance) + (wheelBase * wheelBase)); //Distance between adjectent wheel

        double a = xMag - rMag * (wheelBase / r); // translatedSpeeds[2] * (axleDistance / r) is the ratio of wheel distance from other wheels
        double b = xMag + rMag * (wheelBase / r);
        double c = yMag - rMag * (axelDistance / r);
        double d = yMag + rMag * (axelDistance / r);

        double[][] speeds = new double[][] {
                new double[] {Math.sqrt(b * b + d * d), normalizeRadianAngle(Math.atan2(b, d))}, // Left Front
                new double[] {Math.sqrt(a * a + d * d), normalizeRadianAngle(Math.atan2(a, d))}, // Left Back
                new double[] {Math.sqrt(b * b + c * c), normalizeRadianAngle(Math.atan2(b, c))}, // Right Front
                new double[] {Math.sqrt(a * a + c * c), normalizeRadianAngle(Math.atan2(a, c))}, // Right Back
        };

        if (controlType == Constants.DriveControlType.VELOCITY) {
            return new SwerveModuleState[]{
                    new SwerveModuleState(new Speed2d(speeds[0][0]), new Rotation2d(speeds[0][1]), SwerveModuleState.SwerveModulePositions.LEFT_FRONT),
                    new SwerveModuleState(new Speed2d(speeds[1][0]), new Rotation2d(speeds[1][1]), SwerveModuleState.SwerveModulePositions.LEFT_BACK),
                    new SwerveModuleState(new Speed2d(speeds[2][0]), new Rotation2d(speeds[2][1]), SwerveModuleState.SwerveModulePositions.RIGHT_FRONT),
                    new SwerveModuleState(new Speed2d(speeds[3][0]), new Rotation2d(speeds[3][1]), SwerveModuleState.SwerveModulePositions.RIGHT_BACK),
            };
        } else if(controlType == Constants.DriveControlType.DISTANCE) {
            return new SwerveModuleState[]{
                    new SwerveModuleState(Distance2d.fromUnit(Distance2d.DistanceUnits.INCH, speeds[0][0]), new Rotation2d(speeds[0][1]), SwerveModuleState.SwerveModulePositions.LEFT_FRONT),
                    new SwerveModuleState(Distance2d.fromUnit(Distance2d.DistanceUnits.INCH, speeds[1][0]), new Rotation2d(speeds[1][1]), SwerveModuleState.SwerveModulePositions.LEFT_BACK),
                    new SwerveModuleState(Distance2d.fromUnit(Distance2d.DistanceUnits.INCH, speeds[2][0]), new Rotation2d(speeds[2][1]), SwerveModuleState.SwerveModulePositions.RIGHT_FRONT),
                    new SwerveModuleState(Distance2d.fromUnit(Distance2d.DistanceUnits.INCH, speeds[3][0]), new Rotation2d(speeds[3][1]), SwerveModuleState.SwerveModulePositions.RIGHT_BACK),
            };
        } else {
            return new SwerveModuleState[]{
                    new SwerveModuleState(speeds[0][0], new Rotation2d(speeds[0][1]), SwerveModuleState.SwerveModulePositions.LEFT_FRONT),
                    new SwerveModuleState(speeds[1][0], new Rotation2d(speeds[1][1]), SwerveModuleState.SwerveModulePositions.LEFT_BACK),
                    new SwerveModuleState(speeds[2][0], new Rotation2d(speeds[2][1]), SwerveModuleState.SwerveModulePositions.RIGHT_FRONT),
                    new SwerveModuleState(speeds[3][0], new Rotation2d(speeds[3][1]), SwerveModuleState.SwerveModulePositions.RIGHT_BACK),
            };
        }
    }

    /**
     * Corrected Angle (0~PI)
     */
    public static double normalizeRadianAngle(double value) {
        value %= Math.PI;
        return value < 0 ? value + Math.PI : value;
    }

    public void setSwerveModuleStates(SwerveModuleState[] swerveModuleStates) {
        this.leftFrontModule.setSwerveModuleState(swerveModuleStates[0]);
        this.leftBackModule.setSwerveModuleState(swerveModuleStates[1]);
        this.rightFrontModule.setSwerveModuleState(swerveModuleStates[2]);
        this.rightBackModule.setSwerveModuleState(swerveModuleStates[3]);
    }

    /**
     * Turns the controller input into field centric values
     * @param xSpeed - Desired X speed on field
     * @param ySpeed - Desired Y speed on field
     * @param radianPerSecond - Desired Turn speed on field
     * @param robotAngle - Current robot Angle on field
     * @return Array or double {xSpeed, ySpeed, turnSpeed}
     */
    public static double[] toFieldRelativeChassisSpeeds(double xSpeed, double ySpeed, double radianPerSecond, Rotation2d robotAngle) {
        return new double[]{
                xSpeed * robotAngle.getCos() + ySpeed * robotAngle.getSin(),
                -xSpeed * robotAngle.getSin() + ySpeed * robotAngle.getCos(),
                radianPerSecond};
    }

    @Override
    public void setLeftSpeed(double speed) {
        this.leftFrontModule.setRawDriveSpeed(speed);
        this.leftBackModule.setRawDriveSpeed(speed);
    }

    @Override
    public void setRightSpeed(double speed) {
        this.rightFrontModule.setRawDriveSpeed(speed);
        this.rightBackModule.setRawDriveSpeed(speed);
    }

    @Override
    public Rotation2d getAngle() {
//        double raw = pigeonIMU.getFusedHeading() % 360;
//
//        if(raw >= 180)
//            raw -= 360;
//        else if(raw < -180)
//            raw += 360;
//
//        return Rotation2d.fromDegrees(raw);
        return Rotation2d.fromDegrees(0);
    }

    @Override
    public void setAngleOffset(Rotation2d offset) {
        this.pigeonIMU.setFusedHeading(offset.getDegrees());
    }

    @Override
    public void configDrivetrainControlType(Constants.DriveControlType control) {
        leftFrontModule.configDrivetrainControlType(control);
        rightFrontModule.configDrivetrainControlType(control);
        leftBackModule.configDrivetrainControlType(control);
        rightBackModule.configDrivetrainControlType(control);
    }

    public Motor.MotorTypes getDrivetrainType() {
        return driveTrainType;
    }


}
