package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.functions.io.FileLogger;
import frc.robot.functions.io.FileLogger.EventType;
import frc.robot.functions.io.xmlreader.Step;
import frc.robot.functions.io.xmlreader.XMLSettingReader;
import frc.robot.functions.io.xmlreader.XMLStepReader;
import frc.robot.library.*;
import frc.robot.library.Constants.StepState;
import frc.robot.library.Constants.RobotState;
import frc.robot.library.hardware.DriveTrain;
import frc.robot.library.hardware.swerve.SwerveDrivetrain;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Stack;
import java.util.function.BiConsumer;
import java.util.function.Consumer;

public class Autonomous implements OpMode {

    //Filelogger values
    private FileLogger logger;
    private final int mintDebug = 0;

    //XML Setting reader File
    private XMLSettingReader mSettingReader;
    private final String strSettingFileName = "Settings.xml";
    private final String strSettingFilePath = "\\XMLExamples\\";

    private XMLStepReader mStepReader;
    private final String strStepFileName = "Steps.xml";
    private final String strStepFilePath = "\\XMLExamples\\";

    private CANSubsystem mRobotSubsystem;
    private DriveTrain mDrivetrain;
    
    private List<Step> mStepList;
    private int mintCurrentStep = 0;
    private final List<Step> mCurrentActionSteps = new ArrayList<>();

    //region Main Autonomous Opmode Functions
    @Override
    public void init() {
        //Create Filelogger object and provide the current OPMODE state and debug
        this.logger = new FileLogger(mintDebug, RobotState.AUTONOMOUS);

        this.mSettingReader = new XMLSettingReader(strSettingFilePath, strSettingFileName, logger);
        this.logger.writeEvent(5, EventType.Status, "Setting Reader Initialized");

        this.mRobotSubsystem = mSettingReader.getRobotCANSubsystem();
        if(mintDebug > 8) this.mRobotSubsystem.printHardwareMap(logger);

        this.mStepReader = new XMLStepReader(strStepFilePath, strStepFileName);
        this.mStepList = this.mStepReader.getSteps();

        this.mDrivetrain = new SwerveDrivetrain(mRobotSubsystem.getSubsystemByType("DriveTrain").get(0), mSettingReader, logger);
    }

    @Override
    public void periodic() {
        if (mCurrentActionSteps.size() == 0) {
            mCurrentActionSteps.add(mStepList.get(++mintCurrentStep)); //Add the next step

            for(int i = mintCurrentStep + 1; i < mStepList.size(); i++) {
                Step tmp = mStepList.get(i);

                if (tmp.isParallel()) {
                    mCurrentActionSteps.add(tmp);
                    mintCurrentStep++;
                } else {
                    break;
                }
            }
        }

        for(int i = 0; i < mCurrentActionSteps.size(); i++) {
            Step tmpStep = mCurrentActionSteps.get(i);

            if (tmpStep.getStepState() == StepState.STATE_FINISH) {
                this.logger.writeEvent(3, EventType.Debug, tmpStep.getCommand() + " finished, now removing from operational stack");
                mCurrentActionSteps.remove(tmpStep);
            } else {
                runCommand(tmpStep.getCommand(), tmpStep);
            }
        }
    }

    @Override
    public void end() {
        logger.writeEvent(0, EventType.Status, "Autonomous Ending");
        logger.close();
    }
    //endregion

    //region Autonomous Running Util Functions
    /**
     * Util Functions
     */
    public void runCommand(String commandName, Step step) {
        switch (commandName) {
            case "SetSwerveVelocity":
                setSwerveDrivetrainVelocity(step);
                break;
        }
    }
    //endregion


    //region Swerve Autonomous Commands
    /**
     * Command that sets the Swerve Drivetrain X,Y,R velocities
     *
     * Step layout:
     *  -XDistance is X Velocity
     *  -YDistance is Y Velocity
     *  -Parm 0 is the R Velocity
     *
     *  Command Mode:
     *   -
     *
     * @param step Provided Step
     */
    public void setSwerveDrivetrainVelocity(Step step) {
        logger.setTag("setSwerveDrivetrainVelocity()");
        switch (step.getStepState()) {
            case STATE_INIT:
                SwerveDrivetrain swerve = (SwerveDrivetrain) mDrivetrain;

                Speed2d xVelocity = Speed2d.fromFeetPerSecond(step.getXDistance());
                Speed2d yVelocity = Speed2d.fromFeetPerSecond(step.getYDistance());
                Speed2d rotationVelocity = Speed2d.fromFeetPerSecond(step.getParm(0, 0.0));

                SmartDashboard.putNumber(logger.getTag() + "-XVelocity", xVelocity.getValue(Distance2d.DistanceUnits.FEET, Time2d.TimeUnits.SECONDS));
                SmartDashboard.putNumber(logger.getTag() + "-YVelocity", yVelocity.getValue(Distance2d.DistanceUnits.FEET, Time2d.TimeUnits.SECONDS));
                SmartDashboard.putNumber(logger.getTag() + "-RVelocity", rotationVelocity.getValue(Distance2d.DistanceUnits.FEET, Time2d.TimeUnits.SECONDS));

                //TODO fix the axel track
                swerve.setSwerveModuleStates(swerve.calculateSwerveMotorSpeeds(xVelocity.getValue(), yVelocity.getValue(), rotationVelocity.getValue(), 1, 1, Constants.DriveControlType.VELOCITY));

                step.changeStepState(StepState.STATE_RUNNING);
                step.StartTimer();

                this.logger.writeEvent(5, EventType.Status, "Init Finished for set Swerve Drivetrain Velocity {" + xVelocity + ", " + yVelocity + ", " + rotationVelocity + "}");
                break;

            case STATE_RUNNING:

                if(step.getParm(2, 0d) == 1) {

                }

                if(step.getParm(1, 0d) != 0 && step.hasTimeElapsed(Time2d.fromUnit(Time2d.TimeUnits.MILLISECONDS, step.getParm(1, 0d)))) {
                    mDrivetrain.setSpeed(0);
                    step.changeStepState(StepState.STATE_FINISH);
                    this.logger.writeEvent(5, EventType.Status, "Finished Timed Set Swerve Drive Train Velocity");
                }
                break;

            case STATE_FINISH:
                break;
        }
    }
    //endregion
}
