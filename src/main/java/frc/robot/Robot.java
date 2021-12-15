// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.library.Constants;
import frc.robot.library.OpMode;

public class Robot extends TimedRobot {

  private static final OpMode autonomousClass = new Autonomous();
  private static final OpMode teleopClass = new Teleop();
  private static final OpMode testClass = new Test();
  private static final OpMode disabledClass = new Disabled();

  private static Constants.RobotState lastRobotState = Constants.RobotState.DISABLED;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    switch(lastRobotState) {
      case AUTONOMOUS:
        autonomousClass.end();
        break;
      case TELEOP:
        teleopClass.end();
        break;
      case DISABLED:
        disabledClass.end();
        break;
      case TEST:
        testClass.end();
        break;
    }

    lastRobotState = Constants.RobotState.DISABLED;
    disabledClass.init();
  }

  @Override
  public void disabledPeriodic() {disabledClass.periodic();}

  @Override
  public void autonomousInit() {
    lastRobotState = Constants.RobotState.AUTONOMOUS;
    autonomousClass.init();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {autonomousClass.periodic();}

  @Override
  public void teleopInit() {
    lastRobotState = Constants.RobotState.TELEOP;
    teleopClass.init();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {teleopClass.periodic();}

  @Override
  public void testInit() {
    lastRobotState = Constants.RobotState.TEST;
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    testClass.init();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {testClass.periodic();}
}
