// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.functions.io.xmlreader.Device;
import frc.robot.library.Constants;
import frc.robot.library.OpMode;

import java.util.List;

public class Robot extends TimedRobot {

  private static OpMode autonomousClass;
  private static OpMode teleopClass;
  private static OpMode testClass;
  private static OpMode disabledClass;

  private static Constants.RobotState lastRobotState = Constants.RobotState.DISABLED;
  public static List<Device> deviceCallList;

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
    callEndFunction();
    clearOpModes();

    lastRobotState = Constants.RobotState.DISABLED;

    disabledClass = new Disabled();
    disabledClass.init();
  }

  @Override
  public void disabledPeriodic() {callPeriodicFunction();}

  @Override
  public void autonomousInit() {
    callEndFunction();
    clearOpModes();

    lastRobotState = Constants.RobotState.AUTONOMOUS;

    autonomousClass = new Autonomous();
    autonomousClass.init();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {callPeriodicFunction();}

  @Override
  public void teleopInit() {
    callEndFunction();
    clearOpModes();

    lastRobotState = Constants.RobotState.TELEOP;

    teleopClass = new Teleop();
    teleopClass.init();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {callPeriodicFunction();}

  @Override
  public void testInit() {
    callEndFunction();
    clearOpModes();

    lastRobotState = Constants.RobotState.TEST;
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

    testClass = new Test();
    testClass.init();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {callPeriodicFunction();}

  private void callEndFunction() {
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
  }

  private void callPeriodicFunction() {
    switch(lastRobotState) {
      case AUTONOMOUS:
        autonomousClass.periodic();
        break;
      case TELEOP:
        teleopClass.periodic();
        break;
      case DISABLED:
        disabledClass.periodic();
        break;
      case TEST:
        testClass.periodic();
        break;
    }

    for (Device device : deviceCallList) {
      device.periodic();
    }
  }

  private void clearOpModes() {
    autonomousClass = null; //"Lose" all the pointers to the objects
    teleopClass = null;
    testClass = null;
    disabledClass = null;

    deviceCallList.clear();

    System.gc(); //Hopefully Garbage Collector takes Opmodes
  }
}
