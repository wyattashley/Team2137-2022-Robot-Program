package frc.robot.concept;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;
import frc.robot.library.Constants;

public class ConceptPIDControlSystem extends TimedRobot {

    private double p = 0;
    private double i = 0;
    private double d = 0;

    private double currentPosition = 0;
    private double loopPeriod = 0.02;

    private PIDController pidController = new PIDController(p, i, d);

    // public static void main(String... args) {
    //     RobotBase.startRobot(ConceptPIDControlSystem::new);
    // }

    @Override
    public void robotInit() {}

    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousInit() {
        pidController.setSetpoint(50);
    }

    @Override
    public void autonomousPeriodic() {
        double currentOutput = pidController.calculate(currentPosition);
        SmartDashboard.putNumber("ConceptPIDOutput", pidController.calculate(currentPosition));
        currentPosition += (loopPeriod * currentOutput);
    }
}
