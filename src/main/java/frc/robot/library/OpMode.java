package frc.robot.library;

public interface OpMode {

    /**
     * Runs once when the robot state changed.
     */
    void init();

    /**
     * Repeats until robot state is changed.
     */
    void periodic();

    /**
     * Runs once after the robot state has changed.
     */
    void end();
}
