package frc.robot.library;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import org.ejml.simple.SimpleMatrix;

import java.util.Stack;

public final class Constants {
    public enum RobotState {
        AUTONOMOUS("Auto"),
        DISABLED("Disa"),
        TELEOP("Tele"),
        TEST("Test");

        private final String name;

        RobotState(String _name) {
            name = _name;
        }

        public String toString() {
            return name;
        }
    }

    public enum StepState {
        STATE_INIT ("STATE_INIT"),
        STATE_RUNNING ("STATE_RUNNING"),
        STATE_FINISH ("STATE_FINISHED"),
        STATE_NOT_STARTED ("STATE_NOT_STARTED");

        private String name = "";

        StepState(String name) {
            this.name = name;
        }

        public boolean isFinished() {
            return this == STATE_FINISH;
        }

        public String toString() {
            return this.name;
        }
    }

    public enum DriveControlType {
        RAW ("Raw"),
        VELOCITY ("Velocity"),
        DISTANCE ("Distance");

        private final String name;

        DriveControlType(String name) {
            this.name = name;
        }

        public String toString() {
            return name;
        }
    }

    //////Utility Functions//////

    /**
     * input matrix ->
     * [X1][X2]
     * [Y1][Y2]
     * [R1][R2]
     *
     * @param angleRadians - Frame angle offset
     * @param locationMatrix - Matrix with components to be transformed
     * @return - resultant matrix
     */
    public static SimpleMatrix convertFrame(double angleRadians, SimpleMatrix locationMatrix) {
        SimpleMatrix rotationalMatrix = new SimpleMatrix(
                new double[][] {
                        new double[] {Math.cos(angleRadians), Math.sin(angleRadians), 0.0},
                        new double[] {Math.sin(angleRadians), -Math.cos(angleRadians), 0.0},
                        new double[] {0.0, 0.0, 1.0}
                }
        );

        /* Resultant matrix ->
         * [Y1][Y2]
         * [X1][X2]
         * [R1][R2] (Angles)
         */
        SimpleMatrix returner = rotationalMatrix.mult(locationMatrix);

        SimpleMatrix xLine = returner.rows(1,2);
        SimpleMatrix yLine = returner.rows(0,1);
        SimpleMatrix rLine = returner.rows(2,3);

        return xLine.concatRows(yLine, rLine);
    }

    public static SimpleMatrix createFrameMatrix(double x, double y, double r) {
        return new SimpleMatrix(new double[][]{
                new double[] {x},
                new double[] {y},
                new double[] {r}
        });
    }

    public static SimpleMatrix createFrameMatrix(Distance2d x, Distance2d y, Rotation2d r) {
        return createFrameMatrix(x.getValue(Distance2d.DistanceUnits.INCH), y.getValue(Distance2d.DistanceUnits.INCH), r.getRadians());
    }

    public static double deadband(double value, double deadband) {
        if(Math.abs(value) < deadband) {
            return 0;
        } else {
            return value;
        }
    }

    //////Exception Classes//////

    /**
     * PIDSlotIDOutOfRangeException is used by Motor class when slot id provided in XML File is outside acceptable range
     * @see frc.robot.functions.io.xmlreader.Motor
     */
    public static class PIDSlotIDOutOfRangeException extends Exception {
        public PIDSlotIDOutOfRangeException(String deviceName) {
            super("PID Slot ID is Out of Range in XML File (" + deviceName + ")");
        }
    }

    /**
     * MissingDeviceException is used by CANSubsystem class when required hardware is not contain in XML File
     * @see frc.robot.library.CANSubsystem
     */
    public static class MissingDeviceException extends Exception {
        public MissingDeviceException(CANSubsystem subsystem, String deviceName) {
            super("Missing Device from XML File in " + subsystem.getName() + ". Please add " + deviceName + " to the XML file");
        }
    }
}
