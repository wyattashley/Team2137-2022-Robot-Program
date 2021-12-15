package frc.robot.functions.io.xmlreader;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpiutil.math.Pair;
import frc.robot.library.Constants;
import frc.robot.library.Time2d;

import java.util.HashMap;

public class Step {
    public enum StepValues {
        COMMAND ("COMMAND"),
        TIMEOUT ("TIMEOUT"),
        SPEED ("SPEED"),
        DISTANCE ("DISTANCE"),
        XDISTANCE ("XDISTANCE"),
        YDISTANCE ("YDISTANCE"),
        PARALLEL ("PARALLEL");

        String name = "";

        StepValues(String str) { 
            this.name = str;
        }

        public String toString() { return name; }
    }

    private final HashMap<String, String> values = new HashMap<String, String>();
    private String command;
    private Constants.StepState mCurrentStepState = Constants.StepState.STATE_INIT;

    private double startTime;

    public Step (String _COMMAND, String _TIMEOUT, String _SPEED, String _XDISTANCE, String _YDISTANCE, String _PARALLEL, String[] _PARMs) {
        this.command = _COMMAND;

        this.values.put(StepValues.COMMAND.toString(),  _COMMAND);
        this.values.put(StepValues.TIMEOUT.toString(), _TIMEOUT);
        this.values.put(StepValues.SPEED.toString(),  (_SPEED));
        this.values.put(StepValues.XDISTANCE.toString(),  (_XDISTANCE));
        this.values.put(StepValues.YDISTANCE.toString(),  (_YDISTANCE));
        this.values.put(StepValues.PARALLEL.toString(),  (_PARALLEL));
        for (int i = 0; i < _PARMs.length; i++) {
            this.values.put("PARM" + i,  _PARMs[i]);
        }
    }

    public Step (String _COMMAND, String _TIMEOUT, String _SPEED, String _DISTANCE, String _PARALLEL, String[] _PARMs) {
        this(_COMMAND, _TIMEOUT, _SPEED, _DISTANCE, _DISTANCE, _PARALLEL, _PARMs);
    }

    public Step(String[] _PARMs) {
        for (int i = 0; i < _PARMs.length; i++) {
            this.values.put("PARM" + i,  _PARMs[i]);
        }
    }

    public void setValue(StepValues key, Object val) {
        this.values.replace(key.toString(), String.valueOf(val));
    }

    public void setValue(Pair<StepValues, String> pair) {
        this.values.put(pair.getFirst().toString(), pair.getSecond());
    }

    public String getCommand() {
        return this.command;
    }

    public double getSpeed() {
        return Double.parseDouble(this.values.get(StepValues.SPEED.toString()));
    }

    public double getXDistance() {
        return Double.parseDouble(this.values.get(StepValues.XDISTANCE.toString()));
    }

    public double getYDistance() {
        return Double.parseDouble(this.values.get(StepValues.YDISTANCE.toString()));
    }

    public double getDistance() {
        return (getXDistance() + getYDistance()) / 2;
    }

    public boolean isParallel() {
        return Boolean.parseBoolean(this.values.get(StepValues.PARALLEL.toString()));
    }

    public Double getParm(int PARM) {
        return Double.parseDouble("PARM" + PARM);
    }

    public Double getParm(int PARM, Double falseReturn) {
        if (this.values.containsKey("PARM" + PARM)) {
            return Double.parseDouble("PARM" + PARM);
        } else {
            return falseReturn;
        }
    }

    public Integer getParmInt(int PARM) {
        if (this.values.containsKey("PARM" + PARM)) {
            return Integer.valueOf("PARM" + PARM);
        } else {
            return null;
        }
    }

    public boolean checkParm(int PARM) {
        return this.values.containsKey("PARM" + PARM);
    }

    public void changeStepState(Constants.StepState newStepState) {
        this.mCurrentStepState = newStepState;
    }

    public Constants.StepState getStepState() {
        return this.mCurrentStepState;
    }

    public void StartTimer() {
        startTime = System.currentTimeMillis();
    }

    public Time2d getTime() {
        return Time2d.fromUnit(Time2d.TimeUnits.SECONDS, System.currentTimeMillis() - startTime);
    }

    public boolean hasTimeElapsed(Time2d time) {
        return (System.currentTimeMillis() - startTime) >= time.getValue(Time2d.TimeUnits.MILLISECONDS);
    }
}