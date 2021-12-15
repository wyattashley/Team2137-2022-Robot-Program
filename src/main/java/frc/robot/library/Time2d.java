package frc.robot.library;

public class Time2d {
    public enum TimeUnits {
        MILLISECONDS (1000.0, "Milliseconds"),
        SECONDS (1.0, "Seconds"),
        MINUTES (1.0/60.0, "Minutes"),
        HOURS (1.0/3600.0, "Hours");

        public double secondToUnit;
        private final String name;

        TimeUnits(double _secondsToUnit, String _name) {
            secondToUnit = _secondsToUnit;
            name = _name;
        }

        public String getName() {
            return name;
        }
    }

    private final double secondsValue;

    private Time2d(double value) {
        secondsValue = value;
    }

    public static Time2d fromUnit(TimeUnits unit, double otherUnit) {
        return new Time2d(otherUnit / unit.secondToUnit);
    }

    public static Time2d fromSeconds(double seconds) {
        return new Time2d(seconds);
    }

    public double getValue(TimeUnits unit) {
        return secondsValue * unit.secondToUnit;
    }
}
