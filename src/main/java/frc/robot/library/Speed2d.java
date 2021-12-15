package frc.robot.library;

public class Speed2d {

    private final Distance2d distance;
    private final Time2d time;

    public Speed2d(Distance2d.DistanceUnits distanceUnits, Time2d.TimeUnits timeUnits, double value) {
        distance = Distance2d.fromUnit(distanceUnits, value);
        time = Time2d.fromUnit(timeUnits, 1);
    }

    public Speed2d(Distance2d distance, Time2d time) {
        this(Distance2d.DistanceUnits.FEET, Time2d.TimeUnits.SECONDS, distance.getValue(Distance2d.DistanceUnits.FEET)/time.getValue(Time2d.TimeUnits.SECONDS));
    }

    public Speed2d(double value) {
        this(Distance2d.DistanceUnits.FEET, Time2d.TimeUnits.SECONDS, value);
    }

    public double getValue(Distance2d.DistanceUnits distanceUnit, Time2d.TimeUnits timeUnit) {
        return distance.getValue(distanceUnit) / time.getValue(timeUnit);
    }

    public double getValue() {
        return getValue(Distance2d.DistanceUnits.FEET, Time2d.TimeUnits.SECONDS);
    }

    public static Speed2d fromFeetPerSecond(double value) {
        return new Speed2d(value);
    }

    @Override
    public String toString() {
        return distance.getValue(Distance2d.DistanceUnits.FEET)/time.getValue(Time2d.TimeUnits.SECONDS) + " F/S";
    }
}
