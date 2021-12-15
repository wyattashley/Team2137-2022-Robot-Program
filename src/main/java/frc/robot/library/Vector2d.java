package frc.robot.library;

import edu.wpi.first.wpilibj.geometry.Translation2d;

public class Vector2d extends edu.wpi.first.wpilibj.drive.Vector2d {

    public Vector2d(Distance2d x, Distance2d y) {
        super(x.getValue(Distance2d.DistanceUnits.INCH), y.getValue(Distance2d.DistanceUnits.INCH));
    }

    public Vector2d normalize() {
        double length = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

        return new Vector2d(Distance2d.fromUnit(Distance2d.DistanceUnits.INCH, x / length), Distance2d.fromUnit(Distance2d.DistanceUnits.INCH, y / length));
    }

    public Vector2d scale(double scale) {
        return new Vector2d(Distance2d.fromUnit(Distance2d.DistanceUnits.INCH, x * scale), Distance2d.fromUnit(Distance2d.DistanceUnits.INCH, y * scale));
    }

    public Distance2d getX() {
        return Distance2d.fromUnit(Distance2d.DistanceUnits.INCH, x);
    }

    public Distance2d getY() {
        return Distance2d.fromUnit(Distance2d.DistanceUnits.INCH, y);
    }

    /**
     * Will return in Inches
     * @param translation2d
     * @return
     */
    public Translation2d applyVector(Translation2d translation2d) {
        return new Translation2d(translation2d.getX() + this.x, translation2d.getY() + this.y);
    }

    public double slope() {
        return y / x;
    }
}
