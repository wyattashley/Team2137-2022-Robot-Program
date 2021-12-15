package frc.robot.library.PurePursuit;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.robot.library.Distance2d;

public class Waypoint {
    private Distance2d x;
    private Distance2d y;
    private Rotation2d heading;
    private double curvature;
    private Distance2d distanceFromStart;

    public Waypoint(Distance2d _x, Distance2d _y, Rotation2d _w, double _curvature, Distance2d _distanceFromStart) {
        x = _x;
        y = _y;
        heading = _w;
        curvature = _curvature;
        distanceFromStart = _distanceFromStart;
    }

    public Distance2d getX() {
        return x;
    }

    public void setX(Distance2d x) {
        this.x = x;
    }

    public Distance2d getY() {
        return y;
    }

    public void setY(Distance2d y) {
        this.y = y;
    }

    public Rotation2d getHeading() {
        return heading;
    }

    public void setHeading(Rotation2d heading) {
        this.heading = heading;
    }

    public double getCurvature() {
        return curvature;
    }

    public void setCurvature(double curvature) {
        this.curvature = curvature;
    }

    public Distance2d getDistanceFromStart() {
        return distanceFromStart;
    }

    public void setDistanceFromStart(Distance2d distanceFromStart) {
        this.distanceFromStart = distanceFromStart;
    }
}
