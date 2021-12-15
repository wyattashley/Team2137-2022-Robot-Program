package frc.robot.library.hardware.deadReckoning;

import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.robot.functions.io.xmlreader.Encoder;
import frc.robot.library.Constants;
import frc.robot.library.Distance2d;
import frc.robot.library.hardware.DriveTrain;
import org.ejml.simple.SimpleMatrix;

/**
 * Dead wheel tracking can not work on TeleOp!!
 *
 * If a robot is traveling at full speed circa de 13 fps in the 10ms refresh period the robot can travel a total of
 * 1.56in so any slight change in direction can not be detected properly but in straight lines in accurate.
 */
public class DeadWheelActiveTracking {

    private final CANCoder xCANCoder;
    private final CANCoder yCANCoder;

    private final double xConversionFactor = 1; //Counts for every inch
    private final double yConversionFactor = 1; //Counts for every inch

    private final Distance2d previousXReading = Distance2d.fromUnit(Distance2d.DistanceUnits.INCH, 0);
    private final Distance2d previousYReading = Distance2d.fromUnit(Distance2d.DistanceUnits.INCH, 0);


    private final Distance2d xValue = Distance2d.fromUnit(Distance2d.DistanceUnits.INCH, 0);
    private final Distance2d yValue = Distance2d.fromUnit(Distance2d.DistanceUnits.INCH, 0);

    public DeadWheelActiveTracking(Encoder xDirectionController, Encoder yDirectionController, Distance2d xOffset, Distance2d yOffset) {
        xCANCoder = new CANCoder(xDirectionController.getCANID());
        yCANCoder = new CANCoder(yDirectionController.getCANID());

        xCANCoder.setPosition(xOffset.getValue(Distance2d.DistanceUnits.INCH) * xConversionFactor);
        yCANCoder.setPosition(yOffset.getValue(Distance2d.DistanceUnits.INCH) * yConversionFactor);
    }

    public void updateTracking(Rotation2d robotHeading) {
        Distance2d changeInXDistance = getCurrentXReading().minus(previousXReading);
        Distance2d changeInYDistance = getCurrentYReading().minus(previousYReading);

        SimpleMatrix matrix = Constants.convertFrame(robotHeading.getDegrees(), Constants.createFrameMatrix(changeInXDistance, changeInYDistance, robotHeading));

        xValue.mutablePlus(matrix.get(0,0));
        yValue.mutablePlus(matrix.get(1,0));
    }

    public Distance2d getGlobalXReading() {
        return xValue;
    }

    public Distance2d getGlobalYReading() {
        return yValue;
    }

    private Distance2d getCurrentXReading() {
        return Distance2d.fromUnit(Distance2d.DistanceUnits.INCH, xCANCoder.getPosition() / xConversionFactor);
    }

    private Distance2d getCurrentYReading() {
        return Distance2d.fromUnit(Distance2d.DistanceUnits.INCH, yCANCoder.getPosition() / yConversionFactor);
    }
}
