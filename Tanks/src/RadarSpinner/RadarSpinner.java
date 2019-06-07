package RadarSpinner;
import robocode.*;

import java.awt.geom.Point2D;
/*
 * you want to have the radar spinning independent of the gun so you can actively be scanning while moving/shooting
 * there are three spinning thingies:
 *      the robot itself
 *      the gun
 *      the radar
 */

public class RadarSpinner extends Robot {

    public void run() {

        setAdjustRadarForGunTurn(false); // sets radar independent of gun
        setAdjustGunForRobotTurn(false); // sets radar independent of robot

        double x = getX();
        double y = getY();
        System.out.println(x);
        System.out.println(y);

        while (true) {
            turnRadarRight(Double.POSITIVE_INFINITY); // just yeetin spin it
        }
    }

    @Override
    public void onScannedRobot(ScannedRobotEvent event) {
        System.out.println(event.getName());
    }

    public void getEventCoordinates(ScannedRobotEvent e) { // the goal: take heading data, own coordinates, and distance to approximate coordinates of scanned event
        Point2D.Double p = new Point2D.Double();
    }
}
