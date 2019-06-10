package RadarSpinner;
import robocode.*;

import java.awt.geom.Point2D;
import java.util.ArrayList;
/*
 * you want to have the radar spinning independent of the gun so you can actively be scanning while moving/shooting
 * there are three spinning thingies:
 *      the robot itself
 *      the gun
 *      the radar
 */

public class RadarSpinner extends Robot {

    public ArrayList<RoboGravPoint> gravPoints = new ArrayList<RoboGravPoint>();

    public void run() {

        setAdjustRadarForGunTurn(false); // sets radar independent of gun
        setAdjustGunForRobotTurn(false); // sets radar independent of robot

        double x = getX();
        double y = getY();

        while (true) {
            //turnRadarRight(Double.POSITIVE_INFINITY); // just yeetin spin it
            turnRadarRight(360);
            for (RoboGravPoint gravPoint: gravPoints) {
                System.out.println(gravPoint);
            }
            antiGravMove();
        }
    }

    @Override
    public void onScannedRobot(ScannedRobotEvent event) {
        String name = event.getName();
        for (RoboGravPoint gravPoint: gravPoints) { // make sure we don't have a duplicate gravPoint/can update a gravPoint
            if (name.equals(gravPoint.name)) {
                System.out.println("Duplicate bot '" + name + "' scanned!");
                return;
            }
        }
        System.out.println("New bot '" + name + "' scanned!");
        // trig
        double distance = event.getDistance();
        double enemyAngle = Math.toRadians(getHeading() + event.getBearing());
        double x = Math.sin(enemyAngle) * distance;
        double y = Math.cos(enemyAngle) * distance;
        double power = 1; // magic number for power
        gravPoints.add(new RoboGravPoint(name, x, y, power, distance));
    }

    public void antiGravMove() {
        double xComp = 0;
        double yComp = 0;
        double magn;
        double dir;
        RoboGravPoint p;

        for (RoboGravPoint gravPoint: gravPoints) {
            p = gravPoint;
            magn = p.power / Math.pow(p.distance, 2);
            System.out.println(magn);
            dir = normaliseBearing(Math.PI/2 - Math.atan2(getY() - p.y, getX() - p.x));
            System.out.println(dir);
            xComp += Math.sin(dir) * magn;
            yComp += Math.cos(dir) * magn;
            goTo(xComp, yComp);
        }
    }

    public double normaliseBearing(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    public void goTo(double x, double y) {
        double angle = Math.toDegrees(Math.atan2(y - getY(), x - getX())) - getHeading();
        turnRight(angle);
        ahead(Math.sqrt(Math.pow(y - getY(), 2) + Math.pow(x - getX(), 2)));
    }

    public void getEventCoordinates(ScannedRobotEvent e) { // the goal: take heading data, own coordinates, and distance to approximate coordinates of scanned event
        Point2D.Double p = new Point2D.Double();
    }
}
