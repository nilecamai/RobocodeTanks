package RadarSpinner;
import robocode.*;
import robocode.Robot;

import java.awt.*;
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
            System.out.println("Heading: " + getHeading() + "\n(" + x + ", " + y + ")");
            //turnRadarRight(360);
            for (RoboGravPoint gravPoint: gravPoints) {
                //System.out.println(gravPoint);
            }
            //antiGravMove();
            goTo(0,0);
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
        double power = event.getEnergy() * 10000000;
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
            System.out.println("Magnitude of " + p.name + ": " + magn);
            dir = normaliseBearing(Math.PI/2 - Math.atan2(getY() - p.y, getX() - p.x));
            dir = Math.atan2(getY() - p.y, getX() - p.x);
            xComp += Math.sin(dir) * magn;
            yComp += Math.cos(dir) * magn;
        }
        xComp += getX();
        yComp += getY();
        System.out.println("Going to: " + xComp + " , " + yComp);
        goTo(xComp, yComp);
    }

    public double normaliseBearing(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    public void goTo(double x, double y) {
        double angle = Math.toDegrees(Math.atan2(y - getY(), x - getX()));
        if (x > getX() && y < getY()) {
            angle += 90;
        } else if (x < getX() && y < getY()) {
            angle += 180;
        } else if (x < getX() && y > getY()) {
            angle += 270;
        }
        angle -= getHeading();
        turnRight(angle);
        double dist = Math.sqrt(Math.pow(y - getY(), 2) + Math.pow(x - getX(), 2));
        System.out.println("Turning " + angle + " and going " + dist);
        ahead(dist);
    }

    public void getEventCoordinates(ScannedRobotEvent e) { // the goal: take heading data, own coordinates, and distance to approximate coordinates of scanned event
        Point2D.Double p = new Point2D.Double();
    }
}
