package RadarSpinner;
import robocode.*;
import robocode.Robot;
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
            double dx, dy;
            p = gravPoint;
            dx = p.x - getX();
            dy = p.y - getY();
            System.out.println("Avoiding " + p.name + " located at (" + p.x + ", " + p.y + ")");
            magn = p.power / Math.pow(p.distance, 2); // always positive
            magn = 1;
            //System.out.println("Magnitude of " + p.name + ": " + magn);
            dir = normaliseBearing(Math.PI/2 - Math.atan2(getY() - p.y, getX() - p.x));
            dir = Math.atan2(dy, dx);
            xComp += Math.sin(dir) * magn;
            yComp += Math.cos(dir) * magn;
        }

        System.out.println("Moving " + xComp + " horizontally and " + yComp + " vertically.");
        xComp += getX();
        yComp += getY();
        System.out.println("Going to: " + xComp + " , " + yComp);
        goTo(xComp, yComp);
    }

    public double normaliseBearing(double angle) { // doesn't quite work as intended don't use this
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    public void goTo(double x, double y) {
        // System.out.println("I am at (" + getX() + ", " + getY() + ") and am going to (" + x + ", " + y + ")");
        double dx, dy;
        dx = x - getX();
        dy = y - getY();
        double angle = Math.toDegrees(Math.atan2(dx, dy));
        angle -= getHeading();
        turnRight(angle);
        double dist = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));
        ahead(dist);
        // System.out.println("Turning " + angle + " and going " + dist);
    }
}
