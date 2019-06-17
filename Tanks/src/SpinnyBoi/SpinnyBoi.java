package SpinnyBoi;
import robocode.*;

import java.util.ArrayList;
import robocode.util.*;
/*
 * you want to have the radar spinning independent of the gun so you can actively be scanning while moving/shooting
 * there are three spinning thingies:
 *      the robot itself
 *      the gun
 *      the radar
 */

public class SpinnyBoi extends AdvancedRobot {

    public static final double SAFE_DISTANCE = 400;
    public static final double FIRING_DISTANCE = 500;
    public static final double WALL_AVOIDANCE_CONSTANT = 250000;
    public static final double CHANCE_OF_RANDOM_MOVE = 0;
    public static final double BOT_AVOIDANCE_CONSTANT = 500;
    public static final double FIREPOWER = 1;
    public ArrayList<RoboGravPoint> gravPoints = new ArrayList<RoboGravPoint>();

    public void run() {

        setAdjustRadarForGunTurn(true); // sets radar independent of gun
        setAdjustRadarForRobotTurn(true); // sets radar independent of robot
        setAdjustGunForRobotTurn(true); // sets gun independent of robot

        setTurnRadarRight(Double.POSITIVE_INFINITY); // just yeetin spin it

        while (true) {            
            /*
            if (!randomMove(false)) {
                //fastAntiGravMove();
                antiGravMove();
                System.out.println("Executing antigrav move");
            }
            */
            antiGravMove();
            execute();
            //shootAtClosest();
            predictiveShoot(getClosestGravPoint());
            execute();
            //avoidWalls();
            //execute();
        }
        
    }

    @Override
    public void onScannedRobot(ScannedRobotEvent event) {
        updateGravPoint(event);
    }

    @Override
    public void onRobotDeath(RobotDeathEvent event) {
        for (int i = 0; i < gravPoints.size(); i++) {
            if (event.getName().equals(gravPoints.get(i).name)) {
                gravPoints.remove(i);
            }
        }
    }

    public void updateGravPoint(ScannedRobotEvent event) {
        String name = event.getName();
        // math
        double distance = event.getDistance();
        double enemyAngle = getHeadingRadians() + event.getBearingRadians();
        double x = Math.sin(enemyAngle) * distance + getX();
        double y = Math.cos(enemyAngle) * distance + getY();
        double power = event.getEnergy() * 50;
        double velocity = event.getVelocity();
        double heading = event.getHeadingRadians();
        
        boolean foundDuplicate = false;
        for (int i = 0; i < gravPoints.size(); i++) { // make sure we don't have a duplicate gravPoint/can update a gravPoint
            if (name.equals(gravPoints.get(i).name)) {
                gravPoints.set(i, new RoboGravPoint(name, x, y, power, distance, velocity, heading));
                System.out.println("Updated " + name);
                foundDuplicate = true;
                break;
            }
        }
        if (!foundDuplicate) {
            gravPoints.add(new RoboGravPoint(name, x, y, power, distance, velocity, heading));
            System.out.println("Added robot " + name);
            //gravPoints.add(new RoboGravPoint(event, this)); <- THIS HAS TO WORK TO MAKE EVERTHING EZZZ
        }
    }

    public void fastAntiGravMove() {
        double xForce = 0, yForce = 0;
        for (RoboGravPoint p: gravPoints) {
            double absBearing = Utils.normalAbsoluteAngle(Math.atan2(getX() - p.x,getY() - p.y));
            xForce += Math.sin(absBearing) / Math.pow(p.distance, 2);
            yForce += Math.cos(absBearing) / Math.pow(p.distance, 2);
        }
        // wall repulsion
        
        xForce += (WALL_AVOIDANCE_CONSTANT / getBattleFieldWidth()) / Math.pow(getX(), 2);
        yForce += (WALL_AVOIDANCE_CONSTANT / getBattleFieldWidth()) / Math.pow(getY(), 2);
        xForce -= (WALL_AVOIDANCE_CONSTANT / getBattleFieldHeight()) / Math.pow(getBattleFieldWidth() - getX(), 2);
        yForce -= (WALL_AVOIDANCE_CONSTANT / getBattleFieldHeight()) / Math.pow(getBattleFieldHeight() - getY(), 2);
        
        double angle = Math.atan2(xForce, yForce);
        goToVector(Double.POSITIVE_INFINITY, angle);
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
            dx = getX() - p.x;
            dy = getY() - p.y;
            magn = p.power * BOT_AVOIDANCE_CONSTANT / Math.pow(p.distance, 2); // always positive
            dir = Math.atan2(dx, dy);
            xComp += Math.sin(dir) * magn;
            yComp += Math.cos(dir) * magn;
        }

        /*
        double unitVectorX, unitVectorY;
        unitVectorX = xComp / Math.hypot(xComp, yComp);
        unitVectorY = yComp / Math.hypot(xComp, yComp);
        if (getX() + xComp * unitVectorX > getBattleFieldWidth() || getX() - xComp * unitVectorX < getBattleFieldWidth() ||
            getY() + yComp * unitVectorY > getBattleFieldHeight() || getY() - yComp * unitVectorY < getBattleFieldHeight()) {
                wallSmoothMove(WALL_AVOIDANCE_CONSTANT]);
        }
        */

        double xWallForce, yWallForce;
        xWallForce = WALL_AVOIDANCE_CONSTANT / Math.pow(getX(), 2) - WALL_AVOIDANCE_CONSTANT / Math.pow(getBattleFieldWidth() - getX(), 2);
        yWallForce = yComp += WALL_AVOIDANCE_CONSTANT / Math.pow(getY(), 2) - WALL_AVOIDANCE_CONSTANT / Math.pow(getBattleFieldHeight() - getY(), 2);
        /*
        xComp += WALL_AVOIDANCE_CONSTANT / Math.pow(getX(), 2);
        yComp += WALL_AVOIDANCE_CONSTANT / Math.pow(getY(), 2);
        xComp -= WALL_AVOIDANCE_CONSTANT / Math.pow(getBattleFieldWidth() - getX(), 2);
        yComp -= WALL_AVOIDANCE_CONSTANT / Math.pow(getBattleFieldHeight() - getY(), 2);
        */
        System.out.println("Walls are exerting forces:" + xWallForce + " and " + yWallForce);

        xComp += getX() + xWallForce;
        yComp += getY() + yWallForce;
        System.out.println("Going to (" + xComp + ", " + yComp);
        goTo(xComp, yComp);
    }

    public boolean randomMove(boolean definite) {
        if (getClosestGravPoint() != null) {
            if ((definite && getClosestGravPoint().distance > SAFE_DISTANCE) || Math.random() <= CHANCE_OF_RANDOM_MOVE) {
                double xComp, yComp;
                if (getX() >= getBattleFieldWidth() / 2) {
                    xComp = Math.random() * getBattleFieldWidth() / 2 - WALL_AVOIDANCE_CONSTANT;
                } else {
                    xComp = -Math.random() * getBattleFieldWidth() / 2 - WALL_AVOIDANCE_CONSTANT;
                }
                if (getY() >= getBattleFieldHeight() / 2) {
                    yComp = Math.random() * getBattleFieldHeight() / 2 - WALL_AVOIDANCE_CONSTANT;
                } else {
                    yComp = -Math.random() * getBattleFieldHeight() / 2 - WALL_AVOIDANCE_CONSTANT;
                }
                xComp += getX();
                yComp += getY();
                goTo(xComp, yComp);
                System.out.println("Executing random move");
                return true;                    
            }            
        }
        return false;
    }

    public void huntBot() {
        if (gravPoints.size() > 0) { // hunts the first one
            goTo(gravPoints.get(0).x, gravPoints.get(0).y);
        }
    }

    public void shootAtClosest() {
        if (getClosestGravPoint() != null) {
            RoboGravPoint p = getClosestGravPoint();
            if (p.distance < SAFE_DISTANCE) {
                double xVel = p.velocity * Math.sin(p.heading), yVel = p.velocity * Math.cos(p.heading), gunTurnAngle = 0;
                gunTurnAngle = Utils.normalRelativeAngle(Math.atan2(p.x - getX(), p.y - getY()) - getGunHeadingRadians());
                System.out.println("Turning gun " + Math.toDegrees(gunTurnAngle) + " to shoot at " + p.name + " at (" + p.x + ", " + p.y + ")");
                turnGunRightRadians(gunTurnAngle);
                setFire(FIREPOWER);
            }            
        }
    }

    public void predictiveShoot(RoboGravPoint p) {
        final double ROBOT_DIMENSION = 16;
        if (p != null) {         
            final double firePower = FIRING_DISTANCE / p.distance;
            final double eX = p.x, eY = p.y, eV = p.velocity, eHd = p.heading;
            final double eAbsBearing = Utils.normalAbsoluteAngle(Math.atan2(eX - getX(), eY - getY()));
            final double rX = getX(), rY = getY(), bV = Rules.getBulletSpeed(firePower);
            final double A = (eX - rX) / bV;
            final double B = eV / bV * Math.sin(eHd);
            final double C = (eY - rY) / bV;
            final double D = eV / bV * Math.cos(eHd);
            // a*(1/t)^2 + b*(1/t) + c = 0
            final double a = A * A + C * C;
            final double b = 2 * (A * B + C * D);
            final double c = (B * B + D * D - 1);
            final double disc = b * b - 4 * a * c;
            if (disc >= 0) {
                final double t1 = 2 * a / (-b - Math.sqrt(disc));
                final double t2 = 2 * a / (-b + Math.sqrt(disc));
                final double t = Math.min(t1, t2) >= 0 ? Math.min(t1, t2) : Math.max(t1, t2);
                final double eX2 = limit(eX + eV * t * Math.sin(eHd), ROBOT_DIMENSION / 2, getBattleFieldWidth() - ROBOT_DIMENSION / 2);
                final double eY2 = limit(eY + eV * t * Math.cos(eHd), ROBOT_DIMENSION / 2, getBattleFieldHeight() - ROBOT_DIMENSION / 2);
                turnGunRightRadians(Utils.normalRelativeAngle(Math.atan2(eX2 - rX, eY2 - rY) - getGunHeadingRadians()));
                setFire(firePower);
            }
        }
    }

    public void predictiveShoot(RoboGravPoint p, double firePower) {
        final double ROBOT_DIMENSION = 16;
        if (p != null) {
            final double eX = p.x, eY = p.y, eV = p.velocity, eHd = p.heading;
            final double eAbsBearing = Utils.normalAbsoluteAngle(Math.atan2(eX - getX(), eY - getY()));
            final double rX = getX(), rY = getY(), bV = Rules.getBulletSpeed(firePower);
            final double A = (eX - rX) / bV;
            final double B = eV / bV * Math.sin(eHd);
            final double C = (eY - rY) / bV;
            final double D = eV / bV * Math.cos(eHd);
            // a*(1/t)^2 + b*(1/t) + c = 0
            final double a = A * A + C * C;
            final double b = 2 * (A * B + C * D);
            final double c = (B * B + D * D - 1);
            final double disc = b * b - 4 * a * c;
            if (disc >= 0) {
                final double t1 = 2 * a / (-b - Math.sqrt(disc));
                final double t2 = 2 * a / (-b + Math.sqrt(disc));
                final double t = Math.min(t1, t2) >= 0 ? Math.min(t1, t2) : Math.max(t1, t2);
                final double eX2 = limit(eX + eV * t * Math.sin(eHd), ROBOT_DIMENSION / 2, getBattleFieldWidth() - ROBOT_DIMENSION / 2);
                final double eY2 = limit(eY + eV * t * Math.cos(eHd), ROBOT_DIMENSION / 2, getBattleFieldHeight() - ROBOT_DIMENSION / 2);
                turnGunRightRadians(Utils.normalRelativeAngle(Math.atan2(eX2 - rX, eY2 - rY) - getGunHeadingRadians()));
                setFire(firePower);
            }
        }
    }

    private double limit(double value, double min, double max) {
        return Math.min(max, Math.max(min, value));
    }

    public RoboGravPoint getClosestGravPoint() {
        double closestDistance = Double.POSITIVE_INFINITY;
        RoboGravPoint closest = null;
        for (RoboGravPoint p: gravPoints) {
            if (p.distance < closestDistance) {
                closest = p;
                closestDistance = p.distance;
            }
        }
        return closest;
    }

    public void avoidWalls() {
        if (getX() < WALL_AVOIDANCE_CONSTANT || getBattleFieldWidth() - getX() > WALL_AVOIDANCE_CONSTANT ||
        getY() < WALL_AVOIDANCE_CONSTANT || getBattleFieldHeight() - getY() > WALL_AVOIDANCE_CONSTANT); {
            //turnRightRadians(Math.PI / 2);
            double radius = 2 * WALL_AVOIDANCE_CONSTANT / (2 + Math.sqrt(2)); // worst cast scenario where we're going 45 degrees into a corner
            //setTurnRightRadians(getVelocity() / radius);
            turnLeftRadians(Math.atan(Math.tan(getHeadingRadians())));
            System.out.println("Avoiding a wall");
        }
    }

    public void wallSmoothMove(double w) {
        setTurnRightRadians(getVelocity() * Math.PI / (2 * w));
    }

    public void goTo(double x, double y) {
        // System.out.println("I am at (" + getX() + ", " + getY() + ") and am going to (" + x + ", " + y + ")");
        double dx, dy;
        dx = x - getX();
        dy = y - getY();
        double targetAngle = Utils.normalRelativeAngle(Math.atan2(dx, dy) - getHeadingRadians());
        double dist = Math.hypot(dx, dy);
        double turnAngle = Math.atan(Math.tan(targetAngle));
        setTurnRightRadians(turnAngle);
        if (turnAngle == targetAngle) {
            setAhead(dist);
        } else {
            setBack(dist);
        }
    }

    public void goToVector(double magn, double dir) { // takes radians
        dir = Utils.normalRelativeAngle(dir - getHeadingRadians());
        double turnAngle = Math.atan(Math.tan(dir));
        setTurnRightRadians(turnAngle);
        if (turnAngle == dir) {
            setAhead(magn);
        } else {
            setBack(magn);
        }
    }
}
