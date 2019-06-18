package SpinnyBoi;
import robocode.*;

import java.util.ArrayList;
import java.util.Map;
import java.util.HashMap;
import java.util.Iterator;
import robocode.util.*;
/*
 * you want to have the radar spinning independent of the gun so you can actively be scanning while moving/shooting
 * there are three spinning thingies:
 *      the robot itself
 *      the gun
 *      the radar
 */

public class HashMapTest extends AdvancedRobot {

    public static final double CLOSE_QUARTERS_DISTANCE = 10;
    public static final double FIRING_DISTANCE = 500;
    public static final double WALL_AVOIDANCE_CONSTANT = 250000;
    public static final double CHANCE_OF_RANDOM_MOVE = 0.0/8.0;
    public static final double BOT_AVOIDANCE_CONSTANT = 25000; // used to be 500 but we had magic number 50 hanging out to determine power so.
    public static final double FIREPOWER = 1;
    //public ArrayList<EnemyBot> enemyBots = new ArrayList<EnemyBot>();
    public Map<String, EnemyBot> enemyBots = new HashMap<String, EnemyBot>();

    public void run() {

        setAdjustRadarForGunTurn(true); // sets radar independent of gun
        setAdjustRadarForRobotTurn(true); // sets radar independent of robot
        setAdjustGunForRobotTurn(true); // sets gun independent of robot

        setTurnRadarRight(Double.POSITIVE_INFINITY); // just yeetin spin it

        while (true) {            
            
            if (!randomMove(false)) {
                //fastAntiGravMove();
                antiGravMove();
                System.out.println("Executing antigrav move");
            }
            execute();
            predictiveShoot(getClosestEnemyBot());
            execute();
        }
        
    }

    @Override
    public void onScannedRobot(ScannedRobotEvent event) {
        updateEnemyBot(event);
        // determine firing algorithm AND ONLY SHOOT AT THE BOT IF IT'S UPDATED
        /*
        if (getClosestEnemyBot() != null) {
            if (getClosestEnemyBot().getName().equals(event.getName()) || true) {
                if (getClosestEnemyBot().getDistance() < CLOSE_QUARTERS_DISTANCE) {
                    shootAtClosest(FIREPOWER * 5);
                } else {
                    predictiveShoot(getClosestEnemyBot());
                }
            } else {
                trainGun(getClosestEnemyBot());
            }
        }
        execute();
        */
    }

    @Override
    public void onRobotDeath(RobotDeathEvent event) {
        enemyBots.remove(event.getName());
    }

    
    public void updateEnemyBot(ScannedRobotEvent event) {
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

        enemyBots.put(name, new EnemyBot(name, x, y, power, distance, velocity, heading));
        /*
        for (int i = 0; i < enemyBots.size(); i++) { // make sure we don't have a duplicate gravPoint/can update a gravPoint
            if (name.equals(enemyBots.get(i).name)) {
                enemyBots.set(i, new EnemyBot(name, x, y, power, distance, velocity, heading));
                System.out.println("Updated " + name);
                foundDuplicate = true;
                break;
            }
        }
        if (!foundDuplicate) {
            enemyBots.add(new EnemyBot(name, x, y, power, distance, velocity, heading));
            System.out.println("Added robot " + name);
            //enemyBots.add(new RoboGravPoint(event, this)); <- THIS HAS TO WORK TO MAKE EVERTHING EZZZ
        }
        */
    }

    /*
    public void updateEnemyBot(ScannedRobotEvent event) {
        String name = event.getName();
        boolean foundDuplicate = false;
        for (int i = 0; i < enemyBots.size(); i++) { // make sure we don't have a duplicate gravPoint/can update a gravPoint
            if (name.equals(enemyBots.get(i).name)) {
                enemyBots.set(i, new EnemyBot(event, getX(), getY(), getHeadingRadians()));
                System.out.println("Updated " + name);
                foundDuplicate = true;
                break;
            }
        }
        if (!foundDuplicate) {
            enemyBots.add(new EnemyBot(event, getX(), getY(), getHeadingRadians()));
            System.out.println("Added robot " + name);
        }
    }
    */

    public void fastAntiGravMove() {
        double xForce = 0, yForce = 0;

        for (Map.Entry<String, EnemyBot> entry : enemyBots.entrySet()) {
            double absBearing = Utils.normalAbsoluteAngle(Math.atan2(getX() - entry.getValue().x, getY() - entry.getValue().y));
            xForce += Math.sin(absBearing) / Math.pow(entry.getValue().distance, 2);
            yForce += Math.cos(absBearing) / Math.pow(entry.getValue().distance, 2);
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
        double xForce = 0;
        double yForce = 0;
        double magn;
        double dir;
        for (Map.Entry<String, EnemyBot> entry : enemyBots.entrySet()) {
            double dx, dy;
            dx = getX() - entry.getValue().x;
            dy = getY() - entry.getValue().y;
            magn = entry.getValue().energy * BOT_AVOIDANCE_CONSTANT / Math.pow(entry.getValue().distance, 2); // always positive
            dir = Math.atan2(dx, dy);
            xForce += Math.sin(dir) * magn;
            yForce += Math.cos(dir) * magn;
        }
        double xWallForce, yWallForce;
        xWallForce = WALL_AVOIDANCE_CONSTANT / Math.pow(getX(), 2) - WALL_AVOIDANCE_CONSTANT / Math.pow(getBattleFieldWidth() - getX(), 2);
        yWallForce = yForce += WALL_AVOIDANCE_CONSTANT / Math.pow(getY(), 2) - WALL_AVOIDANCE_CONSTANT / Math.pow(getBattleFieldHeight() - getY(), 2);
        xForce += getX() + xWallForce;
        yForce += getY() + yWallForce;
        //System.out.println("Going to (" + xComp + ", " + yComp);
        goTo(xForce, yForce);
    }

    public boolean randomMove(boolean definite) {
        if (getClosestEnemyBot() != null) {
            if ((definite && getClosestEnemyBot().distance > CLOSE_QUARTERS_DISTANCE) || Math.random() <= CHANCE_OF_RANDOM_MOVE) {
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

    public void trainGun(EnemyBot e) {
        double gunTurnAngle = Utils.normalRelativeAngle(Math.atan2(e.x - getX(), e.y - getY()) - getGunHeadingRadians());
        turnGunRightRadians(gunTurnAngle);
    }

    public void shootAtClosest(double firePower) {
        if (getClosestEnemyBot() != null) {
            EnemyBot e = getClosestEnemyBot();
            /*
            double gunTurnAngle = 0;
            gunTurnAngle = Utils.normalRelativeAngle(Math.atan2(e.getX() - getX(), e.getY() - getY()) - getGunHeadingRadians());
            System.out.println("Turning gun " + Math.toDegrees(gunTurnAngle) + " to shoot at " + e.getName() + " at (" + e.getX() + ", " + e.getY() + ")");
            turnGunRightRadians(gunTurnAngle);
            */
            trainGun(e);
            setFire(firePower);                   
        }
    }

    public void predictiveShoot(EnemyBot e) {
        final double ROBOT_DIMENSION = 16;
        if (e != null) {         
            final double firePower = FIRING_DISTANCE / e.distance;
            final double eX = e.x, eY = e.y, eV = e.velocity, eHd = e.heading;
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

                
                final double turnAngle = Utils.normalRelativeAngle(Math.atan2(eX2 - rX, eY2 - rY) - getGunHeadingRadians());
                setTurnGunRightRadians(turnAngle);
                setFire(firePower);
            }
        }
    }

    public void predictiveShoot(EnemyBot e, double firePower) {
        final double ROBOT_DIMENSION = 16;
        if (e != null) {
            final double eX = e.x, eY = e.y, eV = e.velocity, eHd = e.heading;
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

                final double turnAngle = Utils.normalRelativeAngle(Math.atan2(eX2 - rX, eY2 - rY) - getGunHeadingRadians());
                setTurnGunRightRadians(turnAngle);
                setFire(firePower);
            }
        }
    }

    private double limit(double value, double min, double max) {
        return Math.min(max, Math.max(min, value));
    }

    public EnemyBot getClosestEnemyBot() {
        double closestDistance = Double.POSITIVE_INFINITY;
        EnemyBot closest = null;
        for (Map.Entry<String, EnemyBot> entry : enemyBots.entrySet()) {
            if (entry.getValue().distance < closestDistance) {
                closest = entry.getValue();
                closestDistance = entry.getValue().distance;
            }
        }
        return closest;
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
