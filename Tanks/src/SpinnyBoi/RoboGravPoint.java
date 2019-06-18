package SpinnyBoi;
import robocode.ScannedRobotEvent;
import robocode.AdvancedRobot;

public class RoboGravPoint {
    // idea is to store robot name to identify it and store its coordinates for mapping
    public String name;
    public double x, y, power, distance, velocity, heading;
    public RoboGravPoint(String name, double x, double y, double power, double distance, double velocity, double heading) {
        this.name = name;
        this.x = x;
        this.y = y;
        this.power = power;
        this.distance = distance;
        this.velocity = velocity;
        this.heading = heading;
    }
    public RoboGravPoint(ScannedRobotEvent event, AdvancedRobot robot) {
        // this needs to be fixed - abstraction
        // also i want to rename RoboGravPoint...maybe
        double enemyAngle = robot.getHeadingRadians() + event.getBearingRadians();
        this.name = event.getName();
        this.distance = event.getDistance();
        this.x = Math.sin(enemyAngle) * distance + robot.getX();
        this.y = Math.cos(enemyAngle) * distance + robot.getY();
        this.power = event.getEnergy() * 50;
        this.velocity = event.getVelocity();
        this.heading = event.getHeadingRadians();
    }

    @Override
    public String toString() {
        return "Name: " + name + "\tx: " + x + "\ty:" + y;
    }
}
