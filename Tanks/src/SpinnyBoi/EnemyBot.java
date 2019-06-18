package SpinnyBoi;
import robocode.ScannedRobotEvent;
import robocode.AdvancedRobot;

public class EnemyBot {
    // stores the ScannedRobotEvent and additional information
    public String name;
    public double x, y, absoluteBearing, energy, bearing, distance, heading, velocity;
    //private ScannedRobotEvent robotEvent;
    public EnemyBot(String name, double x, double y, double energy, double distance, double velocity, double heading) {
        this.name = name;
        this.x = x;
        this.y = y;
        this.energy = energy;
        this.distance = distance;
        this.velocity = velocity;
        this.heading = heading;
    }
    public EnemyBot(ScannedRobotEvent event, double rX, double rY, double rHeading) {
        // this needs to be fixed - abstraction
        // also i want to rename RoboGravPoint...maybe
        this.absoluteBearing = rHeading + event.getBearingRadians();
        this.name = event.getName();
        this.x = Math.sin(absoluteBearing) * event.getDistance() + rX;
        this.y = Math.cos(absoluteBearing) * event.getDistance() + rY;
        this.energy = event.getEnergy();
        this.bearing = event.getBearing();
        this.distance = event.getDistance();
        this.heading = event.getHeading();
        this.velocity = event.getVelocity();
        //this.robotEvent = event;
    }

    @Override
    public String toString() {
        return "Name: " + name + "\tx: " + x + "\ty:" + y;
    }
}
