package RadarSpinner;

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

    @Override
    public String toString() {
        return "Name: " + name + "\tx: " + x + "\ty:" + y;
    }
}
