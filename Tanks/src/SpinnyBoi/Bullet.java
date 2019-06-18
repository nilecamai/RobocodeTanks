package SpinnyBoi;

public class Bullet {

    private double x, y, xVel, yVel, firePower;
    public Bullet(double x, double y, double velocity, double headingRadians, double firePower) {
        this.x = x;
        this.y = y;
        this.xVel = velocity * Math.sin(headingRadians);
        this.yVel = velocity * Math.cos(headingRadians);
        this.firePower = firePower;
    }

    public void updatePosition() {
        x += xVel;
        y += yVel;
    }
}