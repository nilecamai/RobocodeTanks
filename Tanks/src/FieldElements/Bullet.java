package FieldElements;

import robocode.Rules;

public class Bullet extends GravPoint{

    private double xVel, yVel, startTime, startX, startY;
    public double velocity, heading;
    public Bullet(double x, double y, double heading, double energy, long startTime) {
        this.x = this.startX = x;
        this.y = this.startY = y;
        this.velocity = Rules.getBulletSpeed(energy);
        this.heading = heading;
        this.xVel = velocity * Math.sin(heading);
        this.yVel = velocity * Math.cos(heading);
        this.energy = energy;
        this.startTime = startTime;
    }

    public void updatePosition(long time) {
        x = startX + xVel * (time - startTime);
        y = startY + yVel * (time - startTime);
    }
}