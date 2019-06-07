package MichaelJackson;
import robocode.*;

public class MichaelJackson extends Robot {
    public void run() {
        while (true) {
            turnRight(5);
        }
    }

    @Override
    public void onScannedRobot(ScannedRobotEvent event) {
        turnRight(event.getBearing());
        double distance = event.getDistance();
        if (distance > 100) {
            ahead(distance - 40); // accurate firing distance
            scan();
        } else {
            fire(2);
            back(150);
        }
    }

    @Override
    public void onHitByBullet(HitByBulletEvent event) {
        back(100);
    }
}
