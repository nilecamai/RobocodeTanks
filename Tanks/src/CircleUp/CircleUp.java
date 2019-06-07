package CircleUp;

import robocode.*;

public class CircleUp extends Robot{

    // have variables to keep track of heading


    public void run() {
        while (true) {
            ahead(50);
            turnRight(10);
        }
    }

    @Override
    public void onScannedRobot(ScannedRobotEvent event) {
        fire(1);
    }
}
