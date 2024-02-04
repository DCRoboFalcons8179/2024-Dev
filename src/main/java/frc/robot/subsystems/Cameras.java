package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Cameras {
    public int cameraControllerLeft() {


        // SmartDashboard.putNumber("test", 40);

        double leftCam = SmartDashboard.getNumber("Left Camera Value", 0);

        SmartDashboard.putNumber("Left Camera Value", 4);

        int leftCamInt = (int) leftCam++;

        return leftCamInt;

    }
}
