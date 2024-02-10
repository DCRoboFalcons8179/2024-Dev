package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Cameras {
    public void cameraControllerLeft(String direction) {

        double leftCam = SmartDashboard.getNumber("Left Cam Value", 0);

        switch (direction) {
            case "left":
                SmartDashboard.putNumber("Left Cam Value", leftCam += 1);
                break;
            case "right":
                SmartDashboard.putNumber("Left Cam Value", leftCam -= 1);
                break;
        }

    }

    public void cameraControllerRight(String direction) {

        double rightCam = SmartDashboard.getNumber("Right Cam Value", 0);

        switch (direction) {
            case "left":
                SmartDashboard.putNumber("Right Cam Value", rightCam += 1);
                break;
            case "right":
                SmartDashboard.putNumber("Right Cam Value", rightCam -= 1);
                break;
        }

    }
}
