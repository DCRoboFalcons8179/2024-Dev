package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Cameras extends SubsystemBase {

    final double FRONT_CAM = 1;
    final double BACK_RIGHT_CAM = 0;
    final double BACK_LEFT_CAM = 2;

    public void cameraSetter(double angle) {

        if (angle == 0) {
            return;
        }

        if (angle < 90 && angle > -45) {
            // Change values eventually, thanks
            SmartDashboard.putNumber("Left Cam Value", FRONT_CAM);
            SmartDashboard.putNumber("Right Cam Value", BACK_RIGHT_CAM);
        } else if (angle > 90 && angle > -135) {
            // Change values eventually, thanks
            SmartDashboard.putNumber("Left Cam Value", BACK_LEFT_CAM);
            SmartDashboard.putNumber("Right Cam Value", FRONT_CAM);
        } else if (angle > -135 && angle < -45) {
            // Change values eventually, thanks
            SmartDashboard.putNumber("Left Cam Value", BACK_LEFT_CAM);
            SmartDashboard.putNumber("Right Cam Value", BACK_RIGHT_CAM);
        }
    }

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
