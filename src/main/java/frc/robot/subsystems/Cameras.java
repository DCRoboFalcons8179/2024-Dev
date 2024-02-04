package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Cameras {
    public void cameraControllerLeft(int add) {

        double leftCam = SmartDashboard.getNumber("Left Camera Value", 0);

        SmartDashboard.putNumber("Left Camera Value", leftCam++);

    }
}
