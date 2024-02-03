package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Cameras {
    public double cameraController() {
    
        SmartDashboard.putNumber("test", 40);

        double test = SmartDashboard.getNumber("test", 0);

        return test;

    }
}
