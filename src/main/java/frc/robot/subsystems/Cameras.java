package frc.robot.subsystems;

import com.fasterxml.jackson.databind.util.PrimitiveArrayBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Cameras extends SubsystemBase {

    private Swerve swerve = new Swerve();

    final double FRONT_CAM = 1;
    final double BACK_RIGHT_CAM = 0;
    final double BACK_LEFT_CAM = 2;

    public void cameraSetter(double angle) {

        // Joystick is 90 degrees turned clockwise idk why -Mason
        // Doing math to give the same offset to gyro in this code wahoo :(

        double gyro = swerve.getGyroYaw().getDegrees() + 90;

        double trueAngle = angle - gyro;

        if (trueAngle > 180) {
            trueAngle -= 180;
        } else if (trueAngle < -180) {
            trueAngle += 180;
        }

        SmartDashboard.putNumber("Cam Gyro", gyro);
        SmartDashboard.putNumber("Controller angle", angle);
        SmartDashboard.putNumber("Controller & Gyro angle", trueAngle);
        
        if (trueAngle < 90 && trueAngle > -45) {
            // Change values eventually, thanks
            SmartDashboard.putNumber("Left Cam Value", FRONT_CAM);
            SmartDashboard.putNumber("Right Cam Value", BACK_RIGHT_CAM);
        } else if (trueAngle > 90 && trueAngle > -135) {
            // Change values eventually, thanks
            SmartDashboard.putNumber("Left Cam Value", BACK_LEFT_CAM);
            SmartDashboard.putNumber("Right Cam Value", FRONT_CAM);
        } else if (trueAngle > -135 && trueAngle < -45) {
            // Change values eventually, thanks
            SmartDashboard.putNumber("Left Cam Value", BACK_LEFT_CAM);
            SmartDashboard.putNumber("Right Cam Value", BACK_RIGHT_CAM);
        }
    }
}
