package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Cameras extends SubsystemBase {

    private Swerve swerve = new Swerve();

    final double FRONT_CAM = 1;
    final double BACK_RIGHT_CAM = 0;
    final double BACK_LEFT_CAM = 2;

    public void cameraSetter(double angle) {

        if (angle == 0) {
            return;
        }

        double trueAngle = angle + swerve.getGyroYaw().getDegrees();

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
