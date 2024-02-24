package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Cameras extends SubsystemBase {

    private Swerve swerve = new Swerve();

    final double FRONT_CAM = 2;
    final double BACK_RIGHT_CAM = 0;
    final double BACK_LEFT_CAM = 4;

    public void cameraSetter(double angle) {

        // Joystick is 90 degrees turned clockwise idk why -Mason

        // Angle is the controller joystick angle

        double gyro = swerve.getGyroYaw().getDegrees();

        if (angle == 0) {
            return;
        }

        // Which zone the bot is in
        int zone;

        // double trueAngle = angle + gyro;

        double heading = swerve.getHeading().getDegrees();

        SmartDashboard.putNumber("Heading", heading);

        SmartDashboard.putNumber("Cam Gyro", gyro);
        SmartDashboard.putNumber("Controller angle", angle);
        // SmartDashboard.putNumber("Controller & Gyro angle", trueAngle);

        if (angle > 90 && angle > -135) {
            // Change values eventually, thanks
            zone = 1;
            DashBoard("Zone", (double) zone);
        } else if (angle < 90 && angle > -45) {
            // Change values eventually, thanks
            zone = 2;
            DashBoard("Zone", (double) zone);
        } else if (angle > -135 && angle < -45) {
            // Change values eventually, thanks
            zone = 3;
            DashBoard("Zone", (double) zone);
        }
    }

    private void DashBoard(String key, Double value) {
        SmartDashboard.putNumber(key, value);
    }
}
