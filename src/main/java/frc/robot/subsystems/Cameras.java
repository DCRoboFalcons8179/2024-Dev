package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Cameras extends SubsystemBase {

    final double FRONT_CAM = 2;
    final double BACK_RIGHT_CAM = 0;
    final double BACK_LEFT_CAM = 4;
    boolean camsKilled = false;

    public void cameraKiller() {
        SmartDashboard.putNumber("Robot Zone", 50);
        camsKilled = false;
    }

    public void cameraSaver() {
        camsKilled = true;
    }

    public void cameraToggler() {
        camsKilled = !camsKilled;

        if (camsKilled) SmartDashboard.putNumber("Robot Zone", 50);
    }

    public void topCam() {
        SmartDashboard.putNumber("Robot Zone", 1);
    }

    public void bottomCam() {
        SmartDashboard.putNumber("Robot Zone", 2);
    }

    public void cameraSetter(double angle, Swerve swerve) {
        if(camsKilled) return;

        // Joystick is 90 degrees turned clockwise idk why -Mason

        // Angle is the controller joystick angle

        double gyro = swerve.getGyroYaw().getDegrees();

        // Which zone the bot is in
        int zone = 1;

        // double trueAngle = angle + gyro;

        Rotation2d trueAngle = Rotation2d.fromDegrees(gyro).plus(Rotation2d.fromDegrees(angle))
                .minus(Rotation2d.fromDegrees(90));

        double heading = swerve.getHeading().getDegrees();

        SmartDashboard.putNumber("Heading", heading);

        SmartDashboard.putNumber("Cam Gyro", gyro);
        SmartDashboard.putNumber("Controller angle", angle);

        if (angle == 0) {
            return;
        }
        // SmartDashboard.putNumber("Controller & Gyro angle", trueAngle);
        gyro = trueAngle.getDegrees();
        if (gyro < 0 && gyro > -135) {
            // Change values eventually, thanks
            zone = 1;
            DashBoard("Robot Zone", zone);
        } else if (gyro > 0 && gyro < 135) {
            // Change values eventually, thanks
            zone = 2;
            DashBoard("Robot Zone", zone);
        } else if (gyro < -135 || gyro > 135) {
            // Change values eventually, thanks
            zone = 3;
            DashBoard("Robot Zone", zone);
        }

        // if (angle < -135 || angle > 90) {
        // // Change values eventually, thanks
        // zone = 1;
        // DashBoard("Controller Zone", zone);
        // } else if (angle < 90 && angle > -45) {
        // // Change values eventually, thanks
        // zone = 2;
        // DashBoard("Controller Zone", zone);
        // } else if (angle > -135 && angle < -45) {
        // // Change values eventually, thanks
        // zone = 3;
        // DashBoard("Controller Zone", zone);
        // }
    }

    private void DashBoard(String key, double value) {
        SmartDashboard.putNumber(key, value);
    }
}
