package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.math.Filter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Cameras;

public class CameraPublisher extends Command {
    private Cameras camera;
    private Swerve swerve;

    private DoubleSupplier driverY;

    private DoubleSupplier driverX;

    public CameraPublisher(Cameras cameras, Swerve swerve, DoubleSupplier driverY, DoubleSupplier driverX) {
        this.swerve = swerve;
        this.camera = cameras;
        this.driverY = driverY;
        this.driverX = driverX;

        addRequirements(cameras);
    }

    @Override
    public void execute() {
        var y = driverY.getAsDouble();
        var x = driverX.getAsDouble();

        x = Filter.deadband(x, 0.1);

        y = Filter.deadband(y, 0.1);

        // In radians
        double theta = Math.atan2(y, x);

        var rot = Rotation2d.fromRadians(theta);

        SmartDashboard.putNumber("Controller Angle Radians", rot.getRadians());
        SmartDashboard.putNumber("Controller Angle Degrees", rot.getDegrees());
        SmartDashboard.putNumber("Controller Angle Y", y);
        SmartDashboard.putNumber("Controller Angle X", x);

        camera.cameraSetter(rot.getDegrees(), swerve);
    }
}
