// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;

public class ApproachTag extends Command {
  /** Creates a new ApproachTag. */
  private Translation3d translationOffset;
  private Rotation3d rotationOffset;
  private Swerve s_Swerve;
  private Limelight limelight;

  public ApproachTag(Swerve s_Swerve, Limelight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Swerve);
    this.s_Swerve = s_Swerve;
    this.limelight = limelight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    translationOffset = limelight.offsetFromTag();
    rotationOffset = limelight.rotationFromTag();

    SmartDashboard.putNumber("tX", translationOffset.getX());
    SmartDashboard.putNumber("tY", translationOffset.getY());
    SmartDashboard.putNumber("tZ", translationOffset.getZ());
    SmartDashboard.putNumber("roll", rotationOffset.getX());
    SmartDashboard.putNumber("pitch", rotationOffset.getY());
    SmartDashboard.putNumber("yaw", rotationOffset.getZ());

    // Mapping inputs to the trapizoidd speed controller]
    Translation2d offset = (new Translation2d(translationOffset.getZ() + 2, -translationOffset.getX()));
    double mag = offset.getNorm();
    Translation2d dir = offset.div(mag);

    if (mag > 0.5) {
      mag = 0.5;
    } else if (mag < 0.2) {
      mag = 0;
    } else {
      mag = 0.2;
    }

    double rotOff = rotationOffset.getZ();

     if (rotOff > 0.5) {
      rotOff = 0.5;
    } else if (mag < -0.5) {
      rotOff = -0.5;
    } else if (Math.abs(rotOff) < 0.2) {
      rotOff = 0;
    } else {
      rotOff = Math.pow(rotOff, 3);
    }


    s_Swerve.drive(dir.times(mag),  -rotOff / 5, false);
      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
