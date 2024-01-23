// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.LinearFilter;
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
  private Translation2d finalTranslationOffset;
  private Rotation3d rotationOffset;
  private Swerve s_Swerve;
  private Limelight limelight;
  private double rot;
  private double mag;
  LinearFilter magFilter;
  LinearFilter rotFilter;

  public ApproachTag(Swerve s_Swerve, Limelight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Swerve);
    this.s_Swerve = s_Swerve;
    this.limelight = limelight;
  }

  // Called when the command is initially scheduled.


  @Override
  public void initialize() {
    magFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
    rotFilter = LinearFilter.singlePoleIIR(0.2, 0.02);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    translationOffset = limelight.offsetFromTag();
    rotationOffset = limelight.rotationFromTag();

    //SmartDashboard offset logging
    SmartDashboard.putNumber("tX", translationOffset.getX());
    SmartDashboard.putNumber("tY", translationOffset.getY());
    SmartDashboard.putNumber("tZ", translationOffset.getZ());
    SmartDashboard.putNumber("roll", rotationOffset.getX());
    SmartDashboard.putNumber("pitch", rotationOffset.getY());
    SmartDashboard.putNumber("rz", limelight.getRobotRZ());

    // Mapping inputs to the trapizoidd speed controller]
    finalTranslationOffset = (new Translation2d(translationOffset.getZ() + 2, -translationOffset.getX()));
    
    Translation2d dir = finalTranslationOffset.div(finalTranslationOffset.getNorm());

    double mag = magFilter.calculate(finalTranslationOffset.getNorm());
    double rot = rotFilter.calculate(limelight.getRobotRZ());

    SmartDashboard.putNumber("mag", mag);
    SmartDashboard.putNumber("rot", rot);



    //mapping numbers
    if (mag > 1) {
      mag = 1;
    } else if (mag < 0.3) {
      mag = 0;
    } else {
      mag = Math.pow(mag, 2);
    }

     if (rot > 2) {
      rot = 1;
    } else if (rot < -2) {
      rot = -1;
    } else if (Math.abs(rot) < 0.6) {
      rot = 0;
    } else {
      
    }

    if (limelight.getTagId() == -1) mag = 0;

    //drive mapped values
    s_Swerve.drive(dir.times(mag).times(0.6),  -rot/5, false);

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
