// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class zeroGyro extends Command {
  /** Creates a new zeroGyro. */

  Swerve s_Swerve;
  /**
   * Zeroes the gyro for field-centric uses.
   * @param s_Swerve
   */
  public zeroGyro(Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    s_Swerve.zeroGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}