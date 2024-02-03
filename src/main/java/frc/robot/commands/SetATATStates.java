// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ATAT;

public class SetATATStates extends Command {
  /** Creates a new SetATATStates. */
  ATAT atat;
  public SetATATStates(ATAT atat) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(atat);
    this.atat = atat;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    atat.setFrontPostPos(atat.getDesiredFrontPostPos());
    atat.setBackPostPos(atat.getDesiredBackPostPos());
    atat.setAngle(atat.getDesiredAngle());
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
