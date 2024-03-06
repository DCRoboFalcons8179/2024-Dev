// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ATAT;

public class SetATATStates extends Command {
  /** Creates a new SetATATStates. */
  private ATAT atat;
  public SetATATStates(ATAT atat) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.atat = atat;
    addRequirements(atat);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    atat.setFrontPostPos(atat.getDesiredFrontPostPos());

    if (atat.getAngle() > 50 && atat.getDesiredBackPostPos() < Constants.ATATConstants.lowAngleBackPostMinLength) {
      atat.setBackPostPos(Constants.ATATConstants.lowAngleBackPostMinLength);
    } else {
      atat.setBackPostPos(atat.getDesiredBackPostPos());
    }
    
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
