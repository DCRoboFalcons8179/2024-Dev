// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ATAT;

public class pullUpOnChain extends Command {
  /** Creates a new pullUpOnChain. */

  ATAT atat;

  public pullUpOnChain(ATAT atat) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.atat = atat;
    addRequirements(atat);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    atat.pullUp();


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    // From Tim - We need to see if we have to make the ending location the setpoint (probably)
    // or the zero position the setpoint. That, or do we ever end? If we get software limits in,
    // we should be good to go.

    atat.makeThisTheSetpoint();
    

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
