// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ATAT;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RequestATATPose extends InstantCommand {
  private ATAT atat;
  private DoubleSupplier distF;
  private DoubleSupplier distB;
  private DoubleSupplier angle;
  public RequestATATPose(ATAT atat, double distF, double distB, double angle) {
    this.atat = atat;
    this.distF = ()-> distF;
    this.distB = ()-> distB;
    this.angle = ()-> angle;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public RequestATATPose(ATAT atat, DoubleSupplier distF, DoubleSupplier distB, DoubleSupplier angle) {
    this.atat = atat;
    this.distF = distF;
    this.distB = distB;
    this.angle = angle;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    atat.setDesiredFrontPostPos(distF.getAsDouble());
    atat.setDesiredBackPostPos(distB.getAsDouble());
    atat.setDesiredAngle(angle.getAsDouble());
  }
}
