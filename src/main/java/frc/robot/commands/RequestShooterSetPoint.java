// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RequestShooterSetPoint extends Command {
  private Shooter shooter;
  private DoubleSupplier speed;
  private boolean instant;

  public RequestShooterSetPoint(Shooter shooter, DoubleSupplier speed) {
    this.shooter = shooter;
    this.speed = speed;
  }
  
  public RequestShooterSetPoint(Shooter shooter, double speed) {
    this(shooter, () -> speed);
    this.instant = true;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void execute() {
    shooter.setShooterSpeed(speed.getAsDouble());
  }

  public boolean isFinished() {
    if (instant) return true;
    return speed.getAsDouble() < 10;
  }
}
