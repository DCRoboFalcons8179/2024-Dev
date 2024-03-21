// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RequestBeaterBarSetSpeed extends Command {
  private Shooter shooter;
  private double speed;
  private boolean waitForRing = false;

  /**
   * Requests a percent output on the beater bar.
   * 
   * @param shooter
   * @param speed
   */
  public RequestBeaterBarSetSpeed(Shooter shooter, double speed) {
    this.speed = speed;
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  /**
   * Requests a percent output on the beater bar. Option to wait for the beater bar to pick up a ring.
   * @param shooter
   * @param speed
   * @param waitForRing
   */
  public RequestBeaterBarSetSpeed(Shooter shooter, double speed, boolean waitForRing) {
    this(shooter, speed);
    this.waitForRing = waitForRing;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("robot set bb speed to " + speed);
    shooter.setBeaterBarSpeed(speed);
  }

  @Override
  public void execute() {
    
  }

  @Override
  public void end(boolean interrupted) {
    if (waitForRing) {
      shooter.stopBeaterbar();
    }
  }

  @Override
  public boolean isFinished() {
    if (waitForRing) {
      return shooter.hasRing();
    }
    return true;
  }
  
}
