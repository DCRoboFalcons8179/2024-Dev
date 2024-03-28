// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.proto.System;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RequestShot extends Command {
  private Shooter shooter;
  private Double speed;

  // used to step to next step
  private Timer timer = new Timer();
  private boolean nextStep = false;


  public RequestShot(Shooter shooter) {
    this(shooter, Constants.ShooterConstants.shooterSpeed);

    this.shooter = shooter;
  }

  public RequestShot(Shooter shooter, Double speed) {
    addRequirements(shooter);

    this.shooter = shooter;
    this.speed = speed;
  }


  

  @Override
  public void initialize() {

    shooter.shoot();
    timer.reset();
    timer.start();

  }


  // Called when the command is initially scheduled.
  @Override
  public void execute() {

    shooter.shoot();

    // Timer - must make sure that we wait for the overshoot to settle out.
    if (timer.hasElapsed(0.6)) {
      nextStep = true;
    }

    if (shooter.isAtSpeed() && nextStep) {

      shooter.startFeedForward();

    }

    if (timer.hasElapsed(2.0)) {
      shooter.startFeedForward();
    }
 

  }

  @Override
  public void end(boolean interrupted) {
    
    // If interupted - aka the user takes their finger off the shot button
    // end() always run when stopped. boolean interrupted is the only thing that changes when the command is either interrupted or ended via isFinished()
    shooter.stop();


  }


  public boolean isFinished() {

    // if (timer.hasElapsed(2.0) && !shooter.hasRing()) {
    //   return true;
    //}


    return false;

    // TODO: Add smart thing to know when a ring has been shot

    // return speed.getAsDouble() < 10;


  }
}
