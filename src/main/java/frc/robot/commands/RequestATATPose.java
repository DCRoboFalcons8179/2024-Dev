// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ATATPose;
import frc.robot.subsystems.ATAT;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RequestATATPose extends Command {  
  private boolean instant = true;
  private ATAT atat;
  private DoubleSupplier distF;
  private DoubleSupplier distB;
  private DoubleSupplier angle;


  
  
  /**
   * Instantly requests ATAT poses individually
   * 
   * @param atat ATAT object
   * @param distF how far the front needs to extend
   * @param distB how far the back needs to extend
   * @param angle which angle to set
   */
  public RequestATATPose(ATAT atat, double distF, double distB, double angle) {
    this.atat = atat;
    this.distF = ()-> distF;
    this.distB = ()-> distB;
    this.angle = ()-> angle;
    // Use addRequirements() here to declare subsystem dependencies.
  }
  
  /**
   * 
   * Requests ATAT poses individually, with an option to wait until it reaches its setpoint
   * 
   * @param atat
   * @param distF
   * @param distB
   * @param angle
   * @param instant
   */
  public RequestATATPose(ATAT atat, double distF, double distB, double angle, boolean instant) {
    this(atat, distF, distB, angle);
    this.instant = instant;
    if (!instant) addRequirements(atat);
  }

  /**
   * 
   * Instantly requests ATAT poses with suppliers.
   * 
   * @param atat
   * @param distF
   * @param distB
   * @param angle
   */
  public RequestATATPose(ATAT atat, DoubleSupplier distF, DoubleSupplier distB, DoubleSupplier angle) {
    this.atat = atat;
    this.distF = distF;
    this.distB = distB;
    this.angle = angle;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  /**
   * 
   * Instantly requests ATAT poses using pre-defined poses.
   * 
   * @param atat
   * @param pose
   */
  public RequestATATPose(ATAT atat, ATATPose pose) {
    this(atat, pose.getDistF(), pose.getDistB(), pose.getAngle());
  }

  /**
   * 
   * Requests ATAT poses using pre-defined poses, with an option to wait until it reaches its setpoint
   * 
   * @param atat
   * @param pose
   *
   */
  public RequestATATPose(ATAT atat, ATATPose pose, boolean instant) {
    this(atat, pose);
    this.instant = instant;
  }

  /**
   * 
   * Requests ATAT poses individually using suppliers, with an option to wait until it reaches its setpoint
   * 
   * @param atat
   * @param distF
   * @param distB
   * @param angle
   * @param instant
   */
  public RequestATATPose(ATAT atat, DoubleSupplier distF, DoubleSupplier distB, DoubleSupplier angle, boolean instant) {
    this(atat, distF, distB, angle);
    this.instant = instant;
    if (!instant) addRequirements(atat);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    atat.setDesiredFrontPostPos(distF.getAsDouble());
    atat.setDesiredBackPostPos(distB.getAsDouble());
    atat.setDesiredAngle(angle.getAsDouble());
  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {

    if (instant) {
      return true;
    }

    return Math.abs(atat.getFrontPostPos() - distF.getAsDouble()) < 0.2 &&
           Math.abs(atat.getBackPostPos() - distF.getAsDouble()) < 0.2 &&
           Math.abs(atat.getAngle() - angle.getAsDouble()) < 0.2;
  }
}
