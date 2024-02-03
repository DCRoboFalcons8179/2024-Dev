// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.lib.math.Filter;
import frc.robot.subsystems.Limelight;

public class ApproachTag extends Command {
  /** Creates a new ApproachTag. */
  private Translation3d translationOffset;
  private Translation2d finalTranslationOffset;
  private Swerve s_Swerve;
  private Limelight limelight;
  private double rot; // [-60, 60] else it loses the tag **degrees
  private double mag; // [0, inf] if infinite resolution
  LinearFilter magFilter;
  LinearFilter rotFilter;

  private final double MAX_MAG; //[0, inf]
  private final double MAX_ROT; //[0, inf]
  private final double MAX_TRANSLATION_SPEED;
  private final double MAX_ROTATION_SPEED;   
  private final double TRANSLATION_DEADBAND; //standardized to 1
  private final double ROTATION_DEADBAND;    //standardized to 1
  private final int MAX_CYCLES_WITHOUT_TAG;
  private final double OFFSET_X; //offset from tag, negative = left
  private final double OFFSET_Z; //offset from tag, negative = into tag
  private final double OFFSET_RY;
  private final boolean SWING_WIDE;


  /**
   * <p>s_Swerve = Swerve object
   * 
   * <p>limelight = limelight object
   *
   * <p>MAX_MAGNITUDE = when the robot should start curving speed. D: double [0, inf]
   * 
   * <p>MAX_ROT = when the robot should start slowing down rotation. D: double [0, inf]
   * 
   * <p>MAX_TRANSLATION_SPEED (meters / second). D: double [0, inf]
   * 
   * <p>MAX_ROTATION_SPEED (degrees / second). D: double [0, inf]
   * 
   * ><p>TRANSLATION_DEADBAND = how far from the desired distance from the tag the bot can be (meters). D: double [0, inf]
   * 
   * <p>ROTATION_DEADBAND = how far off from the desired rotation the bot can be (degrees). D: double [0, inf]
   * 
   * <p>MAX_CYCLES_WITHOUT_TAG = how many cycles the command can run without seeing a tag before it stops. Prevents the robot from losing the tag for a few frames then stopping, but can make the robot keep moving when it actually loses sight of a tag. D: int [0, inf]
   * 
   * <p>OFFSET_FROM_TAG = offset from tag, (X, Z) (meters). negative x is to the left, negative z is into the stage. D: Translation2d([-inf, inf], [-inf, inf])
   * 
   * <p>ROTATION_FROM_TAG = offset from tag, negative = counterclockwise. D: double [-inf, inf]
   * 
   * <p>Swings wide if it is in robot-centric (due to desired vector being offset by the angle the robot is facing to the tag). Not yet ready to support field centric.
   */
  public ApproachTag(Swerve s_Swerve, Limelight limelight, double MAX_MAG, double MAX_ROT, double MAX_TRANSLATION_SPEED, double MAX_ROTATION_SPEED, double TRANSLATION_DEADBAND, double ROTATION_DEADBAND, int MAX_CYCLES_WITHOUT_TAG, Translation2d OFFSET_FROM_TAG, double ROTATION_FROM_TAG, boolean SWING_WIDE) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    addRequirements(s_Swerve);

    //TrapezoidProfile.Constraints() ?
    this.s_Swerve               = s_Swerve;
    this.limelight              = limelight;
    this.MAX_MAG                = MAX_MAG;                              //default 2 (double), distance from desired position when the robot starts ramping down speed
    this.MAX_ROT                = MAX_ROT;                              //default 8 (double), rotation from desired orientation, when the robot starts ramping down rotation speed
    this.MAX_TRANSLATION_SPEED  = MAX_TRANSLATION_SPEED;                //max translation speed of the robot (m/s)
    this.MAX_ROTATION_SPEED     = MAX_ROTATION_SPEED;                   //max rotation speed of the robot (deg/s)
    this.TRANSLATION_DEADBAND   = TRANSLATION_DEADBAND  / this.MAX_MAG; //allowed distance from desired magnitude, prevents shaking
    this.ROTATION_DEADBAND      = ROTATION_DEADBAND     / this.MAX_ROT; //allowed rotation from desired rotation, prevents shaking 
    this.MAX_CYCLES_WITHOUT_TAG = MAX_CYCLES_WITHOUT_TAG;               //allowed cycles the command can run without seeing a tag, ends the program if exceeded. Prevents the robot from ending the command as soon as it loses a tag (e.g. if the tag is too blurry for a frame and can't be found), but may cause the robot to move too far if it no longer has a tag in view. 
    this.OFFSET_X               = OFFSET_FROM_TAG.getX();               //values immediately pulled from Translation2d to prevent continuous calls
    this.OFFSET_Z               = OFFSET_FROM_TAG.getY();               //^^^
    this.OFFSET_RY              = ROTATION_FROM_TAG;
    this.SWING_WIDE             = SWING_WIDE;

  }

  // Called when the command is initially scheduled.


  @Override
  public void initialize() {
    magFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
    rotFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
  }

  // Called every time the scheduler runs while the command is scheduled.
  private int cyclesWithoutTag = 0;

  @Override
  public void execute() {

    translationOffset = limelight.offsetFromTag();

    //Vector2, to desired position
    finalTranslationOffset = (new Translation2d(translationOffset.getZ() + OFFSET_Z, -translationOffset.getX() + OFFSET_X));
    
    Translation2d dir = Filter.unit(finalTranslationOffset);

    if (limelight.getTagId() == -1) { //counts how many times it does not see the camera
      cyclesWithoutTag++;
    } else { // will continue moving in the direction it was even if it does not see a tag, prevents tipping
      mag = magFilter.calculate(finalTranslationOffset.getNorm());
      mag /= MAX_MAG;
      rot = rotFilter.calculate(limelight.getRobotRY() + OFFSET_RY);
      rot /= MAX_ROT;
      cyclesWithoutTag = 0;

      //mapping values

      double rot_rad = Units.degreesToRadians(rot);
      if (!SWING_WIDE) {
        dir = new Translation2d(
          dir.getX() * Math.cos(-rot_rad) + dir.getY() * Math.sin(-rot_rad),
          dir.getY() * Math.cos(-rot_rad) - dir.getX() * Math.sin(-rot_rad)
        );
      }

      mag = Filter.powerCurve(Filter.cutoffFilter(mag, 1, TRANSLATION_DEADBAND), 1.5);

      rot = Filter.powerCurve(Filter.deadband(Filter.cutoffFilter(rot), ROTATION_DEADBAND), 3);

      /*
      if (mag < TRANSLATION_DEADBAND) { //checking deadband first allows for deadband to be more than max limit, prevents unwanted behavior (never moving)
        mag = 0;
      } else if (mag > 1) {
        mag = 1;
      } else {
        mag = Math.pow(mag, 1.5); //power can be any number > 1
      }

      if (Math.abs(rot) < ROTATION_DEADBAND) {
        rot = 0;
      } else if (rot > 1) {
        rot = 1;
      } else if (rot < -1) {
        rot = -1;
      } else {
        rot = Math.pow(rot, 3); //power has to be an odd integer
      }
      */
    }
    

    

    SmartDashboard.putNumber("mag", mag);
    SmartDashboard.putNumber("rot", rot);

    //drive mapped values
    s_Swerve.drive(dir.times(mag).times(MAX_TRANSLATION_SPEED),  -rot * (MAX_ROTATION_SPEED), false);
    
    SmartDashboard.putNumber("dir x", dir.getX());
    SmartDashboard.putNumber("dir y", dir.getY());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("ApproachTag ran out of camera cycles");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // returns true when the camera does not see a tag for MAX_CYCLES_WITHOUT_TAG cycles
    return cyclesWithoutTag > MAX_CYCLES_WITHOUT_TAG;
  }
}
