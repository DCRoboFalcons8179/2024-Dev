// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;

public class ApproachTag extends Command {
  /** Creates a new ApproachTag. */
  private Translation3d translationOffset;
  private Translation2d finalTranslationOffset;
  private Rotation3d rotationOffset;
  private Swerve s_Swerve;
  private Limelight limelight;
  private double rot;
  private double mag;
  LinearFilter magFilter;
  LinearFilter rotFilter;

  private final double MAX_MAG;
  private final double MAX_ROT;
  private final double MAX_TRANSLATION_SPEED;
  private final double MAX_ROTATION_SPEED;   
  private final double TRANSLATION_DEADBAND; //standardized to 1
  private final double ROTATION_DEADBAND;    //standardized to 1
  private final int MAX_CYCLES_WITHOUT_TAG;


  /**
   * s_Swerve = Swerve object
   * 
   * limelight = limelight object
   *
   * MAX_MAGNITUDE = when the robot should start slowing down
   * 
   * MAX_ROT = when the robot should start slowing down rotation
   * 
   * MAX_TRANSLATION_SPEED (meters / second)
   * 
   * MAX_ROTATION_SPEED (degrees / second)
   * 
   * TRANSLATION_DEADBAND = how far from the desired distance from the tag the bot can be (meters)
   * 
   * ROTATION_DEADBAND = how far off from the desired rotation the bot can be (degrees)
   * 
   * MAX_CYCLES_WITHOUT_TAG = how many cycles the command can run without seeing a tag before it stops. Prevents the robot from losing the tag for a few frames then stopping, but can make the robot keep moving when it actually loses sight of a tag.
   */
  public ApproachTag(Swerve s_Swerve, Limelight limelight, double MAX_MAG, double MAX_ROT, double MAX_TRANSLATION_SPEED, double MAX_ROTATION_SPEED, double TRANSLATION_DEADBAND, double ROTATION_DEADBAND, int MAX_CYCLES_WITHOUT_TAG) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    addRequirements(s_Swerve);
    this.s_Swerve = s_Swerve;
    this.limelight = limelight;
    this.MAX_MAG = MAX_MAG; //default 2 (double), distance from desired position when the robot starts ramping down speed
    this.MAX_ROT = MAX_ROT;  //default 8 (double), rotation from desired orientation, when the robot starts ramping down rotation speed
    this.MAX_TRANSLATION_SPEED  = MAX_TRANSLATION_SPEED; //max translation speed of the robot (m/s)
    this.MAX_ROTATION_SPEED     = MAX_ROTATION_SPEED; //max rotation speed of the robot (deg/s)
    this.TRANSLATION_DEADBAND   = TRANSLATION_DEADBAND  / this.MAX_MAG; //allowed distance from desired magnitude, prevents shaking
    this.ROTATION_DEADBAND      = ROTATION_DEADBAND     / this.MAX_ROT; //allowed rotation from desired rotation, prevents shaking 
    this.MAX_CYCLES_WITHOUT_TAG = MAX_CYCLES_WITHOUT_TAG;               //allowed cycles the command can run without seeing a tag, ends the program if exceeded. Prevents the robot from ending the command as soon as it loses a tag (e.g. if the tag is too blurry for a frame and can't be found), but may cause the robot to move too far if it no longer has a tag in view. 
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
    rotationOffset = limelight.rotationFromTag();

    // Mapping inputs to the trapizoidd speed controller]
    finalTranslationOffset = (new Translation2d(translationOffset.getZ() + 3, -translationOffset.getX()));
    
    Translation2d dir = finalTranslationOffset.div(finalTranslationOffset.getNorm());

    double magIn = magFilter.calculate(finalTranslationOffset.getNorm());
    double rotIn = rotFilter.calculate(limelight.getRobotRY());

    if (limelight.getTagId() == -1) { //counts how many times it does not see the camera
      cyclesWithoutTag++;
    } else { // will continue moving in the direction it was even if it does not see a tag, prevents tipping
      mag = magFilter.calculate(magIn);
      rot = rotFilter.calculate(rotIn);
      cyclesWithoutTag = 0;
    }


    SmartDashboard.putNumber("mag", mag);
    SmartDashboard.putNumber("rot", rot);



    //mapping numbers
    mag /= MAX_MAG;
    if (mag < TRANSLATION_DEADBAND) { //checking deadband first allows for deadband to be more than max limit, prevents unwanted behavior (never moving)
      mag = 0;
    } else if (mag > 1) {
      mag = 1;
    } else {
      mag = Math.pow(mag, 2); //power can be any number > 1
    }

    rot /= MAX_ROT;
    if (Math.abs(rot) < ROTATION_DEADBAND) {
      rot = 0;
    } else if (rot > 1) {
      rot = 1;
    } else if (rot < -1) {
      rot = -1;
    } else {
      rot = Math.pow(rot, 3); //power has to be an odd number
    }

    //drive mapped values
    s_Swerve.drive(dir.times(mag).times(MAX_TRANSLATION_SPEED),  -rot/(10d/6 * MAX_ROTATION_SPEED), false);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // returns true when the camera does not see a tag for MAX_CYCLES_WITHOUT_TAG cycles
    return cyclesWithoutTag > MAX_CYCLES_WITHOUT_TAG;
  }
}
