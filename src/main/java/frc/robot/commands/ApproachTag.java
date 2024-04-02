// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.lib.math.Filter;
import frc.robot.Constants;
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
  LinearFilter xFilter;
  LinearFilter zFilter;

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
   * @param s_Swerve = Swerve object
   * 
   * @param limelight = limelight object
   *
   * @param MAX_MAG = when the robot should start curving speed. D: double [0, inf]
   * 
   * @param MAX_ROT = when the robot should start slowing down rotation. D: double [0, inf]
   * 
   * @param MAX_TRANSLATION_SPEED (meters / second). D: double [0, inf]
   * 
   * @param MAX_ROTATION_SPEED (degrees / second). D: double [0, inf]
   * 
   * @param TRANSLATION_DEADBAND = how far from the desired distance from the tag the bot can be (meters). D: double [0, inf]
   * 
   * @param ROTATION_DEADBAND = how far off from the desired rotation the bot can be (degrees). D: double [0, inf]
   * 
   * @param MAX_CYCLES_WITHOUT_TAG = how many cycles the command can run without seeing a tag before it stops. Prevents the robot from losing the tag for a few frames then stopping, but can make the robot keep moving when it actually loses sight of a tag. D: int [0, inf]
   * 
   * @param OFFSET_FROM_TAG = offset from tag, (X, Z) (meters). negative x is to the left, negative z is into the stage. D: Translation2d([-inf, inf], [-inf, inf])
   * 
   * @param ROTATION_FROM_TAG = offset from tag, positive = clockwise. D: double [-inf, inf]
   * 
   * @param SWING_WIDE swings wide if it is in robot-centric (due to desired vector being offset by the angle the robot is facing to the tag). Not yet ready to support field centric.
   */
  public ApproachTag(Swerve s_Swerve, Limelight limelight, double MAX_MAG, double MAX_ROT, double MAX_TRANSLATION_SPEED, double MAX_ROTATION_SPEED, double TRANSLATION_DEADBAND, double ROTATION_DEADBAND, int MAX_CYCLES_WITHOUT_TAG, Translation2d OFFSET_FROM_TAG, double ROTATION_FROM_TAG, boolean SWING_WIDE) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    addRequirements(s_Swerve);
    OFFSET_FROM_TAG.rotateBy(Rotation2d.fromDegrees(-ROTATION_FROM_TAG));
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

  public ApproachTag(Swerve swerve, Limelight limelight, int id, boolean SWING_WIDE) {
    this(swerve, limelight, 0.8, 20,
    4.5, 1,
    0.15, 6, 10,
    Constants.Swerve.getTranslationFromID(id), 0, SWING_WIDE);
  }

  public ApproachTag(Swerve swerve, Limelight limelight, Translation2d offset, boolean SWING_WIDE) {
    this(swerve, limelight, 0.8, 20,
    4.5, 1,
    0.15, 6, 10,
    offset, 7, SWING_WIDE);
  }

  // Called when the command is initially scheduled.

  @Override
  public void initialize() {
    magFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
    rotFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
    xFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
    zFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
    s_Swerve.fieldCentricBoolean = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  private int cyclesWithoutTag = 0;

  @Override
  public void execute() {

    
    if (limelight.getTagId() == -1 || limelight.offsetFromTag().getZ() > 0) { //counts how many times it does not see the camera (or the position flips)
      cyclesWithoutTag++;
    } else { // will continue moving in the direction it was even if it does not see a tag, prevents tipping
      cyclesWithoutTag = 0;
      
      translationOffset = limelight.offsetFromTag();

      //Vector2, to desired position
      finalTranslationOffset = (new Translation2d(translationOffset.getZ() + OFFSET_Z, -(-translationOffset.getX() + OFFSET_X)));
      
      Translation2d dir = Filter.unit(finalTranslationOffset);
      
      mag = finalTranslationOffset.getNorm(); //magFilter.calculate()
      mag /= MAX_MAG;

      //rot = rotFilter.calculate(limelight.getRobotRY() + OFFSET_RY);
      //rot = rotFilter.calculate(limelight.getTX());
      rot = limelight.getTX() + OFFSET_RY;
      double rot_rad = Units.degreesToRadians(limelight.getRobotRY() + OFFSET_RY);

      if (!SWING_WIDE) { // imaginary number rotation magic (x + yi) * (cos(theta) + sin(theta)i)
        dir = new Translation2d(
          dir.getX() * Math.cos(rot_rad) - dir.getY() * Math.sin(rot_rad),
          dir.getX() * Math.sin(rot_rad) + dir.getY() * Math.cos(rot_rad)
        ); 
      }

      rot /= MAX_ROT;

      //mapping values
      
      mag = Filter.powerCurve(Filter.cutoffFilter(mag, 1, TRANSLATION_DEADBAND), 1.5);

      rot = Filter.powerCurve(Filter.deadband(Filter.cutoffFilter(rot), ROTATION_DEADBAND), 2.2);
      

      s_Swerve.drive(dir.times(mag).times(MAX_TRANSLATION_SPEED).rotateBy(Rotation2d.fromDegrees(180)),  rot * (MAX_ROTATION_SPEED), false);

      SmartDashboard.putNumber("dir x", dir.getX());
      SmartDashboard.putNumber("dir y", dir.getY());
    }

    SmartDashboard.putNumber("mag", mag);
    SmartDashboard.putNumber("rot", rot);

    //drive mapped values
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("ApproachTag ended");
    s_Swerve.fieldCentricBoolean = true;
    magFilter.reset();
    rotFilter.reset();
    xFilter.reset();
    zFilter.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // returns true when the camera does not see a tag for MAX_CYCLES_WITHOUT_TAG cycles
    return cyclesWithoutTag > MAX_CYCLES_WITHOUT_TAG || (mag * MAX_MAG < TRANSLATION_DEADBAND && Math.abs(limelight.getRobotRY()) < ROTATION_DEADBAND);
  }
}
