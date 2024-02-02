// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;



//  https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api
public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  
  NetworkTable limelightTable;

  public Limelight() {

  }


  /*
   * ok this is so dumb.
   * within LIMELIGHT SOFTWARE, rotation about the vertical axis (normal to the ground) is the Z axis
   * within LIMELIGHT API,      rotation about the vertical axis (normal to the ground) is the Y AXIS.
   * 
   * change for consistency if you want
   */
  public double getRobotRZ() {
    return LimelightHelpers.getBotPose_TargetSpace("limelight")[5];
  }

  public double getRobotRY() {
    return LimelightHelpers.getBotPose_TargetSpace("limelight")[4];
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {

    Translation3d translationOffset = offsetFromTag();
    Rotation3d rotationOffset = rotationFromTag();

    //SmartDashboard offset logging
    SmartDashboard.putNumber("tX", translationOffset.getX());
    SmartDashboard.putNumber("tY", translationOffset.getY());
    SmartDashboard.putNumber("tZ", translationOffset.getZ());
    SmartDashboard.putNumber("roll",  rotationOffset.getX());
    SmartDashboard.putNumber("ry", getRobotRY());
    SmartDashboard.putNumber("rz", getRobotRZ());

    refreshValues();
    SmartDashboard.putNumber("Tag ID", getTagId());
    //SmartDashboard.putNumberArray("Robot Pose", robotPose)

  }

  private void refreshValues() {

    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

  }

  public long getTagId() {

    return limelightTable.getEntry("tid").getInteger(-1);

  }

  public Rotation3d rotationFromTag() {
      return LimelightHelpers.getBotPose3d_TargetSpace("limelight").getRotation();
    }

    public Translation3d offsetFromTag() {
      return LimelightHelpers.getBotPose3d_TargetSpace("limelight").getTranslation();
    }


}
