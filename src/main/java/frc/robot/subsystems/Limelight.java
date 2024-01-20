// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



//  https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api
public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  
  NetworkTable limelightTable;
  
  public class RelativeRobotPose {

    double[] pose = new double[6];
    //indexes for pose array
    private final int xi = 0;
    private final int yi = 1;
    private final int zi = 2;
    private final int ri = 3;
    private final int pi = 4;
    private final int yawi = 5;
    private final int li = 6; //probably won't use but it's here

    public RelativeRobotPose() {

    }

    public void update() {
      pose = limelightTable.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
    }

    public Rotation3d getRotationOffset() {
      return new Rotation3d(pose[ri],pose[pi],pose[yawi]);
    }

    public Translation3d getTranslationOffset() {
      return new Translation3d(pose[xi], pose[yi], pose[zi]);
    }

  }

  RelativeRobotPose pose;


  public Limelight() {
    pose = new RelativeRobotPose();
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {

    refreshValues();
    SmartDashboard.putNumber("Tag ID", getTagId());
    //SmartDashboard.putNumberArray("Robot Pose", robotPose)

  }

  private void refreshValues() {

    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

    pose.update();

  }

  public long getTagId() {

    return limelightTable.getEntry("tid").getInteger(-1);

  }

  public Rotation3d rotationFromTag() {
    return pose.getRotationOffset().times(-1);
  }

  public Translation3d offsetFromTag() {
    return pose.getTranslationOffset();
  }


}
