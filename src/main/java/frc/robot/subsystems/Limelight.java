// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation3d;
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

    public void update() {
      pose = limelightTable.getEntry("botpose").getDoubleArray(new double[6]);
    }

    public Rotation3d getRotation() {
      return new Rotation3d(0,0,0);
    }
  }


  public Limelight() {
    
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {

    refreshValues();
    SmartDashboard.putNumber("Tag ID", getTagId());
    //SmartDashboard.putNumberArray("Robot Pose", robotPose);
  }

  private void refreshValues() {

    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

  }

  public long getTagId() {
    return limelightTable.getEntry("tid").getInteger(-1);
  }
}
