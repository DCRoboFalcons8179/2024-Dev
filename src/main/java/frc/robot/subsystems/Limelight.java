// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



//  https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api
public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  
  NetworkTable limelightTable;
  double[] robotPose = new double[6];

  public Limelight() {
    
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {

    refreshValues();

    robotPose = limelightTable.getEntry("botpose").getDoubleArray(new double[6]);

  }

  private void refreshValues() {
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  }
}
