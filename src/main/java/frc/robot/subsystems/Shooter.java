// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private TalonFX mBeaterBar;
  private TalonFX mShooter;
  private TalonFX mRightShooter;

  /** Creates a new Shooter. */
  public Shooter() {
    mBeaterBar = new TalonFX(62);
    mShooter = new TalonFX(61);
    mRightShooter = new TalonFX(60);

    mRightShooter.setControl(new Follower(mShooter.getDeviceID(), true));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Speed", getShooterSpeed());
    SmartDashboard.putNumber("mRightShooter speed", mRightShooter.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("mBeaterBar speed", mBeaterBar.getStatorCurrent().getValueAsDouble());
  }

  private double shooterSetSpeed;
  public void setShooterSpeed(double speed) {

    if (speed < -1) {
      shooterSetSpeed = -1;
    } else if (speed > 1) {
      shooterSetSpeed = 1;
    } else {
      shooterSetSpeed = speed;
    }

  }

  public double getShooterSpeed() {
    return shooterSetSpeed;
  }

  private double beaterBarSetSpeed;
  public void setBeaterBarSpeed(double speed) {

      if (speed < -1) {
        beaterBarSetSpeed = -1;
      } else if (speed > 1) {
        beaterBarSetSpeed = 1;
      } else {
        beaterBarSetSpeed = speed;
      }
  } 

  public double getBeaterBarSpeed() {
    return beaterBarSetSpeed;
  }

  public void updateMotors() {
    mBeaterBar.set(beaterBarSetSpeed);
    mShooter.set(shooterSetSpeed);
  }


}
