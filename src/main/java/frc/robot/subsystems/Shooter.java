// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.compound.Diff_VelocityVoltage_Velocity;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Filter;
import frc.robot.Constants;
import frc.robot.Robot;

public class Shooter extends SubsystemBase {

  private TalonFX mBeaterBar;
  private TalonFX mShooter;
  private TalonFX mRightShooter;
  private boolean inShooter = false;
  private DigitalInput limitSwitch = new DigitalInput(10);

  private VelocityVoltage shooterVelocityVoltage = new VelocityVoltage(0);



  /** Creates a new Shooter. */
  public Shooter() {
    mBeaterBar = new TalonFX(62); //TODO: set these to actual motor numbers
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

  private double shooterSetSpeed; // rotations per second
  public void setShooterSpeed(double speed) {
    shooterSetSpeed = Filter.cutoffFilter(speed, Constants.ShooterConstants.shooterWheelMaxRPS);
  }

  public double getShooterSpeed() {
    return shooterSetSpeed;
  }

  private double beaterBarSetSpeed; // [0, 1]
  public void setBeaterBarSpeed(double speed) {

      beaterBarSetSpeed = Filter.cutoffFilter(speed);

  } 

  public double getBeaterBarSpeed() {
    return beaterBarSetSpeed;
  }

  public void updateMotors() {
    mBeaterBar.set(beaterBarSetSpeed);

    if (shooterSetSpeed == 0) {
      mShooter.setControl(new CoastOut());
    } else {
      mShooter.setControl(shooterVelocityVoltage.withVelocity(shooterSetSpeed * Constants.ShooterConstants.shooterGearRatio));
    }
  }

  //TODO: make this read the limit switch state
  public boolean getIntakeLimitSwitchState() {
    return limitSwitch.get();
  }
}
