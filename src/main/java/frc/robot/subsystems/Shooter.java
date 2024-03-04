// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.compound.Diff_VelocityVoltage_Velocity;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Filter;
import frc.robot.CTREConfigs;
import frc.robot.Constants;
import frc.robot.Robot;

public class Shooter extends SubsystemBase {

  private boolean inShooter = false;
  private DigitalInput limitSwitch = new DigitalInput(0);
  
  private TalonSRX shooterSRX;
  private TalonSRX shooterFollowerSRX;
  private VictorSPX beaterBarSPX;

  /** Creates a new Shooter. */
  public Shooter() {
    shooterSRX = new TalonSRX(Constants.ShooterConstants.shooterLeadMotorID);
    shooterFollowerSRX = new TalonSRX(Constants.ShooterConstants.shooterFollowMotorID);
    beaterBarSPX = new VictorSPX(Constants.ShooterConstants.beaterBarMotorID);

    CTREConfigs.configureSRXPIDFfromTalonFXPIDV(shooterSRX, Robot.ctreConfigs.Shooter_shooterFXConfiguration);
    CTREConfigs.configureSRXPIDFfromTalonFXPIDV(shooterFollowerSRX, Robot.ctreConfigs.Shooter_shooterFXConfiguration);
    shooterSRX.setSensorPhase(true);

    shooterSRX.setNeutralMode(NeutralMode.Coast);
    beaterBarSPX.setNeutralMode(NeutralMode.Brake);
    
    shooterSRX.setInverted(InvertType.InvertMotorOutput);
    shooterFollowerSRX.follow(shooterSRX);
    shooterFollowerSRX.setInverted(InvertType.OpposeMaster);

    beaterBarSPX.setInverted(InvertType.InvertMotorOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Speed", getShooterSpeed());
    SmartDashboard.putNumber("Shooter Set Speed", shooterSetSpeed);
    //System.out.println(hasRing());
  }

  private double shooterSetSpeed; // rotations per second
  public void setShooterSpeed(double speed) {
    shooterSetSpeed = Filter.cutoffFilter(speed, Constants.ShooterConstants.shooterWheelMaxRPS, -Constants.ShooterConstants.shooterWheelMaxRPS);
  }

  public double getShooterSpeed() {
    return -shooterSRX.getSelectedSensorVelocity() * 1.0d / Constants.ATATConstants.ThroughBoreTickPerRot * 10;
  }

  public double getShooterSetSpeed() {
    return shooterSetSpeed;
  }

  private double beaterBarSetSpeed; // [-1, 1]
  public void setBeaterBarSpeed(double speed) {
      beaterBarSetSpeed = Filter.cutoffFilter(speed);
  } 

  public double getBeaterBarSpeed() {
    return beaterBarSetSpeed;
  }

  public void updateMotors() {
    beaterBarSPX.set(ControlMode.PercentOutput, beaterBarSetSpeed);

    if (shooterSetSpeed <= 10) {
      shooterSRX.set(ControlMode.PercentOutput, 0);
    } else {
      shooterSRX.set(ControlMode.Velocity, -shooterSetSpeed * Constants.ATATConstants.ThroughBoreTickPerRot / 10, DemandType.ArbitraryFeedForward, 0.01);
    }
  }

  //TODO: make this read the limit switch state
  public boolean getIntakeLimitSwitchState() {
    return limitSwitch.get();
  }

  public boolean hasRing() {
    return !limitSwitch.get();
  }
}
