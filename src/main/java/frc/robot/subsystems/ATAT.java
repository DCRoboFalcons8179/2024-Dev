// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkRelativeEncoder.Type;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Filter;
import frc.robot.CTREConfigs;
import frc.robot.Constants;
import frc.robot.RevConfigs;
import frc.robot.Robot;

public class ATAT extends SubsystemBase {
  /** Creates a new ATAT. */
  private CANSparkMax mAngleMotor = new CANSparkMax(Constants.ATATConstants.leftAngleMotorID, MotorType.kBrushless);
  private CANSparkMax mAngleMotorLeft = new CANSparkMax(Constants.ATATConstants.rightAngleMotorID, MotorType.kBrushed);
  
  private TalonFX mFrontLinearMotor = new TalonFX(58);
  private TalonFX mBackLinearMotor = new TalonFX(57);

  private PositionVoltage anglePosition = new PositionVoltage(0); //params are default positions in rots
  private PositionVoltage frontPosition = new PositionVoltage(0);
  private PositionVoltage backPosition = new PositionVoltage(0);

  
  public ATAT() {

    RevConfigs.configureSparksPIDFFromTalonPIDV(mAngleMotor, Robot.ctreConfigs.ATAT_angleFXConfiguration);
    RevConfigs.configureSparksPIDFFromTalonPIDV(mAngleMotorLeft, Robot.ctreConfigs.ATAT_angleFXConfiguration);

    mAngleMotor.setIdleMode(IdleMode.kBrake);
    mAngleMotorLeft.setIdleMode(IdleMode.kBrake);
    mAngleMotor.getEncoder(Type.kHallSensor, 42);
    mAngleMotor.setSoftLimit(SoftLimitDirection.kForward, 100f/360);
    mAngleMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
    mAngleMotorLeft.follow(mAngleMotor);

    mFrontLinearMotor.getConfigurator().apply(Robot.ctreConfigs.ATAT_postFXConfiguration);
    mFrontLinearMotor.setNeutralMode(NeutralModeValue.Brake);
    
    mBackLinearMotor.getConfigurator().apply(Robot.ctreConfigs.ATAT_postFXConfiguration);
    mBackLinearMotor.setNeutralMode(NeutralModeValue.Brake);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  private double desiredFrontPostPos;
  public void setDesiredFrontPostPos(double dist) {
    desiredFrontPostPos = dist;
  }

  public double getDesiredFrontPostPos() {
    return desiredFrontPostPos;
  }

  private double desiredBackPostPos;
  public void setDesiredBackPostPos(double dist) {
    desiredBackPostPos = dist;
  }

  public double getDesiredBackPostPos() {
    return desiredBackPostPos;
  }

  private double desiredAngle;
  public void setDesiredAngle(double deg) {
    desiredAngle = deg;
  }
  public double getDesiredAngle() {
    return desiredAngle;
  }

  public void setFrontPostPos(double dist /*meters*/) {
    dist = Filter.cutoffFilter(dist, 9);
    double rot = dist * Constants.ATATConstants.frontPostGearRatio / Constants.ATATConstants.distanceBetweenPostParts / 2 / Math.PI;
    mFrontLinearMotor.setControl(frontPosition.withPosition(rot));
  }

  public void setBackPostPos(double dist /*meters*/) {
    dist = Filter.cutoffFilter(dist, 9);
    double rot = dist * Constants.ATATConstants.backPostGearRatio / Constants.ATATConstants.distanceBetweenPostParts / 2 / Math.PI;
    mBackLinearMotor.setControl(backPosition.withPosition(rot));
  }

  public void setAngle(double deg) {
    deg = Filter.cutoffFilter(deg, 120);
    double rot = Units.degreesToRadians(deg) * Constants.ATATConstants.angleGearRatio / 2 / Math.PI ;
    mAngleMotor.getPIDController().setReference(rot, ControlType.kPosition);
  }

  //degrees for printing reasons
  public double getAngle() {
    return mAngleMotor.getEncoder().getPosition() * 360 / Constants.ATATConstants.angleGearRatio;
  }

  public double getFrontPostPos() {
    return mFrontLinearMotor.getPosition().getValueAsDouble() * 2 * Math.PI * Constants.ATATConstants.distanceBetweenPostParts;
  }

  public double getBackPostPos() {
    return mBackLinearMotor.getPosition().getValueAsDouble() * 2 * Math.PI * Constants.ATATConstants.distanceBetweenPostParts;
  }


}
