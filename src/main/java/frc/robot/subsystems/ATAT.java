// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkRelativeEncoder.Type;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Filter;
import frc.robot.CTREConfigs;
import frc.robot.Constants;
import frc.robot.RevConfigs;
import frc.robot.Robot;

public class ATAT extends SubsystemBase {
  /** Creates a new ATAT. */
  private CANSparkMax mAngleMotor = new CANSparkMax(Constants.ATATConstants.leftAngleMotorID, MotorType.kBrushless);
  private RelativeEncoder angleEncoder;
  private CANSparkMax mAngleMotorRight = new CANSparkMax(Constants.ATATConstants.rightAngleMotorID, MotorType.kBrushless);
  
  private TalonSRX mFrontLinearSRX = new TalonSRX(Constants.ATATConstants.frontPostSRXID);
  private TalonSRX mBackLinearSRX = new TalonSRX(Constants.ATATConstants.backPostSRXID);

  
  public ATAT() {

    RevConfigs.configureSparksPIDFFromTalonPIDV(mAngleMotor, Robot.ctreConfigs.ATAT_angleFXConfiguration);
    RevConfigs.configureSparksPIDFFromTalonPIDV(mAngleMotorRight, Robot.ctreConfigs.ATAT_angleFXConfiguration);

    
    mAngleMotor.setIdleMode(IdleMode.kBrake);
    mAngleMotorRight.setIdleMode(IdleMode.kBrake);
    mAngleMotor.setInverted(true);
    //mAngleMotor.setSoftLimit(SoftLimitDirection.kForward, 100f/360);
    //mAngleMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
    mAngleMotorRight.follow(mAngleMotor);
    mAngleMotor.getEncoder().setPosition(0);
    mAngleMotor.getPIDController().setPositionPIDWrappingEnabled(false);
    angleEncoder = mAngleMotor.getAlternateEncoder(com.revrobotics.SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
    mAngleMotor.getPIDController().setFeedbackDevice(angleEncoder);

    mFrontLinearSRX.config_kP(0, Robot.ctreConfigs.ATAT_postFXConfiguration.Slot0.kP);
    mFrontLinearSRX.config_kI(0, Robot.ctreConfigs.ATAT_postFXConfiguration.Slot0.kI);
    mFrontLinearSRX.config_kD(0, Robot.ctreConfigs.ATAT_postFXConfiguration.Slot0.kD);
    mFrontLinearSRX.config_kF(0, Robot.ctreConfigs.ATAT_postFXConfiguration.Slot0.kV);
    mFrontLinearSRX.setInverted(InvertType.InvertMotorOutput);

    mBackLinearSRX.config_kP(0, Robot.ctreConfigs.ATAT_postFXConfiguration.Slot0.kP);
    mBackLinearSRX.config_kI(0, Robot.ctreConfigs.ATAT_postFXConfiguration.Slot0.kI);
    mBackLinearSRX.config_kD(0, Robot.ctreConfigs.ATAT_postFXConfiguration.Slot0.kD);
    mBackLinearSRX.config_kF(0, Robot.ctreConfigs.ATAT_postFXConfiguration.Slot0.kV);
    // mBackLinearSRX.setInverted(//TODO: do this);


    
    // mFrontLinearMotor.getConfigurator().apply(Robot.ctreConfigs.ATAT_postFXConfiguration);
    // mFrontLinearMotor.setNeutralMode(NeutralModeValue.Brake);
    
    // mBackLinearMotor.getConfigurator().apply(Robot.ctreConfigs.ATAT_postFXConfiguration);
    // mBackLinearMotor.setNeutralMode(NeutralModeValue.Brake);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("desiredAngle", getDesiredAngle());
    SmartDashboard.putNumber("actual angle encoder value", getAngle());
    SmartDashboard.putNumber("motor requested speed", mAngleMotor.get());
    SmartDashboard.putNumber("desiredFrontPostPos", getDesiredFrontPostPos());
    SmartDashboard.putNumber("frontPostPos", getFrontPostPos());
  }
  private double desiredFrontPostPos;
  public void setDesiredFrontPostPos(double dist) {
    desiredFrontPostPos = Filter.cutoffFilter(dist, Constants.ATATConstants.frontPostMaxExtension);
  }

  public double getDesiredFrontPostPos() {
    return desiredFrontPostPos;
  }

  private double desiredBackPostPos;
  public void setDesiredBackPostPos(double dist) {
    desiredBackPostPos = Filter.cutoffFilter(dist, Constants.ATATConstants.backPostMaxExtension);
  }

  public double getDesiredBackPostPos() {
    return desiredBackPostPos;
  }

  private double desiredAngle;
  public void setDesiredAngle(double deg) {
    desiredAngle = Filter.cutoffFilter(deg, 120);
  }
  public double getDesiredAngle() {
    return desiredAngle;
  }

  public void setFrontPostPos(double dist /*meters*/) {
    dist = -Filter.cutoffFilter(dist, Constants.ATATConstants.frontPostMaxExtension);
    double rot = dist / Constants.ATATConstants.distanceBetweenPostParts / 2 / Math.PI;
    mFrontLinearSRX.set(ControlMode.Position, rot * Constants.ATATConstants.ThroughBoreTickPerRot, DemandType.ArbitraryFeedForward, 0.008);
  }

  public void setBackPostPos(double dist /*meters*/) {
    dist = Filter.cutoffFilter(dist, 5);
    double rot = dist / Constants.ATATConstants.distanceBetweenPostParts / 2 / Math.PI;
    mBackLinearSRX.set(ControlMode.Position, rot * Constants.ATATConstants.ThroughBoreTickPerRot, DemandType.ArbitraryFeedForward, 0.008);
  }

  public void setAngle(double deg) {
    deg = Filter.cutoffFilter(deg, 120);
    double rot = Units.degreesToRadians(deg) / 2 / Math.PI ;
    mAngleMotor.getPIDController().setReference(rot, ControlType.kPosition);
  }

  //degrees for printing reasons
  public double getAngle() {
    //return mAngleMotor.getEncoder().getPosition() * 360 / Constants.ATATConstants.angleGearRatio;
    // return mAngleMotor.getAlternateEncoder(42).getPosition();
    return angleEncoder.getPosition();
  }

  public double getFrontPostPos() {
    return -mFrontLinearSRX.getSelectedSensorPosition() / Constants.ATATConstants.ThroughBoreTickPerRot * 2 * Math.PI * Constants.ATATConstants.distanceBetweenPostParts;
  }

  public double getBackPostPos() {
    return -mBackLinearSRX.getSelectedSensorPosition() / Constants.ATATConstants.ThroughBoreTickPerRot * 2 * Math.PI * Constants.ATATConstants.distanceBetweenPostParts;
  }


}
