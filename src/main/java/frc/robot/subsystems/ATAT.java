// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
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

    // I made this because I was sick of scrolling down for the functions. -Tim
    bigAnnoyingMotorSetupBlock();

  }


  // Front setpoint
  private double desiredFrontPostPos;

  /**
   * Updates the desired position of the front post, which it constantly tries to go to.
   * @param dist
   */
  public void setDesiredFrontPostPos(double dist) {
    desiredFrontPostPos = Filter.cutoffFilter(dist, Constants.ATATConstants.frontPostMaxExtension);
  }

  /**
   * @return desired position of the front post.
   */
  public double getDesiredFrontPostPos() {
    return desiredFrontPostPos;
  }

  /**
   * Sets the motors to drive to the desired front post position. 
   * <p>Should not be used outside of a command that requires ATAT.
   * @param dist
   */
  public void setFrontPostPos(double dist /*meters*/) {
    dist = Filter.cutoffFilter(dist, Constants.ATATConstants.frontPostMaxExtension);
    double rot = dist / Constants.ATATConstants.distanceBetweenPostParts / 2 / Math.PI;
    mFrontLinearSRX.set(ControlMode.Position, rot * Constants.ATATConstants.ThroughBoreTickPerRot);
  }

  // Back Setpoint
  private double desiredBackPostPos;

  /**
   * Updates the desired position of the back post, which it constantly tries to go to.
   * @param dist
   */
  public void setDesiredBackPostPos(double dist) {
    desiredBackPostPos = Filter.cutoffFilter(dist, Constants.ATATConstants.backPostMaxExtension);
  }

  /**
   * @return desired position of the back post.
   */
  public double getDesiredBackPostPos() {
    return desiredBackPostPos;
  }

  /**
   * Sets the motors to drive to the desired back post position.
   * <p>Should not be used outside of a command that requires ATAT.
   * @param dist
   */
  public void setBackPostPos(double dist /*meters*/) {
    dist = Filter.cutoffFilter(dist, Constants.ATATConstants.backPostMaxExtension);
    double rot = dist / Constants.ATATConstants.distanceBetweenPostParts / 2 / Math.PI;
    mBackLinearSRX.set(ControlMode.Position, rot * Constants.ATATConstants.ThroughBoreTickPerRot);
  }


  // Angle Setpoint
  private double desiredAngle;
  /**
   * Updates the desired position of the angle, which it constantly tries to go to. 
   * <p><b>THIS IS DEGREES DOWN FROM STARTING POSITION. IT DOES NOT START AT 90.
   * </b><p> Soft and hard limits are in place so it shouldn't break.
   * @param deg
   */
  public void setDesiredAngle(double deg) {
    desiredAngle = Filter.cutoffFilter(deg, Constants.ATATConstants.maxAngle, Constants.ATATConstants.minAngle);//Constants.ATATConstants.maxAngle);
  }

  /**
   * @return the desired angle of the ATAT.
   */
  public double getDesiredAngle() {
    return desiredAngle;
  }

  /**
   * Sets the angle motors to move to the desired angle.
   * <p>Should not be used outside of a command that requires ATAT.
   * @param deg
   */
  public void setAngle(double deg) {
    deg = Filter.cutoffFilter(deg, Constants.ATATConstants.maxAngle, Constants.ATATConstants.minAngle);
    double rot = deg / 360;
    mAngleMotor.getPIDController().setReference(rot, ControlType.kPosition);
  }

  /** Manual Pull Up */
  public void pullUp() {

    // Pull up needs to not use the control loop. Just use the raw power
    mBackLinearSRX.set(ControlMode.PercentOutput, -0.9);
    mFrontLinearSRX.set(ControlMode.PercentOutput, -0.9);

    // TBD on if we need this
    // mAngleMotor.set(0);
  }

  public void releasePullUp() {

    // release needs to not use the control loop. Just use the raw power

    mBackLinearSRX.set(ControlMode.PercentOutput, 0.2);
    mFrontLinearSRX.set(ControlMode.PercentOutput, 0.2);

  }

  public void makeThisTheSetpoint() {
    // Call this to make the current position the new setpoint for the control loops
    // do NOT continually call this.
    
    desiredFrontPostPos = getFrontPostPos();
    desiredBackPostPos = getBackPostPos();
    desiredAngle = getAngle();

    setAngle(desiredAngle);
    setFrontPostPos(desiredFrontPostPos);
    setBackPostPos(desiredBackPostPos);

  }

  /**
   * @return angle position in degrees
   */
  public double getAngle() {

    return angleEncoder.getPosition() * 360;
    
  }

  /**
   * @return front post position in inches.
   */
  public double getFrontPostPos() {
    return -mFrontLinearSRX.getSelectedSensorPosition() / Constants.ATATConstants.ThroughBoreTickPerRot * 2 * Math.PI * Constants.ATATConstants.distanceBetweenPostParts;
  }
  
  /**
   * 
   * @return back post position in inches
   */
  public double getBackPostPos() {
    return -mBackLinearSRX.getSelectedSensorPosition() / Constants.ATATConstants.ThroughBoreTickPerRot * 2 * Math.PI * Constants.ATATConstants.distanceBetweenPostParts;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("desiredAngle", getDesiredAngle());
    SmartDashboard.putNumber("actual angle encoder value", getAngle());
    SmartDashboard.putNumber("motor requested speed", mAngleMotor.get());
    SmartDashboard.putNumber("desiredFrontPostPos", getDesiredFrontPostPos());
    SmartDashboard.putNumber("frontPostPos", getFrontPostPos());
    SmartDashboard.putNumber("desiredBackPostPos", getDesiredBackPostPos());
    SmartDashboard.putNumber("backPostPos", getBackPostPos());
    SmartDashboard.putNumber("Angle Motor Integrated Encoder", mAngleMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Right Motor output", mAngleMotorRight.getAppliedOutput());
  }


  private void bigAnnoyingMotorSetupBlock() {

    // i think it's beautiful

    // Set up Angle motors
    mAngleMotor.restoreFactoryDefaults();
    mAngleMotorRight.restoreFactoryDefaults();

    RevConfigs.configureSparksPIDFFromTalonPIDV(mAngleMotor, Robot.ctreConfigs.ATAT_angleFXConfiguration);
    RevConfigs.configureSparksPIDFFromTalonPIDV(mAngleMotorRight, Robot.ctreConfigs.ATAT_angleFXConfiguration);

    mAngleMotor.setIdleMode(IdleMode.kBrake);
    mAngleMotorRight.setIdleMode(IdleMode.kBrake);

    // Set up Current Limits for ALL NEOs
    mAngleMotor.setSmartCurrentLimit(30, 30);
    mAngleMotorRight.setSmartCurrentLimit(30, 30);


    mAngleMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    mAngleMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    mAngleMotor.setSoftLimit(SoftLimitDirection.kForward, 0.25f);
    mAngleMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);

    mAngleMotor.enableVoltageCompensation(12.0);
    mAngleMotorRight.enableVoltageCompensation(12.0);

    mAngleMotor.getPIDController().setOutputRange(-0.7, 0.7);

    // mAngleMotor.getPIDController().
    // Angle Remote encoder setup
    angleEncoder = mAngleMotor.getAlternateEncoder(com.revrobotics.SparkMaxAlternateEncoder.Type.kQuadrature , 8192);
    angleEncoder.setInverted(false);
    
    // Angle motor control loop setups
    mAngleMotor.getPIDController().setFeedbackDevice(angleEncoder);
    mAngleMotor.getPIDController().setPositionPIDWrappingEnabled(false);
    mAngleMotor.setInverted(false);
    
    mAngleMotorRight.follow(mAngleMotor,true);

    // Need to do this line to apply settings
    mAngleMotor.burnFlash();
    mAngleMotorRight.burnFlash();
  
    // Set up Front Motor Control
    CTREConfigs.configureSRXPIDFfromTalonFXPIDV(mFrontLinearSRX, Robot.ctreConfigs.ATAT_frontPostFXConfiguration);
    
    mFrontLinearSRX.setNeutralMode(NeutralMode.Brake);
    
    mFrontLinearSRX.setInverted(InvertType.None);
    mFrontLinearSRX.setSensorPhase(false);

    // From Tim - Use these functions to set up "minimum output value"
    // And Max usable value to get over static friction
    mFrontLinearSRX.configNominalOutputForward(0.01);
    mFrontLinearSRX.configNominalOutputReverse(-0.01);

    mFrontLinearSRX.configPeakOutputForward(1.0);
    mFrontLinearSRX.configPeakOutputReverse(-1.0);

    // TODO: GET SOFTWARE LIMITS
    mFrontLinearSRX.configForwardSoftLimitEnable(true);
    mFrontLinearSRX.configReverseSoftLimitEnable(true);
    mFrontLinearSRX.configReverseSoftLimitThreshold(0);
    mFrontLinearSRX.configForwardSoftLimitThreshold(7800);



    // Set up rear encoder control
    CTREConfigs.configureSRXPIDFfromTalonFXPIDV(mBackLinearSRX, Robot.ctreConfigs.ATAT_backPostFXConfiguration);
    


    mBackLinearSRX.setNeutralMode(NeutralMode.Brake);

    
    mBackLinearSRX.setInverted(false);
    mBackLinearSRX.setSensorPhase(false);

    // From Tim - Use these functions to set up "minimum output value"
    // And Max usable value to get over static friction
    mBackLinearSRX.configNominalOutputForward(0.01);
    mBackLinearSRX.configNominalOutputReverse(-0.01);

    mBackLinearSRX.configPeakOutputForward(1.0);
    mBackLinearSRX.configPeakOutputReverse(-1.0);

    // TODO: GET SOFTWARE LIMITS
    mBackLinearSRX.configForwardSoftLimitEnable(true);
    mBackLinearSRX.configReverseSoftLimitEnable(true);
    mBackLinearSRX.configReverseSoftLimitThreshold(0);
    mBackLinearSRX.configForwardSoftLimitThreshold(11000);



  }


}
