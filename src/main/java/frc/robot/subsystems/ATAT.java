// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Filter;
import frc.robot.Constants;
import frc.robot.Robot;

public class ATAT extends SubsystemBase {
  /** Creates a new ATAT. */
  private TalonFX mAngleMotor = new TalonFX(59);
  private TalonFX mFrontLinearMotor = new TalonFX(58);
  private TalonFX mBackLinearMotor = new TalonFX(57);

  private PositionVoltage anglePosition = new PositionVoltage(0);
  private PositionVoltage frontPosition = new PositionVoltage(0);
  private PositionVoltage backPosition = new PositionVoltage(0);

  private static final double backPostRange  = Constants.ATATConstants.backPostMaxLength  - Constants.ATATConstants.backPostMinLength;
  private static final double frontPostRange = Constants.ATATConstants.frontPostMaxLength - Constants.ATATConstants.frontPostMinLength;

  
  public ATAT() {

    mAngleMotor.getConfigurator().apply(Robot.ctreConfigs.ATAT_postFXConfiguration);
    mAngleMotor.setNeutralMode(NeutralModeValue.Brake);

    mFrontLinearMotor.getConfigurator().apply(Robot.ctreConfigs.ATAT_postFXConfiguration);
    mFrontLinearMotor.setNeutralMode(NeutralModeValue.Brake);
    
    mBackLinearMotor.getConfigurator().apply(Robot.ctreConfigs.ATAT_postFXConfiguration);
    mBackLinearMotor.setNeutralMode(NeutralModeValue.Brake);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setFrontPostOffset(double dist /*meters*/) {

    //dist = dist; conversion here <-- gear ratio and math
    dist = Filter.cutoffFilter(dist, frontPostRange);

    mFrontLinearMotor.setControl(frontPosition.withPosition(dist));
  }

  public void setBackPostOffset(double dist /*meters*/) {

    //dist = dist; conversion here
    dist = Filter.cutoffFilter(dist, backPostRange, 0);

    mBackLinearMotor.setControl(backPosition.withPosition(dist));
  }

  public void setAngleOffset(double deg) {
    deg = Filter.cutoffFilter(deg, 120);
    mAngleMotor.setControl(anglePosition.withPosition(Units.degreesToRadians(deg)));
  }

}
