// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
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

  private final double MAX_SPEED = Constants.ShooterConstants.shooterSpeed;

  /** Creates a new Shooter. */
  public Shooter() {
    shooterSRX = new TalonSRX(Constants.ShooterConstants.shooterLeadMotorID);
    shooterFollowerSRX = new TalonSRX(Constants.ShooterConstants.shooterFollowMotorID);
    beaterBarSPX = new VictorSPX(Constants.ShooterConstants.beaterBarMotorID);

    shooterSRX.configFactoryDefault();
    shooterFollowerSRX.configFactoryDefault();

    CTREConfigs.configureSRXPIDFfromTalonFXPIDV(shooterSRX, Robot.ctreConfigs.Shooter_shooterFXConfiguration);
    CTREConfigs.configureSRXPIDFfromTalonFXPIDV(shooterFollowerSRX, Robot.ctreConfigs.Shooter_shooterFXConfiguration);



    shooterSRX.setInverted(true);
    shooterFollowerSRX.setInverted(false);


    shooterSRX.setSensorPhase(false);
    shooterFollowerSRX.setSensorPhase(false);

    // 12.0 is more realistic
    shooterSRX.configVoltageCompSaturation(12.0);
    shooterFollowerSRX.configVoltageCompSaturation(12.0);

    shooterSRX.setNeutralMode(NeutralMode.Coast);
    shooterFollowerSRX.setNeutralMode(NeutralMode.Coast);

    beaterBarSPX.setNeutralMode(NeutralMode.Brake);
  
    

    // Shooter does NOT follow.
    // shooterFollowerSRX.follow(shooterSRX);
    // shooterFollowerSRX.setInverted(InvertType.FollowMaster);

    beaterBarSPX.setInverted(InvertType.InvertMotorOutput);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Shooter Speed", getShooterSpeed());
    // SmartDashboard.putNumber("Lead Stator Current", getLeadStatorCurrent());
    // SmartDashboard.putNumber("Shooter Set Speed", shooterSetSpeed);
    // SmartDashboard.putNumber("m1 speed", shooterSRX.getSelectedSensorVelocity());
    // SmartDashboard.putNumber("m2 speed", shooterFollowerSRX.getSelectedSensorVelocity());
    SmartDashboard.putBoolean("Has Ring", hasRing());
    //System.out.println(hasRing());
  }

  private double shooterSetSpeed; // rotations per second
  /**
   * Updates the desired set speed of the shooter. 
   * @param speed
   */
  public void setShooterSpeed(double speed) {
    /* Dont do this. See other functions. */
    // shooterSetSpeed = Filter.cutoffFilter(speed, Constants.ShooterConstants.shooterWheelMaxRPS, -Constants.ShooterConstants.shooterWheelMaxRPS) / 100 * Constants.ShooterConstants.shooterSpeed;

    shooterSetSpeed = MAX_SPEED * 0.9;


    shooterSRX.set(ControlMode.Velocity, shooterSetSpeed); //feed forward?
    shooterFollowerSRX.set(ControlMode.Velocity, shooterSetSpeed);
  }
  /**
   * @return desired set speed of the shooter.
   */
  public double getShooterSpeed() {
    /* Dont do this. See other functions. */
    return shooterSRX.getSelectedSensorVelocity();
  }
  
  /**
   * debugging
   * @return current of the main shooter
   */
  public double getLeadStatorCurrent() {
    return shooterSRX.getStatorCurrent();
  }

  /**
   * @return desired speed of the shooter.
   */
  public double getShooterSetSpeed() {
    return shooterSetSpeed;
  }

  private double beaterBarSetSpeed; // [-1, 1]
  /**
   * Updates the desired speed of the beater bar [-1, 1]
   * @param speed
   */
  public void setBeaterBarSpeed(double speed) {
      beaterBarSetSpeed = Filter.cutoffFilter(speed);
      beaterBarSPX.set(ControlMode.PercentOutput, beaterBarSetSpeed);
  } 

  /**
   * @return desired speed of the beater bar [-1, 1]
   */
  public double getBeaterBarSpeed() {
    return beaterBarSetSpeed;
  }


  public void startFeedForward() {
    beaterBarSetSpeed = 1.0;
    beaterBarSPX.set(ControlMode.PercentOutput, beaterBarSetSpeed); // should be handled by SetShooterStates?
  }

  public void shoot() {
    setShooterSpeed(MAX_SPEED * 0.90);
  }



  public void stopShooter() {
    shooterSRX.set(ControlMode.PercentOutput, 0);
    shooterFollowerSRX.set(ControlMode.PercentOutput, 0);

    shooterSetSpeed = 0;
  }

  public void stopBeaterbar() {
    beaterBarSetSpeed = 0;
    beaterBarSPX.set(ControlMode.PercentOutput, beaterBarSetSpeed);
    System.out.println("stopped beaterbar");
  }

  public void stop() {
    stopShooter();
    stopBeaterbar();
  }

  public boolean isAtSpeed() {
    // Important note - this function does not account for overshoot. Make sure to turn the motors 
    // on for 0.20 sec before asking if the motors are at the setpoint
        
    if 
      ((shooterSRX.getSelectedSensorVelocity() - shooterSetSpeed) > -3000 
      && (shooterFollowerSRX.getSelectedSensorVelocity() - shooterSetSpeed) > -3000)
      {
        return true;
      }

    return false;
  }


  public void updateMotors() {
    // Don't use this function. Have your commands call and change motors when you need them to change. 
  }


  //im glad you had no problem with this method
  public boolean hasRing() {
    return !limitSwitch.get();
  }

}
