// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.CANSparkMax;

/** Add your docs here. */
public final class RevConfigs {

    public final double angle_kP = Constants.Swerve.angleKP;
    public final double angle_kI = Constants.Swerve.angleKI;
    public final double angle_kD = Constants.Swerve.angleKD;


    public static final void configureSparksPIDFFromTalonPIDV(CANSparkMax motor, TalonFXConfiguration talonConfig) {
        motor.getPIDController().setP(talonConfig.Slot0.kP);
        motor.getPIDController().setI(talonConfig.Slot0.kI);
        motor.getPIDController().setD(talonConfig.Slot0.kD);
        motor.getPIDController().setFF(talonConfig.Slot0.kV);
    }
}
