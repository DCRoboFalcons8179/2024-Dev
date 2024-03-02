package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.DifferentialSensorSourceValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.wpilibj.motorcontrol.Talon;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

    public TalonFXConfiguration ATAT_postFXConfiguration = new TalonFXConfiguration();
    public TalonFXConfiguration ATAT_angleFXConfiguration = new TalonFXConfiguration();
    public TalonFXConfiguration Shooter_shooterFXConfiguration = new TalonFXConfiguration();

    public CTREConfigs(){
        /** Swerve CANCoder Configuration */
        swerveCANcoderConfig.MagnetSensor.SensorDirection = Constants.Swerve.cancoderInvert;

        /** Swerve Angle Motor Configurations */
        /* Motor Inverts and Neutral Mode */
        swerveAngleFXConfig.MotorOutput.Inverted = Constants.Swerve.angleMotorInvert;
        swerveAngleFXConfig.MotorOutput.NeutralMode = Constants.Swerve.angleNeutralMode;

        /* Gear Ratio and Wrapping Config */
        swerveAngleFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.angleGearRatio;
        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;
        
        /* Current Limiting */
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.angleEnableCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.angleCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentThreshold = Constants.Swerve.angleCurrentThreshold;
        swerveAngleFXConfig.CurrentLimits.SupplyTimeThreshold = Constants.Swerve.angleCurrentThresholdTime;

        /* PID Config */
        swerveAngleFXConfig.Slot0.kP = Constants.Swerve.angleKP;
        swerveAngleFXConfig.Slot0.kI = Constants.Swerve.angleKI;
        swerveAngleFXConfig.Slot0.kD = Constants.Swerve.angleKD;

        /** Swerve Drive Motor Configuration */
        /* Motor Inverts and Neutral Mode */
        swerveDriveFXConfig.MotorOutput.Inverted = Constants.Swerve.driveMotorInvert;
        swerveDriveFXConfig.MotorOutput.NeutralMode = Constants.Swerve.driveNeutralMode;

        /* Gear Ratio Config */
        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.driveGearRatio;

        /* Current Limiting */
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.driveEnableCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.driveCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentThreshold = Constants.Swerve.driveCurrentThreshold;
        swerveDriveFXConfig.CurrentLimits.SupplyTimeThreshold = Constants.Swerve.driveCurrentThresholdTime;

        /* PID Config */
        swerveDriveFXConfig.Slot0.kP = Constants.Swerve.driveKP;
        swerveDriveFXConfig.Slot0.kI = Constants.Swerve.driveKI;
        swerveDriveFXConfig.Slot0.kD = Constants.Swerve.driveKD;
        swerveDriveFXConfig.Slot0.kV = Constants.Swerve.driveKF; //why

        /* Open and Closed Loop Ramping */
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;

        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;






        ATAT_angleFXConfiguration.Slot0.kP = Constants.ATATConstants.a_kP;
        ATAT_angleFXConfiguration.Slot0.kI = Constants.ATATConstants.a_kI;
        ATAT_angleFXConfiguration.Slot0.kD = Constants.ATATConstants.a_kD;
        ATAT_angleFXConfiguration.Slot0.kV = Constants.ATATConstants.a_kF;

        ATAT_postFXConfiguration.Slot0.kP = Constants.ATATConstants.l_kP;
        ATAT_postFXConfiguration.Slot0.kI = Constants.ATATConstants.l_kI;
        ATAT_postFXConfiguration.Slot0.kD = Constants.ATATConstants.l_kD;
        ATAT_postFXConfiguration.Slot0.kV = Constants.ATATConstants.l_kF;

        Shooter_shooterFXConfiguration.Slot0.kP = Constants.ShooterConstants.kP;
        Shooter_shooterFXConfiguration.Slot0.kI = Constants.ShooterConstants.kI;
        Shooter_shooterFXConfiguration.Slot0.kD = Constants.ShooterConstants.kD;
        Shooter_shooterFXConfiguration.Slot0.kV = Constants.ShooterConstants.kF;





    }

    public static void configureSRXPIDFfromTalonFXPIDV(TalonSRX srx, TalonFXConfiguration config) {
        srx.config_kP(0, config.Slot0.kP);
        srx.config_kI(0, config.Slot0.kI);
        srx.config_kD(0, config.Slot0.kD);
        srx.config_kF(0, config.Slot0.kV);
    }
}