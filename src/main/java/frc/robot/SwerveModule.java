package frc.robot;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkRelativeEncoder.Type;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.RevConfigs;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;

    private CANSparkMax mAngleMotor;
    private TalonFX mDriveMotor;
    
    private CANcoder angleEncoder;
    private final RelativeEncoder m_turningEncoder;
    private double offsetFromStartup;
    private double offsetFromDesired;

    // private final SparkPIDController m_turningPIDController;

    private double m_chassisAngularOffset = 0;
    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());
    
    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);


    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        
        this.moduleNumber = moduleNumber;
        
        /* Angle Encoder Config - CANcoder. Should only be used once. */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);



        /* Angle Motor Config */
        mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);

        /* SPARK Encoder. Need to force it to only use CAN once, then use this relitive
         * for the rest of driving. It is used to having the Analog sensor plugged into 
         * the Spark itself. Using some conversions later to get that done.
        */
        m_turningEncoder = mAngleMotor.getEncoder();


        mAngleMotor.restoreFactoryDefaults();

        // m_turningPIDController = mAngleMotor.getPIDController();

        // m_turningPIDController.setFeedbackDevice(m_turningEncoder);

        mAngleMotor.getPIDController().setP(Robot.revConfigs.angle_kP);
        mAngleMotor.getPIDController().setI(Robot.revConfigs.angle_kI);
        mAngleMotor.getPIDController().setD(Robot.revConfigs.angle_kD);
        mAngleMotor.getPIDController().setFF(Robot.ctreConfigs.swerveAngleFXConfig.Slot0.kV);
        mAngleMotor.getPIDController().setPositionPIDWrappingEnabled(true);

        mAngleMotor.getPIDController().setPositionPIDWrappingMaxInput( Constants.Swerve.angleGearRatio / 2);
        mAngleMotor.getPIDController().setPositionPIDWrappingMinInput(-Constants.Swerve.angleGearRatio / 2);

        mAngleMotor.getEncoder(Type.kHallSensor, 1);
        mAngleMotor.getEncoder().setPositionConversionFactor(1);
        //mAngleMotor.getEncoder().setPositionConversionFactor(Constants.Swerve.angleGearRatio);
        //angleOffset.plus(new Rotation2d(mAngleMotor.getEncoder().getPosition() / Constants.Swerve.angleGearRatio * 2 * Math.PI));
        //this.angleOffset = moduleConstants.angleOffset.minus(Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble()));
        //this.angleOffset = moduleConstants.angleOffset;
        this.offsetFromStartup = angleEncoder.getPositionSinceBoot().getValueAsDouble();
        this.offsetFromDesired = angleEncoder.getPosition().getValueAsDouble() * 360 - moduleConstants.desiredCanCoderPos;
        mAngleMotor.getEncoder().setPosition(Rotation2d.fromDegrees(offsetFromDesired).getRotations() * -Constants.Swerve.angleGearRatio);
        //this.angleOffset = moduleConstants.angleOffset.times(-1);
        this.angleOffset = Rotation2d.fromDegrees(0);
        mAngleMotor.burnFlash();


        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
        mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.getConfigurator().setPosition(0.0);

    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        
        desiredState = SwerveModuleState.optimize(desiredState, getPosition().angle); 
        //mAngleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
        //mAngleMotor.getPIDController().setReference(moduleNumber, null);


        mAngleMotor.getPIDController().setReference((desiredState.angle.getRotations()) * Constants.Swerve.angleGearRatio, CANSparkMax.ControlType.kPosition);

        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.setControl(driveDutyCycle);
        }
        else {
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference);
            driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            mDriveMotor.setControl(driveVelocity);
        }
    }

    public Rotation2d getCANcoder(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public void resetToAbsolute(){
        double absolutePosition = (getCANcoder().getRotations() - angleOffset.getRotations());
        // double absolutePosition = getCANcoder().getRadians() - angleOffset.getRadians();

        mAngleMotor.getPIDController().setReference(absolutePosition, CANSparkMax.ControlType.kPosition);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.RPSToMPS(mDriveMotor.getVelocity().getValue(), Constants.Swerve.wheelCircumference), 
            getCANcoder()
            //            Rotation2d.fromRotations(mAngleMotor.getAbsoluteEncoder().getPosition())

            );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(mDriveMotor.getPosition().getValue(), Constants.Swerve.wheelCircumference), 
            //new Rotation2d (((mAngleMotor.getEncoder().getPosition()) / Constants.Swerve.angleGearRatio + angleOffset.getRotations())* 2 * Math.PI)
            new Rotation2d (((mAngleMotor.getEncoder().getPosition()) / Constants.Swerve.angleGearRatio) * 2 * Math.PI)

            // Rotation2d.fromRotations(mAngleMotor.getAbsoluteEncoder().getPosition())
        );
    }

    public void printStats() {

        System.out.println("---------------------");
        System.out.println(getCANcoder().getDegrees());
        System.out.println(mAngleMotor.getEncoder().getPosition());

    }

}