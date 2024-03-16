package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int pigeonID = 1;

        public static final COTSTalonFXSwerveConstants chosenModule = 
        COTSTalonFXSwerveConstants.CHUCK.Falcon500();

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(27 + 3d/4); 
        public static final double wheelBase = Units.inchesToMeters(23 + 7d/8);
        //27 + 3/4;
        //23 7/8
        public static final double wheelCircumference = Units.inchesToMeters(4.0 * Math.PI);

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0,trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = 6.75;
        public static final double angleGearRatio = 150d/7d;//21.428571428571;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 40;
        public static final double driveCurrentThresholdTime = 0.0;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = .18;
        public static final double angleKI = 0.001;
        public static final double angleKD = 0;

        // /* Drive Motor PID Values */
        // public static final double driveKP = 0.025; //TODO: This must be tuned to specific robot
        // public static final double driveKI = 0.0;
        // public static final double driveKD = 0.0;
        // public static final double driveKF = 0.045;

        /* Drive Motor PID Values */
        public static final double driveKP = .005; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.83;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.1388 / 12.0 ; //TODO: This must be tuned to specific robot
        public static final double driveKV = 0.25173 / 12.0;
        public static final double driveKA = 0.014411 / 12.0;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 2.2*1.5;//1 //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 4; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;



        /*
         * Offset tweaking has changed. . .
         * Since we do not have a good absolute encoder on our motors with sparks, we instead use a position to aim for on the CANCoder.
         * No invert needed for the drive motor, flip desiredCanCoderPos.
         */

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 30;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 42;
            public static final double desiredCanCoderPos = 31.85 + 180;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, desiredCanCoderPos);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 29;
            public static final int angleMotorID = 18;
            public static final int canCoderID = 45;
            public static final double desiredCanCoderPos = 50.76;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, desiredCanCoderPos);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 39;
            public static final int angleMotorID = 19;
            public static final int canCoderID = 43;
            public static final double desiredCanCoderPos = 100.15 + 180;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, desiredCanCoderPos);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 20;
            public static final int angleMotorID = 34;
            public static final int canCoderID = 44;
            public static final double desiredCanCoderPos = -152.08;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, desiredCanCoderPos);
        }

        public static final Translation2d getTranslationFromID(int id) {
            switch(id){
                case 7:
                return new Translation2d(0, 2);
                default:
                return getTranslationFromID(7);
            }
        }

        public static HolonomicPathFollowerConfig swervePathFollowerConfig = 
            new HolonomicPathFollowerConfig(new PIDConstants(driveKP, driveKI, driveKD), 
            new PIDConstants(angleKP, angleKI, angleKD), 
            maxSpeed, 
            new Translation2d(wheelBase / 2, trackWidth / 2).getNorm(), 
            new ReplanningConfig(true, true));
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class ATATConstants {
        //linear motor maximums are estimates, please measure!

        public static final int rightAngleMotorID = 7;
        public static final int leftAngleMotorID = 12;

        public static final int frontPostSRXID = 26;
        public static final int backPostSRXID  = 33; //TODO: fix this

        public static final double frontPostMaxExtension = 6.4 + 4d/8;
        public static final double backPostMaxExtension = 8.75; //TODO: fix this

        public static final double distanceBetweenPostParts = (1.123);

        public static final double maxDTheta = Units.degreesToRadians(120);

        public static final double frontPostGearRatio = 90; // input given: output seen
        public static final double backPostGearRatio = 90;
        public static final double angleGearRatio = 185d/3;

        public static final double a_kP = 3.6;
        public static final double a_kI = 0.0004;
        public static final double a_kD = 0.0;
        public static final double a_kF = 0.0;


        public static final double l_kP = 0.4;
        public static final double l_kI = 0.0;
        public static final double l_kD = 0.0;
        public static final double l_kF = 0.0;

        public static final double f_kP = 0.4;
        public static final double f_kI = l_kI;
        public static final double f_kD = l_kD;
        public static final double f_kF = l_kF;
        
        public static final double minAngle = -4;
        public static final double maxAngle = 90;

        public static final double lowAngleBackPostMinLength = 4;

        public static final int ThroughBoreTickPerRot = 8192;

        // ATATPose setpoints for readability in button declarations.
        public static final ATATPose carry = new ATATPose(0, 0, 0);

        public static final ATATPose shootClose = new ATATPose(0, 6.25, 0);
        // public static final ATATPose shootMedium = new ATATPose(8, 8, 27);
        public static final ATATPose shootMedium = new ATATPose(1, 2, 20);
        public static final ATATPose shootFar = new ATATPose(0, 4.5, 0);

        //public static final ATATPose ampSetPoint = new ATATPose(6.4, 4, 20);
        //public static final ATATPose ampSetPoint2 = new ATATPose(6.4, 2.25, 20);

        public static final ATATPose ampSetPoint = new ATATPose(6.9, 8.25, 10);
        public static final ATATPose ampSetPoint2 = new ATATPose(6.9, 2, 24);

        public static final ATATPose pickUpSetPoint = new ATATPose(0, 4, 88);
        public static final ATATPose pickUpHumanPlayer = new ATATPose(6.15, 0, 16);

        public static final ATATPose hangSetPoint = new ATATPose(0, 0, 0);
        public static final ATATPose hangPull = new ATATPose(0, 0, 0);
    }

    public static final class ShooterConstants {

        public static final int shooterLeadMotorID = 35;
        public static final int shooterFollowMotorID = 34;
        public static final int beaterBarMotorID = 36;

        public static final double shooterGearRatio = 1; // input / output, shouldn't matter since encoder is after gears.
        public static final double beaterBarGearRatio = 1; // input / output

        public static final double shooterMotorMaxRPS = 100d; //rotations per second TODO: get correct number
        public static final double shooterWheelMaxRPS = shooterMotorMaxRPS / shooterGearRatio;

        public static final double kP = 0.0065;
        public static final double kI = 0.000006;
        public static final double kD = 0.001;
        public static final double kF = 0.0145;

        public static final double shooterSpeed = 70000;
        public static final double beaterBarFSpeed = 1;
        public static final double beaterBarBSpeed = -0.7;
        public static final double feedSpeed = 0;
        //beater bar does not have pid, no pidf no matter what
    }
}
