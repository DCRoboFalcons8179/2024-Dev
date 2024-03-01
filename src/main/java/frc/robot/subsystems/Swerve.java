package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public AHRS gyro;

    public boolean fieldCentricBoolean = true;

    public Swerve() {

        try {
            System.out.println("--------------");
            gyro = new AHRS(SPI.Port.kMXP);

            System.out.println("NavX plugged in");
            System.out.println("--------------");

        } catch (RuntimeException ex) {
            System.out.println("NavX not plugged in");
            System.out.println("--------------");
        }
        gyro.enableBoardlevelYawReset(true);
        zeroGyro();

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        /*
         * By pausing init for a second before setting module offsets, we avoid a bug
         * with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        // Timer.delay(1.0);
        // zeroHeading();
        // zeroGyro();

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());

        for(SwerveModule mod : mSwerveMods){
            mod.printStats();
        }

        AutoBuilder.configureHolonomic(this::getPose, this::setPose, this::getChassisSpeeds, this::setModuleStates, Constants.Swerve.swervePathFollowerConfig, () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
        }, this);

    }

    // Helper variables for drive command
    private double translationX = 0;
    private double translationY = 0;
    private double rotationCommand = 0;

    public void drive(Translation2d translation, double rotation, boolean isOpenLoop) {

        // Gets the values of x y and rotation and assigns them to their repestive
        // variables
        translationX = translation.getX();

        translationY = translation.getY();

        rotationCommand = rotation;

        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldCentricBoolean ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getGyroYaw().times(-1))
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public ChassisSpeeds getChassisSpeeds() {
        return new ChassisSpeeds(gyro.getVelocityY(), gyro.getVelocityX(), Rotation2d.fromDegrees(gyro.getRawGyroZ()).getRadians());
    }

    public void zeroGyro() {
       
       gyro.zeroYaw();

    }

    public void setModuleStates(ChassisSpeeds robotChassisSpeeds) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(robotChassisSpeeds);
        setModuleStates(swerveModuleStates);
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(-gyro.getYaw());
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getGyroYaw(), getModulePositions());

        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle Motor Absolute Encoder", mod.getAbsolueAngleEncoderPos());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }

        SmartDashboard.putNumber("Commanded Velocity Y", translationY);

        SmartDashboard.putNumber("Commanded Velocity X", translationX);

        SmartDashboard.putNumber("Commanded Velocity", (new Translation2d(translationX, translationY)).getNorm());

        SmartDashboard.putNumber("Commanded Rotation", rotationCommand);

        SmartDashboard.putNumber("Velocity X Error",
                Math.abs(translationX) - Math.abs(mSwerveMods[0].getState().speedMetersPerSecond));

        SmartDashboard.putNumber("Gyro Value", gyro.getAngle());

        SmartDashboard.putNumber("Gyro Nice Value", this.getGyroYaw().getDegrees());

        SmartDashboard.putBoolean("Field Centric", fieldCentricBoolean);

        SmartDashboard.putNumber("Position", mSwerveMods[0].getPosition().distanceMeters);

        SmartDashboard.putNumber("Mod 0 spark state angle", mSwerveMods[0].getState().angle.getRotations());
        SmartDashboard.putNumber("Mod 0 spark angle", mSwerveMods[0].getPosition().angle.getRotations());

    }

    public void toggleFieldCentric() {

        fieldCentricBoolean = !fieldCentricBoolean;
    }
}