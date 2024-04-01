package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants;

import java.util.List;





public class trajs {

    private final static Rotation2d zero = new Rotation2d();

    /**
     * IMPORTANT NOTE: To use paths from Path Planner, Copy the numbers from end points and waypoints
     * into the list of Translation2D()'s, but MAKE SURE TO PUT A NEG SIGN ON THE Y COORDINALTE
     * 
     */

    // This config sets speeds. The faster you go, the less accurate you become.
    private final static TrajectoryConfig config =
    new TrajectoryConfig(
            Constants.AutoConstants.kMaxSpeedMetersPerSecond,
            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(Constants.Swerve.swerveKinematics);



    // Example from the example.
    public final static Trajectory exampleTrajectory =
    TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, 0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);


    
    // Does a circle! NOTE THE NEG Y COORDINATES
    // public final static Trajectory circle = 
    // TrajectoryGenerator.generateTrajectory(
    //     new Pose2d(2.8,-7.21, zero),
    //     List.of(
    //         new Translation2d(3.8 , -5.88),
    //         new Translation2d(2.51, -5.16)
    //     ),
    //     new Pose2d(2.8,-7.21,zero)
    //     ,config);

    public final static Trajectory ampSideBlueFar = 
    TrajectoryGenerator.generateTrajectory(
        new Pose2d(-0.81, 6.66, zero),
        List.of(
            new Translation2d(-3.72, 6.15),
            new Translation2d(-6.01, 6.66)
        ),
        new Pose2d(-8.29, 6.66, zero)
        ,config);

        public final static Trajectory ampSideBlueClose = 
        TrajectoryGenerator.generateTrajectory(
        new Pose2d(-0.81, 6.67, zero),
        List.of(
            new Translation2d(-1, 7.65)
        ),
        new Pose2d(-5.21, 7.65, zero)
        ,config);

         public final static Trajectory kickOffWall = 
    TrajectoryGenerator.generateTrajectory(
        new Pose2d(-0.36, 6.91, zero),
        List.of(
            new Translation2d(-0.35, 6.90)
        ),
        new Pose2d(-1.19, 6.91, zero)
        ,config);

        public final static Trajectory sourceSideBlue = 
    TrajectoryGenerator.generateTrajectory(
        new Pose2d(-0.81, 4.00, zero),
        List.of(
            new Translation2d(-1.92, 1.53),
            new Translation2d(-5.60, 1)
        ),
        new Pose2d(-8.30, 1.88, zero)
        ,config);

        public final static Trajectory centerBlue = 
    TrajectoryGenerator.generateTrajectory(
        new Pose2d(-1.16, 5.52, zero),
        List.of(
            new Translation2d(-1.17, 5.53)
        ),
        new Pose2d(-2.21, 5.52, zero)
        ,config);

        public final static Trajectory centerRed = 
    TrajectoryGenerator.generateTrajectory(
        new Pose2d(-1.16, -5.52, zero),
        List.of(
            new Translation2d(-1.17, -5.53)
        ),
        new Pose2d(-2.21, -5.52, zero)
        ,config);

        public final static Trajectory pullOutFromSourceRed = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(-0.43, -1.96, zero),
            List.of(
                new Translation2d(-0.44, -1.97)
            ),
            new Pose2d(-2.7, -0.7, zero),
            config
        );

        public final static Trajectory centerRedToSpeaker = 
    TrajectoryGenerator.generateTrajectory(
        new Pose2d(-2.21, -5.52, zero),
        List.of(
            new Translation2d(-2.20, -5.53)
        ),
        new Pose2d(-1.16, -5.52, zero)
        ,config);
        

        public final static Trajectory justGoBack = 
    TrajectoryGenerator.generateTrajectory(
        new Pose2d(-0.37, 2.51, zero),
        List.of(
            new Translation2d(-.38, 2.52)
        ),
        new Pose2d(-2.74, 1.34, zero)
        ,config);

         public final static Trajectory sourceSideRed = 
    TrajectoryGenerator.generateTrajectory(
        new Pose2d(-0.81, -4.00, zero),
        List.of(
            new Translation2d(-1.92, -1.53),
            new Translation2d(-5.60, -1)
        ),
        new Pose2d(-8.30, -1.88, zero)
        ,config);

        public final static Trajectory ampSideRedClose = 
    TrajectoryGenerator.generateTrajectory(
        new Pose2d(-0.81, -6.67, zero),
        List.of(
            new Translation2d(-0.82, -6.68)
        ),
        new Pose2d(-2.46, -7.65, zero)
        ,config);

        public final static Trajectory destructionRed = 
    TrajectoryGenerator.generateTrajectory(
        new Pose2d(-1.61, -7.81, zero),
        List.of(
            new Translation2d(-8.27, -7.81),
            new Translation2d(-8.54, -6.68)
        ),
        new Pose2d(-8.54, -1.17, zero)
        ,config);

        public final static Trajectory destructionBlue = 
    TrajectoryGenerator.generateTrajectory(
        new Pose2d(-1.61, 7.81, zero),
        List.of(
            new Translation2d(-8.27, 7.81),
            new Translation2d(-8.54, 6.68)
        ),
        new Pose2d(-8.54, 1.17, zero)
        ,config);
}
