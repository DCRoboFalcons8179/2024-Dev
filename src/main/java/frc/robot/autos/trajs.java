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
    public final static Trajectory circle = 
    TrajectoryGenerator.generateTrajectory(
        new Pose2d(2.8,-7.21, zero),
        List.of(
            new Translation2d(3.8 , -5.88),
            new Translation2d(2.51, -5.16)
        ),
        new Pose2d(2.8,-7.21,zero)
        ,config);

    
}
