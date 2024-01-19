// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.autos;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
// import frc.robot.Constants;
// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
// import com.pathplanner.lib.commands.PPSwerveControllerCommand;
// import com.pathplanner.lib.server.PathPlannerServer;
// import com.pathplanner.lib.server.PathPlannerServerThread;
// import frc.robot.subsystems.Swerve;
// import frc.robot.commands.*;




// public class doPathTrajectory extends SequentialCommandGroup {
//     /** Creates a new doTrajectory. */
//     public doPathTrajectory(Swerve s_Swerve, PathPlannerTrajectory traj) {
//       // Use addRequirements() here to declare subsystem dependencies.
  
//       addRequirements(s_Swerve);
  
//       // var thetaController =
//       // new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
      
//       // thetaController.enableContinuousInput(-Math.PI, Math.PI);
  
//       PPSwerveControllerCommand swerveControllerCommand =
//           new PPSwerveControllerCommand(
//               traj,
//               s_Swerve::getPose,
//               Constants.Swerve.swerveKinematics,
//               // new PIDController(Constants.AutoConstants.kPXController, 0, 0),
//               // new PIDController(Constants.AutoConstants.kPYController, 0, 0),
//               // thetaController,
              
//               // X Axis
//               new PIDController(0, 0, 0),

//               // Y Axis
//               new PIDController(0, 0, 0),

//               // Theta Increase I a little and more paths
//               new PIDController(0.13, 0.00033, 0),
//               s_Swerve::setModuleStates,
//               true,
//               s_Swerve);
  
  
  
//       addCommands(
//       new InstantCommand(() -> s_Swerve.resetOdometry(traj.getInitialPose())),
//       swerveControllerCommand);
      
//     }
  
    
  
//   }

// //  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
// //     return new SequentialCommandGroup(
// //          new InstantCommand(() -> {
// //            // Reset odometry for the first path you run during auto
// //            if(isFirstPath){
// //                this.resetOdometry(traj.getInitialHolonomicPose());
// //            }
// //          }),
// //          new PPSwerveControllerCommand(
// //             traj, 
// //             this::getPose, // Pose supplier
// //             this.kinematics, // SwerveDriveKinematics
// //             new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
// //             new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
// //             new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
// //             this::setModuleStates, // Module states consumer
// //             true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
// //             this // Requires this drive subsystem
// //                 )
// //             );
// //         }