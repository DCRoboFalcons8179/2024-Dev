package frc.robot.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class goToHeading extends Command {
  /** Creates a new command. */

    Swerve swerve;
    Rotation2d target;
    boolean isEnding = false;
    /**
     * TODO - Make the bot go to a certian heading.
     * 
     * @param target rotation 2d that you want to hit. looks at the gyro.
     * 
     */
  public goToHeading(Swerve swerve, Rotation2d target) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.target = target;
    addRequirements(swerve);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {



  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Rotation2d currentLocation = swerve.getGyroYaw();

    double rotPow = 0.0;
    double difference = currentLocation.minus(target).getDegrees();

    double highStep = 25;
    double lowStep = 1;

    if (difference > highStep) {
        rotPow = 3;
    }
    else if (difference > lowStep) {
      rotPow = 0.5;
    }
    else if (difference < -highStep) {
      rotPow = -3;
    }
    else if (difference < -lowStep) {
      rotPow = -0.5;
    }
    else {
      this.isEnding = true;
      // this.cancel() will cancel the entire SequentialCommandGroup - .andThen makes a command group, and what 'this' is gets lost.
    }

    swerve.drive(new Translation2d(0,0), rotPow, false);

    SmartDashboard.putNumber("Gyro Difference", difference);
    SmartDashboard.putNumber("Gyro Rot Power", rotPow);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {


    swerve.stop();
    
    

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    // May need to add in something for sensing if bott/all motors are at the right spot.
    // if so, we go into a more "stable" control loop mode
    
    return isEnding;


  }
}