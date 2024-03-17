package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ATATConstants;
import frc.robot.autos.doTraj;
import frc.robot.autos.exampleAuto;
import frc.robot.autos.goToHeading;
import frc.robot.autos.trajs;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        /* Controllers */
        private final Joystick driver = new Joystick(0);
        private final Joystick board = new Joystick(1);
        private final Joystick board_ext = new Joystick(2);

        /* Drive Controls */
        private final int translationAxis = XboxController.Axis.kLeftY.value;
        private final int strafeAxis = XboxController.Axis.kLeftX.value;
        private final int rotationAxis = XboxController.Axis.kRightX.value;

        /* Subsystems */
        private final Swerve s_Swerve = new Swerve();
        private final Limelight limelight = new Limelight();
        public final Shooter shooter = new Shooter();
        private final Cameras cameras = new Cameras();
        private final ATAT atat = new ATAT();

        /* Driver Buttons */
        private final JoystickButton testButton = new JoystickButton(driver, XboxController.Button.kX.value);
        private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
        // private final JoystickButton zeroGyro = new JoystickButton(driver,
        // XboxController.Button.kRightBumper.value);
        private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
        private final JoystickButton dumpToLogger = new JoystickButton(driver, XboxController.Button.kStart.value);
        // private final JoystickButton approachTag = new JoystickButton(driver,
        // XboxController.Button.kA.value);
        // private final JoystickButton bButton = new JoystickButton(driver,
        // XboxController.Button.kB.value);
        private final JoystickButton hangPullUp = new JoystickButton(board, 2);
        private final JoystickButton ampSetPoint = new JoystickButton(board, 5);
        private final JoystickButton pickUpSetPoint = new JoystickButton(board, 3);
        private final JoystickButton humanPickUpSetPoint = new JoystickButton(board_ext, 6);
        private final JoystickButton carrySetPoint = new JoystickButton(board, 6);
        private final JoystickButton farSetPoint = new JoystickButton(board, 10);
        private final JoystickButton mediumSetPoint = new JoystickButton(board, 9);
        private final JoystickButton closeSetPoint = new JoystickButton(board, 8);
        private final JoystickButton beaterBarF = new JoystickButton(board, 4);
        private final JoystickButton beaterBarB = new JoystickButton(board, 7);
        private final JoystickButton hangPoint = new JoystickButton(board, 1);
        private final JoystickButton feed = new JoystickButton(board, 12);
        private final JoystickButton shoot = new JoystickButton(board, 11);
        private final JoystickButton frontPostManualUp = new JoystickButton(board_ext, 12);
        private final JoystickButton frontPostManualDown = new JoystickButton(board_ext, 11);
        private final JoystickButton backPostManualUp = new JoystickButton(board_ext, 10);
        private final JoystickButton backPostManualDown = new JoystickButton(board_ext, 9);
        private final JoystickButton angleManualUp = new JoystickButton(board_ext, 8);
        private final JoystickButton angleManualDown = new JoystickButton(board_ext, 7);
        private final Trigger leftTriggerPressed = new Trigger(() -> driver.getRawAxis(2) > 0.1);
        private final Trigger rightTriggerPressed = new Trigger(() -> driver.getRawAxis(3) > 0.1);
        private final JoystickButton camToggler = new JoystickButton(board_ext, 5);
        // private final JoystickButton camSaver = new JoystickButton(board_ext, 14);

        private final POVButton povUp = new POVButton(driver, 0);
        private final POVButton povDown = new POVButton(driver, 180);
        private final POVButton povLeft = new POVButton(driver, 270);
        private final POVButton povRight = new POVButton(driver, 90);

        //Arbitrary Triggers
        private Trigger hasRing = new Trigger(()->shooter.hasRing());

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Raw Controller Inputs

                // Controller Filtering and Modification
                // robotCentric.debounce(0.04).onTrue(new toggleFieldCentric(s_Swerve));

                // Calls the swerve command and sends the joystick input to the swerve drive
                // commands
                s_Swerve.setDefaultCommand(
                                new TeleopSwerve(
                                                s_Swerve,
                                                () -> driver.getRawAxis(translationAxis),
                                                () -> -driver.getRawAxis(strafeAxis),
                                                () -> driver.getRawAxis(rotationAxis)));

                // Moves atat
                atat.setDefaultCommand(new SetATATStates(atat));

                // Shooter control
                shooter.setDefaultCommand(new SetShooterStates(shooter));

                // Sets the camera zones unused currently thanks to PETER :(
                // cameras.setDefaultCommand(
                //                 new CameraPublisher(cameras, s_Swerve, () -> -driver.getRawAxis(translationAxis),
                //                                 () -> driver.getRawAxis(strafeAxis)));


                buttonCommands();

                diagnostics();
        }


        private void buttonCommands() {
                
                //robotCentric.onTrue(new InstantCommand(() -> s_Swerve.toggleFieldCentric()));
                
                // approachTag.whileTrue(new ApproachTag(s_Swerve, limelight, 2, 20, 4.5, 2,
                // 0.15, 6, 10, new Translation2d(0, 2), 0, false));
                // bButton.onTrue(new InstantCommand(() ->
                // shooter.setShooterSpeed(0.5)).andThen(() -> shooter.setBeaterBarSpeed(0.1)));
                // bButton.onFalse(new InstantCommand(() ->
                // shooter.setShooterSpeed(0)).andThen(() -> shooter.setBeaterBarSpeed(0)));
                
                
                zeroGyro.debounce(0.04).onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
                
                
                
                // Non-Set Point Buttons
                // shoot.whileTrue(new RequestShooterSetPoint(shooter, Constants.ShooterConstants.shooterSpeed));                
                // shoot.onFalse(new RequestShooterSetPoint(shooter, 0));
                
                // Shooter will spin up and fire.
                shoot.debounce(0.04).whileTrue(new RequestShot(shooter));
                rightTriggerPressed.debounce(0.04).whileTrue(new RequestShot(shooter));

                beaterBarB.onTrue(new RequestBeaterBarSetSpeed(shooter, Constants.ShooterConstants.beaterBarBSpeed));
                beaterBarB.onFalse(new RequestBeaterBarSetSpeed(shooter, 0));
                
                beaterBarF.whileTrue(new RequestBeaterBarSetSpeed(shooter, 0.6, true).andThen(new RequestBeaterBarSetSpeed(shooter, 0.0)));
                beaterBarF.onFalse(new RequestBeaterBarSetSpeed(shooter, 0));
                
                // Sets beater bar speed
                // feed.onTrue(new RequestBeaterBarSetSpeed(shooter, Constants.ShooterConstants.feedSpeed));
                // feed.onFalse(new RequestBeaterBarSetSpeed(shooter, 0));
                
                // Hanging commands wahoo :)
                hangPoint.onTrue(new RequestATATPose(atat, Constants.ATATConstants.hangSetPoint));
                hangPullUp.whileTrue(new pullDownOnChain(atat));

                // Changes the right camera
                // povUp.onTrue(new InstantCommand(() -> cameras.topCam()));
                // povDown.onTrue(new InstantCommand(() -> cameras.bottomCam()));

                // // Set Point Buttons
                closeSetPoint.onTrue(new RequestATATPose(atat, Constants.ATATConstants.shootClose));
                // mediumSetPoint.onTrue(new RequestATATPose(atat, Constants.ATATConstants.shootMedium));
                // farSetPoint.onTrue(new RequestATATPose(atat, Constants.ATATConstants.shootFar));
                ampSetPoint.onTrue(new AmpSequencing(atat, shooter));
                carrySetPoint.onTrue(new RequestATATPose(atat, Constants.ATATConstants.carry));
                pickUpSetPoint.onTrue(new RequestATATPose(atat, Constants.ATATConstants.pickUpSetPoint));
                humanPickUpSetPoint.whileTrue(new HumanPickUpAuto(atat, shooter));
                humanPickUpSetPoint.onFalse(new RequestBeaterBarSetSpeed(shooter, 0));
                // hangSetPoint.onTrue(new RequestATATPose(atat, Constants.ATATConstants.hangSetPoint));

                // // Manual Buttons
                 frontPostManualUp.onTrue(new RequestATATPose(atat, () -> atat.getDesiredFrontPostPos() + .25,
                                () -> atat.getDesiredBackPostPos(), () -> atat.getDesiredAngle()));
                backPostManualUp.onTrue(new RequestATATPose(atat, () -> atat.getDesiredFrontPostPos(),
                                () -> atat.getDesiredBackPostPos() + .25, () -> atat.getDesiredAngle()));
                angleManualUp.debounce(0.04).onTrue(new RequestATATPose(atat, () -> atat.getDesiredFrontPostPos(),
                                () -> atat.getDesiredBackPostPos(), () -> atat.getDesiredAngle() + 2));
                frontPostManualDown.onTrue(new RequestATATPose(atat, () -> atat.getDesiredFrontPostPos() - .25,
                                () -> atat.getDesiredBackPostPos(), () -> atat.getDesiredAngle()));
                backPostManualDown.onTrue(new RequestATATPose(atat, () -> atat.getDesiredFrontPostPos(),
                                () -> atat.getDesiredBackPostPos() - .25, () -> atat.getDesiredAngle()));
                angleManualDown.debounce(0.04).onTrue(new RequestATATPose(atat, () -> atat.getDesiredFrontPostPos(),
                                () -> atat.getDesiredBackPostPos(), () -> atat.getDesiredAngle() - 2));

                // // leftTriggerPressed.whileTrue(new RequestShooterSetPoint(shooter, () ->
                // // driver.getRawAxis(2) * Constants.ShooterConstants.shooterWheelMaxRPS));
                // leftTriggerPressed.whileTrue(new RequestShooterSetPoint(shooter, 100));
                // leftTriggerPressed.onFalse(new RequestShooterSetPoint(shooter, 0));

                pickUpSetPoint.whileTrue(new RequestCarryWhenRing(atat, shooter));
                pickUpSetPoint.onFalse(new RequestBeaterBarSetSpeed(shooter, 0));

                //Arbitrary commands yay
                hasRing.debounce(0.06).onTrue(new RumbleController(driver));
                //hasRing.debounce(0.06).onTrue(new SequentialCommandGroup(new RequestATATPose(atat, Constants.ATATConstants.carry), new RumbleController(driver)));





                // // Cam Toggling
                // camToggler.onTrue(new InstantCommand(() -> cameras.cameraToggler()));

                /*
                 * approachTag.whileTrue(new ApproachTag(s_Swerve, limelight, 0.8, 20,
                 * 1, 1,
                 * 0.15, 6, 10,
                 * new Translation2d(0, 2), 0, false));
                 * //bButton.whileTrue(new ApproachTag(s_Swerve, limelight, 0.8, 20,
                 * 1, 1,
                 * 0.15, 6, 10,
                 * new Translation2d(0, 2), 0, true));
                 */
                /*
                 * bButton.onTrue(new InstantCommand(() -> shooter.setBeaterBarSpeed(0.5)));
                 * bButton.onFalse(new InstantCommand(() -> shooter.setBeaterBarSpeed(0)));
                 * bButton.and(intakeLimitSwitch).onTrue(new InstantCommand(() ->
                 * shooter.setBeaterBarSpeed(0)));
                 */
        }

        private void configureLogger() {

        }

// Diagnostic Shuffleboard Stuff
        private void diagnostics() {
                
                Shuffleboard.getTab("testTab").add(shooter);


        }


        public Command getStopState() {
                // This command runs when the system is disabled. It stops any and all motors.

                Command stops = new InstantCommand(() -> shooter.stop());
                return stops;

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
                        //setting switches
                double switch1 = board.getRawAxis(0); //this one is negative 
                double switch2 = board_ext.getRawAxis(1);// this one is positive
                double switch3 = board_ext.getRawAxis(0); // this one is positive


        public Command getAutonomousCommand() {
                 s_Swerve.setPose(new Pose2d());
                s_Swerve.zeroGyro();
                
                
                return new RequestATATPose(atat, Constants.ATATConstants.shootClose, false)
                                        .andThen(new RequestShot(shooter))
                                        .andThen(new RequestATATPose(atat, Constants.ATATConstants.carry));
                // An ExampleCommand will run in autonomous
                // return new exampleAuto(s_Swerve);
                // return AutoBuilder.followPath(PathPlannerpath.getPathFile("spin"));

                // Load the path you want to follow using its name in the GUI
                //PathPlannerPath path = PathPlannerPat`h.fromPathFile("straight");

                // Create a path following command using AutoBuilder. This will also trigger
                // event markers.
                //return AutoBuilder.followPath(path);
        
                // return AutoBuilder.buildAuto("StraightPath");
                // return AutoBuilder.buildAuto("Test");


                //s_Swerve.setHeading(new Rotation2d(0));

                // return new PathPlannerAuto("Test");
        

                //Pulling color from driver station
               /*  var alliance = DriverStation.getAlliance();
                boolean amIRed;
                if (alliance.isPresent()) {
                        amIRed = (alliance.get() == DriverStation.Alliance.Red);
        
                }
                else {amIRed = false;}

                //IF you are blue setting switches
                if (amIRed = false) {

                        //Blue amp far 1 1 0
                        if (board.getRawAxis(0) < -0.5 && switch2 > 0.5 && switch3 < 0.5) {
                        return new doTraj(s_Swerve, trajs.kickOffWall)
                                .andThen(new goToHeading(s_Swerve, new Rotation2d().fromDegrees(-50)))
                                .andThen(new RequestATATPose(atat, Constants.ATATConstants.shootClose, false))
                                .andThen(new RequestShot(shooter))
                                .andThen(new RequestATATPose(atat, Constants.ATATConstants.carry))
                                .andThen(new goToHeading(s_Swerve, new Rotation2d(0)))
                                .andThen(new doTraj(s_Swerve, trajs.ampSideBlueFar))
                                .andThen(new goToHeading(s_Swerve, new Rotation2d().fromDegrees(-90)))
                                .andThen(new SearchForRIng(atat, shooter, s_Swerve))
                                .andThen(new goToHeading(s_Swerve, new Rotation2d(0)));
                        }else if (switch1 < -0.5 && switch2 < 0.5 && switch3 < 0.5)
                                { // blue amp close 1 0 0
                                return new doTraj(s_Swerve, trajs.kickOffWall)
                                        .andThen(new goToHeading(s_Swerve, new Rotation2d().fromDegrees(-50)))
                                        .andThen(new RequestATATPose(atat, Constants.ATATConstants.shootClose, false))
                                        .andThen(new RequestShot(shooter))
                                        .andThen(new RequestATATPose(atat, Constants.ATATConstants.carry))
                                        .andThen(new goToHeading(s_Swerve, new Rotation2d(0)))
                                        .andThen(new doTraj(s_Swerve, trajs.ampSideBlueClose))
                                        .andThen(new SearchForRIng(atat, shooter, s_Swerve));
                        }else if (switch1 > -0.5 && switch2 < 0.5 && switch3 > 0.5)
                                { //Blue Source far 0 0 1
                                return new doTraj(s_Swerve, trajs.kickOffWall)
                                        .andThen(new goToHeading(s_Swerve, new Rotation2d().fromDegrees(50)))
                                        .andThen(new RequestATATPose(atat, Constants.ATATConstants.shootClose, false))
                                        .andThen(new RequestShot(shooter))
                                        .andThen(new RequestATATPose(atat, Constants.ATATConstants.carry))
                                        .andThen(new goToHeading(s_Swerve, new Rotation2d(0)))
                                        .andThen(new doTraj(s_Swerve, trajs.sourceSideBlue))
                                        .andThen(new goToHeading(s_Swerve, new Rotation2d().fromDegrees(90)))
                                        .andThen(new SearchForRIng(atat, shooter, s_Swerve))
                                        .andThen(new goToHeading(s_Swerve, new Rotation2d(0)));
                        }else if (switch1 > -0.5 && switch2 < 0.5 && switch3 < 0.5)
                                { // Do nothing 0 0 0 
                                return new goToHeading(s_Swerve, new Rotation2d(0));
                        }else if (switch1 > -0.5 && switch2 > 0.5 && switch3 < 0.5)
                                { //Center Blue 0 1 0
                                return new RequestATATPose(atat, Constants.ATATConstants.shootClose, false)
                                        .andThen(new RequestShot(shooter))
                                        .andThen(new RequestATATPose(atat, Constants.ATATConstants.carry))
                                        .andThen(new doTraj(s_Swerve, trajs.centerBlue))
                                        .andThen(new SearchForRIng(atat, shooter, s_Swerve));
                        }else if (switch1 < -0.5 && switch2 < 0.5 && switch3 > 0.5)
                                { // shoot only 1 0 1
                                return new RequestATATPose(atat, Constants.ATATConstants.shootClose, false)
                                        .andThen(new RequestShot(shooter))
                                        .andThen(new RequestATATPose(atat, Constants.ATATConstants.carry));
                        }       
                }



                //if you are red setting switches
                //if (amIRed == true) {

               //}

                //return new doTraj(s_Swerve, trajs.circle).andThen( new doTraj(s_Swerve, trajs.circle)). andThen(new goToHeading(s_Swerve, new Rotation2d(Math.PI /2)));
                //return new doTraj(s_Swerve, trajs.kickOffWall)
                        //.andThen(new goToHeading(s_Swerve, new Rotation2d().fromDegrees(50)))
                        //.andThen(new RequestATATPose(atat, Constants.ATATConstants.shootClose, false))
                        //.andThen(new RequestShot(shooter))
                        //.andThen(new RequestATATPose(atat, Constants.ATATConstants.carry))
                        //.andThen(new goToHeading(s_Swerve, new Rotation2d(0)))
                        //.andThen(new doTraj(s_Swerve, trajs.sourceSideBlue))
                        //.andThen(new goToHeading(s_Swerve, new Rotation2d().fromDegrees(90)))
                        //.andThen(new SearchForRIng(atat, shooter, s_Swerve))
                        //.andThen(new goToHeading(s_Swerve, new Rotation2d(0)));
                        //start using limelight stuff;

                //return new goToHeading(s_Swerve, new Rotation2d(-Math.PI /2));

        // if (board_ext.getRawAxis(1)< -0.5) {
        //         return new RequestATATPose(atat, Constants.ATATConstants.shootClose).andThen(new RequestShooterSetPoint(shooter, 100)).andThen(new WaitCommand(1.5)).andThen(new RequestBeaterBarSetSpeed(shooter, 1)).andThen(new WaitCommand(1)).andThen(new RequestATATPose(atat, Constants.ATATConstants.carry)).andThen(new RequestShooterSetPoint(shooter, 0)).andThen(new RequestBeaterBarSetSpeed(shooter, 0));
        //         //return new RequestATATPose(atat, Constants.ATATConstants.shootClose).andThen(new RequestShooterSetPoint(shooter, 100)).andThen(new WaitCommand(1.5)).andThen(new RequestBeaterBarSetSpeed(shooter, 1)).andThen(new WaitCommand(1)).andThen(new RequestATATPose(atat, Constants.ATATConstants.carry)).andThen(new RequestShooterSetPoint(shooter, 0)).andThen(new RequestBeaterBarSetSpeed(shooter, 0)).andThen(new TeleopSwerve(s_Swerve, () -> -0.8 * 2.2 / Constants.Swerve.maxSpeed, () -> 0, () -> 0).raceWith(new WaitCommand(1.5)));
        // } else
        // {
        //         // waits .5 seconds then moves the swere and waits again
        //         return new WaitCommand(0.5).andThen(new TeleopSwerve(s_Swerve, () -> -0.8 * 2.2/Constants.Swerve.maxSpeed, () -> 0, () -> 0).raceWith(new WaitCommand(1.5)));
        // }*/

    }

}
