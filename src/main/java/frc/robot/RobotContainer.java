package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick board = new Joystick(1);

    /* Drive Controls */
    private final int testPathIndex = XboxController.Axis.kLeftX.value;
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Limelight limelight = new Limelight();
    public final Shooter shooter = new Shooter();

    /* Driver Buttons */
    private final JoystickButton testPath = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton dumpToLogger = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton approachTag = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton bButton = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton hangButton = new JoystickButton(board, 11);
    private final JoystickButton fireButton = new JoystickButton(board, 10);
    private final JoystickButton beaterBarFButton = new JoystickButton(board, 3);
    private final JoystickButton beaterBarBButton = new JoystickButton(board, 4);
    private final JoystickButton lowSetPoint = new JoystickButton(board, 5);
    private final JoystickButton mediumSetPoint = new JoystickButton(board, 0);
    private final JoystickButton highSetPoint = new JoystickButton(board, 1);
    private final JoystickButton ampSetPoint = new JoystickButton(board, 2);
    private final JoystickButton carrySetPoint = new JoystickButton(board, 8);
    private final JoystickButton pickupSetPoint = new JoystickButton(board, 9);
    private final JoystickButton liftSetPoint = new JoystickButton(board, 7);
    private final POVButton povUp = new POVButton(driver, 0);
    private final POVButton povLeft = new POVButton(driver, 270);
    private final POVButton povDown = new POVButton(driver, 180);

    private final Trigger intakeLimitSwitch = new Trigger(() -> shooter.getIntakeLimitSwitchState());

    

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
         // Raw Controller Inputs

        // Controller Filtering and Modification
        //robotCentric.debounce(0.04).onTrue(new toggleFieldCentric(s_Swerve));


        
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> driver.getRawAxis(rotationAxis)
            )
        );

        shooter.setDefaultCommand(new ShooterDefault(shooter));

        // Configure the button bindings
        configureButtonBindings();
        buttonCommands();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        /* Driver Buttons */
        //zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        
    }

    private void buttonCommands() {
        // TODO Auto-generated method stub
        povLeft.whileTrue(new TeleopSwerve(s_Swerve, () -> 0, () -> 1, () -> 0));
        robotCentric.onTrue(new InstantCommand(() -> s_Swerve.toggleFieldCentric()));
        approachTag.whileTrue(new ApproachTag(s_Swerve, limelight, 0.8, 20,
        1, 1,
        0.15, 6, 10,
        new Translation2d(0, 2), 0, false));
        bButton.whileTrue(new ApproachTag(s_Swerve, limelight, 0.8, 20, 
        1, 1, 
        0.15, 6, 10, 
        new Translation2d(0, 2), 0, true));

        fireButton.onTrue(new RequestShooterSpeed(0.5, shooter));
        fireButton.onFalse(new RequestShooterSpeed(0, shooter));
        beaterBarFButton.onTrue(new RequestBeaterBarSpeed(0.5, shooter));
        beaterBarFButton.onFalse(new RequestBeaterBarSpeed(0, shooter));
        beaterBarBButton.onTrue(new RequestBeaterBarSpeed(-0.5, shooter));
        beaterBarBButton.onFalse(new RequestBeaterBarSpeed(0, shooter));




        /*bButton.onTrue(new InstantCommand(() -> shooter.setBeaterBarSpeed(0.5)));
        bButton.onFalse(new InstantCommand(() -> shooter.setBeaterBarSpeed(0)));
        bButton.and(intakeLimitSwitch).onTrue(new InstantCommand(() -> shooter.setBeaterBarSpeed(0)));*/
    }

    private void configureLogger() {
        

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve);
    }
}
