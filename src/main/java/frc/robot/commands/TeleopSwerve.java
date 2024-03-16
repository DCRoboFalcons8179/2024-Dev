package frc.robot.commands;

import frc.lib.math.Filter;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;

    /**
     * Drives the Swerve object. Should only really be used in hard-coded auton and Swerve's default Command.
     * @param s_Swerve
     * @param translationSup
     * @param strafeSup
     * @param rotationSup
     */
    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = translationSup.getAsDouble();
        double strafeVal = strafeSup.getAsDouble();
        double rotationVal = rotationSup.getAsDouble();

        if ((translationVal * translationVal) + (strafeVal * strafeVal) < 0.02) {
            translationVal = 0;
            strafeVal = 0;
        }



        rotationVal = Filter.deadband(rotationVal, 0.08);

        double mag = new Translation2d(translationVal, strafeVal).getNorm();
        mag = Filter.powerCurve(mag, 3);
        rotationVal = Filter.powerCurve(rotationVal, 3);

        



        /* Drive */
        
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity,
            false
        );
    }
}