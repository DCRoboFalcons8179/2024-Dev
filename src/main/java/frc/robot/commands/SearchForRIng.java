// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ATAT;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SearchForRIng extends ParallelRaceGroup {
  /** Creates a new SearchForRIng. */
  public SearchForRIng(ATAT atat, Shooter shooter, Swerve swerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    //Math to figure out backwards if needed

    addCommands (new RequestCarryWhenRing(atat, shooter).handleInterrupt(()-> new RequestATATPose(atat, Constants.ATATConstants.carry)),
                new WaitCommand(1)
                .andThen(new InstantCommand(()-> swerve.drive(new Translation2d(-1, 0), 0, false)))
                .repeatedly().withTimeout(2));

                // addCommands((new RequestBeaterBarSetSpeed(shooter, 0.6, true))
                //             .andThen(new InstantCommand(()-> swerve.drive(new Translation2d(-1, 0), 0, false)))
                //             .repeatedly().withTimeout(2)
                //             .andThen(new RequestBeaterBarSetSpeed(shooter, 0))
                //             .andThen(new RequestATATPose(atat, Constants.ATATConstants.carry)));
  }
}
