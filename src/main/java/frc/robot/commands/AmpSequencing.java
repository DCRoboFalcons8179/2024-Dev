// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ATAT;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AmpSequencing extends SequentialCommandGroup {
  /** Creates a new AmpSequencing. */
  public AmpSequencing(ATAT atat, Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
                new RequestATATPose(atat, Constants.ATATConstants.ampSetPoint).alongWith(new WaitCommand(.80)),
                new RequestBeaterBarSetSpeed(shooter, -0.3),
                new WaitCommand(1.5),
                new RequestBeaterBarSetSpeed(shooter, 0),
                new RequestATATPose(atat, Constants.ATATConstants.ampSetPoint2).alongWith(new WaitCommand(1)),
                new RequestATATPose(atat, Constants.ATATConstants.carry));
  }
}
