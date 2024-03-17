// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ATAT;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HumanPickUpAuto extends SequentialCommandGroup {
  /** Creates a new HumanPickUpAuto. */
  public HumanPickUpAuto(ATAT atat, Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new RequestATATPose(atat, Constants.ATATConstants.pickUpHumanPlayer),
                new RequestBeaterBarSetSpeed(shooter, 0.6, true),
                new RequestBeaterBarSetSpeed(shooter, 0),
                new RequestATATPose(atat, Constants.ATATConstants.carry));
  }
}
