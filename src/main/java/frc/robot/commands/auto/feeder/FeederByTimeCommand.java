// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.feeder;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.feeder.FeederSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FeederByTimeCommand extends ParallelRaceGroup {
  /** Creates a new FeederByTimeCommand. */
  public FeederByTimeCommand(FeederSubsystem feederSubsystem, double time) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new AutonomousFeederCommand(feederSubsystem));

    addCommands(new WaitCommand(time));
  }
}
