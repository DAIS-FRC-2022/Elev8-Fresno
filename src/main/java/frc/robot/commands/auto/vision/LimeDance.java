// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.vision;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.feeder.FeederByTimeCommand;
import frc.robot.commands.teleop.shooter.ShooterCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LimeDance extends SequentialCommandGroup {
  /** Creates a new LimeDance. */
  public LimeDance(ShooterSubsystem shooterSubsystem, FeederSubsystem feederSubsystem, DriveSubsystem driveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new LimeShot(shooterSubsystem, feederSubsystem, driveSubsystem));
    addCommands(new FeederByTimeCommand(feederSubsystem, 7));
  }
}
