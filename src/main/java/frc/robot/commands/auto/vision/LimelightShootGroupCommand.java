// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.vision;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.feeder.FeederByTimeCommand;
import frc.robot.commands.teleop.shooter.ShooterCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LimelightShootGroupCommand extends SequentialCommandGroup {
  /** Creates a new LimelightShootGroupCommand. */
  public LimelightShootGroupCommand(DriveSubsystem driveSubsystem, FeederSubsystem feederSubsystem, ShooterSubsystem shooterSubsystem) {
    // double time = 4;
    // // double ty = RobotContainer
    // // Add your commands in the addCommands() call, e.g.
    // // addCommands(new FooCommand(), new BarCommand());
    // double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0d);
    // double dist = 2.6416 / Math.tan(Math.toRadians(ty+45)); //45 is limelight angle
    // double pow = 2590*dist/0.95;
    addCommands(new LimelightDetectCommand(driveSubsystem));

    addCommands(new LimelightAlignCommand(driveSubsystem));

    // addCommands(new ShooterCommand(shooterSubsystem, pow));

    // addCommands(new FeederByTimeCommand(feederSubsystem, time));
  }
}
