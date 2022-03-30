// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

public class LimelightAlignCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;
  double target;

  /** Creates a new LimelightAlignCommand. */
  public LimelightAlignCommand(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    target = Math.atan(2.6416-0.63/2.15);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    driveSubsystem.setSpeeds(new double[] {0.25*-Math.signum((target-46)-NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0d)), 0.25*-Math.signum((target-46)-NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0d))});;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0d) >= target-47 || NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0d) <= target-45);
  }
}
