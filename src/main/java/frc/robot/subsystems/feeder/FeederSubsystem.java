// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;


public class FeederSubsystem extends SubsystemBase {
  private final CANSparkMax feeder;
  // private final Servo stopper;

  /** Creates a new FeederSubsystem. */
  public FeederSubsystem() {
    this.feeder = new CANSparkMax(FeederConstants.feeder_ID, MotorType.kBrushless);
    // this.stopper = new Servo(FeederConstants.servo_port);
    // this.stopper.setAngle(FeederConstants.initialAngle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Current Servo Speed", feeder.get());
  }

  public void stopFeeder() {
    this.feeder.set(0.0);
    // this.stopper.setAngle(FeederConstants.initialAngle);
  }

  // public double getPosition() {
  //   return this.stopper.getAngle();
  // }

  public void setFeederSpeed(double feederSpeed) {
    double speedmotor = this.feeder.get();
    if (Math.abs(speedmotor) < FeederConstants.deadband) {
      double now = Timer.getFPGATimestamp();
      while (now < 0.6) {
        if (now == 0.2)
          this.feeder.set(feederSpeed / 3);
        if (now == 0.4)
          this.feeder.set(feederSpeed / 2);
      }
      this.feeder.set(feederSpeed);
      // this.stopper.setAngle(FeederConstants.positionAngle);
      // System.out.println("Timer : " + now);
    } else {
      this.feeder.set(0);
      // this.stopper.setAngle(FeederConstants.initialAngle);

    }
  }
}
