// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private final WPI_VictorSPX _shooterController = new WPI_VictorSPX(Constants.kID_Shooter);

  private double kResetActuator = -0.2;
  private double kReleaseActuator = 1.0;

  private int kResetActuatorTime = 15;
  private int kReleaseActuatorTime = 10;

  private int _counter = 0;

  public Shooter() {
    Preferences.initDouble("ShooterResetSpeed", -0.2);
    Preferences.initDouble("ShooterReleaseSpeed", 1.0);

    kResetActuator = Preferences.getDouble("ShooterResetSpeed", -0.2);
    kReleaseActuator = Preferences.getDouble("ShooterReleaseSpeed", 1.0);

    _counter = 0;
  }

  @Override
  public void periodic() {
    // Put code here to be run every loop
    if (_counter > 0) {
      if (--_counter <= 0) {
        stopShooter();
        _counter = 0;
      }
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void logPeriodic() {}

  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public void releaseShooter() {
    _shooterController.set(kReleaseActuator);
    _counter = kReleaseActuatorTime;
    System.out.println("Shooter - releaseShooter ");
  }

  public void resetShooter() {
    _shooterController.set(kResetActuator);
    _counter = kResetActuatorTime;
    System.out.println("Shooter - resetShooter");
  }

  public void stopShooter() {
    _shooterController.set(0.0);
    System.out.println("Shooter - stopped ");
  }

  public boolean isRunning() {
    return _counter > 0;
  }
}
