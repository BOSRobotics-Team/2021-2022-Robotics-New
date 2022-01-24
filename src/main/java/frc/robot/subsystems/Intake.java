// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Intake extends SubsystemBase {
    private final WPI_TalonSRX _intakeController = new WPI_TalonSRX(5);

    public Intake() {
    }

    @Override
    public void periodic() {
        // Put code here to be run every loop
    }

    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }

    public void logPeriodic() {
        // _intakeController.logPeriodic();
     }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public void runIntake(double speed) {
        _intakeController.set(speed);
    }
}

