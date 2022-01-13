// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Climber extends SubsystemBase {
    private final WPI_TalonFX _climberController = new WPI_TalonFX(2);
    private final WPI_TalonFX _leftPivotLinkController = new WPI_TalonFX(3);
    private final WPI_TalonFX _rightPivotLinkController = new WPI_TalonFX(4);
    
    public Climber() {
    }

    @Override
    public void periodic() {
        // Put code here to be run every loop
    }

    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public void runClimber(double speed) {
        _climberController.set(speed);
    }
}

