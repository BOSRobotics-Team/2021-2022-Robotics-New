// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.wrappers.SmartMotor;


public class Climber extends SubsystemBase {
    private final SmartMotor _climberController = new SmartMotor(2);
    private final SmartMotor _leftPivotLinkController = new SmartMotor(3);
    private final SmartMotor _rightPivotLinkController = new SmartMotor(4);
    
    public Climber() {
//        _climberController.configureRatios(gearRatio, wheelRadius);
//        _leftPivotLinkController.configureRatios(gearRatio, wheelRadius);
//        _rightPivotLinkController.configureRatios(gearRatio, wheelRadius);

        _rightPivotLinkController.follow(_leftPivotLinkController);
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

