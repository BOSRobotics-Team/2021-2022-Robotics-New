// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.subsystems.DriveTrain.DriveMode;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CommandDriveTrain extends CommandBase {
@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    public final DriveTrain m_driveTrain;
    public final XboxController m_controller;
    public final JoystickButton m_buttons[] = new JoystickButton[11];

    private double _scaling = 0.2;

    private boolean _lastTriggerL = false;
    private boolean _lastTriggerR = false;

    public final AutonomousCommand m_autoCommand;
    public final AutoDriveStraightCommand m_autoCommand1;
    public final AutoDriveStraightCommand m_autoCommand2;

    public CommandDriveTrain(DriveTrain driveTrain, XboxController controller) {
        m_driveTrain = driveTrain;
        m_controller = controller;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_driveTrain);

        m_buttons[Button.kA.value] = new JoystickButton(m_controller, Button.kA.value);
        m_buttons[Button.kB.value] = new JoystickButton(m_controller, Button.kB.value);
        m_buttons[Button.kLeftBumper.value] = new JoystickButton(m_controller, Button.kLeftBumper.value);
        m_buttons[Button.kRightBumper.value] = new JoystickButton(m_controller, Button.kRightBumper.value);
        m_buttons[Button.kLeftStick.value] = new JoystickButton(m_controller, Button.kLeftStick.value);
        m_buttons[Button.kRightStick.value] = new JoystickButton(m_controller, Button.kRightStick.value);

        m_autoCommand = new AutonomousCommand(m_driveTrain);
        m_autoCommand1 = new AutoDriveStraightCommand(m_driveTrain, 10.0);
        m_autoCommand2 = new AutoDriveStraightCommand(m_driveTrain, 5.0);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        Shuffleboard.addEventMarker("CommandDriveTrain init.", this.getClass().getSimpleName(), EventImportance.kNormal);

        m_buttons[Button.kA.value].whenPressed(m_autoCommand1);    
        m_buttons[Button.kB.value].whenPressed(m_autoCommand2);    

        m_buttons[Button.kLeftBumper.value].whenPressed(() -> m_driveTrain.toggleDriveMode());    
        m_buttons[Button.kRightBumper.value].whenPressed(() -> m_driveTrain.setUseDriveScaling(!m_driveTrain.getUseDriveScaling()));
        m_buttons[Button.kLeftStick.value].whenPressed(() -> m_driveTrain.setUseSquares(!m_driveTrain.getUseSquares()));
        m_buttons[Button.kRightStick.value].whenPressed(() -> m_driveTrain.setQuickTurn(!m_driveTrain.getQuickTurn()));

        m_driveTrain.setDriveMode(DriveMode.ARCADE);
        m_driveTrain.setUseSquares(true);
        m_driveTrain.setUseDriveScaling(true);
        m_driveTrain.setDriveScaling(_scaling);
        m_driveTrain.enableBrakes(true);
        m_driveTrain.enableDriveTrain(true);

        _lastTriggerL = _lastTriggerR = false;
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        double triggerL = m_controller.getLeftTriggerAxis();
        if ((triggerL > 0.5) && !_lastTriggerL) { 
            _scaling = Math.min(_scaling + 0.1, 1.0);
            m_driveTrain.setDriveScaling(_scaling);
        }
        _lastTriggerL = (triggerL > 0.5);

        double triggerR = m_controller.getRightTriggerAxis();
        if ((triggerR > 0.5) && !_lastTriggerR)
        {
            _scaling = Math.max(_scaling - 0.1, 0.1);
            m_driveTrain.setDriveScaling(_scaling);
        }
        _lastTriggerR = (triggerR > 0.5);

        m_driveTrain.drive(m_controller);
        m_driveTrain.logPeriodic();
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        m_driveTrain.driveTank(0, 0);
        m_driveTrain.setUseSquares(true);
        m_driveTrain.enableBrakes(true);
        m_driveTrain.setDriveScaling(1.0);
        m_driveTrain.setDriveMode(DriveMode.ARCADE);
        m_driveTrain.enableDriveTrain(false);

        if (interrupted) {
            Shuffleboard.addEventMarker("CommandDriveTrain Interrupted!", this.getClass().getSimpleName(), EventImportance.kNormal);
        }
        Shuffleboard.addEventMarker("CommandDriveTrain end.", this.getClass().getSimpleName(), EventImportance.kNormal);
    }

    // Make this return true when this Command no longer needs to run execute()
   @Override
   public boolean isFinished() {
       return false;
    }
}