// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.*;
// import frc.robot.subsystems.Lights.LEDColor;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CommandLights extends CommandBase {
@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    public final LEDLights m_lights;
    public final XboxController m_controller;
    public final JoystickButton m_buttons[] = new JoystickButton[11];

    private final LEDAnimationCommand _animationOffCommand;
    private final LEDAnimationRotateCommand _animationRotateCommand;

    // private final LEDOnboardLightCommand _onboardLightCommand;
    // private final LEDOnboardLightCommand _onboardLightOffCommand;

    // private final LEDStripLightCommand _stripLightCommand;
    // private final LEDStripLightCommand _stripLightOffCommand;

    // private boolean _lastTriggerL = false;
    // private boolean _lastTriggerR = false;

    public CommandLights(RobotContainer container) {
        System.out.println("CommandLights constructor ");
        m_lights = container.lights;
        m_controller = container.getOperatorController();

        _animationOffCommand = new LEDAnimationCommand(container);
        _animationRotateCommand = new LEDAnimationRotateCommand(container, true);
        // _onboardLightCommand = new LEDOnboardLightCommand(container, LEDColor.kWhite);
        // _onboardLightOffCommand = new LEDOnboardLightCommand(container, LEDColor.kOff);
        // _stripLightCommand = new LEDStripLightCommand(container, LEDColor.kWhite);
        // _stripLightOffCommand = new LEDStripLightCommand(container, LEDColoer.kOff);

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_lights);

        m_buttons[Button.kLeftStick.value] = new JoystickButton(m_controller, Button.kLeftStick.value);
        m_buttons[Button.kRightStick.value] = new JoystickButton(m_controller, Button.kRightStick.value);

        m_buttons[Button.kLeftStick.value].whenPressed(_animationOffCommand);
        m_buttons[Button.kRightStick.value].whenPressed(_animationRotateCommand);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        System.out.println("CommandLights - initialize");
        Shuffleboard.addEventMarker("CommandLights init.", this.getClass().getSimpleName(), EventImportance.kNormal);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        if (!m_lights.isAnimating()) {
            double lx = m_controller.getLeftX();
            double rx = m_controller.getRightX();

            int red = (lx < 0) ? (int)(-lx * 255) : 0;
            int green = (lx > 0) ? (int)(lx * 255) : 0;
            int blue = (rx < 0) ? (int)(-rx * 255) : 0;
            if (rx > 0) {
                red = (int)(rx * 255); }
            
            m_lights.runLights(red, green, blue);
        }
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        System.out.println("CommandLights end - interrupted = " + interrupted);
        if (interrupted) {
            Shuffleboard.addEventMarker("CommandLights Interrupted!", this.getClass().getSimpleName(), EventImportance.kNormal);
        }
        Shuffleboard.addEventMarker("CommandLights end.", this.getClass().getSimpleName(), EventImportance.kNormal);
    }

    // Make this return true when this Command no longer needs to run execute()
   @Override
   public boolean isFinished() {
       return false;
    }
}