// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.*;
// import frc.robot.subsystems.Lights.LEDColor;

public class CommandLights extends CommandBase {
@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    public final Lights m_lights;
    public final XboxController m_controller;
    // public final JoystickButton right_Bumper;
    public final JoystickButton left_Stick;
    public final JoystickButton right_Stick;

    // private final LEDAnimationCommand _animationCommand;
    private final LEDAnimationRotateCommand _animationRotateCommand;
    private final LEDAnimationOffCommand _animationOffCommand;

    // private final LEDOnboardLightCommand _onboardLightCommand;
    // private final LEDOnboardLightOffCommand _onboardLightOffCommand;

    // private final LEDStripLightCommand _stripLightCommand;
    // private final LEDStripLightOffCommand _stripLightOffCommand;

    // private boolean _lastTriggerL = false;
    // private boolean _lastTriggerR = false;

    public CommandLights(Lights lights, XboxController controller) {
        m_lights = lights;
        m_controller = controller;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(lights);

        left_Stick = new JoystickButton(m_controller, 9);
        right_Stick = new JoystickButton(m_controller, 10);

        // _animationCommand = new LEDAnimationCommand(lights);
        _animationRotateCommand = new LEDAnimationRotateCommand(lights, true);
        _animationOffCommand = new LEDAnimationOffCommand(lights);
        // _onboardLightCommand = new LEDOnboardLightCommand(lights, LEDColor.kWhite);
        // _onboardLightOffCommand = new LEDOnboardLightOffCommand(lights);
        // _stripLightCommand = new LEDStripLightCommand(lights, LEDColor.kWhite);
        // _stripLightOffCommand = new LEDStripLightOffCommand(lights);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        // _lastTriggerL = _lastTriggerR = false;

        left_Stick.whenPressed(_animationRotateCommand);
        right_Stick.whenPressed(_animationOffCommand);
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

        // double triggerL = m_controller.getLeftTriggerAxis();
        // if ((triggerL > 0.5) && !_lastTriggerL) { 
        //     _onCommand.schedule();
        // } else if ((triggerL <= 0.5) && _lastTriggerL) {
        //     _offCommand.schedule();
        // }
        // _lastTriggerL = (triggerL > 0.5);

        // double triggerR = m_controller.getRightTriggerAxis();
        // if ((triggerR > 0.5) && !_lastTriggerR) {
        //     _upCommand.schedule();
        // } else if ((triggerL <= 0.5) && _lastTriggerL) {
        //     _dnCommand.schedule();
        // }
        // _lastTriggerR = (triggerR > 0.5);

    //    m_intake.logPeriodic();
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        m_lights.setColors();
        m_lights.runLights(0, 0, 0);
    }

    // Make this return true when this Command no longer needs to run execute()
   @Override
   public boolean isFinished() {
       return false;
    }
}