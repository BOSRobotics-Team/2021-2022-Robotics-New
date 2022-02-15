// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.*;
import frc.robot.commands.ledlights.*;
import frc.robot.subsystems.LEDLights.*;

public class AutoClimberCommand extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final int kClimberSteps = 19;

  private final String kClimberHeight = "%02d: Climber_Height";
  private final String kClimberTilt = "%02d: Climber_Tilt";

  private final double climberPairs[][] =
      new double[][] {
        {24.5, 10.0},
        {0, 10.0},
        {0, 0},
        {8.0, 0},
        {8.0, -10.0},
        {0, -10.0},
        {0, 10.0},
        {24.5, 10.0},
        {24.5, 25.0},
        {24.5, 30.0},
        {21.5, 30.0},
        {18.0, 30.0},
        {12.0, 40.0},
        {12.0, 50.0},
        {18.0, 50.0},
        {18.0, 10.0},
        {0, 10.0},
        {0, 0},
        {8.0, 0}
      };
  private final LEDColor climberLEDs[] = new LEDColor[kClimberSteps];

  public AutoClimberCommand(RobotContainer container, boolean init) {

    Preferences.initDouble("Climber - MaxHeight", 25.0); // 25 stroke
    Preferences.initDouble("Climber - MaxTilt", 50.0); // 50 degree tilt
    for (Integer step = 0; step < kClimberSteps; ++step) {
      Preferences.initDouble(String.format(kClimberHeight, step), climberPairs[step][0]);
      Preferences.initDouble(String.format(kClimberTilt, step), climberPairs[step][1]);
    }

    double maxHeight = 1.0 / Preferences.getDouble("Climber - MaxHeight", 25.0);
    double maxTilt = 1.0 / (2.0 * Preferences.getDouble("Climber - MaxTilt", 50.0));
    for (Integer step = 0; step < kClimberSteps; ++step) {
      climberPairs[step][0] =
          maxHeight
              * Preferences.getDouble(String.format(kClimberHeight, step), climberPairs[step][0]);
      climberPairs[step][1] =
          0.5
              + maxTilt
                  * Preferences.getDouble(String.format(kClimberTilt, step), climberPairs[step][1]);
    }
    for (Integer step = 0; step < kClimberSteps; ++step) {
      climberLEDs[step] = new LEDColor(255, 255, step, 0, 0, step % 8);
    }

    if (init) {
      LEDColor led1 = new LEDColor(255, 255, 0, 0, 8, 120);
      addCommands(
          new LEDAnimationCommand(container, AnimationTypes.Strobe, led1),
          new LEDOnboardLightCommand(container, climberLEDs[0]),
          new PrintCommand("Climber Extend and Tilt initial position"),
          new ClimberExtendTiltCommand(container, climberPairs[0][0], climberPairs[0][1]));
    } else {
      for (Integer step = 1; step < kClimberSteps; ++step) {
        addCommands(
            new LEDOnboardLightCommand(container, climberLEDs[step]),
            new PrintCommand("Climber Extend and Tilt step " + step.toString()),
            new ClimberExtendTiltCommand(container, climberPairs[step][0], climberPairs[step][1]));
      }
    }
  }

  public AutoClimberCommand(RobotContainer container) {
    this(container, false);
  }
}
