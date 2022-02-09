package frc.robot.unused;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

/** Very simple unidirectional drive control. */
public class SimpleUnidirectionalDriveCommand extends CommandBase {

  /** The OI used for input. */
  public final XboxController m_controller;

  /** The subsystem to execute this command on. */
  private final DriveTrain m_driveTrain;

  /**
   * Default constructor
   *
   * @param subsystem The subsystem to execute this command on
   * @param controller The OI that gives the input to this command.
   */
  public SimpleUnidirectionalDriveCommand(DriveTrain subsystem, XboxController controller) {
    m_controller = controller;
    m_driveTrain = subsystem;
    // Default commands need to require their subsystems.
    addRequirements(subsystem);
  }

  /** Stop the drive for safety reasons. */
  @Override
  public void initialize() {
    m_driveTrain.fullStop();
  }

  /** Give output to the motors based on the stick inputs. */
  @Override
  public void execute() {
    m_driveTrain.drive(this.m_controller);
  }

  /**
   * Run constantly because this is a default drive
   *
   * @return false
   */
  @Override
  public boolean isFinished() {
    return false;
  }

  /** Log and brake when interrupted. */
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      Shuffleboard.addEventMarker(
          "SimpleUnidirectionalDrive Interrupted! Stopping the robot.",
          this.getClass().getSimpleName(),
          EventImportance.kNormal);
    }
    // Brake for safety!
    m_driveTrain.fullStop();
  }
}
