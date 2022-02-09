package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;

public class DriveMotorsEnableCommand extends InstantCommand {
  /** The subsystem to execute this command on. */
  private final DriveTrain m_driveTrain;

  /**
   * Default constructor
   *
   * @param subsystem The subsystem to execute this command on.
   */
  public DriveMotorsEnableCommand(RobotContainer container) {
    m_driveTrain = container.driveTrain;
  }

  /** Log when this command is initialized */
  @Override
  public void initialize() {
    Shuffleboard.addEventMarker(
        "DriveMotorsEnableCommand init.", this.getClass().getSimpleName(), EventImportance.kNormal);
    System.out.println("DriveMotorsEnableCommand - init");
  }

  /** Do the state change. */
  @Override
  public void execute() {
    m_driveTrain.enableDriveTrain(true);
  }

  /** Log when this command ends */
  @Override
  public void end(boolean interrupted) {
    System.out.println("DriveMotorsEnableCommand - end : interrupted = " + interrupted);
    if (interrupted) {
      Shuffleboard.addEventMarker(
          "DriveMotorsEnableCommand Interrupted!",
          this.getClass().getSimpleName(),
          EventImportance.kNormal);
    }
    Shuffleboard.addEventMarker(
        "DriveMotorsEnableCommand end.", this.getClass().getSimpleName(), EventImportance.kNormal);
  }
}
