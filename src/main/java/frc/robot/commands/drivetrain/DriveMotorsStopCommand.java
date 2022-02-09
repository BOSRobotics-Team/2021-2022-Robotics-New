package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.*;
import frc.robot.subsystems.*;

/** Stops the motors of the given drive subsystem. */
public class DriveMotorsStopCommand extends InstantCommand {

  /** The subsystem to execute this command on. */
  private final DriveTrain m_driveTrain;

  /**
   * Default constructor
   *
   * @param subsystem The subsystem to execute this command on.
   */
  public DriveMotorsStopCommand(RobotContainer container) {
    m_driveTrain = container.driveTrain;
  }

  /** Log when this command is initialized */
  @Override
  public void initialize() {
    Shuffleboard.addEventMarker(
        "DriveMotorsStopCommand init.", this.getClass().getSimpleName(), EventImportance.kNormal);
    System.out.println("DriveMotorsStopCommand - init");
  }

  /** Do the state change. */
  @Override
  public void execute() {
    m_driveTrain.fullStop();
  }

  /** Log when this command ends */
  @Override
  public void end(boolean interrupted) {
    System.out.println("DriveMotorsStopCommand - end : interrupted = " + interrupted);
    if (interrupted) {
      Shuffleboard.addEventMarker(
          "DriveMotorsStopCommand Interrupted!",
          this.getClass().getSimpleName(),
          EventImportance.kNormal);
    }
    Shuffleboard.addEventMarker(
        "DriveMotorsStopCommand end.", this.getClass().getSimpleName(), EventImportance.kNormal);
  }
}
