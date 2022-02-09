package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.*;
import frc.robot.subsystems.*;

public class DriveResetPositionCommand extends CommandBase {

  /** The subsystem to execute this command on. */
  private final DriveTrain m_driveTrain;

  private int _counter = 10;

  /**
   * Default constructor
   *
   * @param subsystem The subsystem to execute this command on.
   */
  public DriveResetPositionCommand(RobotContainer container) {
    m_driveTrain = container.driveTrain;
  }

  /** Log when this command is initialized */
  @Override
  public void initialize() {
    Shuffleboard.addEventMarker(
        "DriveResetPositionCommand init.",
        this.getClass().getSimpleName(),
        EventImportance.kNormal);
    System.out.println("DriveResetPositionCommand - init");
    _counter = 10;
    m_driveTrain.resetPosition();
  }

  /** Do the state change. */
  @Override
  public void execute() {
    _counter--;
  }

  /** Log when this command ends */
  @Override
  public void end(boolean interrupted) {
    System.out.println("DriveResetPositionCommand - end : interrupted = " + interrupted);
    if (interrupted) {
      Shuffleboard.addEventMarker(
          "DriveResetPositionCommand Interrupted!",
          this.getClass().getSimpleName(),
          EventImportance.kNormal);
    }
    Shuffleboard.addEventMarker(
        "DriveResetPositionCommand end.", this.getClass().getSimpleName(), EventImportance.kNormal);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return _counter <= 0;
  }
}
