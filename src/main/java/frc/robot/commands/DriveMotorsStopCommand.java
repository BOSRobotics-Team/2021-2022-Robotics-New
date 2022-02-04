package frc.robot.commands;

import frc.robot.*;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;

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
        Shuffleboard.addEventMarker("StopMotors init.", this.getClass().getSimpleName(), EventImportance.kNormal);
    }

    /** Do the state change. */
    @Override
    public void execute() {
        m_driveTrain.fullStop();
    }

    /** Log when this command ends */
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            Shuffleboard.addEventMarker("StopMotors Interrupted!", this.getClass().getSimpleName(), EventImportance.kNormal);
        }
        Shuffleboard.addEventMarker("StopMotors end.", this.getClass().getSimpleName(), EventImportance.kNormal);
    }
}