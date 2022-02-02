package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.*;

public class DriveResetPositionCommand extends InstantCommand {

    /** The subsystem to execute this command on. */
    private final DriveTrain m_driveTrain;
    
    /**
     * Default constructor
     *
     * @param subsystem The subsystem to execute this command on.
     */
    public DriveResetPositionCommand(DriveTrain subsystem) {
        m_driveTrain = subsystem;
    }
    
    /** Log when this command is initialized */
    @Override
    public void initialize() {
        Shuffleboard.addEventMarker("ResetPosition init.", this.getClass().getSimpleName(), EventImportance.kNormal);
    }
    
    /** Do the state change. */
    @Override
    public void execute() {
        m_driveTrain.resetPosition();
    }
    
    /** Log when this command ends */
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            Shuffleboard.addEventMarker("ResetPosition Interrupted!", this.getClass().getSimpleName(), EventImportance.kNormal);
        }
        Shuffleboard.addEventMarker("ResetPosition end.", this.getClass().getSimpleName(), EventImportance.kNormal);
    }
}
