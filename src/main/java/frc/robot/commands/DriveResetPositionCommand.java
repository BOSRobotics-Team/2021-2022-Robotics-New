package frc.robot.commands;

import frc.robot.*;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class DriveResetPositionCommand extends InstantCommand {

    /** The subsystem to execute this command on. */
    private final DriveTrain m_driveTrain;
    
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
        Shuffleboard.addEventMarker("DriveResetPositionCommand init.", this.getClass().getSimpleName(), EventImportance.kNormal);
        System.out.println("DriveResetPositionCommand - init");
    }
    
    /** Do the state change. */
    @Override
    public void execute() {
        m_driveTrain.resetPosition();
    }
    
    /** Log when this command ends */
    @Override
    public void end(boolean interrupted) {
        System.out.println("DriveResetPositionCommand - end : interrupted = " + interrupted);
        if (interrupted) {
            Shuffleboard.addEventMarker("DriveResetPositionCommand Interrupted!", this.getClass().getSimpleName(), EventImportance.kNormal);
        }
        Shuffleboard.addEventMarker("DriveResetPositionCommand end.", this.getClass().getSimpleName(), EventImportance.kNormal);
    }
}
