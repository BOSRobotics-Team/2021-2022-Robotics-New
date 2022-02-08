package frc.robot.commands;

import frc.robot.*;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class DriveResetHeadingCommand extends InstantCommand {

    /** The subsystem to execute this command on. */
    private final DriveTrain m_driveTrain;
    
    /**
     * Default constructor
     *
     * @param subsystem The subsystem to execute this command on.
     */
    public DriveResetHeadingCommand(RobotContainer container) {
        m_driveTrain = container.driveTrain;
    }
    
    /** Log when this command is initialized */
    @Override
    public void initialize() {
        Shuffleboard.addEventMarker("DriveResetHeadingCommand init.", this.getClass().getSimpleName(), EventImportance.kNormal);
        System.out.println("DriveResetHeadingCommand - init");
    }
    
    /** Do the state change. */
    @Override
    public void execute() {
        m_driveTrain.zeroHeading();
    }
    
    /** Log when this command ends */
    @Override
    public void end(boolean interrupted) {
        System.out.println("DriveResetHeadingCommand - end : interrupted = " + interrupted);
        if (interrupted) {
            Shuffleboard.addEventMarker("DriveResetHeadingCommand Interrupted!", this.getClass().getSimpleName(), EventImportance.kNormal);
        }
        Shuffleboard.addEventMarker("DriveResetHeadingCommand end.", this.getClass().getSimpleName(), EventImportance.kNormal);
    }
}
