package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;

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
        Shuffleboard.addEventMarker("EnableMotors init.", this.getClass().getSimpleName(), EventImportance.kNormal);
        m_driveTrain.enableDriveTrain(true);
    }
    
    /** Do the state change. */
    @Override
    public void execute() {
    }
    
    /** Log when this command ends */
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            Shuffleboard.addEventMarker("EnableMotors Interrupted!", this.getClass().getSimpleName(), EventImportance.kNormal);
        }
        Shuffleboard.addEventMarker("EnableMotors end.", this.getClass().getSimpleName(), EventImportance.kNormal);
    }
}
