package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimeLight;
import frc.robot.Constants.AutonomousConstants;

public class AlignWithTag extends Command {

    private final DriveSubsystem drivetrain;
    private final LimeLight limelight;

    /**
 * Aligns the robot with the target using the limelight
 */
    public AlignWithTag(DriveSubsystem drivetrain, LimeLight limelight) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        addRequirements(drivetrain);
        addRequirements(limelight);
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Initialization code, if any
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
            limelight.updateLimeLightTracking();
            // Calculate the heading error to the indicated target
            
            double heading_error = limelight.tx;
            double steering_adjust = 0.0;
            if (Math.abs(heading_error) > 1.0) {
                steering_adjust = -AutonomousConstants.Kp * heading_error + (heading_error > 0 ? -AutonomousConstants.steeringAdjust :AutonomousConstants.steeringAdjust);
            }
            drivetrain.drive(0, 0, steering_adjust, false, true);
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(0, 0, 0, false, true);
    }
    
    @Override
    public boolean isFinished() {
        // Update the LimeLight tracking data to get the latest heading error
        limelight.updateLimeLightTracking();
        double heading_error = -limelight.tx;
    
         // Check if the absolute value of the heading error is less than the threshold
         return Math.abs(heading_error) < AutonomousConstants.headingErrorThreshold;
    }
    
}
