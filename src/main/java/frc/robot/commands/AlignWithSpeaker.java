package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimeLight;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.helpers.LimelightHelpers;

public class AlignWithSpeaker extends Command {

    private final DriveSubsystem drivetrain;
    private final LimeLight limelight;
    private boolean isInAlignmentZone = false;

    public AlignWithSpeaker(DriveSubsystem drivetrain, LimeLight limelight) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        addRequirements(this.drivetrain, this.limelight);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double tx = limelight.tx;
        double ta = limelight.ta;
        double fiducialID = LimelightHelpers.getFiducialID("limelight");
    
        if (fiducialID == 4 || fiducialID == 7) {
            // Calculate the error in tx
            double txError = tx - (AutonomousConstants.TX_MIN + AutonomousConstants.TX_MAX) / 2.0;
    
            // Determine if tx is within the desired range
            boolean txInRange = tx >= AutonomousConstants.TX_MIN && tx <= AutonomousConstants.TX_MAX;
            
            // Calculate steering adjustment based on tx error, only if tx is not in range
            double steeringAdjust = 0.0;
            if (!txInRange) {
                steeringAdjust = AutonomousConstants.KpAim * txError;
                // Ensure minimum command to move the robot if error is non-zero
                if (Math.abs(steeringAdjust) < AutonomousConstants.KpAim) {
                    steeringAdjust = Math.copySign(AutonomousConstants.KpAim, steeringAdjust);
                }
            }
    
            // Adjust for ta if needed, but focus on tx alignment first
            double taAdjust = 0.0;
            if (txInRange) {
                double taError = ta -(AutonomousConstants.TA_MIN + AutonomousConstants.TA_MAX) / 2.0;
                taAdjust = AutonomousConstants.KpDistance * taError;
            }
    
            // Drive the robot. ySpeed is set to 0 as the primary goal is to align tx first
            drivetrain.drive(steeringAdjust, taAdjust, 0, false, true);
        }
    }
    

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(0, 0, 0, false, true);
    }

    @Override
    public boolean isFinished() {
        return isInAlignmentZone;
    }
}