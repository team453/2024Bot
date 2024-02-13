package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimeLight;

public class AlignWithTag extends Command {

    private final DriveSubsystem drivetrain;
    private final LimeLight limelight;
    private static final double Kp = -0.1;
    private static final double min_command = 0.05;

    // Constructor
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
            double heading_error = -limelight.tx;
            double steering_adjust = 0.0;
            if (Math.abs(heading_error) > 1.0) {
                steering_adjust = Kp * heading_error + (heading_error > 0 ? -min_command : min_command);
            }
            drivetrain.drive(0, 0, steering_adjust, false, true);
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(0, 0, 0, false, true);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false; // Modify this as needed based on your command's ending condition
    }
}
