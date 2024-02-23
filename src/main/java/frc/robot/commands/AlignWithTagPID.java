package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimeLight;
import frc.robot.Constants.AutonomousConstants;

public class AlignWithTagPID extends Command {

    private final DriveSubsystem drivetrain;
    private final LimeLight limelight;
    private final PIDController controller;

    /**
     * Aligns the robot with the target using the limelight based on PID control
     */
    public AlignWithTagPID(DriveSubsystem drivetrain, LimeLight limelight) {
        controller = new PIDController(AutonomousConstants.Kp, AutonomousConstants.Ki, AutonomousConstants.Kd);
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        addRequirements(drivetrain, limelight);
    }

    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
        @Override
         public void execute() {
         limelight.updateLimeLightTracking();
         // Calculate the heading error to the indicated target
         
         double heading_error = -limelight.tx;
         double steering_adjust = controller.calculate(heading_error, 0);
         drivetrain.drive(0, 0, steering_adjust, false, true);
     
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
