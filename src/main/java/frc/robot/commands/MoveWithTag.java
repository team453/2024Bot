package frc.robot.commands;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimeLight;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.subsystems.DriveSubsystem;


public class MoveWithTag extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final LimeLight m_LimeLight;
    private final DriveSubsystem m_Drivetrain;
  
    /**
     * Creates a new AutoDrive.
     *
     * @param subsystem The subsystem used by this command.
     */
    public MoveWithTag(LimeLight subsystem1, DriveSubsystem subsystem2) {
        m_LimeLight = subsystem1;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(subsystem1);

      m_Drivetrain = subsystem2;

      addRequirements(subsystem2);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
  
    @Override
    public void execute() {
        // Check if there's a valid target
        if (!m_LimeLight.m_ValidTarget) {
            return; // No valid target, do nothing
        }
    
        double yAdjust = 0.0;
        double xAdjust = 0.0;
        double rotateAdjust = 0.0; 
        double desiredTa = 1; // Desired target area percentage
    
        double taError = m_LimeLight.ta - desiredTa;
        yAdjust = (taError * AutonomousConstants.kAutoCorrectSpeed); 

        xAdjust = (m_LimeLight.tx * AutonomousConstants.kAutoCorrectStrafe); 
/* 
        // Proportional control for rotation based on skew
        if (m_LimeLight.ts > 5.0) { 
            rotateAdjust = (m_LimeLight.ts * AutonomousConstants.kAutoCorrectTurn); 
        } else if (m_LimeLight.ts < -1.0) {
            rotateAdjust = (m_LimeLight.ts * AutonomousConstants.kAutoCorrectTurn);
        }
    */
        // Command the drivetrain to adjust position, orientation, and rotation
        SmartDashboard.putNumber("moveX", xAdjust);
        SmartDashboard.putNumber("moveY", yAdjust);
        SmartDashboard.putNumber("rotate", rotateAdjust);
        m_Drivetrain.drive(xAdjust, yAdjust, rotateAdjust, false, true); // Combine turn and rotation adjustments
    }
    

  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}