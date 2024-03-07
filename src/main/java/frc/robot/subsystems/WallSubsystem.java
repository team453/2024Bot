package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WallSubsystemConstants;

public class WallSubsystem extends SubsystemBase {
    private final TalonFX m_wallMotor;

    public WallSubsystem() {
        m_wallMotor = new TalonFX(WallSubsystemConstants.kWallMotorCanId);

        // Set neutral mode to brake to hold the wall position when motor power is not applied
        m_wallMotor.setNeutralMode(NeutralModeValue.Brake);

        // Set the safety enabled to ensure the motor stops if not regularly updated
        m_wallMotor.setSafetyEnabled(true);
        m_wallMotor.setExpiration(0.1); // 100ms expiration time for the safety feature
    }

    
    public void invertDirection(boolean isInverted) {
        // Invert the motor direction if necessary
        m_wallMotor.setInverted(isInverted);
    }

    @Override
    public void periodic() {
        // Feed the motor safety object to ensure the motor does not get disabled
        m_wallMotor.feed();
    }

    public void setWallMotor(double speed)
    {
        m_wallMotor.set(speed);
    }

    // Inner class for operating the wall
    public class MoveWallCommand extends Command {
    private final double speed;

    public MoveWallCommand(double speed) {
        this.speed = speed;
        // Add requirements to ensure this command has exclusive access to the WallSubsystem
        addRequirements(WallSubsystem.this);
    }

    @Override
    public void execute() {
      
        // Acquire a refreshed TalonFX rotor position signal
        var rotorPosSignal = m_wallMotor.getRotorPosition();

        // Retrieve position value that we just refreshed, units are rotations
        var rotorPos = rotorPosSignal.getValue();
        SmartDashboard.putNumber("Wall Rotation", rotorPos);
        
        //if not at limit it can move down
        if(speed < 0)
        {
            if(rotorPos > WallSubsystemConstants.kBottomLimit)
            {
                 setWallMotor(speed);
            }
            else
            {
                setWallMotor(0);
            }
        }
       
         //if not at limit it can move up
        if(speed > 0)
        {
            if(rotorPos > WallSubsystemConstants.kTopLimit)
            {
                 setWallMotor(speed);
            }
            else
            {
                setWallMotor(0);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Command end action: Stop the wall motor, whether the command ends normally or is interrupted
        setWallMotor(0);
    }

    

    private void setWallMotor(double speed) {
        WallSubsystem.this.setWallMotor(speed); // Set the wall motor's speed
    }
}

}
