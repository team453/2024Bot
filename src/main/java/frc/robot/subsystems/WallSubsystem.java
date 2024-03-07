package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WallSubsystemConstants;

public class WallSubsystem extends SubsystemBase {
    private final CANSparkMax m_wallMotor;

    public WallSubsystem() {
        // Initialize the wall motor with its CAN ID and motor type
        m_wallMotor = new CANSparkMax(WallSubsystemConstants.kWallMotorCanId, MotorType.kBrushless);
    }

    public void moveWall(double speed) {
        // Set the motor speed, where positive is up and negative is down
        m_wallMotor.set(speed);
    }

    public void stopWall() {
        // Stop the wall motor
        m_wallMotor.set(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
