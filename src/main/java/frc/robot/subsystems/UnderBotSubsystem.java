package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.UnderBotSubsystemConstants;

public class UnderBotSubsystem extends SubsystemBase {

    //the whole thing is controlled by one neo
    private final CANSparkMax m_motor;

    //beam break sensor
    private final DigitalInput m_beamBreak;
    
  /** Creates a new ExampleSubsystem. */
  public UnderBotSubsystem() {
    m_motor = new CANSparkMax(UnderBotSubsystemConstants.kMotorCanId, MotorType.kBrushless);
    m_beamBreak = new DigitalInput(UnderBotSubsystemConstants.kBeamBreakDioPort);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  
   public Command IntakeCommand() {
    // Create a new run command that will run the motor
    return new RunCommand(() -> {
        // Continuously check if the beam break sensor is tripped
        if (getBeamBreak()) {
            // If the beam break is tripped, stop the motor
            m_motor.set(0);
        } else {
            // If the beam break is not tripped, keep running the motor
            m_motor.set(UnderBotSubsystemConstants.kIntakeSpeed); 
        }
    }, this) {
        // Override the isFinished method to stop the command when the beam break is tripped
        @Override
        public boolean isFinished() {
            return getBeamBreak();
        }
    };
}

public Command stop()
{
  return new RunCommand(() -> {
    m_motor.set(0);
  }, this);

}

  public Command ShooterCommand() {
    //run the motor for a set speed
    return new RunCommand(() -> {
        m_motor.set(UnderBotSubsystemConstants.kShooterSpeed);
    }, this);
    }


  
  public boolean getBeamBreak() {
    return !m_beamBreak.get();
  }
  
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("UnderBot Beam Break Sensor", !m_beamBreak.get());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}