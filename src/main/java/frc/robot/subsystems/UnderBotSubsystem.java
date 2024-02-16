package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.UnderBotSubsystemConstants;

public class UnderBotSubsystem extends SubsystemBase {

    private final CANSparkMax m_intake;
    private final CANSparkMax m_shooter;

    //beam break sensor
    private final AnalogInput m_beamBreak;    
  /** Creates a new ExampleSubsystem. */
  public UnderBotSubsystem() {
    m_beamBreak = new AnalogInput(UnderBotSubsystemConstants.kBeamBreakAnalogPort);
    m_intake = new CANSparkMax(UnderBotSubsystemConstants.kIntakeMotorCanId, MotorType.kBrushless);
    m_shooter = new CANSparkMax(UnderBotSubsystemConstants.kShooterMotorCanId, MotorType.kBrushed);
    // Sets the AnalogInput to 8-bit averaging.  64 samples will be averaged together.
    // The update rate will decrease by a factor of 64.
    m_beamBreak.setAverageBits(8);
  }
  

   public Command IntakeCommand() {
      return new FunctionalCommand(
    // No initialization action needed for the intake command
    () -> {},

    // Command execution: Run the motor unless the beam break is tripped
    () -> {
        if (!isBeamBroken()) {
            m_intake.set(UnderBotSubsystemConstants.kIntakeSpeed);
        } else {
            m_intake.set(0);
        }
    },

    // Command end action: Stop the motor, whether the command ends normally or is interrupted
    interrupted -> m_intake.set(0),

    // Command is finished when the beam break sensor is tripped
    this::isBeamBroken,

    // Require the intake subsystem
    this
);

}
/*// Create a new run command that will run the motor
   */

public Command EjectCommand() {
  // The startEnd helper method takes a method to call when the command is initialized and one to
    // call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          m_intake.set(UnderBotSubsystemConstants.kOuttakeSpeed);
        },
        // When the command stops, stop the wheels
        () -> {
         m_intake.set(0);
        });

}

public Command HighSpeedShootCommand() {
 return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          m_shooter.set(UnderBotSubsystemConstants.kHighShooterSpeed);
          m_intake.set(UnderBotSubsystemConstants.kIntakeFeederSpeed);
        },
        // When the command stops, stop the wheels
        () -> {
         m_intake.set(0);
          m_shooter.set(0);
        });

}

public Command LowSpeedShootCommand() {
     return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          m_shooter.set(UnderBotSubsystemConstants.kLowShooterSpeed);
          m_intake.set(UnderBotSubsystemConstants.kIntakeFeederSpeed);
        },
        // When the command stops, stop the wheels
        () -> {
         m_intake.set(0);
          m_shooter.set(0);
        });

}

public Command StopUnderbot()
{
  return new RunCommand(() -> {
   //stop all motors
   m_intake.set(0);
   m_shooter.set(0);
}, this); 
}


  public double getBeamBreak() {
    return m_beamBreak.getValue();
  }
  
  public boolean isBeamBroken() {
    return getBeamBreak() > UnderBotSubsystemConstants.kBeamBreakThreshold;
  }

  @Override
  public void periodic() {
        // This method will be called once per scheduler run
    SmartDashboard.putNumber("UnderBot Beam Break Sensor", m_beamBreak.getValue());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}