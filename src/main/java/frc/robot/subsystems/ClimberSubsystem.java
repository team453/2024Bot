package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.fasterxml.jackson.databind.deser.std.StdScalarDeserializer;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.WallSubsystemConstants;

public class ClimberSubsystem extends SubsystemBase {
    private final CANSparkMax m_wench;
    private final CANSparkMax m_hook;
    private String state;

    public ClimberSubsystem() {
        m_wench = new CANSparkMax(ClimberConstants.kWenchMotorCanId, MotorType.kBrushless);
        m_hook = new CANSparkMax(ClimberConstants.kClimberMotorCanId, MotorType.kBrushless);
        state = "Ready";
    }

    @Override
    public void periodic() {
       SmartDashboard.putString("Climber State", state);
    }

    // Inner class for operating the hook
    public class MoveHookCommand extends Command {
    private boolean isMovingUp;

    public MoveHookCommand(boolean isMovingUp) {
        this.isMovingUp = isMovingUp;
        // Add requirements to ensure this command has exclusive access to the WallSubsystem
        addRequirements(ClimberSubsystem.this);
    }

    @Override
    public void execute() {
      if(isMovingUp)
      {
        state = "Moving Hook Up";
        m_hook.set(ClimberConstants.kHookSpeed);
      }
      else
      {
        state = "Moving Hook Down";
        m_hook.set(-ClimberConstants.kHookSpeed);
      }
    }

    @Override
    public void end(boolean interrupted) {
        // Command end action: Stop the motor, whether the command ends normally or is interrupted
        m_hook.set(0);
        state = "Idle";
    }
    }
    // Inner class for operating the wall
    public class MoveWenchCommand extends Command {
    private boolean isPullingUp;

    public MoveWenchCommand(boolean isPullingUp) {
        this.isPullingUp = isPullingUp;
        // Add requirements to ensure this command has exclusive access to the WallSubsystem
        addRequirements(ClimberSubsystem.this);
    }

    @Override
    public void execute() {
      if(isPullingUp)
      {
        state = "Pulling Wench";
        m_wench.set(ClimberConstants.kWenchSpeed);
      }
      else
      {
        state = "Releasing Wench";
        m_wench.set(-ClimberConstants.kWenchSpeed);
      }
    }

     @Override
    public void end(boolean interrupted) {
        // Command end action: Stop the motor, whether the command ends normally or is interrupted
        m_wench.set(0);
        state = "Idle";
    }
    }
}



