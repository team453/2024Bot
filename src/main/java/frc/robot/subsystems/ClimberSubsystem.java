package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    private final CANSparkMax m_extenderMotor;
    private final CANSparkMax m_winchMotor;

    public ClimberSubsystem() {
        m_extenderMotor = new CANSparkMax(ClimberConstants.kExtenderMotorCanId, MotorType.kBrushless);
        m_winchMotor = new CANSparkMax(ClimberConstants.kWinchMotorCanId, MotorType.kBrushless);
    }

    public void moveHook(double speed) {
        m_extenderMotor.set(speed);
    }

    public void moveWinch(double speed) {
        m_winchMotor.set(speed);
    }

    // Inner class for moving the hook
    public class MoveHookCommand extends Command {
        double speed;
        public MoveHookCommand(double speed) {
            this.speed = speed;
            addRequirements(ClimberSubsystem.this);
        }

        @Override
        public void execute() {
            moveHook(speed);
        }

        @Override
        public void end(boolean interrupted) {
            moveHook(0);
        }
    }

    // Inner class for pulling the winch
    public class MoveWinchCommand extends Command {
        double speed;
        public MoveWinchCommand(double speed) {
            this.speed = speed;
            addRequirements(ClimberSubsystem.this);
        }

        @Override
        public void execute() {
            moveWinch(ClimberConstants.kWinchPullInSpeed);
        }

        @Override
        public void end(boolean interrupted) {
            moveWinch(0);
        }
    }

}
