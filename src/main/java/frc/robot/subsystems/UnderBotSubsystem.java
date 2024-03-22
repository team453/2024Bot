package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.UnderBotSubsystemConstants;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class UnderBotSubsystem extends SubsystemBase {

    private final CANSparkMax m_intake;
    private final CANSparkMax m_shooter;
    private String state;
  
  /** Creates a new ExampleSubsystem. */
  public UnderBotSubsystem() {
    m_intake = new CANSparkMax(UnderBotSubsystemConstants.kIntakeMotorCanId, MotorType.kBrushless);
    m_shooter = new CANSparkMax(UnderBotSubsystemConstants.kShooterMotorCanId, MotorType.kBrushless);
  state = "Ready";
  }

  @Override
    public void periodic() {
        SmartDashboard.putString("Underbot State", state);
    }
    
    public void setIntakeMotor(double speed) {
    m_intake.set(speed);
    }

    public void setShooterMotor(double speed) {
    m_shooter.set(speed);
    }



  // Inner class for operating the intake
public class IntakeCommand extends Command {
    public IntakeCommand() {
        // Add requirements to ensure this command has exclusive access to the IntakeSubsystem
        addRequirements(UnderBotSubsystem.this);
    }

    @Override
    public void execute() {
             state = "Floor intake";
            setIntakeMotor(UnderBotSubsystemConstants.kIntakeSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        // Command end action: Stop the motor, whether the command ends normally or is interrupted
        setIntakeMotor(0);
        state = "Idle";
    }


    private void setIntakeMotor(double speed) {
        UnderBotSubsystem.this.setIntakeMotor(speed);
    }
}

 // Inner class for operating the intake
public class EjectCommand extends Command {
    public EjectCommand() {
        // Add requirements to ensure this command has exclusive access to the IntakeSubsystem
        addRequirements(UnderBotSubsystem.this);
    }

    @Override
    public void execute() {
        // Command execution: Run the motor unless the beam break is tripped
            state = "Floor Eject";
            setIntakeMotor(UnderBotSubsystemConstants.kOuttakeSpeed);
       
    }

    @Override
    public void end(boolean interrupted) {
        // Command end action: Stop the motor, whether the command ends normally or is interrupted
        setIntakeMotor(0);
        state = "Idle";
    }

   

    private void setIntakeMotor(double speed) {
        UnderBotSubsystem.this.setIntakeMotor(speed);
    }
}

public class ShootCommand extends Command {
    private final double speed;
    Timer timer = new Timer();
    public ShootCommand(double speed) {
        this.speed = speed;
        // Add requirements to ensure this command has exclusive access to the IntakeSubsystem
        addRequirements(UnderBotSubsystem.this);
        
    }

     @Override
    public void execute() {
         // Check if 3 seconds have passed since the command started
         if (timer.get() >= 0.5) {
            // 3 seconds after starting, run the intake motor
            state = "Shooting";
            setIntakeMotor(UnderBotSubsystemConstants.kIntakeFeederSpeed);
        }
        else{
            setIntakeMotor(0);
        }
    }


    @Override
    public void initialize() {
        timer.start();
        timer.reset();
        state = "Preparing Shoot";
        // Start the shooter motor at the specified speed
        setShooterMotor(speed);
    }

    @Override
    public void end(boolean interrupted) {
        // Command end action: Stop the motors, whether the command ends normally or is interrupted
        setShooterMotor(0);
        setIntakeMotor(0);
        state = "Idle";
    }

    private void setShooterMotor(double speed) {
        UnderBotSubsystem.this.setShooterMotor(speed);
    }

    private void setIntakeMotor(double speed) {
        UnderBotSubsystem.this.setIntakeMotor(speed);
    }
}


public class ShootAmpCommand extends Command {
    private final double speed1;
    private final double speed2;
    Timer timer = new Timer();
    public ShootAmpCommand(double speed1, double speed2) {
        this.speed1 = speed1;
        this.speed2 = speed2;
        // Add requirements to ensure this command has exclusive access to the IntakeSubsystem
        addRequirements(UnderBotSubsystem.this);
    }

     @Override
    public void execute() {
         // Check if 3 seconds have passed since the command started
         if (timer.get() >= 0.5) {
            state = "Amp Shoot";
            // 3 seconds after starting, run the intake motor
            setIntakeMotor(speed2);
        }
        else{
            setIntakeMotor(0);
        }
    }


    @Override
    public void initialize() {
        timer.start();
        timer.reset();
        state = "Preparing Amp Shoot";
        // Start the shooter motor at the specified speed
        setShooterMotor(speed1);
    }

    @Override
    public void end(boolean interrupted) {
        // Command end action: Stop the motors, whether the command ends normally or is interrupted
        setShooterMotor(0);
        setIntakeMotor(0);
        state = "Idle";
    }

    private void setShooterMotor(double speed) {
        UnderBotSubsystem.this.setShooterMotor(speed);
    }

    private void setIntakeMotor(double speed) {
        UnderBotSubsystem.this.setIntakeMotor(speed);
    }
}

// Inner class for thing
public class SourceIntakeCommand extends Command {
    double speed;
    public SourceIntakeCommand(double speed) {
        this.speed = speed;
        // Add requirements to ensure this command has exclusive access to the IntakeSubsystem
        addRequirements(UnderBotSubsystem.this);
    }

    @Override
    public void execute() {
        state = "Source Intake";
       setShooterMotor(speed);
       setIntakeMotor(UnderBotSubsystemConstants.kOuttakeSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        // Command end action: Stop the motor, whether the command ends normally or is interrupted
        setShooterMotor(0);
        setIntakeMotor(0);
        state = "Idle";
    }

     private void setShooterMotor(double speed) {
        UnderBotSubsystem.this.setShooterMotor(speed);
    }

    private void setIntakeMotor(double speed) {
        UnderBotSubsystem.this.setIntakeMotor(speed);
    }
}

//reverse the shooter for that kind of intake
public class ReverseIntakeCommand extends Command {
    double speed;
    public ReverseIntakeCommand(double speed) {
        this.speed = -speed;
        // Add requirements to ensure this command has exclusive access to the IntakeSubsystem
        addRequirements(UnderBotSubsystem.this);
    }

    @Override
    public void execute() {
        state = "Reverse Intake";
       setShooterMotor(speed);
       setIntakeMotor(UnderBotSubsystemConstants.kOuttakeSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        // Command end action: Stop the motor, whether the command ends normally or is interrupted
        setShooterMotor(0);
        setIntakeMotor(0);
        state = "Idle";
    }

     private void setShooterMotor(double speed) {
        UnderBotSubsystem.this.setShooterMotor(speed);
    }

    private void setIntakeMotor(double speed) {
        UnderBotSubsystem.this.setIntakeMotor(speed);
    }
}

//innerclass for feed
public class PrepareShootCommand extends Command {
    double speed;
    public PrepareShootCommand(double speed) {
        this.speed = speed;
        // Add requirements to ensure this command has exclusive access to the IntakeSubsystem
        addRequirements(UnderBotSubsystem.this);
    }

    @Override
    public void execute() {
        state = "Preparing Shoot";
       setShooterMotor(speed);
    }

    @Override
    public void end(boolean interrupted) {
        // Command end action: Stop the motor, whether the command ends normally or is interrupted
        state = "Idle";
    }

    private void setShooterMotor(double speed) {
        UnderBotSubsystem.this.setShooterMotor(speed);
    }
}

public Command StopUnderbot()
{
  return new RunCommand(() -> {
   //stop all motors
   m_intake.set(0);
   m_shooter.set(0);
}, this); 
}

  
  //auto stuff
  public class StartShooterCommand extends Command {
    private final double speed;

    public StartShooterCommand(double speed) {
        this.speed = speed;
    }

    @Override
    public void initialize() {
        state = "Start Shooter";
       UnderBotSubsystem.this.setShooterMotor(speed);
    }

    @Override
    public boolean isFinished() {
        state = "Active Shooter";
        return true; 
        // This command completes immediately after starting the motor
        //It does NOT STOP THE MOTOR
        //Use other commands to do that please :)
    }
}

public class StartFeederCommand extends Command {
    

    public StartFeederCommand() {
    }

    @Override
    public void initialize() {
        state = "Start Feeder";
        UnderBotSubsystem.this.setIntakeMotor(UnderBotSubsystemConstants.kIntakeFeederSpeed);
    }

    @Override
    public boolean isFinished() {
        state = "Active Feeder";
        return true; 
        // This command completes immediately after starting the motor
        //It does NOT STOP THE MOTOR
        //Use other commands to do that please :)
    }
}

public class StartIntakeCommand extends Command {
    

    public StartIntakeCommand() {
    }

    @Override
    public void initialize() {
        state = "Started Intake";
        UnderBotSubsystem.this.setIntakeMotor(UnderBotSubsystemConstants.kIntakeSpeed);
    }

    @Override
    public boolean isFinished() {
        state = "Active Intake";
        return true; 
        // This command completes immediately after starting the motor
        //It does NOT STOP THE MOTOR
        //Use other commands to do that please :)
    }
}

public class StopMotorsCommand extends Command {

    public StopMotorsCommand() {
    }

    @Override
    public void initialize() {
        state = "Stopping motors";
    UnderBotSubsystem.this.setShooterMotor(0);
     UnderBotSubsystem.this.setIntakeMotor(0);
    }

    @Override
    public boolean isFinished() {
        state = "Idle";
        return true; // This command completes immediately after stopping the motors
    }
}

// Then, to sequence them:
public class SequentialShootCommand extends SequentialCommandGroup {
    public SequentialShootCommand(double shooterSpeed) {
        addCommands(
            new StartShooterCommand(shooterSpeed),
            new WaitCommand(0.5),
            new StartFeederCommand(),
             new WaitCommand(3.0),
            new StopMotorsCommand()
        );
    }
}

public class SequentialIntakeCommand extends SequentialCommandGroup
{
      public SequentialIntakeCommand() {
        addCommands(
            new StartIntakeCommand(),
            new WaitCommand(2),
            new StopMotorsCommand()
        );
    }
}
}

