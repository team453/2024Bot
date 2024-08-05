package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.UnderBotSubsystemConstants;

public class UnderBotSubsystem extends SubsystemBase {

    private final CANSparkMax m_intake;
    private final CANSparkMax m_leftGuide;
    private final CANSparkMax m_rightGuide;
    private final CANSparkMax m_shooter;
    private final SparkPIDController  m_shooterPIDController;
    private String state;
  
  /** Creates a new ExampleSubsystem. */
  public UnderBotSubsystem() {
    m_intake = new CANSparkMax(UnderBotSubsystemConstants.kIntakeMotorCanId, MotorType.kBrushless);
    m_shooter = new CANSparkMax(UnderBotSubsystemConstants.kShooterMotorCanId, MotorType.kBrushless);
    m_leftGuide = new CANSparkMax(UnderBotSubsystemConstants.kLeftGuideMotorCanId, MotorType.kBrushed);
    m_rightGuide = new CANSparkMax(UnderBotSubsystemConstants.kRightGuideMotorCanId, MotorType.kBrushed);

    m_shooterPIDController = m_shooter.getPIDController();

    // PID Coefficients, these should be tuned for your specific robot
    m_shooterPIDController.setP(0.00000481);
    m_shooterPIDController.setI(0.00000000015);
    m_shooterPIDController.setD(0);
    m_shooterPIDController.setFF(0.00015);
    m_shooterPIDController.setOutputRange(-1, 0);

  state = "Ready";
  }

  @Override
    public void periodic() {
        SmartDashboard.putString("Underbot State", state);
        SmartDashboard.putNumber("intake volts", UnderBotSubsystem.this.m_intake.getOutputCurrent());
    }
    
    public void setIntakeMotor(double speed) {
    m_intake.set(speed);
    }

    public void setShooterMotor(double speed) {
    m_shooter.set(speed);
    }

    // Methods for intake and eject guide wheels
    public void intakeGuideWheels() {
        m_leftGuide.set(UnderBotSubsystemConstants.kGuideWheelIntakeSpeed);
         m_rightGuide.set(UnderBotSubsystemConstants.kGuideWheelIntakeSpeed);
}

    public void ejectGuideWheels() {
    m_leftGuide.set(UnderBotSubsystemConstants.kGuideWheelEjectSpeed);
    m_rightGuide.set(UnderBotSubsystemConstants.kGuideWheelEjectSpeed);
    }

     public void stopGuideWheels() {
    m_leftGuide.set(0);
    m_rightGuide.set(0);
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
            UnderBotSubsystem.this.setIntakeMotor(UnderBotSubsystemConstants.kIntakeSpeed);
            UnderBotSubsystem.this.intakeGuideWheels();
    }

    @Override
    public void end(boolean interrupted) {
        // Command end action: Stop the motor, whether the command ends normally or is interrupted
        UnderBotSubsystem.this.setIntakeMotor(0);
        stopGuideWheels();
        state = "Idle";
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
             state = "Floor eject";
            UnderBotSubsystem.this.setIntakeMotor(UnderBotSubsystemConstants.kEjectSpeed);
            UnderBotSubsystem.this.m_shooter.set(0.2);
            UnderBotSubsystem.this.ejectGuideWheels();
    }

    @Override
    public void end(boolean interrupted) {
        // Command end action: Stop the motor, whether the command ends normally or is interrupted
        UnderBotSubsystem.this.setIntakeMotor(0); 
         UnderBotSubsystem.this.m_shooter.set(0);
        stopGuideWheels();
        state = "Idle";
    }

}

public class ShootCommand extends Command {
    private final double speed;
    Timer timer = new Timer();
    
    public ShootCommand(double speed) {
        timer.reset();
        this.speed = speed;
        // Add requirements to ensure this command has exclusive access to the IntakeSubsystem
        addRequirements(UnderBotSubsystem.this);
        
    }

     @Override
    public void execute() {
         // Check if 3 seconds have passed since the command started
         
         if (timer.get() >= 0.6) {
            //0.6 seconds after starting, run the intake motor
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
        setShooterRPM(speed);
    }

    @Override
    public void end(boolean interrupted) {
        // Command end action: Stop the motors, whether the command ends normally or is interrupted
        setShooterMotor(0);
       setIntakeMotor(0);
        state = "Idle";
    }

    private void setShooterRPM(double rpm) {
        m_shooterPIDController.setReference(rpm, CANSparkBase.ControlType.kVelocity);
}
}

public class ShootCrazyCommand extends Command {
    private final double speed;
    Timer timer = new Timer();
    
    public ShootCrazyCommand(double speed) {
        timer.reset();
        this.speed = speed;
        // Add requirements to ensure this command has exclusive access to the IntakeSubsystem
        addRequirements(UnderBotSubsystem.this);
        
    }

     @Override
    public void execute() {
         // Check if 3 seconds have passed since the command started
         
         if (timer.get() >= 0.6) {
            //0.6 seconds after starting, run the intake motor
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
        setShooterRPM(speed);
    }

    @Override
    public void end(boolean interrupted) {
        // Command end action: Stop the motors, whether the command ends normally or is interrupted
        setShooterMotor(0);
       setIntakeMotor(0);
        state = "Idle";
    }

    private void setShooterRPM(double rpm) {
        m_shooterPIDController.setReference(rpm, CANSparkBase.ControlType.kVelocity);
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
         // Check if 1 seconds have passed since the command started
         if (timer.get() >= 1) {
            state = "Amp Shoot";
            // 1 seconds after starting, run the intake motor
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
        setShooterRPM(speed1);
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

     private void setShooterRPM(double rpm) {
        m_shooterPIDController.setReference(rpm, CANSparkBase.ControlType.kVelocity);
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
       setIntakeMotor(UnderBotSubsystemConstants.kEjectSpeed);
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
       setIntakeMotor(UnderBotSubsystemConstants.kEjectSpeed);
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

public class StopCommand extends Command {
    
    public StopCommand() {
      
        // Add requirements to ensure this command has exclusive access to the IntakeSubsystem
        addRequirements(UnderBotSubsystem.this);
    }

    @Override
    public void execute() {
        state = "Stopping";
        UnderBotSubsystem.this.setShooterMotor(0);
        UnderBotSubsystem.this.setIntakeMotor(0);
    }

    @Override
    public void end(boolean interrupted) {
        // Command end action: Stop the motor, whether the command ends normally or is interrupted
        state = "Idle";
    }

  
}

public Command StopUnderbot()
{
  return new RunCommand(() -> {
   //stop all motors
   m_intake.set(0);
   m_shooter.set(0);
   stopGuideWheels();
}); 
}

  
  //auto stuff
  public class StartShooterCommand extends Command {

    public StartShooterCommand() {

    }

    @Override
    public void initialize() {
        state = "Start Shooter";
         m_shooterPIDController.setReference(UnderBotSubsystemConstants.kLaunchShooterRPM, CANSparkBase.ControlType.kVelocity);
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
        //stop the guide wheels
        stopGuideWheels();
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
        //stop shooter
        UnderBotSubsystem.this.setShooterMotor(0);
        UnderBotSubsystem.this.setIntakeMotor(0.6);
        UnderBotSubsystem.this.intakeGuideWheels();
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


public class StartEjectCommand extends Command {
    

    public StartEjectCommand() {
    }

    @Override
    public void initialize() {
        state = "Started Intake";
        UnderBotSubsystem.this.setIntakeMotor(-0.7f);
        UnderBotSubsystem.this.ejectGuideWheels();
    }

    @Override
    public boolean isFinished() {
        state = "Active Eject";
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
     stopGuideWheels();
    }

    @Override
    public boolean isFinished() {
        state = "Idle";
        return true; // This command completes immediately after stopping the motors
    }
}

// Then, to sequence them:
public class SequentialShootCommand extends SequentialCommandGroup {
    public SequentialShootCommand() {
        addCommands(
            new LoadNoteCommand(),
            new WaitCommand(0.2),
            new StopMotorsCommand(),
            new StartShooterCommand(),
            new WaitCommand(0.6),
            new StartFeederCommand(),
            new WaitCommand(0.2)
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

public class SequentialEjectCommand extends SequentialCommandGroup
{
      public SequentialEjectCommand() {
        addCommands(
            new StartEjectCommand(),
            new WaitCommand(4),
            new StopMotorsCommand()
        );
    }
}
public class LoadNoteCommand extends Command {
 Timer timer = new Timer();
    public LoadNoteCommand() {
    }

    @Override
    public void initialize() {
       timer.start();
        timer.reset();
    stopGuideWheels();
     UnderBotSubsystem.this.setIntakeMotor(UnderBotSubsystemConstants.kEjectSpeed);
     
    }

    @Override
    public boolean isFinished() {
        state = "Loading Note";
        //if grabbed note or waited more than 0.3 sec
        if(UnderBotSubsystem.this.m_intake.getOutputCurrent() > 48  || timer.get() > 0.6)
        {
            //don't stop the motor
            return true;
        }
        else
        {
            return false;
        } 


    }
}
}


