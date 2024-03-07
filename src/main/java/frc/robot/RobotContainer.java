// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AlignWithTag;
import frc.robot.helpers.LimelightHelpers;
import frc.robot.subsystems.ClimberSubsystem.MoveHookCommand;
import frc.robot.subsystems.ClimberSubsystem.MoveWinchCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.UnderBotSubsystem;
import frc.robot.subsystems.WallSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final SendableChooser<Command> autoChooser;
  // The robot's subsystems
  private final DriveSubsystem m_drivetrain = new DriveSubsystem();
  private final PoseEstimatorSubsystem m_poseEstimator = new PoseEstimatorSubsystem(m_drivetrain);
  private final LimeLight m_limeLight = new LimeLight();
  private final AlignWithTag m_AlignWithTag = new AlignWithTag(m_drivetrain, m_limeLight);
    private final UnderBotSubsystem m_underBot = new UnderBotSubsystem();
    private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
    // The robot's subsystems
private final WallSubsystem m_wallSubsystem = new WallSubsystem();



  // Add a boolean variable to track field position state
  private boolean isFieldPositionEnabled = true; // Initialize to true by default

  // The driver's controller
  Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
    Joystick m_operatorController = new Joystick(OIConstants.kOperatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
     autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);
    // Configure the button bindings
    configureButtonBindings();
    // Update the SmartDashboard with the initial field position state
    SmartDashboard.putBoolean("Field Position Enabled", isFieldPositionEnabled);

  // Configure default commands
m_drivetrain.setDefaultCommand(
    // The joystick's Y axis controls forward/backward movement,
    // the X axis controls left/right movement, and
    // Z controls turning.
   new RunCommand(() -> {
          // Toggle the field position state when button 1 is pressed
          isFieldPositionEnabled = true;
          // Update the SmartDashboard with the new state
          SmartDashboard.putBoolean("Field Position Enabled", isFieldPositionEnabled);
          m_drivetrain.drive(
            -MathUtil.applyDeadband(m_driverController.getY(), OIConstants.kDriveDeadband) * OIConstants.kSpeedMultiplier,
            -MathUtil.applyDeadband(m_driverController.getX(), OIConstants.kDriveDeadband) * OIConstants.kSpeedMultiplier,
            -MathUtil.applyDeadband(m_driverController.getZ(), OIConstants.kDriveDeadband) * OIConstants.kSpeedMultiplier,
            true, true);
        }, 
        m_drivetrain));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
     autoChooser.addOption("Example Auto", new PathPlannerAuto("Example Path"));

    new JoystickButton(m_driverController, 9)
        .whileTrue(new RunCommand(
            () -> m_drivetrain.setX(),
             m_drivetrain));

                   //When holding the tri gger, use the driverDriveSpeed
    new JoystickButton(m_driverController, OIConstants.kDriverDisableFieldPositionButton) // Create a new JoystickButton binding for button 9 on m_driver joystick
    .whileTrue(new RunCommand(() -> {
          // Toggle the field position state when button 1 is pressed
          isFieldPositionEnabled = false;
          // Update the SmartDashboard with the new state
          SmartDashboard.putBoolean("Field Position Enabled", isFieldPositionEnabled);
          m_drivetrain.drive(
            -MathUtil.applyDeadband(m_driverController.getY(), OIConstants.kDriveDeadband) * OIConstants.kSpeedMultiplier,
            -MathUtil.applyDeadband(m_driverController.getX(), OIConstants.kDriveDeadband) * OIConstants.kSpeedMultiplier,
            -MathUtil.applyDeadband(m_driverController.getZ(), OIConstants.kDriveDeadband) * OIConstants.kSpeedMultiplier,
            false, true);
        }, 
        m_drivetrain));
    new JoystickButton(m_driverController, OIConstants.kDriverAlignButton).whileTrue(m_AlignWithTag);

    //underbot stuff
    
    new JoystickButton(m_operatorController, OIConstants.kUnderbotIntakeButton) // Binding for trigger on m_operatorController joystick
    .whileTrue(m_underBot.IntakeCommand())
    .onFalse(m_underBot.StopUnderbot());

    new JoystickButton(m_operatorController, OIConstants.kUnderbotEjectButton) // Binding for trigger on m_operatorController joystick
    .whileTrue(m_underBot.EjectCommand());

    new JoystickButton(m_operatorController, OIConstants.kUnderbotShooterHighButton) // Binding for trigger on m_operatorController joystick
    .whileTrue(m_underBot.HighSpeedShootCommand());

    new JoystickButton(m_operatorController, OIConstants.kUnderbotShooterLowButton) // Binding for trigger on m_operatorController joystick
    .whileTrue(m_underBot.LowSpeedShootCommand());

   //wall stuff
    // Wall movement button binding
    new JoystickButton(m_operatorController, OIConstants.kEnableWallButton)
        .whileTrue(new RunCommand(() -> {
            double speed = -MathUtil.applyDeadband(m_driverController.getY(), OIConstants.kDriveDeadband) * OIConstants.kWallSpeedMultiplier;
            if (OIConstants.kUsingWallSpeedAdjustment) {
                // Apply square root function for smoother control
                speed = Math.copySign(Math.sqrt(Math.abs(speed)), speed);
            }
            m_wallSubsystem.moveWall(speed);
        }, m_wallSubsystem));

        //hook stuff
         // Move Hook Up
        new JoystickButton(m_operatorController, OIConstants.kClimberMoveHookUpButton)
            .whileTrue(m_AlignWithTag)(m_climberSubsystem.new MoveHookCommand(ClimberConstants.kHookMoveUpSpeed));

        // Move Hook Down
        new JoystickButton(m_operatorController, OIConstants.kClimberMoveHookDownButton)
            .whileTrue(m_climberSubsystem.new MoveHookCommand(ClimberConstants.kHookMoveDownSpeed));

        // Pull Winch
        new JoystickButton(m_operatorController, OIConstants.kClimberPullWinchButton)
            .whileTrue(m_climberSubsystem.new MoveWinchCommand(ClimberConstants.kWinchPullInSpeed));

        // Release Winch
        new JoystickButton(m_operatorController, OIConstants.kClimberReleaseWinchButton)
            .whileTrue(m_climberSubsystem.new MoveWinchCommand(ClimberConstants.kWinchReleaseSpeed));
  }

   /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //CURRENT PATHS: rotCurvePath, curvePath, straightPath, rotatePath
    PathPlannerPath path = PathPlannerPath.fromPathFile("straightPath");

    return AutoBuilder.followPath(path);
  }
}