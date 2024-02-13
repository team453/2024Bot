// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.UnderBotSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.UnderBotSubsystemConstants;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_drivetrain = new DriveSubsystem();
    private final UnderBotSubsystem m_underBot = new UnderBotSubsystem();

  // The driver's controller
  Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
    // The operator's controller
    Joystick m_operatorController = new Joystick(OIConstants.kOperatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

  // Configure default commands
m_drivetrain.setDefaultCommand(
    // The joystick's Y axis controls forward/backward movement,
    // the X axis controls left/right movement, and
    // Z controls turning.
    new RunCommand(
        () -> m_drivetrain.drive(
            -MathUtil.applyDeadband(m_driverController.getY(), OIConstants.kDriveDeadband) * OIConstants.kSpeedMultiplier, // Forward/backward
            -MathUtil.applyDeadband(m_driverController.getX(), OIConstants.kDriveDeadband)* OIConstants.kSpeedMultiplier, // Left/right
            -MathUtil.applyDeadband(m_driverController.getZ(), OIConstants.kDriveDeadband)* OIConstants.kSpeedMultiplier, // Turning
            false, true),
        m_drivetrain));

   
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, 9)
        .whileTrue(new RunCommand(
            () -> m_drivetrain.setX(),
             m_drivetrain));

                   //When holding the tri gger, use the driverDriveSpeed
    new JoystickButton(m_driverController, 1) // Create a new JoystickButton binding for button 9 on m_driver joystick
    .whileTrue(
        new RunCommand(
        () -> m_drivetrain.drive(
            -MathUtil.applyDeadband(m_driverController.getY(), OIConstants.kDriveDeadband)* OIConstants.kSpeedMultiplier, // Forward/backward
            -MathUtil.applyDeadband(m_driverController.getX(), OIConstants.kDriveDeadband)* OIConstants.kSpeedMultiplier, // Left/right
            -MathUtil.applyDeadband(m_driverController.getZ(), OIConstants.kDriveDeadband)* OIConstants.kSpeedMultiplier, // Turning
            true, true),
        m_drivetrain));


    //underbot stuff
    
   //Launch with set speeds
    new JoystickButton(m_operatorController, OIConstants.kShooterButton) // Create a new JoystickButton binding for button 9 on m_driver joystick
    .whileTrue(
     new RunCommand(() -> m_underBot.stop()
        .withTimeout(UnderBotSubsystemConstants.kShooterDelay) // Set the timeout to fully stop motors
           .andThen(m_underBot.ShooterCommand())) // Shoot the note
        .handleInterrupt(() -> m_underBot.stop())); // Handle any interruption by stopping the launcher
  
    new JoystickButton(m_operatorController, OIConstants.kIntakeButton) // Binding for trigger on m_operatorController joystick
    .whileTrue(
     new RunCommand(() -> m_underBot.stop()
           .andThen(m_underBot.IntakeCommand())) // Intake the note
        .handleInterrupt(() -> m_underBot.stop())); // Handle any interruption by stopping the launcher

  }
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
         m_drivetrain::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
         m_drivetrain::setModuleStates,
        m_drivetrain);

    // Reset odometry to the starting pose of the trajectory.
     m_drivetrain.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() ->  m_drivetrain.drive(0, 0, 0, false, false));
  }
}
