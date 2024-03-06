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
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AlignWithTag;
import frc.robot.helpers.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.PoseEstimatorSubsystem;
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
  //private final AlignWithTag m_AlignWithTag = new AlignWithTag(m_drivetrain, m_limeLight);
  

  // The driver's controller
  Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
     autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);
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
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
     autoChooser.addOption("Example Auto", new PathPlannerAuto("Example Path"));

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

    //new JoystickButton(m_driverController, 7).whileTrue(m_AlignWithTag);
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