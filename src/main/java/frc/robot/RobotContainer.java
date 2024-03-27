package frc.robot;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
/*import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;*/
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.Constants.AutoConstants;
//import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.UnderBotSubsystemConstants;
import frc.robot.commands.pathfindingCommands;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
//import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.UnderBotSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
//import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import java.util.List;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
//import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

public class RobotContainer {
  private final DriveSubsystem m_drivetrain;
  //private final PoseEstimatorSubsystem m_poseEstimator = new PoseEstimatorSubsystem(m_drivetrain);
  Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
  Joystick m_operatorController = new Joystick(OIConstants.kOperatorControllerPort);
  private final SendableChooser<Double> speedChooser = new SendableChooser<>();
  private final UnderBotSubsystem m_underBot = new UnderBotSubsystem();
  private final ClimberSubsystem m_climber = new ClimberSubsystem();
  private final SendableChooser<Command> autoChooser;
  // Add a boolean variable to track field position state
  private boolean isFieldPositionEnabled = true; // Initialize to false by default
  private final Field2d field;
  private final pathfindingCommands paths = new pathfindingCommands();

  public RobotContainer() {
    m_drivetrain = new DriveSubsystem();
    field = new Field2d();
    SmartDashboard.putData("Field", field);


    //log current robot pose to the 2D field
    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      field.setRobotPose(pose);
    });


    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      field.getObject("target pose").setPose(pose);
    });

    //logs data to add to the current path to the 2D field widget
    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
    
    // Register Named Commands for PathPlanner
    NamedCommands.registerCommand("shootCommand", m_underBot.new SequentialShootCommand(-0.75));
    NamedCommands.registerCommand("intakeCommand", m_underBot.new SequentialIntakeCommand());
    autoChooser = AutoBuilder.buildAutoChooser();

    autoChooser.setDefaultOption("Shooter Routine", new PathPlannerAuto("basicShoot"));
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureButtonBindings();
    configureDefaultDriveCommand();
    configureUnderBotButtonBindings();
   configureClimberButtonBindings();
    updateShuffleboard();
    configureAutonomousButtonBindings();
  }

  private void updateShuffleboard() {
    speedChooser.setDefaultOption("Low Speed", OIConstants.kLowSpeedMultiplier);
    speedChooser.addOption("Medium Speed", OIConstants.kMediumSpeedMultiplier);
    speedChooser.addOption("High Speed", OIConstants.kHighSpeedMultiplier);
    SmartDashboard.putData("Speed Multiplier", speedChooser);
    
    
    // Update the SmartDashboard with the initial field position state
    SmartDashboard.putBoolean("Field Position Enabled", isFieldPositionEnabled);
  }

  private void configureDefaultDriveCommand() {
    m_drivetrain.setDefaultCommand(
      new RunCommand(() -> {
          double speedMultiplier = speedChooser.getSelected();
           isFieldPositionEnabled = false;
              // Update the SmartDashboard with the current state after changing it
          SmartDashboard.putBoolean("Field Position Enabled", isFieldPositionEnabled);
          m_drivetrain.drive(
            -MathUtil.applyDeadband(m_driverController.getY(), OIConstants.kDriveDeadband) * speedMultiplier,
            -MathUtil.applyDeadband(m_driverController.getX(), OIConstants.kDriveDeadband) * speedMultiplier,
            -MathUtil.applyDeadband(m_driverController.getZ(), OIConstants.kDriveDeadband) * speedMultiplier,
            false, isFieldPositionEnabled);
        }, 
        m_drivetrain)
    );
  }

  private void configureButtonBindings() {
    new JoystickButton(m_driverController, 12).onTrue(
      new RunCommand(
            () -> m_drivetrain.resetOdometry()));
    new JoystickButton(m_driverController, 9)
        .whileTrue(new RunCommand(
            () -> m_drivetrain.setX(),
             m_drivetrain));

    new JoystickButton(m_driverController, 1)
    .whileTrue(new RunCommand(() -> {
          double speedMultiplier = speedChooser.getSelected();
          // Toggle the field position state when button 1 is pressed
          isFieldPositionEnabled = true;
          // Update the SmartDashboard with the new state
          SmartDashboard.putBoolean("Field Position Enabled", isFieldPositionEnabled);
          m_drivetrain.drive(
            -MathUtil.applyDeadband(m_driverController.getY(), OIConstants.kDriveDeadband) * speedMultiplier,
            -MathUtil.applyDeadband(m_driverController.getX(), OIConstants.kDriveDeadband) * speedMultiplier,
            -MathUtil.applyDeadband(m_driverController.getZ(), OIConstants.kDriveDeadband) * (speedMultiplier/2),
            true, isFieldPositionEnabled);
        }, 
        m_drivetrain));
  }

  private void configureUnderBotButtonBindings() {
    // UnderBot subsystem bindings
    new JoystickButton(m_operatorController, OIConstants.kUnderbotIntakeButton)
        .whileTrue(m_underBot.new IntakeCommand());

    new JoystickButton(m_operatorController, OIConstants.kUnderbotEjectButton)
        .whileTrue(m_underBot.new EjectCommand());

   new JoystickButton(m_operatorController, OIConstants.kUnderbotShooterHighButton)
       .whileTrue(m_underBot.new SourceIntakeCommand(-0.1));

    new JoystickButton(m_operatorController, OIConstants.kUnderbotShooterLowButton)
        .whileTrue(m_underBot.new ShootCommand(UnderBotSubsystemConstants.kHighShooterRPM));

   new JoystickButton(m_operatorController, 12)
        .whileTrue(m_underBot.new ShootCommand(0.9));

        new JoystickButton(m_operatorController, 11)
       .whileTrue(m_underBot.new ShootAmpCommand(-0.6,0.35));

}


private void configureClimberButtonBindings()
{
 new JoystickButton(m_operatorController, OIConstants.kMoveHookUpButton)
        .whileTrue(m_climber.new MoveHookCommand(true));

        new JoystickButton(m_operatorController, OIConstants.kMoveHookDownButton)
        .whileTrue(m_climber.new MoveHookCommand(false));

         new JoystickButton(m_operatorController, OIConstants.kPullWinchUpButton)
        .whileTrue(m_climber.new MoveWenchCommand(true));

          new JoystickButton(m_operatorController, OIConstants.kReleaseWinchButton)
        .whileTrue(m_climber.new MoveWenchCommand(false));

}


private void configureAutonomousButtonBindings()
{
    new JoystickButton(m_driverController, 2).onTrue(paths.toLoadingStation());

    //This follows a sequence of points. Change bezierPoints to change where the paths go.
    new JoystickButton(m_driverController, 3).onTrue(Commands.runOnce(() -> {
      //Created a list of bezier points. These are essentially waypoints. The rotation component acts as the direction of travel.
      //DO NOT SET HOLONOMIC ROTATION HERE!!
      List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
        new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
        new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
        new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90))
      );

      PathPlannerPath path = new PathPlannerPath(
        bezierPoints,
        new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), // The constraints for this path.
        new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // Goal end state. You can set a holonomic rotation here.
      );

      //Uncomment this if the paths are acting strangely on the red alliance.
      //path.preventFlipping = true;

      AutoBuilder.followPath(path).schedule();
    }));
}


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
   //return new PathPlannerAuto("Example Auto");

    
    return autoChooser.getSelected();
  }
}
