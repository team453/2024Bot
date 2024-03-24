package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class pathfindingCommands extends Command{

    public Command toLoadingStation()
    {
        Pose2d targetPose2d = new Pose2d(15.51, 1.06, Rotation2d.fromDegrees(-1.54));

      PathConstraints constraints = new PathConstraints(
        3, 4, 
        Units.degreesToRadians(540), Units.degreesToRadians(720));

      Command pathfindingCommand = AutoBuilder.pathfindToPose(
        targetPose2d, constraints,
        0,0);

        return pathfindingCommand;
    }
    
}

