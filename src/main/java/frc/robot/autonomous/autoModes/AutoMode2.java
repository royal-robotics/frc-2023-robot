package frc.robot.autonomous.autoModes;

import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.PathPlannerTrajectory;

import javax.lang.model.util.ElementScanner14;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AutoMode2 extends SequentialCommandGroup {
    public AutoMode2 (RobotContainer robotContainer) {
        PathPlannerTrajectory blueTrajectory = PathPlanner.loadPath("Another Test Path", 2.5, 1.5);
        PathPlannerTrajectory redTrajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(blueTrajectory, Alliance.Red);

        this.addCommands(
            new InstantCommand(() -> {
                if (DriverStation.getAlliance() == Alliance.Red)
                {
                // Reset odometry for the first path you run during auto
                robotContainer.s_Swerve.resetOdometry(redTrajectory.getInitialHolonomicPose());
                }
                else
                {
                    robotContainer.s_Swerve.resetOdometry(blueTrajectory.getInitialHolonomicPose());
                }
            }),
            new PPSwerveControllerCommand(
                blueTrajectory, 
                robotContainer.s_Swerve::getPose, // Pose supplier
                Constants.Drivebase.swerveKinematics, // SwerveDriveKinematics
                new PIDController(12, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                new PIDController(12, 0, 0), // Y controller (usually the same values as X controller)
                new PIDController(11, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                robotContainer.s_Swerve::setModuleStates, // Module states consumer
                true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                robotContainer.s_Swerve // Requires this drive subsystem
            )
        );
    }
}
