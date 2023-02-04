package frc.robot.autonomous.autoModes;

import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AutoMode extends SequentialCommandGroup {
    public AutoMode(RobotContainer robotContainer) {
        PathPlannerTrajectory examplePath = PathPlanner.loadPath("New Path", 3, 2);

        this.addCommands(
            new InstantCommand(() -> {
                // Reset odometry for the first path you run during auto
                robotContainer.s_Swerve.resetOdometry(examplePath.getInitialHolonomicPose());
            }),
            new PPSwerveControllerCommand(
                examplePath, 
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
