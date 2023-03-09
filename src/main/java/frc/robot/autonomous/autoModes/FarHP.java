package frc.robot.autonomous.autoModes;

import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.AutoExtendIntake;
import frc.robot.commands.AutoGripClose;
import frc.robot.commands.AutoGripDown;
import frc.robot.commands.AutoGripOpen;
import frc.robot.commands.AutoGripUp;
import frc.robot.commands.MoveArm;

public class FarHP extends SequentialCommandGroup {
    public FarHP(RobotContainer robotContainer) {
        PathPlannerTrajectory blueTrajectory = PathPlanner.loadPath("FarHP", 3, 2);
        PathPlannerTrajectory redTrajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(blueTrajectory, Alliance.Red);

        this.addCommands(
            new InstantCommand(() -> {
                // Reset odometry for the first path you run during auto
                if (DriverStation.getAlliance() == Alliance.Red)
                {
                    robotContainer.s_Swerve.resetOdometry(redTrajectory.getInitialHolonomicPose());
                }
                else
                {
                    robotContainer.s_Swerve.resetOdometry(blueTrajectory.getInitialHolonomicPose());
                }
            }),
            new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                    new AutoGripOpen(robotContainer.s_Arm, 0.5),
                    new AutoExtendIntake(robotContainer.s_Arm, robotContainer.s_Intake, -0.6, 4.5),
                    new AutoGripClose(robotContainer.s_Arm, 2.0),
                    new AutoGripUp(robotContainer.s_Arm, robotContainer.s_Intake, 1.0),
                    new MoveArm(robotContainer.s_Arm, robotContainer.s_Intake, Constants.armTopSetpoint),
                    new AutoGripOpen(robotContainer.s_Arm, 0.5),
                    new AutoGripClose(robotContainer.s_Arm, 0.5),
                    new AutoGripDown(robotContainer.s_Arm, robotContainer.s_Intake, 0.5),
                    new MoveArm(robotContainer.s_Arm, robotContainer.s_Intake, Constants.armBottomSetpoint)
                ),
                new PPSwerveControllerCommand(
                    blueTrajectory, 
                    robotContainer.s_Swerve::getPose, // Pose supplier
                    Constants.Drivebase.swerveKinematics, // SwerveDriveKinematics
                    new PIDController(Constants.Auto.kPXController, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                    new PIDController(Constants.Auto.kPYController, 0, 0), // Y controller (usually the same values as X controller)
                    new PIDController(Constants.Auto.kPThetaController, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                    robotContainer.s_Swerve::setModuleStates, // Module states consumer
                    true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                    robotContainer.s_Swerve // Requires this drive subsystem
                )
            )
            //new AutoBalanceCommand(robotContainer.s_Swerve)
        );
    }
}
