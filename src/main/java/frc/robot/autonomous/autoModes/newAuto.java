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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.AutoExtendIntake;
import frc.robot.commands.AutoGripClose;
import frc.robot.commands.AutoGripDown;
import frc.robot.commands.AutoGripOpen;
import frc.robot.commands.AutoGripUp;
import frc.robot.commands.MoveArm;
import frc.robot.commands.ScoreAlignCommand;
import frc.robot.commands.TagAlignCommand;
import frc.robot.subsystems.Swerve;

public class newAuto extends SequentialCommandGroup {
    public newAuto(RobotContainer robotContainer) {
        PathPlannerTrajectory blueTrajectory = PathPlanner.loadPath("newAuto", 3, 2);
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
            new SequentialCommandGroup(  
                new AutoExtendIntake(robotContainer.s_Arm, robotContainer.s_Intake, 0, 0.5),
                new AutoGripUp(robotContainer.s_Arm, robotContainer.s_Intake, 0.5),
                new MoveArm(robotContainer.s_Arm, robotContainer.s_Intake, Constants.armTopSetpoint),
                new ParallelDeadlineGroup(
                    new WaitCommand(2.0),
                    new ScoreAlignCommand(robotContainer.s_Swerve, robotContainer.s_Visions, Constants.xLeftCone, Constants.yLeftCone)
                ),
                // new ScoreAlignCommand(robotContainer.s_Swerve, robotContainer.s_Visions, Constants.xLeftCone, Constants.yLeftCone),
                new AutoGripOpen(robotContainer.s_Arm, 0.5),
                new AutoGripClose(robotContainer.s_Arm, 0.25)),
            new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                    new MoveArm(robotContainer.s_Arm, robotContainer.s_Intake, Constants.armBottomSetpoint),
                    new AutoGripDown(robotContainer.s_Arm, robotContainer.s_Intake, 0.5),
                    new AutoGripOpen(robotContainer.s_Arm, 0.25),
                    new AutoExtendIntake(robotContainer.s_Arm, robotContainer.s_Intake, Constants.cubeIntakeSpeedAuto, 2.0),
                    new AutoGripClose(robotContainer.s_Arm, 0.5),
                    new AutoGripUp(robotContainer.s_Arm, robotContainer.s_Intake, 0.5),
                    new MoveArm(robotContainer.s_Arm, robotContainer.s_Intake, Constants.armTopSetpoint),
                    new AutoGripOpen(robotContainer.s_Arm, 0.25)
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
            ),
            new InstantCommand(() -> robotContainer.s_Swerve.setGyro(180))
            //new AutoBalanceCommand(robotContainer.s_Swerve)
        );
    }
}
