package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.sensors.Limelight;
import frc.robot.subsystems.Swerve;

/* Command that aligns robot to desired positions in front of pole/platform to score cone/cube using AprilTag data. 
 * Requires a visible AprilTag from limelight,
 * 
 * This version of this command uses the AprilTags ID to get the saved tag pose in FIELD SPACE and returns a translation
 * and rotation to line up the robot in a scoring position. Translation and rotation are therefore relative
 * to field. 
 * 
 * Benefits of this version:
 * - More reliable tag poses. Only pulling tag IDs, so won't have oscillating tag pose values like in targetPose_robotspace
 * Limitation to this version:
 * - Space to save tag locations
 * - Does Alliance color factor into whether saved poses and generated trajectories are correct?
*/
public class GridAlignCommand extends SequentialCommandGroup {
    // whether we want to line up with left pole, right pole, or center nodes in the tag's grid 
    public enum Align {
        LEFT,
        RIGHT,
        CENTER;
    }

    // AprilTag locations on the field
    private final Pose2d[] tagLocations = 
        {new Pose2d(4, 4, new Rotation2d())}; // example tag location

    /* These transforms document how far we want the bot be from the AprilTag to align with certain scoring targets. 
     * When scoring, we probably don't want to be on the tag or on the pole, but rather in front of and facing it. 
     * Think of these as if the tag is the origin and the values of the transform are the x,y, and rotation we want
     * the robot to be oriented relative to the tag.
    */
    private final Transform2d leftPoleAlign = 
        new Transform2d(new Translation2d(1, 1), // places goal pose 1 meter in front (x) and 1 meter to left(y) of tag
        new Rotation2d(Math.PI));                     // set to 180 degrees so robot ends up facing the tag/pole
    private final Transform2d rightPoleAlign =
        new Transform2d(new Translation2d(1, -1), // 1 meter in front(x) and 1 meter to right(-y) of tag
        new Rotation2d(Math.PI));   
    private final Transform2d centerAlign = 
        new Transform2d(new Translation2d(1, 0), // 1 meter in front of tag
        new Rotation2d(Math.PI));   


    // ideally, we could connect this command to trigger buttons on Xbox controller that map to which alignment we want
    public GridAlignCommand(Swerve drivetrain, Align goal) {
        Limelight limelight = drivetrain.s_Visions;
        
        if (limelight.hasTarget() == 1) {
            //int tagID = limelight.getTagID();
            int tagID = 1;

            // get tag's pose relative to field space from saved tag locations (assuming locations saved sequentially).
            Pose2d tagPose = tagLocations[tagID - 1];
            Pose2d goalPose = new Pose2d();
            
            // transform tag pose to get goal pose (ie. pose in front cone). Again, goalPose is relative to the fieldspace
            // because the initial pose is in fieldspace.
            switch (goal) {
                case LEFT:
                    goalPose = tagPose.transformBy(leftPoleAlign);  // TODO: shuffleboard test these values
                    break;
                case RIGHT:
                    goalPose = tagPose.transformBy(rightPoleAlign);
                    break;
                case CENTER:
                    goalPose = tagPose.transformBy(centerAlign);
                    break;
            }
            
            // ----- DRIVE ROBOT TO GOAL POSE ------ 
            /* sourced from BaseFalconSwerve AutoExample. uses swervecontrollercommand and trajectory generator to create
             * a hopefully straight path (will need to be tested) from current robot position to goal pose. unsure of how
             * else to PID control drive to a given position so this is by no means the best way to do this.
             */
            TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.Auto.kMaxSpeedMetersPerSecond,
                    Constants.Auto.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Drivebase.swerveKinematics);

            // Generate trajectory to goal position 
            Trajectory exampleTrajectory =
                TrajectoryGenerator.generateTrajectory(
                    drivetrain.getPose(),   // initial pose = bot's current pose. 
                    List.of(),              // no waypoints in between
                    goalPose,               // final pose = goal pose that we found from our transformations
                    config);

            var thetaController =
                new ProfiledPIDController(
                    Constants.Auto.kPThetaController, 0, 0, Constants.Auto.kThetaControllerConstraints);
            thetaController.enableContinuousInput(-Math.PI, Math.PI);

            // Create command to drive along trajectory to goal 
            SwerveControllerCommand swerveControllerCommand =
                new SwerveControllerCommand(
                    exampleTrajectory,
                    drivetrain::getPose,   
                    Constants.Drivebase.swerveKinematics,
                    new PIDController(Constants.Auto.kPXController, 0, 0),
                    new PIDController(Constants.Auto.kPYController, 0, 0),
                    thetaController,
                    drivetrain::setModuleStates,
                    drivetrain);

            // schedule drive to goal
            addCommands(swerveControllerCommand);
        }
    }
}
