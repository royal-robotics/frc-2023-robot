package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Visions;
import frc.robot.subsystems.Swerve;

/* Command that aligns robot to desired positions in front of pole/platform to score cone/cube using AprilTag data. 
 * Requires a visible AprilTag from limelight,
 * 
 * The RobotSpace version of this command grabs the tag's pose from the ROBOT'S PERSPECTIVE and returns a translation
 * and rotation to line up robot in a scoring position. Translation and rotation are therefore relative
 * to robot, NOT FIELD. 
 * 
 * Benefits of this version:
 * - Less stored info
 * Limitation to this version:
 * - Unstable readings from tagPose 
 * - requires that tag remains in camera view while it moves to aligned position, which is not always possible
*/
public class GridAlignRobotSpaceCommand extends CommandBase {
    // whether we want to line up with left pole, right pole, or center nodes in the tag's grid 
    public enum Align {
        LEFT,
        RIGHT,
        CENTER;
    }

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
        
    private Swerve drivetrain;
    private Visions limelight;
    private Pose2d goalPose;    // Translation and rotation from the robot current Pose to the goal position. (bot perspective)
    private Align goal;
    private boolean hasTarget; 

    public GridAlignRobotSpaceCommand(Swerve drivetrain, Align goal) {
        this.drivetrain = drivetrain;
        this.limelight = drivetrain.s_Visions;
        this.goal = goal;
    }

    @Override
    public void initialize() {
        this.goalPose = null;
        this.hasTarget = false;
    }

    @Override
    public void execute() {
        this.hasTarget = limelight.hasTarget() == 1;
        if (this.hasTarget) {
            // get pose of target relative to robot space. This should essentially be a transform to move
            // robot from current position to tag's current position AND orientation.
            Pose2d tagPose = limelight.robotSpaceTagPose();
            
            // transform tag pose to goal pose. This is essentially a concatenation on the last transform
            // to move the robot from the tag's pose to our desired position and orientation.
            // This should return a transform to get to the goal pose FROM THE BOT'S PERSPECTIVE. 
            switch (goal) {
                case LEFT:
                    this.goalPose = tagPose.transformBy(leftPoleAlign);  // TODO: shuffleboard test these values
                    break;
                case RIGHT:
                    this.goalPose = tagPose.transformBy(rightPoleAlign);
                    break;
                case CENTER:
                    this.goalPose = tagPose.transformBy(centerAlign);
                    break;
            }
            Shuffleboard.getTab("Camera").addNumber("AprilTagIDx1", () -> goalPose.getX()).withPosition(4, 0);
            Shuffleboard.getTab("Camera").addNumber("AprilTagIDy1", () -> goalPose.getY()).withPosition(4, 1);
            // -----DRIVE TO GOAL-------
            /* Uses swerve's drive method to get to goal pose as another possible example versus the SwerveControllerCommand
             * Possible limitations to this method:
             *  - I am unfamiliar with how this drive() implementation works, so I don't know if it is compatible with 
             *      the command scheduler
             *  - No PID control
             * However, we can extend CommandBase here versus CommandGroup, so we may have more control of behavior during 
             * course of command
            */
            // drivetrain.drive(
            //     this.goalPose.getTranslation(), 
            //     this.goalPose.getRotation().getRadians(), 
            //     false,  // transform should be from robot's perspective, therefore is false
            //     false);     // not sure what this is
        }

    }

    @Override
    public void end(boolean interrupted) {
        // stop driving
        drivetrain.drive(new Translation2d(), 0, false, false);
    }

    @Override
    public boolean isFinished(){ 
        // finish command when robot has reached goal (ie when bot's relative position to goal is zero) or if no visible target
        return !this.hasTarget || ((this.goalPose != null) && 
            (this.goalPose.getX() == 0) && 
            (this.goalPose.getY() == 0) && 
            (this.goalPose.getRotation().getDegrees() == 0)); 
    }
}
