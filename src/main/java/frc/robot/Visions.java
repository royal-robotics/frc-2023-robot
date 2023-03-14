package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.sensors.Limelight;

public class Visions {
    public Limelight m_Limelight;
    private Pose2d[] blueTagPoses = {new Pose2d(15.53, 1.05, new Rotation2d(Math.PI)),
        new Pose2d(15.53, 2.75, new Rotation2d(Math.PI)),
        new Pose2d(15.53, 4.44, new Rotation2d(Math.PI)),
        new Pose2d(16.24, 6.73, new Rotation2d(Math.PI)),
        new Pose2d(0.3, 6.73, new Rotation2d()),
        new Pose2d(1.0, 4.44, new Rotation2d()),
        new Pose2d(1.0, 2.75, new Rotation2d()),
        new Pose2d(1.0, 1.05, new Rotation2d())
    };
    
    public Pose3d targetPoseRobotSpace;
    public enum Align {
        LEFT,
        RIGHT,
        CENTER;
    }

    private final Transform2d leftPoleAlign =
        new Transform2d(new Translation2d(1, -1), // places goal pose 1 meter in front (x) and 1 meter to left(y) of tag
        new Rotation2d(0));                     // set to 180 degrees so robot ends up facing the tag/pole
    private final Transform2d rightPoleAlign =
        new Transform2d(new Translation2d(1, 1), // 1 meter in front(x) and 1 meter to right(-y) of tag
        new Rotation2d(0));
    private final Transform2d centerAlign =
        new Transform2d(new Translation2d(1, 0), // 1 meter in front of tag
        new Rotation2d(0));

    private final Translation2d centerPositionBlue = new Translation2d(1.9, 2.75); // Position in front of blue center cube spot for blue alliance
    private final Translation2d centerPositionRed = new Translation2d(1.9, 5.27); // Position in front of red center cube spot for red alliance

    // public Visions() {
    //     this.m_Limelight = new Limelight();
    // }

    public Visions(String camName) {
        this.m_Limelight = new Limelight(camName);
    }

    public Pose2d doubleArrayToRobotPose(double[] poseArray) {
        if (poseArray.length == 6) {
            return new Pose2d(
                new Translation2d(poseArray[0], poseArray[2]),
                new Rotation2d(
                    Units.degreesToRadians(poseArray[4]))
            );
        }
        return new Pose2d();
    }
    public Pose2d doubleArrayToFieldPose(double[] poseArray) {
        if (poseArray.length == 6) {
            return new Pose2d(
                new Translation2d(poseArray[0], poseArray[1]),
                new Rotation2d(
                    Units.degreesToRadians(poseArray[5]))
            );
        }
        return new Pose2d();
    }
    public Pose2d blueAllianceBotPose() {
        double[] botpose = m_Limelight.getPoseBlue();
        return doubleArrayToFieldPose(botpose);
    }

    public Pose2d redAllianceBotPose() {
        double[] botpose = m_Limelight.getPoseRed();
        return doubleArrayToFieldPose(botpose);
    }

    public Pose2d tagSpaceRobotPose() {
        double[] botpose = m_Limelight.botPoseTargetSpace();
        return doubleArrayToFieldPose(botpose);
    }

    public Pose2d robotSpaceTagPose() {
        double[] targetpose = m_Limelight.targetPoseRobotSpace();
        return doubleArrayToRobotPose(targetpose);
    }

    public PathPlannerTrajectory generatePath() {
        // Get our current field position
        Pose2d botPose = blueAllianceBotPose();
        // Transform for red alliance?
        // Calculate the closest target to our current position?
        Translation2d goalPosition = centerPositionBlue;
        // Calculate difference between current and goal position to get heading angle
        Translation2d positionDifference = goalPosition.minus(botPose.getTranslation());
        // Return path between these points
        return PathPlanner.generatePath(
            new PathConstraints(3, 2),
            new PathPoint(botPose.getTranslation(), positionDifference.getAngle()),
            new PathPoint(goalPosition, positionDifference.getAngle()));
    }

    public Pose2d getBlueTagPose(int tagID) {
        return this.blueTagPoses[tagID - 1];
    }

    public Pose2d tagPoseToGoalPose(Pose2d tagPose, Align goalAlign) {
        Pose2d goalPose = new Pose2d();
        switch (goalAlign) {
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
        return goalPose;
    }

    public double zDistRobotToTag() {
        return m_Limelight.targetPoseRobotSpace()[2];
    }

    public double angleRobotToTag() {
        return m_Limelight.targetPoseRobotSpace()[4];
    }

    public double yDistRobotToTag() {
        return m_Limelight.targetPoseRobotSpace()[0];
    }
}