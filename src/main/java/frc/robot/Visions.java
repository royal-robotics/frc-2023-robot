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
    private Pose3d[] tagPoses;
    public Pose3d targetPoseRobotSpace;
    public enum Align {
        LEFT,
        RIGHT,
        CENTER;
    }

    private final Transform2d leftPoleAlign =
        new Transform2d(new Translation2d(1, -1), // places goal pose 1 meter in front (x) and 1 meter to left(y) of tag
        new Rotation2d(Math.PI));                     // set to 180 degrees so robot ends up facing the tag/pole
    private final Transform2d rightPoleAlign =
        new Transform2d(new Translation2d(1, 1), // 1 meter in front(x) and 1 meter to right(-y) of tag
        new Rotation2d(Math.PI));
    private final Transform2d centerAlign =
        new Transform2d(new Translation2d(1, 0), // 1 meter in front of tag
        new Rotation2d(Math.PI));

    private final Translation2d centerPositionBlue = new Translation2d(1.9, 2.75); // Position in front of blue center cube spot for blue alliance
    private final Translation2d centerPositionRed = new Translation2d(1.9, 5.27); // Position in front of red center cube spot for red alliance

    public Visions() {
        this.tagPoses = new Pose3d[8];
        this.m_Limelight = new Limelight();
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
}