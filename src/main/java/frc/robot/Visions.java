package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.sensors.Limelight;

public class Visions extends Limelight{
    private Pose3d[] tagPoses;
    public Pose3d targetPoseRobotSpace;
    public enum Align {
        LEFT,
        RIGHT,
        CENTER;
    }

    private final Transform2d leftPoleAlign = 
        new Transform2d(new Translation2d(1, 1), // places goal pose 1 meter in front (x) and 1 meter to left(y) of tag
        new Rotation2d(Math.PI));                     // set to 180 degrees so robot ends up facing the tag/pole
    private final Transform2d rightPoleAlign =
        new Transform2d(new Translation2d(1, -1), // 1 meter in front(x) and 1 meter to right(-y) of tag
        new Rotation2d(Math.PI));   
    private final Transform2d centerAlign = 
        new Transform2d(new Translation2d(1, 0), // 1 meter in front of tag
        new Rotation2d(Math.PI));   
    public Visions(){
        this.tagPoses = new Pose3d[8];
    }

    public Pose2d doubleArrayToRobotPose(double[] poseArray){
        if (poseArray.length == 6) {
            return new Pose2d(
                new Translation2d(poseArray[0], poseArray[2]),
                new Rotation2d(
                    Units.degreesToRadians(poseArray[5]))
            );
        }
        return new Pose2d();
    }
    public Pose2d doubleArrayToFieldPose(double[] poseArray){
        if (poseArray.length == 6) {
            return new Pose2d(
                new Translation2d(poseArray[0], poseArray[1]),
                new Rotation2d(
                    Units.degreesToRadians(poseArray[5]))
            );
        }
        return new Pose2d();
    }
    public Pose2d blueAllianceBotPose(){
        double[] botpose = getPoseBlue();
        return doubleArrayToFieldPose(botpose);
    }

    public Pose2d redAllianceBotPose(){
        double [] botpose = getPoseRed();
        return doubleArrayToFieldPose(botpose);
    }

    public Pose2d tagSpaceRobotPose(){
        double[] botpose = botPoseTargetSpace();
        return doubleArrayToFieldPose(botpose);
    }

    public Pose2d robotSpaceTagPose(){
        double[] targetpose = targetPoseRobotSpace();
        return doubleArrayToRobotPose(targetpose);
    }
    
    

}

