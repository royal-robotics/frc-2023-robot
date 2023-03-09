package src.test.java;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Visions.Align;
import frc.robot.Visions;

class VisionTest {
    Visions vision;

    @BeforeEach // this method will run before each test
    void setup() {
        vision = new Visions();
    }

    // @Test 
    // void testRedCenterAlignTransform() {
    //     Pose2d tagPose = new Pose2d(3, 1, new Rotation2d(Math.PI));
    //     Pose2d goalPose = vision.tagPoseToGoalPose(tagPose, Align.CENTER);
    //     // System.out.println(goalPose.getX() + "\n" + goalPose.getY() + "\n" + goalPose.getRotation().getDegrees());
    //     assertEquals(new Pose2d(2, 1, new Rotation2d(2*Math.PI)), goalPose);
    // }

    // @Test 
    // void testRedLeftAlignTransform() {
    //     Pose2d tagPose = new Pose2d(3, 1, new Rotation2d(Math.PI));
    //     Pose2d goalPose = vision.tagPoseToGoalPose(tagPose, Align.LEFT);
    //     // System.out.println(goalPose.getX() + "\n" + goalPose.getY() + "\n" + goalPose.getRotation().getDegrees());
    //     assertEquals(new Pose2d(2, 2, new Rotation2d(2*Math.PI)), goalPose);
    // }

    // @Test 
    // void testRedRightAlignTransform() {
    //     Pose2d tagPose = new Pose2d(3, 1, new Rotation2d(Math.PI));
    //     Pose2d goalPose = vision.tagPoseToGoalPose(tagPose, Align.RIGHT);
    //     // System.out.println(goalPose.getX() + "\n" + goalPose.getY() + "\n" + goalPose.getRotation().getDegrees());
    //     assertEquals(new Pose2d(2, 0, new Rotation2d(2*Math.PI)), goalPose);
    // }

    // @Test 
    // void testBlueCenterAlignTransform() {
    //     Pose2d tagPose = new Pose2d(3, 1, new Rotation2d(0));
    //     Pose2d goalPose = vision.tagPoseToGoalPose(tagPose, Align.CENTER);
    //     // System.out.println(goalPose.getX() + "\n" + goalPose.getY() + "\n" + goalPose.getRotation().getDegrees());
    //     assertEquals(new Pose2d(4, 1, new Rotation2d(Math.PI)), goalPose);
    // }

    // @Test 
    // void testBlueLeftAlignTransform() {
    //     Pose2d tagPose = new Pose2d(3, 1, new Rotation2d(0));
    //     Pose2d goalPose = vision.tagPoseToGoalPose(tagPose, Align.LEFT);
    //     // System.out.println(goalPose.getX() + "\n" + goalPose.getY() + "\n" + goalPose.getRotation().getDegrees());
    //     assertEquals(new Pose2d(4, 0, new Rotation2d(Math.PI)), goalPose);
    // }

    // @Test 
    // void testBlueRightAlignTransform() {
    //     Pose2d tagPose = new Pose2d(3, 1, new Rotation2d(0));
    //     Pose2d goalPose = vision.tagPoseToGoalPose(tagPose, Align.RIGHT);
    //     // System.out.println(goalPose.getX() + "\n" + goalPose.getY() + "\n" + goalPose.getRotation().getDegrees());
    //     assertEquals(new Pose2d(4.0, 2.0, new Rotation2d(Math.PI)), goalPose);
    // }

    // @Test 
    // //testing tagPoseRobotSpace compatibility
    // void testOffCenterAlign() {
    //     Pose2d tagPose = new Pose2d(1,  0, new Rotation2d(Units.degreesToRadians(135)));
    //     Pose2d goalPose = vision.tagPoseToGoalPose(tagPose, Align.CENTER);
    //     System.out.println(goalPose.getX() + "\n" + goalPose.getY() + "\n" + goalPose.getRotation().getDegrees());
    //     assertEquals(new Pose2d(1.0-Math.sqrt(2)/2, Math.sqrt(2)/2, new Rotation2d(Units.degreesToRadians(-45))), goalPose);
    // }

    // @Test 
    // //testing GridAlignTagPose funtionality
    // void testMultipleTransformsFaceRed() {
    //     //bot pose field space
    //     Pose2d botPose = new Pose2d(2, 6, new Rotation2d(0));

    //     //transform botpose using botToTag (aka tagPose in robot space) to get tag pose field space
    //     Transform2d botToTag = new Transform2d(new Translation2d(0.35, 5), new Rotation2d(Math.PI));
    //     Pose2d tagPose = botPose.transformBy(botToTag);

    //     //transform tag pose to goal pose (field space)
    //     Pose2d goalPose = vision.tagPoseToGoalPose(tagPose, Align.CENTER);

    //     // System.out.println(goalPose.getX() + "\n" + goalPose.getY() + "\n" + goalPose.getRotation().getDegrees());
    //     assertEquals(new Pose2d(1.35, 11, new Rotation2d(0)), goalPose);
    // }

    // @Test 
    // //testing GridAlignTagPose funtionality
    // void testMultipleTransformsFaceBlue() {
    //     //bot pose field space
    //     Pose2d botPose = new Pose2d(2, 6, new Rotation2d(Math.PI));

    //     //transform botpose using botToTag (aka tagPose in robot space) to get tag pose field space
    //     Transform2d botToTag = new Transform2d(new Translation2d(0.35, 5), new Rotation2d(Math.PI));
    //     Pose2d tagPose = botPose.transformBy(botToTag);

    //     //transform tag pose to goal pose (field space)
    //     Pose2d goalPose = vision.tagPoseToGoalPose(tagPose, Align.CENTER);

    //     // System.out.println(goalPose.getX() + "\n" + goalPose.getY() + "\n" + goalPose.getRotation().getDegrees());
    //     assertEquals(new Pose2d(2.65, 1, new Rotation2d(Math.PI)), goalPose);
    // }
}
