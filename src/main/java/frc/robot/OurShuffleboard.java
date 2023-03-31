package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.autonomous.AutoModeSelector;
import frc.robot.sensors.Limelight;
import frc.robot.subsystems.*;

public class OurShuffleboard {
    public OurShuffleboard(Robot robot) {
        RobotContainer container = robot.m_robotContainer;
        AutoModeSelector autoModeSelector = robot.m_autoModeSelector;
        Swerve swerve = container.s_Swerve;
        Intake intake = container.s_Intake;
        Arm arm = container.s_Arm;
        Visions vision = container.s_Visions;
        Visions vision2 = container.s_leftVisions;

        // ShuffleboardTab cameraTab = Shuffleboard.getTab("Camera");
        // increasing going right, decreasing going left 
        
        // cameraTab.addNumber("hasTarget", () -> vision.m_Limelight.hasTarget()).withPosition(4, 0);
        // cameraTab.addNumber("zDistToTag", () -> vision.zDistRobotToTag()).withPosition(5, 0);
        // cameraTab.addNumber("yDistFromTag", ()->vision.yDistRobotToTag()).withPosition(6, 0);
        // cameraTab.addNumber("hasTarget2", () -> vision2.m_Limelight.hasTarget()).withPosition(4, 2);
        // cameraTab.addNumber("zDistToTag2", () -> vision2.zDistRobotToTag()).withPosition(5, 2);
        // cameraTab.addNumber("yDistFromTag2", ()->vision2.yDistRobotToTag()).withPosition(6, 2);
        // cameraTab.addCamera("Camera Stream", "lightlime", "http://10.s25.22.11:5800/").withPosition(0, 0).withSize(4, 2);
        // cameraTab.addCamera("Camera2 Stream", "lightlime-two", "http://10.25.22.12:5800/").withPosition(0, 2).withSize(4, 2);
        // cameraTab.addString("TagID", () -> {
        //     double[] id = vision.m_Limelight.getTagID();
        //     String str = "[" + id[0] + ", " + id[1] + ", " + id[2] + ", " + id[3] + ", " + id[4] + ", " + id[5] + "]";
        //     return str;
        // }).withPosition(2, 0);
        
        // cameraTab.addNumber("botposeBlueTranX", () -> vision.blueAllianceBotPose().getX()).withPosition(1, 0);
        // cameraTab.addNumber("botposeBlueTranY", () -> vision.blueAllianceBotPose().getY()).withPosition(2, 0);
        // cameraTab.addNumber("botposeBlueTranR", () -> vision.blueAllianceBotPose().getRotation().getDegrees()).withPosition(3, 0);
        // cameraTab.addNumber("botposeRedTranX", () -> vision.redAllianceBotPose().getX()).withPosition(1, 1);
        // cameraTab.addNumber("botposeRedTranY", () -> vision.redAllianceBotPose().getY()).withPosition(2, 1);
        // cameraTab.addNumber("botposeRedTranR", () -> vision.redAllianceBotPose().getRotation().getDegrees()).withPosition(3, 1);
        // cameraTab.addNumber("botSpacePoseTranX", () -> vision.tagSpaceRobotPose().getX()).withPosition(1, 2);
        // cameraTab.addNumber("botSpacePoseTranY", () -> vision.tagSpaceRobotPose().getY()).withPosition(2, 2);
        // cameraTab.addNumber("botSpacePoseTranR", () -> vision.tagSpaceRobotPose().getRotation().getDegrees()).withPosition(3, 2);
        // cameraTab.addNumber("targetPoseTranX", () -> vision.robotSpaceTagPose().getX()).withPosition(1, 3);
        // cameraTab.addNumber("targetPoseTranY", () -> vision.robotSpaceTagPose().getY()).withPosition(2, 3);
        // cameraTab.addNumber("targetPoseTranR", () -> vision.robotSpaceTagPose().getRotation().getDegrees()).withPosition(3, 3);
        // cameraTab.addNumber("robotAngleGoal", () -> {
        //     // Translation2d targetPos = new Translation2d(limelight.robotSpaceTagPose().getX(), limelight.robotSpaceTagPose().getY());
        //     // Translation2d polePos = targetPos.plus(new Translation2d(0.56, 0.23));
        //     // return polePos.getAngle().getDegrees();
        //     Pose2d bogusPos = new Pose2d(1,0, new Rotation2d(Math.PI));
        //     Transform2d bogusTrans = new Transform2d(new Translation2d(1, 0), new Rotation2d(Math.PI));
        //     Pose2d polePos = bogusPos.transformBy(bogusTrans);
        //     return polePos.getX();
        // }).withPosition(0, 1);
        // cameraTab.addString("TagID", () -> vision.m_Limelight.getTagID().toString()).withPosition(0, 4);

        ShuffleboardTab competitionTab = Shuffleboard.getTab("Competition");
        competitionTab.add("Auto Mode", autoModeSelector._chooser).withPosition(0, 0).withSize(2, 1);
        competitionTab.addCamera("Camera Stream", "lightlime", "http://10.25.22.11:5800/").withPosition(3, 0).withSize(3, 3);
        //competitionTab.addCamera( "Camera2 Stream", "lightlime-two", "http://10.25.22.12:5800/").withPosition(6, 0).withSize(2, 2);
        competitionTab.addNumber("Arm Power", () -> arm.getPidValue()).withPosition(0, 1);
        competitionTab.addBoolean("Arm At Setpoint", () -> arm.atSetpoint()).withPosition(1, 1);
        competitionTab.addBoolean("Claw Open", () -> arm.getSolenoidGrip() == Constants.gripOpen).withPosition(0, 2);
        // competitionTab.addNumber("Tag Distance", () -> vision.zDistRobotToTag()).withPosition(1, 2);
        //competitionTab.addNumber("Gyro Pitch", () -> swerve.gyro.getPitch()).withPosition(0, 1);
        //competitionTab.addNumber("Gyro Roll", () -> swerve.gyro.getRoll()).withPosition(1, 1);
        competitionTab.addNumber("Sonic Distance", () -> swerve.getSonicDistance()).withPosition(2,0);
        competitionTab.addNumber("Gyro Yaw", () -> swerve.gyro.getYaw()).withPosition(2, 1);
        // competitionTab.addNumber("Sonic Raw Voltage", ()->swerve.sonicSensorRange.getVoltage()).withPosition(2, 2);

        // ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake");
        // /*intakeTab.add("Speed Override", false)
        //     .withWidget(BuiltInWidgets.kToggleButton)
        //     .withPosition(0, 0)
        //     .withSize(2, 1);*/

        ShuffleboardTab armTab = Shuffleboard.getTab("Arm");
        armTab.addNumber("Arm Position", () -> arm.getEncoder()).withPosition(0, 2);
        armTab.addNumber("Arm Goal Position", () -> arm.getSetpoint()).withPosition(1,2);
        armTab.addNumber("Arm Power", () -> arm.getPidValue()).withPosition(2, 2);
        armTab.addBoolean("Arm At Setpoint", () -> arm.atSetpoint()).withPosition(0, 3);

        ShuffleboardTab testTab = Shuffleboard.getTab("Drivebase");
        testTab.addNumber("FL Speed", () -> (swerve.m_ModuleState[0].speedMetersPerSecond)).withPosition(0, 0);
        testTab.addNumber("FL Angle", () -> (swerve.m_ModuleState[0].angle.getDegrees())).withPosition(0, 1);
        testTab.addNumber("FR Speed", () -> (swerve.m_ModuleState[1].speedMetersPerSecond)).withPosition(1, 0);
        testTab.addNumber("FR Angle", () -> (swerve.m_ModuleState[1].angle.getDegrees())).withPosition(1, 1);
        testTab.addNumber("BL Speed", () -> (swerve.m_ModuleState[2].speedMetersPerSecond)).withPosition(2, 0);
        testTab.addNumber("BL Angle", () -> (swerve.m_ModuleState[2].angle.getDegrees())).withPosition(2, 1);
        testTab.addNumber("BR Speed", () -> (swerve.m_ModuleState[3].speedMetersPerSecond)).withPosition(3, 0);
        testTab.addNumber("BR Angle", () -> (swerve.m_ModuleState[3].angle.getDegrees())).withPosition(3, 1);
        testTab.addNumber("OdoX", () -> (swerve.getPose().getX())).withPosition(0, 2);
        testTab.addNumber("OdoY", () -> (swerve.getPose().getY())).withPosition(1, 2);
        testTab.addNumber("OdoR", () -> (swerve.getPose().getRotation().getDegrees())).withPosition(2, 2);
    }
}
