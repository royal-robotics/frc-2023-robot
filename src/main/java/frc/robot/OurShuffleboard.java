package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.autonomous.AutoModeSelector;
import frc.robot.subsystems.*;

public class OurShuffleboard {
    public OurShuffleboard(Robot robot) {
        RobotContainer container = robot.m_robotContainer;
        AutoModeSelector autoModeSelector = robot.m_autoModeSelector;
        Swerve swerve = container.s_Swerve;
        Intake intake = container.s_Intake;
        Arm arm = container.s_Arm;
        Visions vision = container.s_Visions;

        ShuffleboardTab cameraTab = Shuffleboard.getTab("Camera");
        
        cameraTab.addCamera("Camera Stream", "lightlime", "http://10.25.22.11:5800/").withPosition(4, 0).withSize(4, 4);
        cameraTab.addNumber("hasTarget", () -> vision.m_Limelight.hasTarget()).withPosition(0, 0);
        cameraTab.addNumber("botposeBlueTranX", () -> vision.blueAllianceBotPose().getX()).withPosition(1, 0);
        cameraTab.addNumber("botposeBlueTranY", () -> vision.blueAllianceBotPose().getY()).withPosition(2, 0);
        cameraTab.addNumber("botposeBlueTranR", () -> vision.blueAllianceBotPose().getRotation().getDegrees()).withPosition(3, 0);
        cameraTab.addNumber("botposeRedTranX", () -> vision.redAllianceBotPose().getX()).withPosition(1, 1);
        cameraTab.addNumber("botposeRedTranY", () -> vision.redAllianceBotPose().getY()).withPosition(2, 1);
        cameraTab.addNumber("botposeRedTranR", () -> vision.redAllianceBotPose().getRotation().getDegrees()).withPosition(3, 1);
        cameraTab.addNumber("botSpacePoseTranX", () -> vision.tagSpaceRobotPose().getX()).withPosition(1, 2);
        cameraTab.addNumber("botSpacePoseTranY", () -> vision.tagSpaceRobotPose().getY()).withPosition(2, 2);
        cameraTab.addNumber("botSpacePoseTranR", () -> vision.tagSpaceRobotPose().getRotation().getDegrees()).withPosition(3, 2);
        cameraTab.addNumber("targetPoseTranX", () -> vision.robotSpaceTagPose().getX()).withPosition(1, 3);
        cameraTab.addNumber("targetPoseTranY", () -> vision.robotSpaceTagPose().getY()).withPosition(2, 3);
        cameraTab.addNumber("targetPoseTranR", () -> vision.robotSpaceTagPose().getRotation().getDegrees()).withPosition(3, 3);
        cameraTab.addNumber("robotAngleGoal", () -> {
            // Translation2d targetPos = new Translation2d(limelight.robotSpaceTagPose().getX(), limelight.robotSpaceTagPose().getY());
            // Translation2d polePos = targetPos.plus(new Translation2d(0.56, 0.23));
            // return polePos.getAngle().getDegrees();
            Pose2d bogusPos = new Pose2d(1,0, new Rotation2d(Math.PI));
            Transform2d bogusTrans = new Transform2d(new Translation2d(1, 0), new Rotation2d(Math.PI));
            Pose2d polePos = bogusPos.transformBy(bogusTrans);
            return polePos.getX();
        }).withPosition(0, 1);

        ShuffleboardTab competitionTab = Shuffleboard.getTab("Competition");
        competitionTab.add("Auto Mode", autoModeSelector._chooser).withPosition(0, 0).withSize(2, 1);
        competitionTab.addNumber("Gyro Pitch", () -> swerve.gyro.getPitch()).withPosition(0, 1);
        competitionTab.addNumber("Gyro Roll", () -> swerve.gyro.getRoll()).withPosition(1, 1);
        competitionTab.addNumber("Gyro Yaw", () -> swerve.gyro.getYaw()).withPosition(2, 1);

        ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake");
        /*intakeTab.add("Speed Override", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withPosition(0, 0)
            .withSize(2, 1);*/

        ShuffleboardTab armTab = Shuffleboard.getTab("Arm");
        armTab.addNumber("Encoder", () -> arm.getEncoder()).withPosition(0, 2);
        armTab.addNumber("Arm PID", () -> arm.getPidValue()).withPosition(1, 2);

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
