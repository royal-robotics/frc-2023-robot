package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.autonomous.AutoModeSelector;
import frc.robot.subsystems.*;
import frc.robot.sensors.Limelight;

public class OurShuffleboard {
    public OurShuffleboard(Robot robot) {
        RobotContainer container = robot.m_robotContainer;
        AutoModeSelector autoModeSelector = robot.m_autoModeSelector;
        Swerve swerve = container.s_Swerve;
        Limelight limelight = new Limelight();

        ShuffleboardTab cameraTab = Shuffleboard.getTab("Camera");
        cameraTab.addCamera("Camera Stream", "lightlime", "http://10.25.22.11:5800/").withPosition(4, 0).withSize(4, 4);
        cameraTab.addNumber("hasTarget", () -> limelight.hasTarget()).withPosition(0, 0);
        cameraTab.addNumber("botposeBlueTranX", () -> limelight.getPoseBlue().getX()).withPosition(1, 0);
        cameraTab.addNumber("botposeBlueTranY", () -> limelight.getPoseBlue().getY()).withPosition(2, 0);
        cameraTab.addNumber("botposeBlueTranZ", () -> limelight.getPoseBlue().getZ()).withPosition(3, 0);
        cameraTab.addNumber("botposeRedTranX", () -> limelight.getPoseRed().getX()).withPosition(1, 1);
        cameraTab.addNumber("botposeRedTranY", () -> limelight.getPoseRed().getY()).withPosition(2, 1);
        cameraTab.addNumber("botposeRedTranZ", () -> limelight.getPoseRed().getZ()).withPosition(3, 1);
        cameraTab.addNumber("botSpacePoseTranX", () -> limelight.botPoseTargetSpace().getX()).withPosition(1, 2);
        cameraTab.addNumber("botSpacePoseTranY", () -> limelight.botPoseTargetSpace().getY()).withPosition(2, 2);
        cameraTab.addNumber("botSpacePoseTranZ", () -> limelight.botPoseTargetSpace().getZ()).withPosition(3, 2);
        cameraTab.addNumber("targetPoseTranX", () -> limelight.targetPoseRobotSpace().getX()).withPosition(1, 3);
        cameraTab.addNumber("targetPoseTranY", () -> limelight.targetPoseRobotSpace().getY()).withPosition(2, 3);
        cameraTab.addNumber("targetPoseTranZ", () -> limelight.targetPoseRobotSpace().getZ()).withPosition(3, 3);
        cameraTab.addNumber("robotAngleGoal", () -> {
            Translation2d targetPos = new Translation2d(limelight.targetPoseRobotSpace().getX(), limelight.targetPoseRobotSpace().getZ());
            Translation2d polePos = targetPos.plus(new Translation2d(0.56, 0.23));
            return polePos.getAngle().getDegrees();
        }).withPosition(0, 1);

        ShuffleboardTab competitionTab = Shuffleboard.getTab("Competition");
        competitionTab.add("Auto Mode", autoModeSelector._chooser).withPosition(0, 0).withSize(2, 1);
        competitionTab.addNumber("Gyro Pitch", () -> swerve.gyro.getPitch()).withPosition(1, 0);
        competitionTab.addNumber("Gyro Roll", () -> swerve.gyro.getRoll()).withPosition(2, 0);
        competitionTab.addNumber("Gyro Yaw", () -> swerve.gyro.getYaw()).withPosition(3, 0);

        ShuffleboardTab testTab = Shuffleboard.getTab("Test");
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
