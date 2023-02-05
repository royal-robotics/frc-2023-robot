package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.sensors.Limelight;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class OurShuffleboard {
    public final ShuffleboardTab m_dashboardTab = Shuffleboard.getTab("Drivetrain");

    public OurShuffleboard(Robot m_Robot){
    RobotContainer container = m_Robot.m_robotContainer;
    Swerve m_swerve = container.s_Swerve;
    Limelight m_limelight = new Limelight();
    ShuffleboardTab cameraTab = Shuffleboard.getTab("Camera");
    cameraTab.addNumber("hasTarget", () -> m_limelight.hasTarget()).withPosition(0, 0);

    cameraTab.addNumber("botposeBlueTranX", () -> m_limelight.getPoseBlue()[0]).withPosition(1,0);
    cameraTab.addNumber("botposeBlueTranY", () -> m_limelight.getPoseBlue()[1]).withPosition(2,0);
    cameraTab.addNumber("botposeBlueTranZ", () -> m_limelight.getPoseBlue()[2]).withPosition(3,0);
    cameraTab.addNumber("botposeRedTranX", () -> m_limelight.getPoseRed()[0]).withPosition(1,1);
    cameraTab.addNumber("botposeRedTranY", () -> m_limelight.getPoseRed()[1]).withPosition(2,1);
    cameraTab.addNumber("botposeRedTranZ", () -> m_limelight.getPoseRed()[2]).withPosition(3,1);
    cameraTab.addNumber("targetPoseTranX", () -> m_limelight.botPoseTargetSpace()[0]).withPosition(1,2);
    cameraTab.addNumber("targetPoseTranY", () -> m_limelight.botPoseTargetSpace()[1]).withPosition(2,2);
    cameraTab.addNumber("targetPoseTranZ", () -> m_limelight.botPoseTargetSpace()[2]).withPosition(3,2);
    cameraTab.addNumber("botSpacePoseTranX", () -> m_limelight.targetPoseRobotSpace()[0]).withPosition(1,3);
    cameraTab.addNumber("botSpacePoseTranY", () -> m_limelight.targetPoseRobotSpace()[1]).withPosition(2,3);
    cameraTab.addNumber("botSpacePoseTranZ", () -> m_limelight.targetPoseRobotSpace()[2]).withPosition(3,3);
    /* cameraTab.addNumber("botSpacePoseTranZ", () -> {

            Pose2d targetPose = new Pose2d(m_limelight.targetPoseRobotSpace()[0], m_limelight.targetPoseRobotSpace()[1], new Rotation2d(m_limelight.targetPoseRobotSpace()[5]));
            targetPose.transformBy(new Transform2d(null, 1))
            })
        .withPosition(6,3);*/

    cameraTab.addCamera("Camera Stream", "lightlime", "http://10.25.22.11:5800/").withPosition(4,0).withSize(4, 4);
    
    ShuffleboardTab testTab = Shuffleboard.getTab("Test");
        testTab.addNumber("FL Speed", () -> (m_swerve.m_ModuleState[0].speedMetersPerSecond)).withPosition(0, 0);
        testTab.addNumber("FL Angle", () -> (m_swerve.m_ModuleState[0].angle.getDegrees())).withPosition(0, 1);
        testTab.addNumber("FR Speed", () -> (m_swerve.m_ModuleState[1].speedMetersPerSecond)).withPosition(1, 0);
        testTab.addNumber("FR Angle", () -> (m_swerve.m_ModuleState[1].angle.getDegrees())).withPosition(1, 1);
        testTab.addNumber("BL Speed", () -> (m_swerve.m_ModuleState[2].speedMetersPerSecond)).withPosition(2, 0);
        testTab.addNumber("BL Angle", () -> (m_swerve.m_ModuleState[2].angle.getDegrees())).withPosition(2, 1);
        testTab.addNumber("BR Speed", () -> (m_swerve.m_ModuleState[3].speedMetersPerSecond)).withPosition(3, 0);
        testTab.addNumber("BR Angle", () -> (m_swerve.m_ModuleState[3].angle.getDegrees())).withPosition(3, 1);
        testTab.addNumber("OdoX", () -> (m_swerve.getPose().getX())).withPosition(0,2);
        testTab.addNumber("OdoY", () -> (m_swerve.getPose().getY())).withPosition(1,2);
        testTab.addNumber("OdoR", () -> (m_swerve.getPose().getRotation().getDegrees())).withPosition(2,2);
    }
}
