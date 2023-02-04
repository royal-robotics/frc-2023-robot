package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.sensors.Limelight;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class OurShuffleboard {
    public final ShuffleboardTab m_dashboardTab = Shuffleboard.getTab("Drivetrain");

    public OurShuffleboard(Robot m_Robot){
    RobotContainer container = m_Robot.m_robotContainer;
    Swerve m_swerve = container.s_Swerve;
    Limelight limelight = new Limelight();
    ShuffleboardTab cameraTab = Shuffleboard.getTab("Camera");
    cameraTab.addNumber("hasTarget", () -> limelight.hasTarget()).withPosition(1, 0);

    cameraTab.addNumber("botposeTranX", () -> limelight.getPose()[0]).withPosition(2,0);
    cameraTab.addNumber("botposeTranY", () -> limelight.getPose()[1]).withPosition(3,0);
    cameraTab.addNumber("botposeTranZ", () -> limelight.getPose()[2]).withPosition(4,0);
    
    
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
