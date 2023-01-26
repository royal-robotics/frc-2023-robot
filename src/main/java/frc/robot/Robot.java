// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.math.geometry.*;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  //private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;
  //private final AutoModeSelector m_autoModeSelector;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    //m_autoModeSelector = new AutoModeSelector(m_robotContainer);
}

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    PathPlannerTrajectory examplePath = PathPlanner.loadPath("New Path", 3, 2);
  Command autoCommand =
   // Assuming this method is part of a drivetrain subsystesm that provides the necessary methods
       new SequentialCommandGroup(
          new InstantCommand(() -> {
            // Reset odometry for the first path you run during auto
                m_robotContainer.s_Swerve.resetOdometry(examplePath.getInitialHolonomicPose());
          }),
          new PPSwerveControllerCommand(
              examplePath, 
              m_robotContainer.s_Swerve::getPose, // Pose supplier
              Constants.Drivebase.swerveKinematics, // SwerveDriveKinematics
              new PIDController(12, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
              new PIDController(12, 0, 0), // Y controller (usually the same values as X controller)
              new PIDController(11, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
              m_robotContainer.s_Swerve::setModuleStates, // Module states consumer
              true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
              m_robotContainer.s_Swerve // Requires this drive subsystem
          )
      );
      autoCommand.schedule();  
    }

  /** This function is called periodically during autonomous. */
  //@Override
  //public void autonomousPeriodic() 

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    //m_robotContainer.s_Swerve.swerveOdometry.resetPosition(new Rotation2d(), m_robotContainer.s_Swerve.getModulePositions(), new Pose2d());
    //TODO: built in odometry reset? 
    //  m_robotContainer.s_Swerve.resetOdometry(new Pose2d());
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
