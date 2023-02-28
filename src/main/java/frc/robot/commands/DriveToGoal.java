package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class DriveToGoal extends CommandBase {
    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(Constants.Auto.kMaxSpeedMetersPerSecond, Constants.Auto.kMaxAccelerationMetersPerSecondSquared);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(Constants.Auto.kMaxSpeedMetersPerSecond, Constants.Auto.kMaxAccelerationMetersPerSecondSquared);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(8, 8);
  
    private final Swerve drivetrain;
  
    private final ProfiledPIDController xController = new ProfiledPIDController(1, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(1, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(1, 0, 0, OMEGA_CONSTRAINTS);
  
    private final Pose2d goalPose = new Pose2d(1, 1, new Rotation2d(Math.PI));
  
    public DriveToGoal(Swerve drivetrain) {
        this.drivetrain = drivetrain;
    
        xController.setTolerance(0.2);
        yController.setTolerance(0.2);
        omegaController.setTolerance(Units.degreesToRadians(3));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);
    
        addRequirements(drivetrain);
    }
  
    @Override
    public void initialize() {
        Pose2d robotPose = drivetrain.getPose();
        omegaController.reset(robotPose.getRotation().getRadians());
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());

        xController.setGoal(goalPose.getX());
        yController.setGoal(goalPose.getY());
        omegaController.setGoal(goalPose.getRotation().getRadians());
    }  
  
    @Override
    public void execute() {
        Pose2d robotPose = drivetrain.getPose();

        double xSpeed = xController.calculate(robotPose.getX());
        if (xController.atGoal()) {
            xSpeed = 0;
        }

        double ySpeed = yController.calculate(robotPose.getY());
        if (yController.atGoal()) {
            ySpeed = 0;
        }

        double omegaSpeed = omegaController.calculate(robotPose.getRotation().getRadians());
        if (omegaController.atGoal()) {
            omegaSpeed = 0;
        }
        
        drivetrain.drive(new Translation2d(xSpeed, ySpeed), omegaSpeed, true, false);

    }
  
    @Override
    public void end(boolean interrupted) {
        drivetrain.setStableModuleStates();
    }

    @Override
    public boolean isFinished() {
        // return xController.atGoal() && y Controller.atGoal() && omegaController.atGoal();
        return false;
    }
}
