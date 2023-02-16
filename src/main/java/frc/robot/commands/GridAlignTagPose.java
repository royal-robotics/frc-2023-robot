package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Visions;
import frc.robot.Visions.Align;
import frc.robot.subsystems.Swerve;

public class GridAlignTagPose extends CommandBase{
    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(Constants.Auto.kMaxSpeedMetersPerSecond, Constants.Auto.kMaxAccelerationMetersPerSecondSquared);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(Constants.Auto.kMaxSpeedMetersPerSecond, Constants.Auto.kMaxAccelerationMetersPerSecondSquared);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(8, 8);
    
    private final Visions vision;
    private final Swerve drivetrain;
    private final Align goalAlign;
  
    private final ProfiledPIDController xController = new ProfiledPIDController(1, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(1, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(1, 0, 0, OMEGA_CONSTRAINTS);
  
  
    private int lastTagID;

    // fields for shuffleboard purposes
    public Pose2d robotPose;
    public Pose2d tagPoseRobotSpace;
    public Pose2d tagPose;
    public Pose2d goalPose;
  
    public GridAlignTagPose(Swerve drivetrain, Visions vision, Align goalAlign) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        this.goalAlign = goalAlign;
    
        xController.setTolerance(0.2);
        yController.setTolerance(0.2);
        omegaController.setTolerance(Units.degreesToRadians(3));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);
    
        addRequirements(drivetrain);
    }
  
    @Override
    public void initialize() {
        lastTagID = 0;
        robotPose = drivetrain.getPose();
        // robotPose = vision.blueAllianceBotPose();
        omegaController.reset(robotPose.getRotation().getRadians());
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());
    }
  
    @Override
    public void execute() {
        robotPose = drivetrain.getPose();
        // robotPose = vision.blueAllianceBotPose();

        ShuffleboardTab test = Shuffleboard.getTab("GridAlignVisionTesting");
        test.addDouble("BotPoseX", ()->robotPose.getX()).withPosition(0, 0);
        test.addDouble("BotPoseY", ()->robotPose.getY()).withPosition(0, 1);
        test.addDouble("BotPoseAngle", ()->robotPose.getRotation().getDegrees()).withPosition(0, 2);
        
        if (vision.m_Limelight.onTarget()) {
            // lastTagID = limelight.getTagID();
            lastTagID = 1;

            // Transform the robot's pose to find the tag's pose in field space
            tagPoseRobotSpace = vision.robotSpaceTagPose();
            // Pose2d tagPoseRobotSpace = new Pose2d(limelight.targetPoseRobotSpace()[0], -limelight.targetPoseRobotSpace()[1], 
            //                                     new Rotation2d(limelight.targetPoseRobotSpace()[5]));
            Transform2d botToTag = new Transform2d(tagPoseRobotSpace.getTranslation(), tagPoseRobotSpace.getRotation());
            test.addDouble("TagPoseRobotSpaceX", ()->tagPoseRobotSpace.getX()).withPosition(1, 0);
            test.addDouble("TagPoseRobotSpaceY", ()->tagPoseRobotSpace.getY()).withPosition(1, 1);
            test.addDouble("TagPoseRobotSpaceAngle", ()->tagPoseRobotSpace.getRotation().getDegrees()).withPosition(1, 2);

            tagPose = robotPose.transformBy(botToTag);
            test.addDouble("TagPoseFieldSpaceX", ()->tagPose.getX()).withPosition(2, 0);
            test.addDouble("TagPoseFieldSpaceY", ()->tagPose.getY()).withPosition(2, 1);
            test.addDouble("TagPoseFieldSpaceAngle", ()->tagPose.getRotation().getDegrees()).withPosition(2, 2);
            
            // Transform the tag's pose to set our goal
            goalPose = vision.tagPoseToGoalPose(tagPose, goalAlign);
            test.addDouble("goalPoseX", ()->tagPose.getX()).withPosition(3, 0);
            test.addDouble("goalPoseY", ()->tagPose.getY()).withPosition(3, 1);
            test.addDouble("goalPoseAngle", ()->tagPose.getRotation().getDegrees()).withPosition(3, 2);
            
            
            xController.setGoal(goalPose.getX());
            yController.setGoal(goalPose.getY());
            omegaController.setGoal(goalPose.getRotation().getRadians());
        }
        
        // Drive
        if (lastTagID == 0) {
            // No target has been visible
            drivetrain.drive(new Translation2d(), 0, true, false);
        } else {
            // Drive to the target
            var xSpeed = xController.calculate(robotPose.getX());
            if (xController.atGoal()) {
                xSpeed = 0;
            }
    
            var ySpeed = yController.calculate(robotPose.getY());
            if (yController.atGoal()) {
                ySpeed = 0;
            }
    
            var omegaSpeed = omegaController.calculate(robotPose.getRotation().getRadians());
            if (omegaController.atGoal()) {
                omegaSpeed = 0;
            }
            
            // drivetrain.drive(new Translation2d(xSpeed, ySpeed), omegaSpeed, true, false);
        }
    }
  
    @Override
    public void end(boolean interrupted) {
        // drivetrain.drive(new Translation2d(), 0, true, false);
    }

    @Override
    public boolean isFinished(){ 
        return xController.atGoal() && yController.atGoal() && omegaController.atGoal();
        // return false;
    }
  
}
  