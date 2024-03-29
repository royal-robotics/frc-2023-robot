package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Visions;
import frc.robot.subsystems.Swerve;

public class ScoreAlignCommand extends CommandBase {
    private Swerve swerve;
    private Visions vision; 

    private PIDController xController;
    private PIDController yController;
    private PIDController angleController;

    private double xSetPoint;
    private double ySetPoint;

    public ScoreAlignCommand(Swerve swerve, Visions vision, double xSetPoint, double ySetPoint) {
        this.swerve = swerve;
        this.vision = vision;
        addRequirements(swerve);
        
        this.angleController = new PIDController(Constants.scoreAlignAnglePidKp, 0, 0);  //this.angleController = new PIDController(4, 0, 0);
        this.xController = new PIDController(Constants.scoreAlignDrivePidKp, 0, 0);  //this.xController = new PIDController(2, 0, 0);
        this.yController = new PIDController(Constants.scoreAlignDrivePidKp, 0, 0);  //this.yController = new PIDController(2, 0, 0);

        this.xSetPoint = xSetPoint; 
        this.ySetPoint = ySetPoint;
    }

    @Override
    public void initialize() {
        angleController.enableContinuousInput(0, Math.PI);
        angleController.setSetpoint(0.0);
        yController.setSetpoint(ySetPoint);
        xController.setSetpoint(xSetPoint);
    }

    @Override 
    public void execute() {
        double xSpeed = 0;
        double ySpeed = 0;
        if(vision.m_Limelight.onTarget()) {
            double distFromTag = vision.zDistRobotToTag();
            double yTranslationToTag = vision.yDistRobotToTag();

            xSpeed = xController.calculate(distFromTag); //-.6
            ySpeed = yController.calculate(yTranslationToTag);
            double angleSpeed = angleController.calculate(swerve.getYaw().getRadians());

            swerve.drive(new Translation2d(-xSpeed, ySpeed), angleSpeed, false, false);
        } else {
            swerve.setStableModuleStates();
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setStableModuleStates();
    }   
    
    @Override
    public boolean isFinished() {
        // return xController.atSetpoint() && yController.atSetpoint() && angleController.atSetpoint();
        return false;
    }
}
