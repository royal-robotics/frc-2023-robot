package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Visions;
import frc.robot.subsystems.Swerve;

public class ScoreAlign extends CommandBase {
    private Swerve swerve;
    private Visions vision; 

    private PIDController xController;
    private PIDController yController;
    private PIDController angleController;

    public ScoreAlign(Swerve swerve, Visions vision) {
        this.swerve = swerve;
        this.vision = vision;
        addRequirements(swerve);
        
        this.angleController = new PIDController(4, 0, 0);
        this.xController = new PIDController(2, 0, 0);
        this.yController = new PIDController(2, 0, 0);
    }

    @Override
    public void initialize() {
        angleController.enableContinuousInput(0, Math.PI);
        angleController.setSetpoint(0.0);
        yController.setSetpoint(-0.61);
        xController.setSetpoint(0.845);
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
        return false;
    }
}
