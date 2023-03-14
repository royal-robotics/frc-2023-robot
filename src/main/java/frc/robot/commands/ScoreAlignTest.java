package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Visions;
import frc.robot.subsystems.Swerve;

public class ScoreAlignTest extends CommandBase {
    private Swerve swerve;
    private Visions strVision; 
    private Visions tVision;
    private double goalYFromTag;


    private PIDController xController;
    private PIDController yController;
    private PIDController angleController;

    public ScoreAlignTest(Swerve swerve, Visions tiltedVision, Visions straightVision, double goalYFromTag) {
        this.swerve = swerve;
        this.strVision = straightVision;
        this.tVision = tiltedVision;
        this.goalYFromTag = goalYFromTag;
        addRequirements(swerve);
        
        this.angleController = new PIDController(4, 0, 0);
        this.xController = new PIDController(1.5, 0, 0);
        this.yController = new PIDController(1.5, 0, 0);
    }

    @Override
    public void initialize() {
        angleController.enableContinuousInput(0, Math.PI);
        angleController.setSetpoint(0.0);
        yController.setSetpoint(goalYFromTag);
        xController.setSetpoint(0.845);
    }

    @Override 
    public void execute() {
        double xSpeed = 0;
        double ySpeed = 0;
        if(tVision.m_Limelight.onTarget()) {
            double distFromTag = tVision.zDistRobotToTag();
            double yTranslationToTag = tVision.yDistRobotToTag();

            xSpeed = xController.calculate(distFromTag); //-.6
            ySpeed = yController.calculate(yTranslationToTag);
        } else if (strVision.m_Limelight.onTarget()) {
            double distFromTag = strVision.zDistRobotToTag();
            double yTranslationToTag = strVision.yDistRobotToTag();

            xSpeed = xController.calculate(distFromTag); //-.6
            ySpeed = yController.calculate(yTranslationToTag);
        }
        double angleSpeed = angleController.calculate(swerve.getYaw().getRadians()) ;
        
        // Drive 
        // if (DriverStation.getAlliance() == Alliance.Red) {

        // }
        // swerve.drive(new Translation2d(xSpeed, -ySpeed), angleSpeed, true, false);
        swerve.drive(new Translation2d(-xSpeed, ySpeed), angleSpeed, false, false);
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
