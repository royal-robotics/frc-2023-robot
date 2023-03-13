package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Visions;
import frc.robot.subsystems.Swerve;

public class GridAlign extends CommandBase {
    private Swerve swerve;
    private Visions vision;
    private double goalYFromTag;
    private boolean hasTarget;

    private PIDController xController;
    private PIDController yController;
    private PIDController angleController;

    // --- shuffleboard fields ------
    public double deltaX;
    public double deltaY;

    public GridAlign(Swerve swerve, Visions vision, double goalYFromTag) {
        this.swerve = swerve;
        this.vision = vision;
        this.goalYFromTag = goalYFromTag;
        this.hasTarget = false;
        addRequirements(swerve);

        this.angleController = new PIDController(4, 0, 0);
        this.xController = new PIDController(1.5, 0, 0);
        this.yController = new PIDController(1.5, 0, 0);

        ShuffleboardTab scoreAlignTab = Shuffleboard.getTab("ScoreAlign"); 
        scoreAlignTab.addNumber("Current X", () -> swerve.getPose().getX()).withPosition(0, 0); 
        scoreAlignTab.addNumber("Current Y", () -> swerve.getPose().getX()).withPosition(0, 1); 
        scoreAlignTab.addNumber("Goal X", () -> getXSetPoint()).withPosition(1, 0); 
        scoreAlignTab.addNumber("Goal Y", () -> getYSetPoint()).withPosition(1, 1);
        scoreAlignTab.addNumber("Delta X", () -> deltaX).withPosition(2, 0); 
        scoreAlignTab.addNumber("Delta Y", () -> deltaY).withPosition(2, 1);
    }

    @Override 
    public void initialize() {
        angleController.enableContinuousInput(0, Math.PI);
        angleController.setSetpoint(0.0);

        double xSetpoint = 0.0;
        double ySetpoint = 0.0;

        if (vision.m_Limelight.onTarget()) {
            this.hasTarget = true;

            xSetpoint = getXSetPoint();
            ySetpoint = getYSetPoint();
        }   

        yController.setSetpoint(ySetpoint);
        xController.setSetpoint(xSetpoint);
    }

    @Override 
    public void execute() {
        if (hasTarget) {
            if (vision.m_Limelight.onTarget()) {
                xController.setSetpoint(getXSetPoint());
                yController.setSetpoint(getYSetPoint());
            }
            
            double angleSpeed = angleController.calculate(swerve.getYaw().getRadians());
            double xSpeed = xController.calculate(swerve.getPose().getX());
            double ySpeed = yController.calculate(swerve.getPose().getY());

            // swerve.drive(new Translation2d(xSpeed, ySpeed), angleSpeed, true, false);
            swerve.drive(new Translation2d(0, ySpeed), angleSpeed, true, false);
        } else {
            swerve.setStableModuleStates();
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setStableModuleStates();
    }

    @Override 
    public boolean isFinished(){
        return false;
    }

    public double getXSetPoint() {
        double xFromTag = vision.zDistRobotToTag();
        double goalX = xFromTag - 0.845;
        deltaX = goalX;
        double botPosX = swerve.getPose().getX();
        return goalX + botPosX;
    }

    public double getYSetPoint() {
        double yFromTag = vision.yDistRobotToTag();
        double goalY = -yFromTag + goalYFromTag; 
        deltaY = goalY;
        double botPosY = swerve.getPose().getY();
        return goalY + botPosY;
    }
}
