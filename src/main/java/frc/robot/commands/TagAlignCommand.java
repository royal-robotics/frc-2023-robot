package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Visions;
import frc.robot.subsystems.Swerve;

public class TagAlignCommand extends CommandBase {
    private PIDController zController;
    private PIDController angleController;
    private Swerve s_Swerve;
    private Visions s_Vision;
    public double distToTag;
    public double zError;
    public double angleToTag;
    public double angleError;
    

    public TagAlignCommand(Swerve swerve, Visions vision) {
        zController = new PIDController(1.5, 0, 0);
        angleController = new PIDController(4, 0, 0);
        s_Swerve = swerve;
        s_Vision = vision;
        addRequirements(s_Swerve);

        distToTag = 0;
        zError = 0; 
        angleToTag = 0;
        angleError = 0;

        ShuffleboardTab autoBalanceTest = Shuffleboard.getTab("Tag Align");
        autoBalanceTest.addNumber("distToTag", () -> distToTag).withPosition(0, 0); 
        autoBalanceTest.addNumber("zError", () -> zError).withPosition(0, 1);
        autoBalanceTest.addNumber("angleToTag", () -> angleToTag).withPosition(1, 0); 
        autoBalanceTest.addNumber("angleError", () -> angleError).withPosition(1, 1);
    }   

    @Override
    public void initialize() {
        zController.setSetpoint(0.925);
        angleController.setSetpoint(0.0);
    }

    @Override 
    public void execute() {
        if (s_Vision.m_Limelight.onTarget()) {
            distToTag = s_Vision.zDistRobotToTag();
            zError = zController.calculate(distToTag);
            if (zError > 1) {
                zError = 1;
            } else if (zError < -1) {
                zError = -1;
            }
            
            angleToTag = Units.degreesToRadians(s_Vision.angleRobotToTag());
            angleError = angleController.calculate(angleToTag);

            s_Swerve.drive(new Translation2d(-zError, 0), angleError, false, true);
        } else {
            s_Swerve.setStableModuleStates();
        }
    
    }

    @Override
    public void end(boolean interrupted) {
        s_Swerve.setStableModuleStates();
    }

}

