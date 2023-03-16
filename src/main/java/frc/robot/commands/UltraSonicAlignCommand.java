package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class UltraSonicAlignCommand extends CommandBase {
    private Swerve swerve;
    private PIDController xController;
    private PIDController angleController;
    public double xError;
    public double angleError;

    public UltraSonicAlignCommand(Swerve swerve){
        this.swerve = swerve;
        xController = new PIDController(4, 0.00, 0);
        angleController = new PIDController(4, 0, 0);
        //xController.reset();
        //xController.setIntegratorRange(xError, angleError);

        xError = 0;
        angleError = 0;
        addRequirements(swerve);
        ShuffleboardTab sonicTab = Shuffleboard.getTab("UltraSonic");
        sonicTab.addNumber("Average Voltage", () -> swerve.getVoltage()).withPosition(0,0);
        //sonicTab.addNumber("Sonic Distance", () -> swerve.getSonicDistance()).withPosition(1,0);
        sonicTab.addString("Voltage Readings", () -> swerve.voltageReadings.toString()).withPosition(0, 2);
        sonicTab.addNumber("XError", () -> xError).withPosition(1, 2);
    }

    @Override
    public void initialize(){
        xController.setSetpoint(0.65);
        angleController.enableContinuousInput(0, Math.PI);
        angleController.setSetpoint(0);
    }

    @Override
    public void execute(){
        if(Math.abs(xError-xController.calculate(swerve.getSonicDistance()))>=0.01){
            xController.reset();
        } 
        xError = xController.calculate(swerve.getSonicDistance());
        // xError = xController.calculate();
        if(xError > 1.0){
            xError = 1.0;
        } else if(xError < -1.0){
            xError = -1.0;
        }
        angleError = angleController.calculate(swerve.getYaw().getRadians());
        swerve.drive(new Translation2d(-xError,0), angleError, false, true);
    
    }

    @Override
    public void end(boolean interrupted){
        swerve.setStableModuleStates();
    }

    @Override
    public boolean isFinished(){
        // if(xController.atSetpoint()){
        //     return true;
        // }
        return false;
    }
}

    
