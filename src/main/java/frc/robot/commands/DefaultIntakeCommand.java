package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Intake;

public class DefaultIntakeCommand extends CommandBase{

    private Intake s_Intake;
    private DoubleSupplier s_Speed;

    private DoubleSolenoid.Value bottomValue;
    private DoubleSolenoid.Value topValue;
    private boolean bottomPressed;
    private boolean topPressed;

    public DefaultIntakeCommand(Intake intake, DoubleSupplier speed){
        s_Intake = intake;
        addRequirements(s_Intake);

        s_Speed = speed;

        bottomValue = DoubleSolenoid.Value.kReverse;
        topValue = DoubleSolenoid.Value.kReverse;
        bottomPressed = false;
        topPressed = false;
    }
    
    @Override
    public void initialize(){
        s_Intake.setMotorSpeed(0);
        //s_Intake.setSolenoidValue(DoubleSolenoid.Value.kReverse);
    }

    @Override
    public void execute(){
        // Toggle bottom
        
        // Toggle top

        s_Intake.setMotorSpeed(s_Speed.getAsDouble());
        //s_Intake.setBottomSolenoidValue(bottomValue);
        //s_Intake.setTopSolenoidValue(topValue);
    }

    @Override
    public void end(boolean interrupted){
        s_Intake.setMotorSpeed(0);
        //s_Intake.setSolenoidValue(DoubleSolenoid.Value.kReverse);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
