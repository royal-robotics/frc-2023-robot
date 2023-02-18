package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Intake;


public class DefaultIntakeCommand extends CommandBase{

    private Intake s_Intake;
    private DoubleSupplier s_Speed;
    private BooleanSupplier s_Solenoid;

    public DefaultIntakeCommand(Intake intake, DoubleSupplier speed, BooleanSupplier solenoid){
        s_Intake = intake;
        addRequirements(s_Intake);

        s_Speed = speed;
        s_Solenoid = solenoid;
    }
    
    @Override
    public void initialize(){
        s_Intake.setMotorSpeed(0);
        s_Intake.setSolenoidValue(DoubleSolenoid.Value.kReverse);
    }

    @Override
    public void execute(){
        DoubleSolenoid.Value solenoid = (s_Solenoid.getAsBoolean()) ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse;

        s_Intake.setMotorSpeed(s_Speed.getAsDouble());
        s_Intake.setSolenoidValue(solenoid);
    }

    @Override
    public void end(boolean interrupted){
        s_Intake.setMotorSpeed(0);
        s_Intake.setSolenoidValue(DoubleSolenoid.Value.kReverse);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}