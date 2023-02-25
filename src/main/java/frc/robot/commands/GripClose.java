package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class GripClose extends CommandBase{
    private Arm s_Arm;
    

    public GripClose(Arm arm){
        s_Arm = arm;
    }
    
    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
      s_Arm.setSolenoidGrip(Value.kReverse);

    }

    @Override
    public void end(boolean interrupted){
        
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}