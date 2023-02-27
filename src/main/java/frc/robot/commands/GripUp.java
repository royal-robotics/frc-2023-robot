package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class GripUp extends CommandBase{
    private Arm s_Arm;
    private Intake s_Intake;

    public GripUp(Arm arm, Intake intake){
        s_Intake = intake;
        s_Arm = arm;
        addRequirements(s_Intake, s_Arm);
    }
    
    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
      if(s_Intake.getTopSolenoidValue() == Constants.intakeExtend && s_Intake.getBottomSolenoidValue() == Constants.intakeExtend && s_Arm.getSolenoidGrip() == Constants.gripClose){
        s_Arm.setSolenoidAngle(Constants.armUp); //Value.kForward
        s_Intake.setUpperMotorSpeed(-0.2);
        s_Intake.setLowerMotorSpeed(0.2);
      }

    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return false;
    }
}