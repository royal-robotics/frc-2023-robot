package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class ChargeStationPullUp extends CommandBase {
    private Intake s_Intake;
    

    public ChargeStationPullUp(Intake intake){
        s_Intake = intake;
        addRequirements(s_Intake);
    }
    
    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
      s_Intake.setBottomSolenoidValue(Constants.intakeExtend);
      s_Intake.setLowerMotorSpeed(Constants.Drivebase.chargeStationWheelSpeed); //0.3

    }

    @Override
    public void end(boolean interrupted){    
        s_Intake.setMotorSpeed(0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
