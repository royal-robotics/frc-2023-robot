package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class MoveArm extends CommandBase {
    private Arm s_Arm;
    private Intake s_Intake;
    private double s_setPoint;

    public MoveArm(Arm arm, Intake intake, double setPoint){
        s_Intake = intake;
        s_Arm = arm;
        s_setPoint = setPoint;
        addRequirements(s_Intake, s_Arm);
    }
    
    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        if (s_Arm.getEncoder() < Constants.encoderAndSetPointLimit || s_setPoint < Constants.encoderAndSetPointLimit) { //s_Arm.getEncoder() < 0.5 || s_setPoint < 0.5
            if (s_Intake.getBottomSolenoidValue() == Constants.intakeExtend &&
                s_Intake.getTopSolenoidValue() == Constants.intakeExtend &&
                (s_Arm.getSolenoidGrip() == Constants.gripClose || s_Arm.getSolenoidAngle() == Constants.armUp)) {
                    s_Arm.setSetpoint(s_setPoint);
            }
        } else {
            s_Arm.setSetpoint(s_setPoint);
        }
    }
    /*s_Intake.getBottomSolenoidValue() == DoubleSolenoid.Value.kForward &&
                s_Intake.getTopSolenoidValue() == DoubleSolenoid.Value.kForward &&
                s_Arm.getSolenoidGrip() == DoubleSolenoid.Value.kReverse 
    //backup code in case constants replacement breaks something           
    */

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
