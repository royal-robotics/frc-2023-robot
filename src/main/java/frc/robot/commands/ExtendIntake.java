package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ExtendIntake extends CommandBase {

    private Intake s_Intake;
    private Arm s_Arm;
    private double s_wheelSpeed;
    private final double armDownDistance = 0.1;

    public ExtendIntake(Arm arm, Intake intake, double wheelSpeed){
        s_Intake = intake;
        s_Arm = arm;
        addRequirements(s_Intake, s_Arm);
        
        s_wheelSpeed = wheelSpeed;
    }
    
    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        // Toggle bottom
        
        // Toggle tops
        if (s_Arm.getEncoder() < armDownDistance) {        
            s_Intake.setMotorSpeed(s_wheelSpeed); 
            //s_Arm.setSolenoidGrip(Constants.gripOpen); //open grip?
        }
        s_Intake.setBottomSolenoidValue(Constants.intakeExtend);
        s_Intake.setTopSolenoidValue(Constants.intakeExtend);
    
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
