package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ExtendIntake extends CommandBase {

    private Intake s_Intake;
    private Arm s_Arm;
    private boolean s_isCube;
    private final double armDownDistance = 0.1;

    public ExtendIntake(Arm arm, Intake intake, boolean isCube){
        s_Intake = intake;
        s_Arm = arm;
        s_isCube = isCube;
    }
    
    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        // Toggle bottom
        
        // Toggle tops
        if (s_Arm.getEncoder() < armDownDistance) {        
            if (s_isCube) {
                s_Intake.setMotorSpeed(-0.5);
            } else {
                s_Intake.setMotorSpeed(-0.8);
            }
        }
        s_Intake.setBottomSolenoidValue(Value.kForward);
        s_Intake.setTopSolenoidValue(Value.kForward);
    
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
