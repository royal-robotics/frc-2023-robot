package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class RetractIntake extends CommandBase {
    private Arm s_Arm;
    private Intake s_Intake;
    private final double armDownDistance = 0.1;

    public RetractIntake(Arm arm, Intake intake){
        s_Arm = arm;
        s_Intake = intake;
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        // Toggle bottom
        
        // Toggle top
        
        s_Intake.setMotorSpeed(0);

        if (s_Arm.getEncoder() < armDownDistance) {
            s_Intake.setBottomSolenoidValue(Value.kReverse);
            s_Intake.setTopSolenoidValue(Value.kReverse);
        }
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