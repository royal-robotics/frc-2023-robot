package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DefaultArmCommand extends CommandBase{
    private Arm s_Arm;
    private DoubleSupplier s_SetpointChange;

    private DoubleSolenoid.Value gripValue;
    private DoubleSolenoid.Value angleValue;
    private boolean gripPressed;
    private boolean anglePressed;

    public DefaultArmCommand(Arm arm, DoubleSupplier speed){
        s_Arm = arm;
        addRequirements(s_Arm);

        s_SetpointChange = speed;

        gripValue = DoubleSolenoid.Value.kReverse;
        angleValue = DoubleSolenoid.Value.kReverse;
        gripPressed = false;
        anglePressed = false;
    }

    @Override
    public void initialize(){
        s_Arm.setMotorSpeed(0);
        //s_Arm.setSolenoidGrip(DoubleSolenoid.Value.kReverse);
        //s_Arm.setSolenoidAngle(DoubleSolenoid.Value.kReverse);
    }

    @Override
    public void execute(){

        //s_Arm.setMotorSpeed(s_SetpointChange.getAsDouble());
        double currentSetpoint = s_Arm.getSetpoint();
        double setPointChange = -MathUtil.applyDeadband(s_SetpointChange.getAsDouble(), Constants.stickDeadband) *0.02;

        currentSetpoint = currentSetpoint + setPointChange;
        if(currentSetpoint < Constants.minArmDistance){
            currentSetpoint = Constants.minArmDistance;
        }
        if(currentSetpoint > Constants.maxArmDistance){
            currentSetpoint = Constants.maxArmDistance;
        }
        s_Arm.setSetpoint(currentSetpoint);

        
    }

    @Override
    public void end(boolean interrupted){
        s_Arm.setMotorSpeed(0);
       
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
