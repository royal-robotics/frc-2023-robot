package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class DefaultArmCommand extends CommandBase{
    private Arm s_Arm;
    private DoubleSupplier s_Speed;
    private BooleanSupplier s_Grip;
    private BooleanSupplier s_Angle;

    public DefaultArmCommand(Arm arm, DoubleSupplier speed, BooleanSupplier grip, BooleanSupplier angle){
        s_Arm = arm;
        addRequirements(s_Arm);

        s_Speed = speed;
        s_Grip = grip;
        s_Angle = angle;
    }

    @Override
    public void initialize(){
        s_Arm.setMotorSpeed(0);
        s_Arm.setSolenoidGrip(DoubleSolenoid.Value.kReverse);
        s_Arm.setSolenoidAngle(DoubleSolenoid.Value.kReverse);
    }

    @Override
    public void execute(){
        DoubleSolenoid.Value grip = (s_Grip.getAsBoolean()) ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse;
        DoubleSolenoid.Value angle = (s_Angle.getAsBoolean()) ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse;

        s_Arm.setMotorSpeed(s_Speed.getAsDouble());
        s_Arm.setSolenoidGrip(grip);
        s_Arm.setSolenoidAngle(angle);
    }

    @Override
    public void end(boolean interrupted){
        s_Arm.setMotorSpeed(0);
        s_Arm.setSolenoidGrip(DoubleSolenoid.Value.kReverse);
        s_Arm.setSolenoidAngle(DoubleSolenoid.Value.kReverse);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
