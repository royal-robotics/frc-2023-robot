package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class AutoDown extends SequentialCommandGroup {
    public AutoDown(Arm arm, Intake intake) {
        //if (intake.getBottomSolenoidValue() == Constants.intakeRetract ||
                //intake.getTopSolenoidValue() == Constants.intakeRetract) {
            this.addCommands(new AutoExtendIntake(arm, intake, 0, 0.5));
        //}
        //if (arm.getSolenoidGrip() == Constants.gripOpen) {
            this.addCommands(new AutoGripClose(arm, 0.5));
        //}
        //if (arm.getSolenoidAngle() == Constants.armUp) {
            this.addCommands(new AutoGripDown(arm, intake, 1.0));
        //}
        //if (arm.getEncoder() > Constants.encoderAndSetPointLimit) {
            this.addCommands(new MoveArm(arm, intake, Constants.armBottomSetpoint));
        //}
        this.addCommands(new AutoGripOpen(arm, 0.5));
    }
}
