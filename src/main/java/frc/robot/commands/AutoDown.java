package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class AutoDown extends SequentialCommandGroup {
    public AutoDown(Arm arm, Intake intake) {
        if (arm.getEncoder() > Constants.encoderAndSetPointLimit) {
            if (intake.getBottomSolenoidValue() == Constants.intakeRetract ||
                    intake.getTopSolenoidValue() == Constants.intakeRetract) {
                this.addCommands(new AutoExtendIntake(arm, intake, 0, 1.0));
            }
            if (arm.getSolenoidGrip() == Constants.gripClose) {
                this.addCommands(new AutoGripClose(arm, 0.5));
            }
            if (arm.getSolenoidAngle() == Constants.armUp) {
                this.addCommands(new AutoGripDown(arm, intake, 1.0));
            }
            //this.addCommands(new MoveArm(arm, intake, Constants.armBottomSetpoint));
        }
    }
}
