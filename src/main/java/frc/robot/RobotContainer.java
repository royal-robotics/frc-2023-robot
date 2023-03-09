package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Visions.Align;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

    /* Drive Controls */
    //private final int translationAxis = XboxController.Axis.kLeftY.value;
    //private final int strafeAxis = XboxController.Axis.kLeftX.value;
    //private final int rotationAxis = XboxController.Axis.kRightX.value;

    private final int translationAxis = Constants.Container.translationAxis;
    private final int strafeAxis = Constants.Container.strafeAxis;
    private final int rotationAxis = Constants.Container.rotationAxis;

    /* Driver Buttons */
    //private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    //private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    
    // private final JoystickButton chargeStationDrive = new JoystickButton(driver, 3);
    // private final JoystickButton zeroGyro = new JoystickButton(driver, 11);
    //private final JoystickButton zeroArmEncoder = new JoystickButton(operator, 8);
    // private final JoystickButton robotCentric = new JoystickButton(driver, 10);
    // private final JoystickButton slow = new JoystickButton(driver, 1); //Trigger
    // private final JoystickButton spinSpeed = new JoystickButton(driver, 2); // Thumb button
    //private final JoystickButton alignAprilTagField = new JoystickButton(operator, 1); //placeholder #
    // private final JoystickButton alignAprilTagRobot = new JoystickButton(operator, 2); //placeholder #
    // private final JoystickButton gridAlignTagPose = new JoystickButton(driver, 8); //placeholder #
    // private final JoystickButton tagAlignCommand = new JoystickButton(driver, 9);
    //private final JoystickButton driveToGoal = new JoystickButton(operator, 4); //placeholder #
    //private final JoystickButton setpointPID = new JoystickButton(operator, 6);
    //intake
    private final int intakeSpeed = Constants.Container.intakeTranslationAxis;
    private final JoystickButton cubeIntake = new JoystickButton(operator, 2); //B
    private final JoystickButton coneIntake = new JoystickButton(operator, 4); //Y
    private final JoystickButton retractIntake = new JoystickButton(operator, 7); 


    //private final JoystickButton autoBalance = new JoystickButton(driver, 4); 

    private final JoystickButton slow = new JoystickButton(driver, 6);
    private final JoystickButton chargeStationDrive = new JoystickButton(driver, 5);
    private final JoystickButton zeroGyro = new JoystickButton(driver, 7);
    //private final JoystickButton robotCentric = new JoystickButton(driver, 6);
    private final JoystickButton spinSpeed = new JoystickButton(driver, 10);
    private final JoystickButton lockForward = new JoystickButton(driver, 4);
    // private final JoystickButton gridAlignTagPose = new JoystickButton(driver, 1); //placeholder #
    private final JoystickButton tagAlignCommand = new JoystickButton(driver, 2);
    private final JoystickButton lockBackward = new JoystickButton(driver, 1);
    
    

    //arm
    private final int armSpeed = Constants.Container.armTranslationAxis;

    private final JoystickButton gripDown = new JoystickButton(operator, 1); //A
    private final JoystickButton gripUp = new JoystickButton(operator, 3); //X
    private final JoystickButton gripToggle = new JoystickButton(operator, 5); //LB
    // private final JoystickButton gripClose = new JoystickButton(operator, 6); //RB
    //private final JoystickButton autoUp = new JoystickButton(operator, 3); //X
    //private final JoystickButton autoDown = new JoystickButton(operator, 1); //A

    private final Trigger setpointTop = new Trigger(() -> operator.getPOV() == 0);
    private final Trigger setpointMiddleCone = new Trigger(() -> operator.getPOV() == 90);
    private final Trigger setpointMiddleCube = new Trigger(() -> operator.getPOV() == 270);
    private final Trigger setpointBottom = new Trigger(() -> operator.getPOV() == 180);

    /* Subsystems */
    public final Visions s_Visions = new Visions();
    public final Swerve s_Swerve = new Swerve(s_Visions);
    public final Arm s_Arm = new Arm();
    public final Intake s_Intake = new Intake();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                //() -> robotCentric.getAsBoolean(),
                () -> slow.getAsBoolean(),
                () -> lockForward.getAsBoolean(),
                () -> lockBackward.getAsBoolean()
            )
        );

        s_Intake.setDefaultCommand(
            new DefaultIntakeCommand(
                s_Intake,
                () -> operator.getRawAxis(intakeSpeed)
            )

        );

        s_Arm.setDefaultCommand(
            new DefaultArmCommand(
                s_Arm,
                () -> operator.getRawAxis(armSpeed)
            )
        );

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        
        chargeStationDrive.whileTrue(new ChargeStationPullUp(s_Intake));
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        //zeroArmEncoder.onTrue(new InstantCommand(() -> s_Arm.zeroArmEncoder()));
        //setpointPID.onTrue(new InstantCommand(() -> s_Arm.setSetpoint(0.5)));
        //setpointPID.onFalse(new InstantCommand(() -> s_Arm.setSetpoint(0)));
        //alignAprilTagField.whileTrue(new GridAlignCommand(s_Swerve, s_Visions, Align.CENTER));
        //gridAlignTagPose.whileTrue(new GridAlignTagPose(s_Swerve, s_Visions, Align.CENTER));  // button2
        tagAlignCommand.whileTrue(new TagAlignCommand(s_Swerve, s_Visions));
        //driveToGoal.whileTrue(new DriveToGoal(s_Swerve));  //button3
        //alignAprilTagRobot.onTrue(new GridAlignRobotSpaceCommand(s_Swerve,  GridAlignRobotSpaceCommand.Align.CENTER));
       // alignAprilTagRobot.onTrue(s_Swerve.driveToPoint());
        slow.onTrue(new InstantCommand(() -> s_Swerve.m_speedMultiplier = Constants.slowMode));
        slow.onFalse(new InstantCommand(() -> s_Swerve.m_speedMultiplier = Constants.fastMode));
        spinSpeed.onTrue(new InstantCommand(() -> s_Swerve.m_spinMultiplier = Constants.fastSpin));
        spinSpeed.onFalse(new InstantCommand(() -> s_Swerve.m_spinMultiplier = Constants.slowSpin));
        cubeIntake.whileTrue(new ExtendIntake(s_Arm, s_Intake, Constants.cubeIntakeSpeed));
        coneIntake.whileTrue(new ExtendIntake(s_Arm, s_Intake, Constants.coneIntakeSpeed));
        retractIntake.whileTrue(new RetractIntake(s_Arm, s_Intake));
        gripDown.whileTrue(new GripDown(s_Arm, s_Intake));
        gripUp.whileTrue(new GripUp(s_Arm, s_Intake));

        gripToggle.onTrue(new InstantCommand(() -> {
            if (s_Arm.getSolenoidGrip() == Constants.gripOpen) {
                s_Arm.setSolenoidGrip(Constants.gripClose);
            } else {
                s_Arm.setSolenoidGrip(Constants.gripOpen);
            }
        }));

        //sautoUp.onTrue(new AutoUp(s_Arm, s_Intake));
        //autoDown.onTrue(new AutoDown(s_Arm, s_Intake));
        // gripClose.whileTrue(new GripClose(s_Arm));
       // autoIntake.whileFalse(new RetractIntake(s_Arm, s_Intake));
       setpointTop.onTrue(new MoveArm(s_Arm, s_Intake, Constants.armTopSetpoint));
       setpointMiddleCone.onTrue(new MoveArm(s_Arm, s_Intake, Constants.armMiddleConeSetpoint));
       setpointMiddleCube.onTrue(new MoveArm(s_Arm, s_Intake, Constants.armMiddleCubeSetpoint));
       setpointBottom.onTrue(new MoveArm(s_Arm, s_Intake, Constants.armBottomSetpoint));
       //autoBalance.whileTrue(new AutoBalanceCommand(s_Swerve));
    }
}