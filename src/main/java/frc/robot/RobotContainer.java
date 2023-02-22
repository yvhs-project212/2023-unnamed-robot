// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.NavxSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.TurretConstants;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ArmCommands;
import frc.robot.commands.ClawIntakeCommand;
import frc.robot.commands.ClawOpenCommand;
import frc.robot.commands.ClawRollersOuttakeCommand;
import frc.robot.commands.ControlTurretWithJoystickCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static ArmSubsystem arm = new ArmSubsystem();
  private final ArmCommands armWithDPadsCmd = new ArmCommands(arm);

  public static XboxController driverController = new XboxController(Constants.OperatorConstants.DRIVER_CONTROLLER_PORT);
  public static XboxController operatorController = new XboxController(Constants.OperatorConstants.OPERATOR_CONTROLLER_PORT);

  private final NavxSubsystem m_NavxSubsystem = new NavxSubsystem();

  public static TurretSubsystem turret = new TurretSubsystem();
  private final ControlTurretWithJoystickCommand turretWithJoystick = new ControlTurretWithJoystickCommand(turret);

    //Claw Files
    private final ClawSubsystem clawSub = new ClawSubsystem();
    private final ClawIntakeCommand clawIntakeComm = new ClawIntakeCommand(clawSub);
    private final ClawRollersOuttakeCommand clawRollersOuttakeComm = new ClawRollersOuttakeCommand(clawSub);
    private final ClawOpenCommand clawOpenComm = new ClawOpenCommand(clawSub);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    turret.setDefaultCommand(turretWithJoystick);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

    //Claw Binds
    //Claw Intake
    final JoystickButton clawIntake = new JoystickButton(operatorController, XboxController.Button.kRightBumper.value);
    clawIntake.whileTrue(clawIntakeComm);
    //Claw Rollers Outtake
    final JoystickButton clawOpen = new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value);
    clawOpen.whileTrue(clawOpenComm);
    //Claw Open
    final JoystickButton clawRollersOuttake = new JoystickButton(operatorController, XboxController.Button.kY.value);
    clawRollersOuttake.whileTrue(clawRollersOuttakeComm);
    
    arm.setDefaultCommand(armWithDPadsCmd);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() { 
    // An example command will be run in autonomous
    return null;
  }
}
