// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ArcadeDriveCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.ClawIntakeCommand;
import frc.robot.commands.ClawOpenCommand;
import frc.robot.commands.ClawRollersOuttakeCommand;
import frc.robot.commands.ElevatorLiftWithjoystickCommand;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Controllers
  public static XboxController driverController = new XboxController(Constants.OperatorConstants.DRIVER_CONTROLLER_PORT);
  public static XboxController operatorController = new XboxController(Constants.OperatorConstants.OPERATOR_CONTROLLER_PORT);

  // Drivetrain Files
  private final DrivetrainSubsystem drivetrainSub = new DrivetrainSubsystem();
  private final ArcadeDriveCommand arcadeDriveComm = new ArcadeDriveCommand(drivetrainSub);

  //Claw Files
  private final ClawSubsystem clawSub = new ClawSubsystem();
  private final ClawIntakeCommand clawIntakeComm = new ClawIntakeCommand(clawSub);
  private final ClawRollersOuttakeCommand clawRollersOuttakeComm = new ClawRollersOuttakeCommand(clawSub);
  private final ClawOpenCommand clawOpenComm = new ClawOpenCommand(clawSub);

  //Elevator Files
  private final ElevatorSubsystem elevatorSub = new ElevatorSubsystem();
  private final ElevatorLiftWithjoystickCommand elevatorLiftComm = new ElevatorLiftWithjoystickCommand(elevatorSub);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    drivetrainSub.setDefaultCommand(arcadeDriveComm);
    elevatorSub.setDefaultCommand(elevatorLiftComm);

    configureBindings();
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
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }

}
