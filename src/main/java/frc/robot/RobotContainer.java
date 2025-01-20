// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Stages;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final Drivetrain drivetrain = new Drivetrain();
  public final Stages stages = new Stages();
  final PhotonCamera camera = new PhotonCamera("ArducamOV9281");

  public final XboxController m_driverController =
  new XboxController(OperatorConstants.kDriverControllerPort);
  
  //  See below
  /** Uncomment this section, uncomment drivetrain.setDefault..., and comment in 
   *  m_drivetrain.arcadeDrive... in Drivetrain.java to put drive command control 
   *  in RobotContainer.java.

    -------
        public final XboxController m_driverController =
            new XboxController(OperatorConstants.kDriverControllerPort);

        
        TalonFX rightFrontMotor = new TalonFX(11);
        TalonFX rightBackMotor = new TalonFX(10);
        MotorControllerGroup m_right = new MotorControllerGroup(rightFrontMotor, rightBackMotor);
        TalonFX leftFrontMotor = new TalonFX(13);
        TalonFX leftBackMotor = new TalonFX(12);
        MotorControllerGroup m_left = new MotorControllerGroup(leftFrontMotor, leftBackMotor);
        DifferentialDrive m_drivetrain = new DifferentialDrive(m_left, m_right);
    -------

  */

  // Replace with CommandPS4Controller or CommandJoystick if needed

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    //drivetrain.setDefaultCommand(drivetrain.drive(m_driverController.getRightX(), m_driverController.getLeftY()));
    //stages.setDefaultCommand(stages.runStageTestVision(stages, camera));
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
    //new Trigger(m_drivetrain::exampleCondition).onTrue(new ExampleCommand(m_drivetrain));
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //drivetrain.setDefaultCommand(drivetrain.drive(-0.7*m_driverController.getRightX(), -0.7*m_driverController.getLeftY()));
    //stages.setDefaultCommand(stages.stage());
    new Trigger(() -> m_driverController.getAButton()).onTrue(stages.stage()).onFalse(stages.unstage()); 
    new Trigger(() -> stages.target != null).onTrue(stages.stage().alongWith(new PrintCommand("Running"))).onFalse(stages.unstage().alongWith(new PrintCommand("Not Running")));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  /*
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_drivetrain);
  }
  */
}
