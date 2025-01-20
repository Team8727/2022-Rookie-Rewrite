// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;
import java.util.function.Supplier;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.XboxController;
import org.photonvision.PhotonCamera;
import org.photonvision.*;


public class Drivetrain extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  public final XboxController m_driverController =
      new XboxController(OperatorConstants.kDriverControllerPort);

  TalonFX rightFrontMotor = new TalonFX(11);
  TalonFX rightBackMotor = new TalonFX(10);
  MotorControllerGroup m_right = new MotorControllerGroup(rightFrontMotor, rightBackMotor);
  TalonFX leftFrontMotor = new TalonFX(13);
  TalonFX leftBackMotor = new TalonFX(12);
  MotorControllerGroup m_left = new MotorControllerGroup(leftFrontMotor, leftBackMotor);
  public DifferentialDrive m_drivetrain = new DifferentialDrive(m_left, m_right);
  Stages stages = new Stages();

  public Drivetrain() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
   
  }

//-=-=-=-=-=-=- Change the below code in the future to put the drive method in the Robot.java file
//              with teleopPeriodic or in the RobotContainer.java file with 

  @Override
  public void periodic() {
    m_drivetrain.feed();
    // This method will be called once per scheduler run
    //m_drivetrain.arcadeDrive(-0.7*m_driverController.getRightX(), -0.7*m_driverController.getLeftY());
    //Move this to RobotContainer.java
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

// DO NOT DELETE BELOW COMMAND UNLESS YOU WANT TO GET RID OF DRIVETRAIN CONTROL FROM RobotContainer.java
// This command is CURRENTLY not being used, as the drive controls are run from this subsystem, the drivetrain, instead.

  public Command drive(Supplier<Double> xSpeed_getRightX, Supplier<Double> zRotation_getLeftY) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(() -> {
      m_drivetrain.arcadeDrive(-0.1*xSpeed_getRightX.get(), -0.1*zRotation_getLeftY.get());
      //System.out.println(xSpeed_getRightX + zRotation_getLeftY);
    });
  
  }
}
