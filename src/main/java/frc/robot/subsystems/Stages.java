// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.XboxController;


public class Stages extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  public final XboxController m_driverController =
      new XboxController(OperatorConstants.kDriverControllerPort);


  TalonSRX stage1 = new TalonSRX(5);
  TalonSRX stage2 = new TalonSRX(4);
  TalonFX flywheel = new TalonFX(40);
  
  final PhotonCamera camera = new PhotonCamera("ArducamOV9281");
  public PhotonPipelineResult result = camera.getLatestResult();
  public boolean hasTargets = result.hasTargets();
  //public List<PhotonTrackedTarget> targets = result.getTargets();
  public PhotonTrackedTarget target = result.getBestTarget();

  public Stages() {}

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
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

// DO NOT DELETE BELOW COMMAND UNLESS YOU WANT TO GET RID OF DRIVETRAIN CONTROL FROM RobotContainer.java
// This command is CURRENTLY not being used, as the drive controls are run from this subsystem, the drivetrain, instead.

  public Command stage() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          stage1.set(TalonSRXControlMode.PercentOutput, 1);
          stage2.set(TalonSRXControlMode.PercentOutput, 1);
          flywheel.setVoltage(12);
        });
  }

  public Command unstage() {

    return runOnce(
        () -> {
          stage1.set(TalonSRXControlMode.PercentOutput, 0);
          stage2.set(TalonSRXControlMode.PercentOutput, 0);
          flywheel.setVoltage(0);
        });
  }
}