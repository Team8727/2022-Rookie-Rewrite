// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonUtils;
// import org.photonvision.targeting.PhotonPipelineResult;
// import org.photonvision.targeting.PhotonTrackedTarget;
// import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Stages;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private final Stages stages = new Stages();

  private final Drivetrain m_drivetrain = new Drivetrain();
  
  public final XboxController m_driverController =
      new XboxController(OperatorConstants.kDriverControllerPort);

  //public final PhotonCamera camera = new PhotonCamera("Arducam_OV9281_USB_Camera");

  //===================================================================
  // Constants such as camera and target height stored. Change per robot and goal!
    // Change this to match the name of your camera

    //XboxController xboxController = new XboxController(0);

    // Drive motors
    //PWMVictorSPX leftMotor = new PWMVictorSPX(0);
    //PWMVictorSPX rightMotor = new PWMVictorSPX(1);
    //DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);
  //===================================================================
  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    //setDefaultCommand(m_drivetrain.drive(m_driverController.getRightX(), m_driverController.getLeftY()));

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    System.out.println(m_drivetrain.getCurrentCommand());

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  
  public void autonomousInit() {
    //m_autonomousCommand = m_robotContainer.getAutonomousCommand();
  

    // schedule the autonomous command (example)
    /*
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    */
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
    m_drivetrain.setDefaultCommand(m_drivetrain.drive(() -> m_driverController.getRightX(), () -> m_driverController.getLeftY()));
    
    // double turn = -0.1*m_driverController.getRightX();
    // double forward = -0.1*m_driverController.getLeftY();

    // // Read in relevant data from the Camera
    // boolean targetVisible = false;
    // double targetYaw = 0.0;
    // PhotonPipelineResult results = camera.getLatestResult();

    /*
          if (result.hasTargets()) {
            //System.out.println("hasTargets() confirmed");
            // At least one AprilTag was seen by the camera
            for (var target : result.getTargets()) {
              //System.out.println("for loop is working");
                if (target.getFiducialId() == 1) {
                    // Found Tag 7, record its information
                    targetYaw = target.getYaw();
                    targetVisible = true;
                    System.out.println("Target 1 found, yaw set");
                }
                
            }
        }
    }
    */

    // if (m_driverController.getYButton() && targetVisible) {
    //   // Driver wants auto-alignment to tag 7
    //   // And, tag 7 is in sight, so we can turn toward it.
    //   // Override the driver's turn command with an automatic one that turns toward the tag.
    //   System.out.println("Target Visible, and recognizing Y Button");
    //   //turn = -1.0 * targetYaw;
    // }
    
    //System.out.println("Driving wheels");
    //m_drivetrain.drive(turn, forward);
    //m_drivetrain.m_drivetrain.feed();
    
    
    /*
    var result = camera.getLatestResult();
    boolean hasTargets = result.hasTargets();
    //List<PhotonTrackedTarget> targets = result.getTargets();
    PhotonTrackedTarget target = result.getBestTarget();
    // Get information from target.
    int targetID = target.getFiducialId();
    double poseAmbiguity = target.getPoseAmbiguity();
    Transform3d bestCameraToTarget = target.getBestCameraToTarget();
    stages.stage();
    //new Trigger(() -> target != null && RobotState.isTeleop()).onTrue(stages.stage().alongWith(new PrintCommand("Staged"))).onFalse(stages.unstage().alongWith(new PrintCommand("Un-Staged")));
    */
    
    /*
    if (target == null) {
      System.out.println("Un - Staging");
      stages.unstage();
    } else {
      System.out.println("Staging");
      stages.stage();
    }
    */
    
    /*
    double forwardSpeed;
        double rotationSpeed = m_driverController.getLeftX();

        if (m_driverController.getAButton()) {
            // Vision-alignment mode
            // Query the latest result from PhotonVision
            var result = camera.getLatestResult();
            List<PhotonTrackedTarget> targets = result.getTargets();
            System.out.println(targets);


            if (result.hasTargets()) {
                // First calculate range
                double range =
                        PhotonUtils.calculateDistanceToTargetMeters(
                                CAMERA_HEIGHT_METERS,
                                TARGET_HEIGHT_METERS,
                                CAMERA_PITCH_RADIANS,
                                Units.degreesToRadians(result.getBestTarget().getPitch()));

                // Use this range as the measurement we give to the PID controller.
                // -1.0 required to ensure positive PID controller effort _increases_ range
                forwardSpeed = -controller.calculate(range, GOAL_RANGE_METERS);
            } else {
                // If we have no targets, stay still.
                forwardSpeed = 0;
            }
        } else {
            // Manual Driver Mode
            forwardSpeed = -m_driverController.getRightY();
        }

        // Use our forward/turn speeds to control the drivetrain
        m_drivetrain.drive(forwardSpeed, rotationSpeed);
    */
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
