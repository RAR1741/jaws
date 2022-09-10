/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.util.Date;
import java.util.Timer;
import java.util.TimerTask;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.logging.Loggable;
import frc.robot.logging.LoggableCompressor;
import frc.robot.logging.LoggableTimer;
import frc.robot.logging.Logger;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  XboxController driver = null;
  Collection collection = null;

  Drivetrain drive = null;

  Compressor compressor;

  Logger logger;
  TimerTask logTimer;
  Timer runTimer;

  LoggableTimer timer;

  private static final double DEADBAND_LIMIT = 0.01;
  private static final double SPEED_CAP = 0.6;
  InputScaler joystickDeadband = new Deadband(DEADBAND_LIMIT);
  InputScaler joystickSquared = new SquaredInput(DEADBAND_LIMIT);
  BoostInput boost = new BoostInput(SPEED_CAP);

  public double deadband(double in) {
    double out = joystickSquared.scale(in);
    return joystickDeadband.scale(out);
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    timer = new LoggableTimer("Time");
    logger = new Logger();
    runTimer = new Timer();

    logTimer = new TimerTask() {
      @Override
      public void run() {
        logger.collectData();
        try {
          logger.writeData();
        } catch (IOException io) {
          io.printStackTrace();
        }
      }
    };

    try {
      logger.createLog();
    } catch (IOException e) {
      e.printStackTrace();
    }

    System.out.print("Initializing drivetrain...");
    DriveModule leftModule = new DriveModule(new Talon(13), new Talon(3));
    DriveModule rightModule = new DriveModule(new Talon(1), new Talon(15));

    compressor = new LoggableCompressor(1, PneumaticsModuleType.CTREPCM);

    collection = new Collection(new Talon(7), new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3)); //2, 3

    drive = new Drivetrain(leftModule, rightModule, new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1)); //0, 1
    System.out.println("done");

    driver = new XboxController(0);

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    Config.loadFromFile("config.txt"); // It doesn't actually read from any file

    logger.addLoggable((Loggable) compressor);
    logger.addLoggable((Loggable) timer);
    // logger.addLoggable((Loggable) leftModule); // TODO: Figure out what to log
    // logger.addLoggable((Loggable) rightModule); // TODO: Figure out what to log
    
    logger.collectHeaders();
    try {
		  logger.writeHeaders();
    } catch (IOException io) {
		  io.printStackTrace();
	  }

	  runTimer.schedule(logTimer, new Date(), 33);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // autoSelected = SmartDashboard.getString("Auto Selector",
    // defaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    // TODO before next test add in gearshift solenoid and temp for testing main arm
    // solenoid

    double leftDrive = deadband(driver.getRightY());
    double rightDrive = deadband(driver.getLeftY());

    // Limit speed input to a lower percentage unless boost mode is on
    boost.setEnabled(driver.getLeftTriggerAxis() > 0.5);
    // speedInput = boost.scale(speedInput);

    drive.tankDrive(leftDrive, rightDrive);
    if (driver.getXButtonPressed()) {
      drive.setPTO(!drive.engaged);
    }

    // drive.arcadeDrive(turnInput, speedInput);

    collection.setCollecting(driver.getYButton());

    if (driver.getLeftBumperPressed()) {
      collection.setExtended(!collection.engaged);
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
