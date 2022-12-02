/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
  XboxController operator = null;
  Collection collection = null;

  Drivetrain drive = null;

  Compressor compressor;

  CamShooter camShooter;
  boolean tankDriveEnabled = false;
  boolean shooterEnabled = true;

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
    System.out.println("Reading config file...");
    Config.loadFromFile();
    System.out.println("Done loading initial config");
    System.out.print("Initializing drivetrain...");
    DriveModule leftModule = new DriveModule(new Talon(3), new Talon(2));
    DriveModule rightModule = new DriveModule(new Talon(1), new Talon(0));
    compressor = new Compressor(1, PneumaticsModuleType.CTREPCM);
    collection = new Collection(new Talon(7), new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3)); //2, 3
    drive = new Drivetrain(leftModule, rightModule, new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1)); //0, 1
    System.out.println("done");

    driver = new XboxController(0);
    operator = new XboxController(1);

    if(shooterEnabled) {
      camShooter = new CamShooter(5, 4, 2, 3, 0, 8, 9, Config.getSetting("cam_loop_period", 0.004));
      camShooter.enable();
    }

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
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
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all the chooser code and uncomment the getString line to get the
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

  @Override
  public void teleopInit() {
    camShooter.enable();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    //Logger.info("Hello World!");
    if (tankDriveEnabled) {
      double leftDrive = deadband(driver.getLeftY());
      double rightDrive = deadband(driver.getRightY());

      drive.tankDrive(leftDrive, rightDrive);
    } else {
      double speedInput = boost.scale(deadband(driver.getRightY()));
      double turnInput = deadband(driver.getLeftX());

      drive.arcadeDrive(turnInput, speedInput);
    }

    // Limit speed input to a lower percentage unless boost mode is on
    boost.setEnabled(driver.getLeftTriggerAxis() > 0.5);

    if (driver.getXButtonPressed()) {
      drive.setPTO(!drive.engaged);
    }

    collection.setCollecting(driver.getYButton());

    if (driver.getLeftBumperPressed()) {
      collection.setExtended(!collection.engaged);
    }

    if(shooterEnabled) {
      camShooter.process(operator.getRightTriggerAxis() > 0, operator.getXButton(), operator.getBButton());
      // camShooter.debug(log);
    }
  }

  @Override
  public void disabledInit() {
    camShooter.disable();
    System.out.println("Reloading configuration from file...");
    Config.loadFromFile();
    System.out.println("Configuration loaded, dumping...");
    Config.dump(System.out);
    System.out.println("Reconfiguring shooter...");
    camShooter.reconfigure();
    System.out.println("Done reconfiguring shooter");
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {

    System.out.println(camShooter.getPosition());
    System.out.println(camShooter.indexTripped());

  }
}
