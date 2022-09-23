package frc.robot;

import java.lang.Math;
import java.util.concurrent.locks.ReentrantLock;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;

public class CamShooter implements Runnable {

	private final double CAM_READY_TO_FIRE_POSITION = Config.getSetting("cam_ready_to_fire_position", 35);
	private final double CAM_FIRE_TO_POSITION = Config.getSetting("cam_fire_to_position", 45);
	private final double CAM_FIRE_POSITION_TOLERANCE = Config.getSetting("cam_fire_position_tolerance", 3);
	private final double CAM_POINT_OF_NO_RETURN = Config.getSetting("cam_point_of_no_return", 58);
	private final double CAM_SETPOINT_ERROR_LIMIT = Config.getSetting("cam_setpoint_error_limit", 25);
	private final ReentrantLock accessSemaphore;

	boolean scopeToggleState = false;
	double m_P;
	double m_I;
	double m_D;
	double m_totalError;
	double m_prevError;
	double m_setpoint;
	boolean m_pidEnabled;
	double m_minimumInput;
	double m_maximumInput;
	double m_minimumOutput;
	double m_maximumOutput;
	String m_state;
	boolean indexHasBeenSeen;
	boolean m_enabled;
	boolean shouldFire = false;
	boolean shouldRearm = false;
	boolean shouldEject = false;

	Notifier controlLoop;
	Talon shooterMotorLeft;
	Talon shooterMotorRight;
	CamMotors cam_outputter;
	Encoder shooterEncoder;
	DigitalInput indexSensor;
	DigitalOutput scopeToggle;
	DigitalOutput scopeCycle;

	public class CamMotors {
		final Talon LEFT;
		final Talon RIGHT;

		public CamMotors(Talon left, Talon right) {
			this.LEFT = left;
			this.RIGHT = right;
		}

		public void PIDWrite(double out) {
			LEFT.set(out);
			RIGHT.set(out);
		}
	}

	public CamShooter(int motorLeft, int motorRight, int encoderA, int encoderB, int indexInput, int scopeTogglePort, int scopeCyclePort, double period) {		
		accessSemaphore = new ReentrantLock();
		accessSemaphore.lock(); //I know, there is no try(){}finally{} following it; sue me
		controlLoop = new Notifier(this);
		final double LINES_PER_REV = 360;
		final double GEAR_REDUCTION_RATIO = 2250.0 / 49.0;
		shooterMotorLeft = new Talon(motorLeft);
		shooterMotorRight = new Talon(motorRight);
		cam_outputter = new CamMotors(shooterMotorLeft, shooterMotorRight);
		shooterEncoder = new Encoder(encoderA, encoderB, false, EncodingType.k4X);
		indexSensor = new DigitalInput(indexInput);
		scopeToggle = new DigitalOutput(scopeTogglePort);
		scopeCycle = new DigitalOutput(scopeCyclePort);

		shooterEncoder.setDistancePerPulse(100.0 / (GEAR_REDUCTION_RATIO * LINES_PER_REV));
		//ShooterEncoder->SetPIDSourceParameter(Encoder::kDistance); (trying to find java equivalent (so far I only see things working with PIDController))
		shooterEncoder.reset();

		//Found this in code:
		/*
		PID = new PIDController(Config::GetSetting("cam_p", 0.04),
											Config::GetSetting("cam_i", 0.005), 
											Config::GetSetting("cam_d", 0.03), 
											ShooterEncoder, 
											cam_outputter);
		*/
		setUpPID(Config.getSetting("cam_p", 0.04),
					Config.getSetting("cam_i", 0.005),
					Config.getSetting("cam_d", 0.03));
		m_setpoint = 0;
		disablePID();
		m_minimumInput = 0;
		m_maximumInput = 100 + 10;
		m_minimumOutput = -1;
		m_maximumOutput = 1;

		m_state = "homing";

		indexHasBeenSeen = false;
		m_enabled = false;
		controlLoop.startPeriodic(period);
	}

	public void setUpPID(double p, double i, double d) {
		m_P = p;
		m_I = i;
		m_D = d;
		resetPID();
	}
	
	public void resetPID() {
		m_totalError = m_prevError = 0;
	}

	public void setPIDSetpoint(double setpoint) {
		m_setpoint = setpoint;
	}

	public double PIDCompute(double input) {
		double m_error = m_setpoint - input;
		double m_result = 0;
		if (Math.abs(m_error) > (m_maximumInput - m_minimumInput) / 2) {
			if (m_error > 0) {
				m_error = m_error - m_maximumInput + m_minimumInput;
			} else {
				m_error = m_error + m_maximumInput - m_minimumInput;
			}
		}

		if (m_I != 0) {
			double potentialIGain = (m_totalError + m_error) * m_I;
			if (potentialIGain < m_maximumOutput) {
				m_totalError = potentialIGain >m_minimumOutput ? m_totalError + m_error : m_minimumOutput / m_I;
			} else {
				m_totalError = m_maximumOutput / m_I;
			}
		}
		double m_F = 0; //"What is this anyway?"
		m_result = m_P * m_error + m_I * m_totalError + m_D * (m_error - m_prevError) + m_setpoint * m_F;
		m_prevError = m_error;

		if (m_result > m_maximumOutput) {
			m_result = m_maximumOutput;
		} else if (m_result < m_minimumOutput) {
			m_result = m_minimumOutput;
		}

		return m_result;
	}

	public void enable() {
		synchronized(accessSemaphore) {
			m_enabled = true;
		}
	}

	public boolean isEnabled() {
		boolean enabled = false;
		synchronized(accessSemaphore) {
			enabled = m_enabled;
		}
		return m_enabled; //TODO this is probably supposed to just be enabled, not m_enabled
	}

	public void disable() {
		synchronized(accessSemaphore) {
			m_enabled = false;
		}
	}

	public void enablePID() {
		m_pidEnabled = true;
	}

	public void disablePID() {
		m_pidEnabled = false;
		resetPID();
	}

	public boolean indexTripped() {
		boolean index_tripped = false;
		
		synchronized(accessSemaphore) {
			index_tripped = indexHasBeenSeen;
		}
		return index_tripped;
	}

	@Override
	public void run() {
		innerProcess();
	}

	public void reset() {
		synchronized(accessSemaphore) {
			setUpPID(Config.getSetting("cam_p", 0.04),
				Config.getSetting("cam_i", 0.005),
				Config.getSetting("cam_d", 0.03));
			m_state = "homing";
			shooterEncoder.reset();
			disablePID();
		}
	}

	public double getPosition() {
		double position = 0;
		
		synchronized(accessSemaphore) {
			position = shooterEncoder.get();
		}
		return position;
	}

	private void innerProcess() {
		scopeToggleState = !scopeToggleState;
		scopeToggle.set(scopeToggleState);
		scopeCycle.set(true);
		
		boolean fire = false;
		boolean rearm = false;
		boolean eject = false;
		boolean enabled = false;

		double shooterEncoderDistance;
		double PIDSetpoint = 0;
		boolean setSetpoint = false;
		double PWMOutput = 0;
		boolean setPWMOutput = false;

		boolean resetEncoder = false;

		boolean PIDEnable = false;
		boolean controlPID = false;

		double homeSpeed = .1;

		boolean PIDOnTarget = false;

		double ejectPosition = 30;
		double pointOfNoReturn = 0;
		double readyToFirePosition = 0;
		double fireToPosition = 0;
		double camFirePositionTolerance = 0;

		String nextState;

		synchronized(accessSemaphore) {
			//The 2014 programmers wanted this out of the main loop at some point
				homeSpeed = Config.getSetting("cam_home_speed", .5);
				ejectPosition = Config.getSetting("CAM_EJECT_POSITION", 30);
				readyToFirePosition = CAM_READY_TO_FIRE_POSITION;
				pointOfNoReturn = CAM_POINT_OF_NO_RETURN;
				fireToPosition = CAM_FIRE_TO_POSITION;
				camFirePositionTolerance = CAM_FIRE_POSITION_TOLERANCE;
			enabled = m_enabled;
			fire = shouldFire;
			rearm = shouldRearm;
			eject = shouldEject;
			indexHasBeenSeen = indexSensor.get();

			shooterEncoderDistance = shooterEncoder.getDistance();
			PIDSetpoint = m_setpoint;

			PIDEnable = m_pidEnabled;

			PIDOnTarget = Math.abs(m_setpoint - shooterEncoderDistance) < camFirePositionTolerance;

			nextState = m_state;
		}

		if (enabled) { 
			synchronized(accessSemaphore) {
				switch (m_state) {
					case "rearming":
						if (PIDOnTarget) {
							nextState = "readyToFire";
						}
						if (shooterEncoderDistance >= pointOfNoReturn) {
							nextState = "firing";
						}
						break;
					case "readyToFire":
						if (fire) {
							nextState = "firing";
							PIDEnable = true;
							controlPID = true;
						}
						
						if (shooterEncoderDistance >= pointOfNoReturn) {
							nextState = "firing";
							System.out.println("OK, I guess we're firing now. Fine. Whatever."); //I thought this was funny
						} else if (eject) {
							setSetpoint = true;
							PIDSetpoint = ejectPosition;
							nextState = "ejecting";
						}
						break;
					case "firing":
						PIDSetpoint = fireToPosition;
						setSetpoint = true;
						
						if (PIDOnTarget && rearm) {
							PIDEnable = false;
							controlPID = true;
							nextState = "homing";
						}
						break;
					case "homing":
						// Switch to voltage mode, drive to the index.
						PWMOutput = homeSpeed;
						setPWMOutput = true;
						
						if (indexHasBeenSeen) {
							resetEncoder = true;
							PIDEnable = true;
							controlPID = true;
							PIDSetpoint = readyToFirePosition;
							setSetpoint = true;
							nextState = "rearming";
						}
						break;
					case "calibration":
						// "this isn't used anymore" I kept this just in case
						break;
					case "ejecting":
						if (!eject) {
							PIDSetpoint = readyToFirePosition;
							setSetpoint = true;
							nextState = "reaming";
						
						}
						break;
					case "testing":
						PIDEnable = false;
						controlPID = true;
						break;
					default:
						// ERROR
						break;
				}

				m_state = nextState;

				if (resetEncoder) {
					shooterEncoder.reset();
				}

				if (controlPID) {
					if (PIDEnable) {
						enablePID();
					} else {
						disablePID();
						resetPID();
					}
				}

				if (setSetpoint) {
					setPIDSetpoint(PIDSetpoint);
				}

				if (PIDEnable) {
					PWMOutput = PIDCompute(shooterEncoderDistance);
					setPWMOutput = true;
				}

				if (setPWMOutput) {
					cam_outputter.PIDWrite(PWMOutput);
				}

			}
		}

		scopeCycle.set(false);
	}

	public void process(boolean fire, boolean rearm, boolean eject) {
		System.out.println("Start of process()");
		synchronized(accessSemaphore) {
			shouldFire = fire;
			shouldRearm = rearm;
			shouldEject = eject;
		}
		System.out.println("End of process()");
	}

	public void setPosition(double pos) {}
}