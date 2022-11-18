package frc.robot;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.lang.Math;
import java.util.Calendar;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;

public class CamShooter implements Runnable {
	private final double CAM_READY_TO_FIRE_POSITION = Config.getSetting("cam_ready_to_fire_position", 35);
	private final double CAM_FIRE_TO_POSITION = Config.getSetting("cam_fire_to_position", 45);
	private final double CAM_FIRE_POSITION_TOLERANCE = Config.getSetting("cam_fire_position_tolerance", 3);
	private final double CAM_POINT_OF_NO_RETURN = Config.getSetting("cam_point_of_no_return", 58);
	//private final double CAM_SETPOINT_ERROR_LIMIT = Config.getSetting("cam_setpoint_error_limit", 25);

	boolean scopeToggleState = false;
	double p, i, d;
	double totalError, prevError;
	double setpoint;
	boolean pidEnabled = false;
	final double minimumInput, maximumInput;
	final double minimumOutput, maximumOutput;
	int state;
	boolean indexHasBeenSeen;
	boolean enabled;
	boolean shouldFire = false;
	boolean shouldRearm = false;
	boolean shouldEject = false;

	private BufferedWriter logOutput;
	private Timer timer;
	private StringBuilder sb;

	final Notifier controlLoop;
	final Talon shooterMotorLeft, shooterMotorRight;
	final CamMotors camMotors;
	final Encoder shooterEncoder;
	final DigitalInput indexSensor;
	final DigitalOutput scopeToggle, scopeCycle;

	public static class CamMotors {
		final Talon LEFT;
		final Talon RIGHT;

		public CamMotors(Talon left, Talon right) {
			this.LEFT = left;
			this.RIGHT = right;

			LEFT.setInverted(true);
		}

		public void PIDWrite(double out) {
			LEFT.set(out);
			RIGHT.set(out);
		}
	}

	public CamShooter(int motorLeft, int motorRight, int encoderA, int encoderB, int indexInput, int scopeTogglePort, int scopeCyclePort, double period) {
		controlLoop = new Notifier(this);
		final double LINES_PER_REV = 360;
		final double GEAR_REDUCTION_RATIO = 2250.0 / 49.0;
		shooterMotorLeft = new Talon(motorLeft);
		shooterMotorRight = new Talon(motorRight);
		camMotors = new CamMotors(shooterMotorLeft, shooterMotorRight);
		shooterEncoder = new Encoder(encoderA, encoderB, false, EncodingType.k4X);
		indexSensor = new DigitalInput(indexInput);
		scopeToggle = new DigitalOutput(scopeTogglePort);
		scopeCycle = new DigitalOutput(scopeCyclePort);

		shooterEncoder.setDistancePerPulse(100.0 / (GEAR_REDUCTION_RATIO * LINES_PER_REV));
		shooterEncoder.reset();

		setUpPID(0.08, 0.00005, 0.8);
		//setUpPID(Config.getSetting("cam_p", 0.04),
		//		Config.getSetting("cam_i", 0.005),
		//		Config.getSetting("cam_d", 0.03));
		setpoint = 0;
		disablePID();
		minimumInput = 0;
		maximumInput = 100 + 10;
		minimumOutput = -1;
		maximumOutput = 1;

		state = 3;

		indexHasBeenSeen = false;
		enabled = false;

		timer = new Timer();
		controlLoop.startPeriodic(period);
		sb = new StringBuilder(100);
	}

	public void setUpPID(double p, double i, double d) {
		this.p = p;
		this.i = i;
		this.d = d;
		resetPID();
	}

	public void resetPID() {
		totalError = prevError = 0;
	}

	public void setPIDSetpoint(double setpoint) {
		this.setpoint = setpoint;
	}

	public double PIDCompute(double input) {
		double m_error = setpoint - input;
		double m_result;
		if (Math.abs(m_error) > (maximumInput - minimumInput) / 2) {
			if (m_error > 0) {
				m_error = m_error - maximumInput + minimumInput;
			} else {
				m_error = m_error + maximumInput - minimumInput;
			}
		}

		if (i != 0) {
			double potentialIGain = (totalError + m_error) * i;
			if (potentialIGain < maximumOutput) {
				totalError = potentialIGain > minimumOutput ? totalError + m_error : minimumOutput / i;
			} else {
				totalError = maximumOutput / i;
			}
		}
		double m_F = 0; //"What is this anyway?"
		m_result = p * m_error + i * totalError + d * (m_error - prevError) + setpoint * m_F;
		prevError = m_error;

		if (m_result > maximumOutput) {
			m_result = maximumOutput;
		} else if (m_result < minimumOutput) {
			m_result = minimumOutput;
		}

		return m_result;
	}

	public void enable() {
		synchronized(this) {
			enabled = true;
			try {
				timer.reset();
				logOutput = new BufferedWriter(new FileWriter("/home/lvuser/cam-shooter.csv"));
				logOutput.write("time,state,encoder,setpoint");
				logOutput.newLine();
			} catch (IOException e) {
				// Don't bother recovering this is test code
				throw new RuntimeException("Problem opening log file in camshooter", e);
			}
		}
	}

	public boolean isEnabled() {
		boolean enabled;
		synchronized(this) {
			enabled = this.enabled;
		}
		return enabled;
	}

	public void disable() {
		synchronized(this) {
			enabled = false;
			try {
				logOutput.close();
			} catch (IOException e) {
				throw new RuntimeException("Problem with closing log file in camshooter", e);
			}
		}
	}

	public void enablePID() {
		pidEnabled = true;
	}

	public void disablePID() {
		pidEnabled = false;
		resetPID();
	}

	public boolean indexTripped() {
		boolean index_tripped;

		synchronized(this) {
			index_tripped = indexHasBeenSeen;
		}
		return index_tripped;
	}

	@Override
	public void run() {
		innerProcess();
	}

	public void reset() {
		synchronized(this) {
			setUpPID(Config.getSetting("cam_p", 0.04),
					Config.getSetting("cam_i", 0.005),
					Config.getSetting("cam_d", 0.03));
			state = 3;
			shooterEncoder.reset();
			disablePID();
		}
	}

	public double getPosition() {
		double position;

		synchronized(this) {
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

		/*System.out.println("setSetpoint: " + setSetpoint);
		System.out.println("setPWMOutput: " + setPWMOutput);
		System.out.println("PIDEnable: " + PIDEnable);
		System.out.println("controlPID: " + controlPID);*/

		double homeSpeed = 0.1;

		boolean PIDOnTarget = false;

		double ejectPosition;
		double pointOfNoReturn;
		double readyToFirePosition;
		double fireToPosition;
		double camFirePositionTolerance;

		int nextState;

		synchronized(this) {
			//The 2014 programmers wanted this out of the main loop at some point
				homeSpeed = Config.getSetting("cam_home_speed", .5);
				ejectPosition = Config.getSetting("CAM_EJECT_POSITION", 30);
				readyToFirePosition = CAM_READY_TO_FIRE_POSITION;
				pointOfNoReturn = CAM_POINT_OF_NO_RETURN;
				fireToPosition = CAM_FIRE_TO_POSITION;
				camFirePositionTolerance = CAM_FIRE_POSITION_TOLERANCE;
			enabled = this.enabled;
			fire = shouldFire;
			rearm = shouldRearm;
			eject = shouldEject;
			indexHasBeenSeen = indexSensor.get();

			shooterEncoderDistance = shooterEncoder.getDistance();
			PIDSetpoint = setpoint;

			PIDEnable = pidEnabled;

			PIDOnTarget = Math.abs(setpoint - shooterEncoderDistance) < camFirePositionTolerance;

			nextState = state;
		}

 
		if (enabled) {
			synchronized(this) {
				//camLogger.info(state + ", " + shooterEncoderDistance);

				//camLogger.info("Hello!");
				System.out.println("p: " + p + ", i: " + i + ", d: " + d);
				switch (state) {
					case 0: //rearming
						if (PIDOnTarget) {
							nextState = 1;
						}
						if (shooterEncoderDistance >= pointOfNoReturn) {
							nextState = 2;
						}
						break;
					case 1: //readyToFire
						if (fire) {
							nextState = 2;
							PIDEnable = true;
							controlPID = true;
						}
						if (shooterEncoderDistance >= pointOfNoReturn) {
							nextState = 2;
							System.out.println("OK, I guess we're firing now. Fine. Whatever."); //I thought this was funny
						} else if (eject) {
							setSetpoint = true;
							PIDSetpoint = ejectPosition;
							nextState = 5;
						}
						break;
					case 2: //firing
						PIDSetpoint = fireToPosition;
						setSetpoint = true;

						if (PIDOnTarget && rearm) {
							PIDEnable = false;
							controlPID = true;
							nextState = 3;
						}
						break;
					case 3: //homing
						// Switch to voltage mode, drive to the index.
						PWMOutput = homeSpeed;
						setPWMOutput = true;

						if (indexHasBeenSeen) {
							resetEncoder = true;
							PIDEnable = true;
							controlPID = true;
							PIDSetpoint = readyToFirePosition;
							setSetpoint = true;
							nextState = 0;
						}
						break;
					case 4: //calibration
						// "this isn't used anymore" I kept this just in case
						break;
					case 5: //ejecting
						if (!eject) {
							PIDSetpoint = readyToFirePosition;
							setSetpoint = true;
							nextState = 0;
						}
						break;
					case 6: //testing
						PIDEnable = false;
						controlPID = true;
						break;
					default:
						// ERROR
						break;
				}

				state = nextState;

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
					camMotors.PIDWrite(PWMOutput);
				}
				
				try {
					sb.setLength(0);
					// time,state,encoder,setpoint
					sb.append(timer.get());
					sb.append(',');
					sb.append(state);
					sb.append(',');
					sb.append(shooterEncoderDistance);
					sb.append(',');
					sb.append(PIDSetpoint);
					logOutput.write(sb.toString());
					logOutput.newLine();
				} catch (IOException e) {
					throw new RuntimeException("Exception thrown during inner loop in CamShooter", e);
				}
			}
		}

		scopeCycle.set(false);
	}

	public void process(boolean fire, boolean rearm, boolean eject) {
		synchronized(this) {
			shouldFire = fire;
			shouldRearm = rearm;
			shouldEject = eject;
		}
	}

	public BufferedWriter openLogger() {
        Calendar calendar = Calendar.getInstance();
        String dir = "/home/lvuser/logs";
        new File(dir).mkdirs();
        if (new File("/media/sda").exists()) {
            dir = "/media/sda";
        }
        String name = dir + "/log-" + calendar.get(Calendar.YEAR) + "-"
                + calendar.get(Calendar.MONTH) + "-" + calendar.get(Calendar.DAY_OF_MONTH) + "_"
                + calendar.get(Calendar.HOUR_OF_DAY) + "-" + calendar.get(Calendar.MINUTE) + "-"
                + calendar.get(Calendar.SECOND) + ".csv";

        System.out.printf("Logging to file: '%s'%n", new File(name).getAbsolutePath());
        return this.openLogger(name);
    }

    /**
     * Opens a file to log to.
     *
     * @param filepath Path of the file to open
     * @return Whether opening the file succeeded
     */
    public BufferedWriter openLogger(String filepath) {
		BufferedWriter log;
        try {
            log = new BufferedWriter(new FileWriter(filepath));
			log.write("Encoder, Setpoint, Motor, Sensor, State,\n");
        } catch (IOException e) {
            return (BufferedWriter) null;
        }
        return log;
    }

	public void debug(BufferedWriter out) {
		double distance, setpoint, left;
		boolean index_tripped;
		int state;
		synchronized(this) {
			setpoint = this.setpoint;
			distance = shooterEncoder.getDistance();
			index_tripped = indexHasBeenSeen;
			left = shooterMotorLeft.get();
			state = this.state;
		}
		try {
			out.write(String.valueOf(distance) + ",");
			out.write(String.valueOf(setpoint) + ",");
			out.write(String.valueOf(left) + ",");
			out.write((index_tripped ? "1" : "0") + ",");
			out.write(String.valueOf(state) + ",\n");
			out.flush();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}