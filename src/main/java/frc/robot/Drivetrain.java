package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Drivetrain {

    private static final double DEADBAND_LIMIT = 0.02;

    private final DriveModule left;
    private final DriveModule right;
    private final DoubleSolenoid pto;
    public boolean engaged = false;

    Drivetrain(DriveModule left, DriveModule right, DoubleSolenoid pto) {
        this.left = left;
        this.right = right;
        right.setInverted(true);

        this.pto = pto;
    }

    /**
     * Drives the left side of the robot either forward or backward.
     *
     * @param speed the speed at which to drive (ranges from -1.0 to +1.0)
     */
    public void driveLeft(double speed) {
        double sp = deadband(speed);
        left.set(sp);

        Logger.addEntry("Drivetrain/Left/Speed", sp);
    }

    /**
     * Drives the right side of the robot either forward or backward.
     *
     * @param speed the speed at which to drive (ranges from -1.0 to +1.0)
     */
    public void driveRight(double speed) {
        double sp = deadband(speed);
        right.set(sp);

        Logger.addEntry("Drivetrain/Right/Speed", sp);
    }

    /**
     * Normalizes the input to 0.0 if it is below the value set by
     * {@link #DEADBAND_LIMIT} This is primarily used for reducing the strain on
     * motors.
     *
     * @param in the input to check
     * @return 0.0 if {@code in} is less than abs(DEADBAND_LIMIT) else {@code in}
     */
    public double deadband(double in) {
        return Math.abs(in) > DEADBAND_LIMIT ? in : 0.0;
    }

    public void setPTO(boolean engaged) {
        this.engaged = engaged;
        pto.set(engaged ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
    }

    /**
     * Drives the robot with an arcade style drive
     *
     * @param turnInput  The speed to drive the drivetrain in the x direction
     *                   (ranges
     *                   from -1.0 to +1.0)
     * @param speedInput The speed to drive the drivetrain in the y direction
     *                   (ranges
     *                   from -1.0 to +1.0)
     */
    public void arcadeDrive(double turnInput, double speedInput) {
        this.driveLeft(speedInput - turnInput);
        this.driveRight(speedInput + turnInput);
    }

    /**
     * Drives the robot with a tank style drive
     *
     * @param leftDrive  The speed to drive the left drivetrain (ranges from -1.0 to
     *                   +1.0)
     * @param rightDrive The speed to drive the right drivetrain (ranges from -1.0
     *                   to
     *                   +1.0)
     */
    public void tankDrive(double leftDrive, double rightDrive) {
        this.driveLeft(leftDrive);
        this.driveRight(rightDrive);
    }
}
