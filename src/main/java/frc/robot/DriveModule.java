package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Talon;

public class DriveModule {

    private Talon master;
    private Talon slave;

    private DoubleSolenoid pto;

    DriveModule(Talon master, Talon slave, DoubleSolenoid pto) {
        this.master = master;
        this.slave = slave;

        this.pto = pto;
    }

    public void setInverted(boolean isInverted) {
        master.setInverted(isInverted);
        slave.setInverted(isInverted);
    }
    
    public void set(double input) {
        master.setSpeed(input);
        slave.setSpeed(input);
    }

    public void setPTO(boolean engaged) {
        pto.set(engaged ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
    }
}