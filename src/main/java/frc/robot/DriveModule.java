package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

public class DriveModule {

    private Talon master;
    private Talon slave;

    DriveModule(Talon master, Talon slave) {
        this.master = master;
        this.slave = slave;
    }

    public void setInverted(boolean isInverted) {
        master.setInverted(isInverted);
        slave.setInverted(isInverted);
    }

    public void set(double input) {
        master.set(input);
        slave.set(input);
    }

}
