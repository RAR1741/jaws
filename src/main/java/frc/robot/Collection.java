package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Talon;

public class Collection {

    private Talon spinWheel;
    private DoubleSolenoid collecSol;

    Collection (Talon spinWheel, DoubleSolenoid collecSol) {
        this.spinWheel = spinWheel;
        this.collecSol = collecSol;
    }

    /**
     * Sets if collecting balls.
     * 
     * @param collect true if collecting, false if not.
     */
    public void setCollecting(boolean collect) {
        spinWheel.setSpeed(collect ? 1 : 0);
    }

    /**
     * Sets if collector is extended.
     * 
     * @param extend true if extended, false if not.
     */
    public void setExtended(boolean extend) {
        collecSol.set(extend ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
    }
}