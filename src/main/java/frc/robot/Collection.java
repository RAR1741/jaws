package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

public class Collection {

    private final Talon spinWheel;
    private final DoubleSolenoid collecSol;
    public boolean engaged = false;

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
        spinWheel.set(collect ? -0.7 : 0);
    }

    /**
     * Sets if collector is extended.
     *
     * @param extend true if extended, false if not.
     */
    public void setExtended(boolean extend) {
        engaged = extend;
        collecSol.set(extend ? Value.kForward : Value.kReverse);
        //System.out.println(collecSol.get());
    }
}