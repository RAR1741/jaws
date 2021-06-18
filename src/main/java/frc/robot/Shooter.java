package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;

public class Shooter {

    private final double CamFirePositionTolerance = 0; //TODO: Determine tolerance
    private final double CamPointOfNoReturn = 0; //TODO: Determine point of no return

    private Talon motorLeft, motorRight;
    private Encoder ShooterEncoder;
    private DigitalInput indexSensor, scopeToggle, scopeCycle;
    private boolean enabled;
    private PIDController PID;
    public State state;

    public enum State {
        Rearming, Firing, ReadyToFire, Calibration, Homing, Ejecting;
    }

    Shooter (Talon moterLeft, Talon motorRight, int encoderA, int encoderB, DigitalInput indexSensor, DigitalInput scopeToggle, DigitalInput scopeCycle){
        this.motorLeft = moterLeft;
        this.motorRight = motorRight;
        this.indexSensor = indexSensor;
        this.scopeCycle = scopeCycle;
        this.scopeToggle = scopeToggle;

        ShooterEncoder = new Encoder(encoderA, encoderB, false, EncodingType.k4X);

        PID = new PIDController(0.04, 0.005, 0.03);
        // PID = new PIDController(0.04, 0.005, 0.03, ShooterEncoder);


        state = State.Homing;
    }

    private void enable(){
        enabled = true;
    }
    
    private void disable(){
        enabled = false;
    }

    private boolean isEnabled(){
        return enabled;
    }

    private boolean getIndexTripped(){
        return indexSensor.get();
    }

    private double getPosition(){
        return motorLeft.get();
    }

    private void run(){
        State nextState;
        if(enabled){
            switch(state){
                case Rearming:
                    if(Math.abs(PID.getPositionError()) < CamFirePositionTolerance)
                        nextState = State.ReadyToFire;
                    if(ShooterEncoder.get() >= CamPointOfNoReturn)
                        nextState = State.Firing;
                    break;




                    
            }
        }
    }
}
