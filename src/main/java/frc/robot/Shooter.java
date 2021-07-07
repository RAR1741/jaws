package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.Talon;

public class Shooter {

    private final double FirePositionTolerance = 0; //TODO: Determine tolerance
    private final double PointOfNoReturn = 0; //TODO: Determine point of no return
    private final double ReadyToFirePosition = 0; //TODO: Determine ready to fire position
    private final double EjectPosition = 0; //TODO: Determine eject position
    private final double FireToPosition = 0; //TODO: Determing fire to position

    private Talon motorLeft, motorRight;
    private Encoder ShooterEncoder;
    private DigitalInput indexSensor;
    private boolean enabled;
    private PIDController PID;
    public State state;

    private double startPosition;

    public enum State {
        Rearming, Firing, ReadyToFire, Homing, Ejecting;
    }

    Shooter (Talon moterLeft, Talon motorRight, DigitalInput indexSensor){
        this.motorLeft = moterLeft;
        this.motorRight = motorRight;
        this.indexSensor = indexSensor;

        PID = new PIDController(0.04, 0.005, 0.03);

        startPosition = motorLeft.getPosition();
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
        return motorLeft.getPosition() - startPosition;
    }

    private void setPosition(double setpoint){
        PID.setSetpoint(setpoint);
    }

    private void resetEncoders(){
        startPosition = motorLeft.getPosition();
    }

    private void run(boolean fire, boolean rearm, boolean eject){
        State nextState = state;
        if(enabled){
            switch(state){
                case Rearming:
                    if(Math.abs(PID.getPositionError()) < FirePositionTolerance)
                        nextState = State.ReadyToFire;
                    if(ShooterEncoder.get() >= PointOfNoReturn)
                        nextState = State.Firing;
                    break;

                case ReadyToFire:
                    if(fire)
                        nextState = State.Firing;
                    if(getPosition() >= PointOfNoReturn)
                        nextState = State.Firing;
                    else if (eject){
                        setPosition(EjectPosition);
                        nextState = State.Ejecting;
                    }
                    break;

                case Firing:
                    setPosition(FireToPosition);
                    if(Math.abs(PID.getPositionError()) < FirePositionTolerance && rearm){
                        nextState = State.Homing;
                    }
                    break;

                case Homing:
                    if(getIndexTripped()){
                        resetEncoders();
                        setPosition(ReadyToFirePosition);
                        nextState = State.Rearming;
                    }
                    break;

                case Ejecting:
                    if(!eject){
                        setPosition(ReadyToFirePosition);
                        nextState = State.Rearming;
                    }
                    break;

                default:
                    break;
            }
            motorLeft.set(PID.calculate(motorLeft.getPosition()));
            motorRight.set(PID.calculate(motorRight.getPosition()));
            state = nextState;
        }
    }
}
