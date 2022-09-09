package frc.robot.logging;

import edu.wpi.first.wpilibj.Timer;

public class LoggableTimer extends Timer implements Loggable {
    String name;

    public LoggableTimer(String name) {
        this.name = name;
    }

    @Override
    public void logHeaders(Logger logger) {
        logger.addHeader(this.name);
    }

    @Override
    public void logData(Logger logger) {
        logger.addData(this.name, this.get());
    }

}
