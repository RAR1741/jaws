package frc.robot.logging;

import edu.wpi.first.wpilibj.XboxController;

public class LoggableController extends XboxController implements Loggable {
    public String name;

    public LoggableController(String name, int port) {
        super(port);
        this.name = name;
    }

    @Override
    /**
     * Sets up logging for the controller.
     */
    public void logHeaders(Logger logger) {
        logger.addHeader(name + "/LeftX");
        logger.addHeader(name + "/LeftY");
        logger.addHeader(name + "/RightX");
        logger.addHeader(name + "/RightY");
    }

    @Override
    /**
     * Logs the controller's values.
     */
    public void logData(Logger logger) {
        logger.addData(name + "/LeftX", this.getLeftX());
        logger.addData(name + "/LeftY", this.getLeftY());
        logger.addData(name + "/RightX", this.getRightX());
        logger.addData(name + "/RightY", this.getRightY());
    }

}
