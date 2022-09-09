package frc.robot.logging;

import edu.wpi.first.wpilibj.PowerDistribution;

public class LoggablePowerDistribution extends PowerDistribution implements Loggable {
    public LoggablePowerDistribution() {
        super();
    }

    public LoggablePowerDistribution(int module, ModuleType moduleType) {
        super(module, moduleType);
    }

    @Override
    public void logHeaders(Logger logger) {
        logger.addHeader("PDH/voltage");
    }

    @Override
    public void logData(Logger logger) {
        logger.addData("PDH/voltage", this.getVoltage());
    }

}
