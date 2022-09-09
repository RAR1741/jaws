package frc.robot.logging;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class LoggableCompressor extends Compressor implements Loggable {
    public LoggableCompressor(int module, PneumaticsModuleType moduleType) {
        super(module, moduleType);
    }

    public LoggableCompressor(PneumaticsModuleType moduleType) {
        super(moduleType);
    }

    @Override
    public void logHeaders(Logger logger) {
        logger.addHeader("PH/pressure");
    }

    @Override
    public void logData(Logger logger) {
        logger.addData("PH/pressure", this.getPressure());
    }
}
