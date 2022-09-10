package frc.robot.logging;

import com.kauailabs.navx.frc.AHRS;

public class LoggableGyro extends AHRS implements Loggable {
    public LoggableGyro() {
        super();
    }

    @Override
    public void logHeaders(Logger logger) {
        logger.addHeader("AHRS/velocityX");
        logger.addHeader("AHRS/yaw");
        logger.addHeader("AHRS/accelerationX");
        logger.addHeader("AHRS/velocityY");
        logger.addHeader("AHRS/pitch");
        logger.addHeader("AHRS/accelerationY");
        logger.addHeader("AHRS/velocityZ");
        logger.addHeader("AHRS/roll");
        logger.addHeader("AHRS/accelerationZ");
    }

    @Override
    public void logData(Logger logger) {
        logger.addData("AHRS/velocityX", this.getVelocityX());
        logger.addData("AHRS/yaw", this.getYaw());
        logger.addData("AHRS/accelerationX", this.getWorldLinearAccelX());
        logger.addData("AHRS/velocityY", this.getVelocityY());
        logger.addData("AHRS/pitch", this.getPitch());
        logger.addData("AHRS/accelerationY", this.getWorldLinearAccelY());
        logger.addData("AHRS/velocityZ", this.getVelocityZ());
        logger.addData("AHRS/roll", this.getRoll());
        logger.addData("AHRS/accelerationZ", this.getWorldLinearAccelZ());
    }
}
