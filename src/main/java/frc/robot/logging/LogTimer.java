package frc.robot.logging;

import java.io.IOException;
import java.util.TimerTask;

public class LogTimer extends TimerTask {
	private Logger logger;

	public LogTimer(Logger log) {
		logger = log;
	}

	@Override
	public void run() {
		logger.collectData();
        try {
			logger.writeData();
		} catch (IOException io) {
			io.printStackTrace();
		}
	}
}
