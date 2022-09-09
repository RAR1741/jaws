package frc.robot.logging;

public class LoggableFirstOrderFilter implements Loggable {
    private double[] previousValues;
    private int index = 0;
    private int range;
    private String name;

    public LoggableFirstOrderFilter(int range, String name) {
        this.range = range;
        this.name = name;
        this.previousValues = new double[range];
    }

    public void update(double value) {
        this.previousValues[this.index] = value;
        this.index = (this.index + 1) % this.range;
    }

    public double get() {
        double total = 0.0;

        for (int i = 0; i < range; i++) {
            total += this.previousValues[i];
        }

        return total / this.range;
    }

    @Override
    public void logHeaders(Logger logger) {
        logger.addHeader(this.name + "/Average");
    }

    @Override
    public void logData(Logger logger) {
        logger.addData(this.name + "/Average", get());
    }
}
