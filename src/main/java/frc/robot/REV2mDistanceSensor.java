package frc.robot;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Unit;

/**
 * Wrapper for the {@link Rev2mDistanceSensor} object from RevLib. Most of the
 * javadocs for these methods are the same.
 */
public class REV2mDistanceSensor {
    private final MUX m_mux;
    private final MUX.Port m_port;
    private final Rev2mDistanceSensor m_sensor;

    /**
     * Creates a new {@link Rev2mDistanceSensor}.
     * 
     * @param mux  The {@link MUX} that the distance sensor is connected to.
     * @param port The port on the {@link MUX} that the distance sensor is connected
     *             to.
     */
    public REV2mDistanceSensor(MUX mux, MUX.Port port) {
        m_mux = mux;
        m_port = port;
        m_mux.switchToPort(m_port);
        m_sensor = new Rev2mDistanceSensor(Rev2mDistanceSensor.Port.kMXP);
        m_sensor.setAutomaticMode(true);
    }

    /**
     * Returns distance from sensor.
     * 
     * @return Distance in meters.
     */
    public double getDistance() {
        m_mux.switchToPort(m_port);
        return m_sensor.getRange(Unit.kMillimeters) / 1000;
    }

}
