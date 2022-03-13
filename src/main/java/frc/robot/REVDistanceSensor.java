package frc.robot;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Unit;

import edu.wpi.first.wpilibj.I2C;

public class REVDistanceSensor {
    private final MUX m_mux;
    private final MUX.Port m_port;
    private final Rev2mDistanceSensor m_sensor;

    public REVDistanceSensor(MUX mux, MUX.Port port) {
        m_mux = mux;
        m_port = port;
        m_mux.switchToPort(m_port);
		m_sensor = new Rev2mDistanceSensor(Rev2mDistanceSensor.Port.kMXP);
    }

    /**
     * Returns distance from sensor.
     * @return distance in meters
     */
    public double getDistance() {
		m_mux.switchToPort(m_port);
        return m_sensor.getRange(Unit.kMillimeters) / 1000;
    }
  
}
