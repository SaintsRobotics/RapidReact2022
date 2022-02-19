package frc.robot;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Wrapper for the ColorSensorV3 object from RevLib. Most of the javadocs for
 * these methods are copypasta from Rev's javadocs.
 */
public class REVColorSensorV3 {

    private MUX m_mux;
    private MUX.Port m_muxPort;
    private ColorSensorV3 m_sensor;

    /**
     * 
     * @param mux  The MUX object on the I2C bus. There shold only be one instance
     *             of this object existing in the code.
     * @param port The port on the MUX that the color sensor is plugged into. Should
     *             be unique for each color sensor.
     */
    public REVColorSensorV3(MUX mux, MUX.Port port) {
        m_mux = mux;
        m_muxPort = port;

        m_mux.switchToPort(m_muxPort);
        m_sensor = new ColorSensorV3(I2C.Port.kMXP);
    }

    /**
     * Configure the color sensor.
     * These settings are only needed for advanced users, the defaults will work
     * fine for most teams. Consult the APDS-9151 for more information on these
     * configuration settings
     * and how they will affect color sensor measurements.
     * 
     * @param res  Bit resolution output by the respective light sensor ADCs
     * @param rate Measurement rate of the light sensor
     * @param gain Gain factor applied to light sensor (color) outputs
     */
    public void configureColorSensor(ColorSensorV3.ColorSensorResolution res,
            ColorSensorV3.ColorSensorMeasurementRate rate, ColorSensorV3.GainFactor gain) {
        m_mux.switchToPort(m_muxPort);
        m_sensor.configureColorSensor(res, rate, gain);
    }

    /**
     * Configure the proximity sensor.
     * These settings are only needed for advanced users, the defaults will work
     * fine for most teams. Consult the APDS-9151 for more information on these
     * configuration settings and how they will affect proximity sensor
     * measurements.
     * 
     * @param res  Bit resolution output by the proximity sensor ADC.
     * @param rate Measurement rate of the proximity sensor.
     */
    public void configureProximitySensor​(ColorSensorV3.ProximitySensorResolution res,
            ColorSensorV3.ProximitySensorMeasurementRate rate) {
        m_mux.switchToPort(m_muxPort);
        m_sensor.configureProximitySensor(res, rate);
    }

    /**
     * Configure the IR LED used by the proximity sensor.
     * These settings are only needed for advanced users, the defaults will work
     * fine for most teams. Consult the APDS-9151 for more information on these
     * configuration settings and how they will affect proximity sensor
     * measurements.
     * 
     * @param freq   The pulse width modulation frequency for the proximity sensor
     *               LED
     * @param curr   The pulse current for the prximity sensor LED
     * @param pulses The number of pulses per measurement of the proximity sensor
     *               LED. I think the range is [0-255]
     */
    public void configureProximitySensorLED​(ColorSensorV3.LEDPulseFrequency freq, ColorSensorV3.LEDCurrent curr,
            int pulses) {
        m_mux.switchToPort(m_muxPort);
        m_sensor.configureProximitySensorLED(freq, curr, pulses);
    }

    /**
     * Get the most likely color. Works best when within 2 inches and perpendicular
     * to surface of interest.
     * 
     * @return Color enum of the most likely color, including unknown if the minimum
     *         threshold is not met
     */
    public Color getColor() {
        m_mux.switchToPort(m_muxPort);
        return m_sensor.getColor();
    }

    /**
     * Get the raw color value from the red ADC
     * 
     * @return Red ADC value
     */
    public int getRed() {
        m_mux.switchToPort(m_muxPort);
        return m_sensor.getRed();
    }

    /**
     * Get the raw color value from the green ADC
     * 
     * @return Green ADC value
     */
    public int getGreen() {
        m_mux.switchToPort(m_muxPort);
        return m_sensor.getGreen();
    }

    /**
     * Get the raw color value from the blue ADC
     * 
     * @return Blue ADC value
     */
    public int getBlue() {
        m_mux.switchToPort(m_muxPort);
        return m_sensor.getBlue();
    }

    /**
     * 
     * Get the raw proximity value from the sensor ADC (11 bit)
     * 
     * @return Proximity measurement value, ranging from 0 to 2047
     */
    public int getProximity() {
        m_mux.switchToPort(m_muxPort);
        return m_sensor.getProximity();
    }

    /**
     * Indicates if the device reset. Based on the power on status flag in the
     * status register. Per the datasheet:
     * Part went through a power-up event, either because the part was turned on or
     * because there was power supply voltage disturbance (default at first register
     * read).
     * This flag is self-clearing
     * 
     * @return True if the device was reset.
     */
    public boolean hasReset() {
        m_mux.switchToPort(m_muxPort);
        return m_sensor.hasReset();
    }

    /**
     * Indicates if the device can currently be communicated with.
     * 
     * @return True if yes, false if no
     */
    public boolean isConnected() {
        m_mux.switchToPort(m_muxPort);
        return m_sensor.isConnected();
    }

}
