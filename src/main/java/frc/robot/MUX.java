package frc.robot;

import edu.wpi.first.wpilibj.I2C;

/**
 * Wrapper for multiplexer plugged into the I2C port on the roboRIO. There
 * should only be one instance
 */
public class MUX {
    private I2C m_i2cBus;

    public enum Port {
        ONE(new byte[] { (byte) 1 }), // single byte array to configure mux for port 1 0b00000001
        TWO(new byte[] { (byte) 2 }), // single byte array to configure mux for port 2 0b00000010
        THREE(new byte[] { (byte) 4 }); // single byte array to configure mux for port 3 0b00000100

        public final byte[] value;

        Port(byte[] value) {
            this.value = value;
        }
    }

    /**
     * Constructor.
     * 
     * The I2C port and address is hardcoded in the constructor.
     */
    public MUX() {
        m_i2cBus = new I2C(I2C.Port.kMXP, 0x70);
    }

    public void switchToPort(MUX.Port port) {
        m_i2cBus.writeBulk(port.value);
    }

    public I2C getI2CBus() {
        return m_i2cBus;
    }
}
