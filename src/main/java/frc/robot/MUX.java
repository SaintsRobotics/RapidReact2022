package frc.robot;

import edu.wpi.first.wpilibj.I2C;

/**
 * Wrapper for multiplexer plugged into the I2C port on the roboRIO. There
 * should only be one instance
 */
public class MUX {
	private final I2C m_I2C;

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
	 * Creates a new {@link MUX}. There should only be one instance.
	 */
	public MUX() {
		m_I2C = new I2C(I2C.Port.kMXP, 0x70);
	}

	public void switchToPort(MUX.Port port) {
		m_I2C.writeBulk(port.value);
	}

	public I2C getI2CBus() {
		return m_I2C;
	}
}
