package frc.robot;

import edu.wpi.first.wpilibj.I2C;

/**
 * Wrapper for multiplexer plugged into the {@link I2C} port on the roboRIO.
 * There should only be one instance
 */
public class MUX {
	private final I2C m_I2C = new I2C(I2C.Port.kMXP, 0x70);

	public enum Port {
		kOne(new byte[] { (byte) 1 }), // single byte array to configure mux for port 1 0b00000001
		kTwo(new byte[] { (byte) 2 }), // single byte array to configure mux for port 2 0b00000010
		kThree(new byte[] { (byte) 4 }); // single byte array to configure mux for port 3 0b00000100

		public final byte[] value;

		Port(byte[] value) {
			this.value = value;
		}
	}

	/**
	 * Creates a new {@link MUX}. There should only be one instance.
	 */
	public MUX() {
	}

	public void switchToPort(MUX.Port port) {
		m_I2C.writeBulk(port.value);
	}
}
