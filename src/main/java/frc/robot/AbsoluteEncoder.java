package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.simulation.AnalogInputSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AbsoluteEncoder {
	private final AnalogInput m_analogIn;
	private final AnalogInputSim m_analogInputSim;

	private final double m_offset;

	/**
	 * Construct an absolute encoder, most likely a US Digital MA3 encoder.
	 * 
	 * @param channel analog in (aka sometime also refered to as AIO) port on the
	 *                robotRIO
	 * @param offset  Offset of the analog input in volts. Set this to the voltage
	 *                the analog input returns when the wheel is pointed forward.
	 */
	public AbsoluteEncoder(int channel, double offset) {
		m_analogIn = new AnalogInput(channel);
		m_analogInputSim = new AnalogInputSim(m_analogIn);

		m_offset = offset;
	}

	/**
	 * Returns the angle of the encoder. Zero points toward the front of the robot.
	 * The value increases as the wheel is turned counterclockwise.
	 * 
	 * @return The angle between -pi and pi.
	 */
	public double get() {
		// Takes the voltage of the analog input (0 to 5) and converts it to an angle.
		// This value needs to be negated because the analog input value increases as
		// the module is turned clockwise, which is the opposite of what we need.
		return MathUtil.angleModulus(-(m_analogIn.getVoltage() - m_offset) / 5 * 2 * Math.PI);
	}

	/**
	 * Figures out the value that the simulated absolute encoder should be at. Read
	 * comments in source code below.
	 * 
	 * @param turnVoltage Voltage that will go to the turning motor, range: [-1, 1].
	 */
	public void simulateVoltage(double turnVoltage) {
		// gear ratio between motor and wheel / encoder
		double gearRatio = 12.8;

		// maximum RPM for motor under 0 load
		double motorRPM = 5000;

		turnVoltage = MathUtil.clamp(turnVoltage, -1, 1);

		// started with motorRPM and converted to wheel rotations per tick
		// RPM * (min / sec) * (s / tick) * (wheel rotations / motor rotations)
		double wheelRotationsPerTick = motorRPM / 60 * Robot.kDefaultPeriod / gearRatio; // 0.143
		double wheelRotationsSinceLastTick = wheelRotationsPerTick * turnVoltage;
		double voltsSinceLastTick = 5 * wheelRotationsSinceLastTick;
		double polarity = -1;
		voltsSinceLastTick *= polarity;

		double outputVoltage = m_analogIn.getVoltage() + voltsSinceLastTick;

		SmartDashboard.putNumber("turning voltage" + m_analogIn.getChannel(), turnVoltage);

		outputVoltage = MathUtil.inputModulus(outputVoltage, 0, 5);
		// basically this "hijacks" the simulated absolute encoder to say that it's
		// reading the voltage that u give it, range: [0, 5]
		m_analogInputSim.setVoltage(outputVoltage);
	}
}
