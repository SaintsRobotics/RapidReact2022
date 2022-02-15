package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.simulation.AnalogInputSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AbsoluteEncoder {
  private final AnalogInput m_analogIn;
  private final AnalogInputSim m_analogInputSim;

  private final boolean m_reversed;
  private final double m_offset;

  /**
   * Construct an absolute encoder, most likely a US Digital MA3 encoder.
   * 
   * @param channel  analog in (aka sometime also refered to as AIO) port on the
   *                 robotRIO
   * @param reversed set this to <i>TRUE</i> if physically turning the swerve
   *                 wheel <i>CLOCKWISE</i> (looking down from the top of the bot)
   *                 <i>INCREASES</i> the raw voltage that the encoder provides.
   * @param offset   swerve offset in <i>RADIANS</i>. This value is
   *                 <i>SUBTRACTED</i> from the encoder output.
   */
  public AbsoluteEncoder(int channel, boolean reversed, double offset) {
    m_analogIn = new AnalogInput(channel);
    m_analogInputSim = new AnalogInputSim(m_analogIn);

    m_reversed = reversed;
    m_offset = offset;
  }

  /**
   * Returns the angle as a {@link Rotation2d}. Zero points toward the front of
   * the robot.
   * <i>The value INCREASES as the wheel is turned COUNTER-CLOCKWISE</i>
   * 
   * @return The angle as a {@link Rotation2d}.
   */
  public Rotation2d get() {
    double angle = (m_analogIn.getVoltage() / 5 * 2 * Math.PI) - m_offset;

    return m_reversed ? new Rotation2d(5 - angle) : new Rotation2d(angle);
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
    double polarity = m_reversed ? -1 : 1;
    voltsSinceLastTick *= polarity;

    double outputVoltage = m_analogIn.getVoltage() + voltsSinceLastTick;

    SmartDashboard.putNumber("turning voltage" + m_analogIn.getChannel(), turnVoltage);

    outputVoltage = MathUtil.inputModulus(outputVoltage, 0, 5);
    // basically this "hijacks" the simulated absolute encoder to say that it's
    // reading the voltage that u give it, range: [0, 5]
    m_analogInputSim.setVoltage(outputVoltage);
  }
}
