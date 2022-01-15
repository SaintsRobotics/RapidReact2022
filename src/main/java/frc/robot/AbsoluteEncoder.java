package frc.robot;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;

public class AbsoluteEncoder {
  private AnalogInput m_analogIn;
  private AnalogEncoder m_analogEncoder;

  private double m_offset;

  public AbsoluteEncoder(int channel, double offset) {
    m_analogIn = new AnalogInput(channel);
    m_analogEncoder = new AnalogEncoder(m_analogIn);

    m_offset = offset;
  }

  /**
   * Returns the angle in radians.
   * 
   * @todo check if this is equal to the 0
   * @return the angle in radians
   */
  public double getAngleRadians() {
    return ((m_analogEncoder.get() % 5 / 5) * 2 * Math.PI) - m_offset;
  }

  /**
   * Returns the angle in degrees.
   * 
   * @todo check if this is equal to the 0
   * @return the angle in degrees
   */
  public double getAngleDegrees() {
    return Math.toDegrees(getAngleRadians());
  }
}
