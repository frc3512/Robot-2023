package frc3512.lib.swervelib.encoders;

public abstract class SwerveAbsoluteEncoder {

  /** Reset the encoder to factory defaults. */
  public abstract void factoryDefault();

  /** Clear sticky faults on the encoder. */
  public abstract void clearStickyFaults();

  /**
   * Configure the absolute encoder to read from [0, 360) per second.
   *
   * @param inverted Whether the encoder is inverted.
   */
  public abstract void configure(boolean inverted);

  /**
   * Get the absolute position of the encoder.
   *
   * @return Absolute position in degrees from [0, 360).
   */
  public abstract double getAbsolutePosition();
}
