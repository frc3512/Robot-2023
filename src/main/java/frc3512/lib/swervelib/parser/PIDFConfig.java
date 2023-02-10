package frc3512.lib.swervelib.parser;

import edu.wpi.first.math.controller.PIDController;

/** Hold the PIDF and Integral Zone values for a PID. */
public class PIDFConfig {

  /** PIDF Values and integral zone. */
  public double p, i, d, f, iz;

  /** Used when parsing PIDF values from JSON. */
  public PIDFConfig() {}

  /**
   * PIDF Config constructor to contain the values.
   *
   * @param p P gain.
   * @param i I gain.
   * @param d D gain.
   * @param f F gain.
   * @param iz Intergral zone.
   */
  public PIDFConfig(double p, double i, double d, double f, double iz) {
    this.p = p;
    this.i = i;
    this.d = d;
    this.f = f;
    this.iz = iz;
  }

  /**
   * PIDF Config constructor to contain the values.
   *
   * @param p P gain.
   * @param i I gain.
   * @param d D gain.
   * @param f F gain.
   */
  public PIDFConfig(double p, double i, double d, double f) {
    this(p, i, d, f, 0);
  }

  /**
   * PIDF Config constructor to contain the values.
   *
   * @param p P gain.
   * @param i I gain.
   * @param d D gain.
   */
  public PIDFConfig(double p, double i, double d) {
    this(p, i, d, 0, 0);
  }

  /**
   * PIDF Config constructor to contain the values.
   *
   * @param p P gain.
   * @param d D gain.
   */
  public PIDFConfig(double p, double d) {
    this(p, 0, d, 0, 0);
  }

  /**
   * Create a PIDController from the PID values.
   *
   * @return PIDController.
   */
  public PIDController createPIDController() {
    return new PIDController(p, i, d);
  }
}
