package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
	
	/**
	 * How many sensor units per rotation.
	 * Using Talon FX Integrated Sensor.
	 * @link https://github.com/CrossTheRoadElec/Phoenix-Documentation#what-are-the-units-of-my-sensor
	 */
	public final static int kSensorUnitsPerRotation = 2048;
	
	/**
	 * Motor neutral dead-band, set to the minimum 0.1%.
	 */
	public final static double kNeutralDeadband = 0.001;
	
	/**
	 * PID Gains may have to be adjusted based on the responsiveness of control loop
	 * 	                                    			  kP   kI    kD     kF             Iz    PeakOut */
	// FIXME: !!! find kF for the rotation
	public final static Gains kGains_MotProf = new Gains( 3.25, 0.01,  0.0, 0.07,  400,  0.5 ); /* measured 6800 velocity units at full motor output */
	
	public final static int kPrimaryPIDSlot = 0; // any slot [0,3]

	public final static boolean TUNING_MODE = true;

	public static final double ROTATION_MAX_ELEVATOR_EXTENSION_VELOCITY_DEGREES_PER_SECOND = 20;
  public static final double ROTATION_MAX_ELEVATOR_RETRACTION_VELOCITY_DEGREES_PER_SECOND = 20;
  public static final double ROTATION_ELEVATOR_ACCELERATION_DEGREES_PER_SECOND_PER_SECOND = 20;

  public static final double ROTATION_GEAR_RATIO = 20;
  public static final double ROTATION_DRUM_CIRCUMFERENCE = Units.inchesToMeters(2.0) * Math.PI;

  public static final int PIGEON_ID = 4;
  public static final int PIGEON_UNITS_PER_ROTATION = 8192;

  
}
