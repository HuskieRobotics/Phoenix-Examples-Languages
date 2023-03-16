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
	public final static Gains kGains_MotProf = new Gains( 0.3, 0.0,  0.0, 0.05,  400,  1.00 ); /* measured 6800 velocity units at full motor output */
	
	public final static int kPrimaryPIDSlot = 0; // any slot [0,3]

	public static final boolean TUNING_MODE = true;

	public static final double EXTENSION_PULLEY_CIRCUMFERENCE = Units.inchesToMeters(1.128) * Math.PI;
  public static final double EXTENSION_GEAR_RATIO = 3.0;

	public static final double EXTENSION_MAX_ELEVATOR_EXTENSION_VELOCITY_METERS_PER_SECOND = 0.5;
  public static final double EXTENSION_MAX_ELEVATOR_RETRACTION_VELOCITY_METERS_PER_SECOND = 0.5;
  public static final double EXTENSION_ELEVATOR_ACCELERATION_METERS_PER_SECOND_PER_SECOND = 1.0;
  
}
