package frc.robot;

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
}
