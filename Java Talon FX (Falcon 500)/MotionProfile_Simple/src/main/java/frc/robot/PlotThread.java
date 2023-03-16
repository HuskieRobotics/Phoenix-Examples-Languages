package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Quick and dirty threaded plotter class
 */
class PlotThread implements Runnable {

	private TalonFX _extensionTalon;
	private TalonFX _rotationTalon;
	private Thread _thread;

	public PlotThread(TalonFX rotationTalon, TalonFX extensionTalon) {
		_rotationTalon = rotationTalon;
		_extensionTalon = extensionTalon;

		_thread = new Thread(this);
		_thread.start();
	}

	public void run() {
		/**
		 * Speed up network tables, this is a test project so eat up all of the network
		 * possible for the purpose of this test.
		 */

		while (true) {
			/* Yield for a Ms or so - this is not meant to be accurate */
			try {
				Thread.sleep(1);
			} catch (Exception e) {
				/* Do Nothing */
			}

			/* Grab the latest signal update from our 1ms frame update */
			double sen_pos = _rotationTalon.getSelectedSensorPosition(0);
			double sen_vel = _rotationTalon.getSelectedSensorVelocity(0);
			double mtr_volts = _rotationTalon.getMotorOutputVoltage();
			double trgt_pos = _rotationTalon.getActiveTrajectoryPosition(0);
			double trgt_vel = _rotationTalon.getActiveTrajectoryVelocity(0);
			double trgt_arbF = _rotationTalon.getActiveTrajectoryArbFeedFwd(0);
			SmartDashboard.putNumber("rot_sen_pos", sen_pos);
			SmartDashboard.putNumber("rot_sen_vel", sen_vel);
			SmartDashboard.putNumber("rot_mtr_volts", mtr_volts);
			SmartDashboard.putNumber("rot_trgt_pos", trgt_pos);
			SmartDashboard.putNumber("rot_trgt_vel", trgt_vel);
			SmartDashboard.putNumber("rot_trgt_arbF", trgt_arbF);

			sen_pos = _extensionTalon.getSelectedSensorPosition(0);
			sen_vel = _extensionTalon.getSelectedSensorVelocity(0);
			mtr_volts = _extensionTalon.getMotorOutputVoltage();
			trgt_pos = _extensionTalon.getActiveTrajectoryPosition(0);
			trgt_vel = _extensionTalon.getActiveTrajectoryVelocity(0);
			trgt_arbF = _extensionTalon.getActiveTrajectoryArbFeedFwd(0);
			SmartDashboard.putNumber("ext_sen_pos", sen_pos);
			SmartDashboard.putNumber("ext_sen_vel", sen_vel);
			SmartDashboard.putNumber("ext_mtr_volts", mtr_volts);
			SmartDashboard.putNumber("ext_trgt_pos", trgt_pos);
			SmartDashboard.putNumber("ext_trgt_vel", trgt_vel);
			SmartDashboard.putNumber("ext_trgt_arbF", trgt_arbF);
		}
	}
}