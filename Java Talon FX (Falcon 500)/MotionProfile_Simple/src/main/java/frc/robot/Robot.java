/**
 * Phoenix Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and 
 * Phoenix Software API Libraries ONLY when in use with CTR Electronics hardware products
 * as well as the FRC roboRIO when in use in FRC Competition.
 * 
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * CROSS THE ROAD ELECTRONICS BE LIABLE FOR ANY INCIDENTAL, SPECIAL, 
 * INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE
 */

/**
 * Description:
 * This Java FRC robot application is meant to demonstrate an example using the Motion Profile control mode
 * in Talon FX.  The TalonFX class gives us the ability to buffer up trajectory points and execute them
 * as the roboRIO streams them into the Talon FX.
 * 
 * There are many valid ways to use this feature and this example does not sufficiently demonstrate every possible
 * method.  Motion Profile streaming can be as complex as the developer needs it to be for advanced applications,
 * or it can be used in a simple fashion for fire-and-forget actions that require precise timing.
 * 
 * This application is a TimedRobot project to demonstrate a minimal implementation not requiring the command 
 * framework, however these code excerpts could be moved into a command-based project.
 * 
 * The project also includes instrumentation.java which simply has debug printfs, and a MotionProfile.java which is generated
 * in @link https://docs.google.com/spreadsheets/d/1PgT10EeQiR92LNXEOEe3VGn737P7WDP4t0CQxQgC8k0/edit#gid=1813770630&vpid=A1
 * or find Motion Profile Generator.xlsx in the Project folder.
 * 
 * Controls:
 * Button 1: When held, streams and fires the MP.  When released, contorl is back to PercentOutput Mode.
 * Button 2: Prints MP status to the console when held.
 * Left Joystick Y-Axis: Throttle Talon FX forward and reverse when not running MP.
 * 
 * Gains for Motion Profile may need to be adjusted in Constants.java
 */
package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motion.*;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

import frc.robot.sim.PhysicsSim;

import static frc.robot.Constants.*;

public class Robot extends TimedRobot {

    private static final int LOOP_DT_MS = 10;

    private final TunableNumber rotationMotionProfileAcceleration = new TunableNumber(
            "ElevatorRotation/MPAcceleration",
            ROTATION_ELEVATOR_ACCELERATION_METERS_PER_SECOND_PER_SECOND);
    private final TunableNumber rotationMotionProfileExtensionCruiseVelocity = new TunableNumber(
            "ElevatorRotation/MPExtensionVelocity",
            ROTATION_MAX_ELEVATOR_EXTENSION_VELOCITY_METERS_PER_SECOND);
    private final TunableNumber rotationMotionProfileRetractionCruiseVelocity = new TunableNumber(
            "ElevatorRotation/MPRetractionVelocity",
            ROTATION_MAX_ELEVATOR_RETRACTION_VELOCITY_METERS_PER_SECOND);

    private final TunableNumber rotationSetpoint = new TunableNumber(
            "ElevatorRotation/Setpoint(deg)",
            45);


    /** very simple state machine to prevent calling set() while firing MP. */
    int _state = 0;

    /** a master talon, add followers if need be. */
    WPI_TalonFX _master = new WPI_TalonFX(19, "canbus1");

    /** gamepad for control */
    Joystick _joy = new Joystick(0);

    /** new class type in 2019 for holding MP buffer. */
    BufferedTrajectoryPointStream _bufferedStream = new BufferedTrajectoryPointStream();

    /* talon configs */
    TalonFXConfiguration _config = new TalonFXConfiguration(); // factory default settings
    
    /* quick and dirty plotter to smartdash */
    PlotThread _plotThread = new PlotThread(_master);

    public void simulationInit() {
        PhysicsSim.getInstance().addTalonFX(_master, 0.5, 6800);
    }
    public void simulationPeriodic() {
        PhysicsSim.getInstance().run();
    }

    public void robotInit() {
        /* fill our buffer object with the excel points */
        initBuffer(0, 20);

        /* _config the master specific settings */
        _config.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        _config.neutralDeadband = Constants.kNeutralDeadband; /* 0.1 % super small for best low-speed control */
        _config.slot0.kF = Constants.kGains_MotProf.kF;
        _config.slot0.kP = Constants.kGains_MotProf.kP;
        _config.slot0.kI = Constants.kGains_MotProf.kI;
        _config.slot0.kD = Constants.kGains_MotProf.kD;
        _config.slot0.integralZone = (int) Constants.kGains_MotProf.kIzone;
        _config.slot0.closedLoopPeakOutput = Constants.kGains_MotProf.kPeakOutput;
        // _config.slot0.allowableClosedloopError // left default for this example
        // _config.slot0.maxIntegralAccumulator; // left default for this example
        // _config.slot0.closedLoopPeriod; // left default for this example
        _master.configAllSettings(_config);

        /* pick the sensor phase and desired direction */
        _master.setInverted(TalonFXInvertType.CounterClockwise);
		/*
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
		 * 
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
		 */
        // _master.setSensorPhase(true);
    }

    public void robotPeriodic() {
        /* get joystick button and stick */
        boolean bPrintValues = _joy.getRawButton(2);
        boolean bFireMp = _joy.getRawButton(1);
        double axis = _joy.getRawAxis(1);

        /* if button is up, just drive the motor in PercentOutput */
        if (bFireMp == false) {
            _state = 0;
        }

        switch (_state) {
            /* drive master talon normally */
            case 0:
                _master.set(TalonFXControlMode.PercentOutput, axis);
                if (bFireMp == true) {
                    /* go to MP logic */
                    _state = 1;
                }
                break;

            /* fire the MP, and stop calling set() since that will cancel the MP */
            case 1:
                /* wait for 10 points to buffer in firmware, then transition to MP */
                _master.startMotionProfile(_bufferedStream, 10, TalonFXControlMode.MotionProfile.toControlMode());
                _state = 2;
                Instrum.printLine("MP started");
                break;

            /* wait for MP to finish */
            case 2:
                if (_master.isMotionProfileFinished()) {
                    Instrum.printLine("MP finished");
                    _state = 3;
                }
                break;

            /* MP is finished, nothing to do */
            case 3:
                break;
        }

        /* print MP values */
        Instrum.loop(bPrintValues, _master);
    }

    public void initBuffer(double extension, double rotation) {
        // public static BufferedTrajectoryPointStream generateTrajectory(double
        // theta_0, double
        // theta_f, Constraints constraints) { //TODO: consider generating motion
        // profiles for the
        // elevator and the arm together, that way the kG feedforward can be perfect
        /*
         * Trapezoidal profile for the angle of the arm as a function of time. The motor
         * profile is generated using
         * calculateMotorPosition() and calculateMotorVelocity such that the motion of
         * the arm matches this trajectory
         */
        Constraints constraints = new Constraints(
                mpsToFalconMotionMagicUnits(
                        rotationMotionProfileExtensionCruiseVelocity.get(),
                        ROTATION_DRUM_CIRCUMFERENCE,
                        ROTATION_GEAR_RATIO),
                mpsToFalconMotionMagicUnits(
                        rotationMotionProfileAcceleration.get(),
                        ROTATION_DRUM_CIRCUMFERENCE,
                        ROTATION_GEAR_RATIO));
        TrapezoidProfile profile = new TrapezoidProfile(
                constraints,
                new State(
                        0,
                        0),
                new State(_master.getSelectedSensorPosition(Constants.kPrimaryPIDSlot), 0));

        // based on
        // https://v5.docs.ctr-electronics.com/en/stable/ch16_ClosedLoop.html#motion-profiling-closed-loop
        BufferedTrajectoryPointStream stream = new BufferedTrajectoryPointStream();
        TrajectoryPoint point = new TrajectoryPoint();

        /* clear the buffer, in case it was used elsewhere */
        _bufferedStream.Clear();

        for (double t = 0; !profile.isFinished(t - LOOP_DT_MS / 1000.0); t += LOOP_DT_MS / 1000.0) {
            double extensionPosition = profile.calculate(t).position;
            double extensionVelocity = profile.calculate(t).velocity;

            point.timeDur = LOOP_DT_MS;
            point.position = extensionPosition;
            point.velocity = extensionVelocity;
            point.auxiliaryPos = 0;
            point.auxiliaryVel = 0;
            point.profileSlotSelect0 = Constants.kPrimaryPIDSlot; /* which set of gains would you like to use [0,3]? */
            point.profileSlotSelect1 = 0; /* auxiliary PID [0,1], leave zero */
            point.zeroPos = (t == 0); /* set this to true on the first point */
            point.isLastPoint = profile.isFinished(t); /* set this to true on the last point */
            point.arbFeedFwd = 0; // calculatekG(theta_arm); //apply kG as an arbitrary feedforward

            _bufferedStream.Write(point);
        }
    }

    private static double mpsToFalconMotionMagicUnits(
            double mps, double circumference, double gearRatio) {
        double pulleyRotationsPerSecond = mps / circumference;
        double motorRotationsPerSecond = pulleyRotationsPerSecond * gearRatio;
        double ticksPerSecond = motorRotationsPerSecond * 2048.0;
        return ticksPerSecond / 10.0; // per 100 ms
    }
}