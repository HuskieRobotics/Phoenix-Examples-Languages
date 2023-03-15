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

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2Configuration;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.ctre.phoenix.motion.*;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

import frc.robot.sim.PhysicsSim;

import static frc.robot.Constants.*;

public class Robot extends TimedRobot {

    private static final int LOOP_DT_MS = 10;

    private final TunableNumber rotationMotionProfileAcceleration = new TunableNumber(
            "ElevatorRotation/MPAcceleration(deg/sec/sec)",
            ROTATION_ELEVATOR_ACCELERATION_DEGREES_PER_SECOND_PER_SECOND);
    private final TunableNumber rotationMotionProfileExtensionCruiseVelocity = new TunableNumber(
            "ElevatorRotation/MPExtensionVelocity(deg/s)",
            ROTATION_MAX_ELEVATOR_EXTENSION_VELOCITY_DEGREES_PER_SECOND);
    private final TunableNumber rotationMotionProfileRetractionCruiseVelocity = new TunableNumber(
            "ElevatorRotation/MPRetractionVelocity(deg/s)",
            ROTATION_MAX_ELEVATOR_RETRACTION_VELOCITY_DEGREES_PER_SECOND);

            private final TunableNumber extensionMotionProfileAcceleration = new TunableNumber(
            "ElevatorExtension/MPAcceleration(m/s/s)",
            EXTENSION_ELEVATOR_ACCELERATION_METERS_PER_SECOND_PER_SECOND);
    private final TunableNumber extensionMotionProfileExtensionCruiseVelocity = new TunableNumber(
            "ElevatorExtension/MPExtensionVelocity(m/s)",
            EXTENSION_MAX_ELEVATOR_EXTENSION_VELOCITY_METERS_PER_SECOND);
    private final TunableNumber extensionMotionProfileRetractionCruiseVelocity = new TunableNumber(
            "ElevatorExtension/MPRetractionVelocity(m/s)",
            EXTENSION_MAX_ELEVATOR_RETRACTION_VELOCITY_METERS_PER_SECOND);

    

    private final TunableNumber rotationSetpoint = new TunableNumber(
            "ElevatorRotation/Setpoint(deg)",
            45);

            private final TunableNumber extensionSetpoint = new TunableNumber(
            "ElevatorExtension/Setpoint(in)",
            44);

            // positive values rotation finishes after the extension
            // negative values: extension finishes after the rotation
    private final TunableNumber extensionRotationProfileDelta = new TunableNumber(
                "Elevator/ProfileDelta(s)",
                2.0);


    /** very simple state machine to prevent calling set() while firing MP. */
    int _state = 0;

    int loopCount = 0;
    double durationDifference = 0;

    /** a master talon, add followers if need be. */
    WPI_TalonFX rotationTalon = new WPI_TalonFX(19, "canbus1");
    WPI_TalonFX extensionTalon = new WPI_TalonFX(5, "canbus1");

    Pigeon2 pigeon;

    /** gamepad for control */
    Joystick _joy = new Joystick(0);

    /** new class type in 2019 for holding MP buffer. */
    BufferedTrajectoryPointStream rotationBufferedStream = new BufferedTrajectoryPointStream();
    BufferedTrajectoryPointStream extensionBufferedStream = new BufferedTrajectoryPointStream();

    /* talon configs */
    TalonFXConfiguration rotationConfig = new TalonFXConfiguration(); // factory default settings
    TalonFXConfiguration extensionConfig = new TalonFXConfiguration(); // factory default settings
    
    /* quick and dirty plotter to smartdash */
    PlotThread _plotThread = new PlotThread(rotationTalon, extensionTalon);

    public void simulationInit() {
        PhysicsSim.getInstance().addTalonFX(rotationTalon, 0.5, 6800);
    }
    public void simulationPeriodic() {
        PhysicsSim.getInstance().run();
    }

    public void robotInit() {

        /* create and configure the Pigeon */
    this.pigeon = new Pigeon2(PIGEON_ID, "canbus1");
    Pigeon2Configuration config = new Pigeon2Configuration();
    // set mount pose as rolled 90 degrees clockwise
    config.MountPoseYaw = 0;
    config.MountPoseRoll = -90.0;
    this.pigeon.configAllSettings(config);
    this.pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, 9);

        /* fill our buffer object with the excel points */
        //initBuffer(0, 20);

        /* rotationConfig the master specific settings */
        //rotationConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        rotationConfig.neutralDeadband = Constants.kNeutralDeadband; /* 0.1 % super small for best low-speed control */
        rotationConfig.slot0.kF = Constants.kRotationGains_MotProf.kF;
        rotationConfig.slot0.kP = Constants.kRotationGains_MotProf.kP;
        rotationConfig.slot0.kI = Constants.kRotationGains_MotProf.kI;
        rotationConfig.slot0.kD = Constants.kRotationGains_MotProf.kD;
        rotationConfig.slot0.integralZone = (int) Constants.kRotationGains_MotProf.kIzone;
        rotationConfig.slot0.closedLoopPeakOutput = Constants.kRotationGains_MotProf.kPeakOutput;

        rotationConfig.remoteFilter0.remoteSensorDeviceID = PIGEON_ID;
        rotationConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.Pigeon_Pitch;


        // rotationConfig.slot0.allowableClosedloopError // left default for this example
        // rotationConfig.slot0.maxIntegralAccumulator; // left default for this example
        // rotationConfig.slot0.closedLoopPeriod; // left default for this example
        rotationTalon.configAllSettings(rotationConfig);

        rotationTalon.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
        rotationTalon.setSensorPhase(true);

        /* pick the sensor phase and desired direction */
        rotationTalon.setInverted(TalonFXInvertType.CounterClockwise);

        /* _config the master specific settings */
        extensionConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        extensionConfig.neutralDeadband = Constants.kNeutralDeadband; /* 0.1 % super small for best low-speed control */
        extensionConfig.slot0.kF = Constants.kExtensionGains_MotProf.kF;
        extensionConfig.slot0.kP = Constants.kExtensionGains_MotProf.kP;
        extensionConfig.slot0.kI = Constants.kExtensionGains_MotProf.kI;
        extensionConfig.slot0.kD = Constants.kExtensionGains_MotProf.kD;
        extensionConfig.slot0.integralZone = (int) Constants.kExtensionGains_MotProf.kIzone;
        extensionConfig.slot0.closedLoopPeakOutput = Constants.kExtensionGains_MotProf.kPeakOutput;
        // extensionConfig.slot0.allowableClosedloopError // left default for this example
        // extensionConfig.slot0.maxIntegralAccumulator; // left default for this example
        // extensionConfig.slot0.closedLoopPeriod; // left default for this example
        extensionTalon.configAllSettings(extensionConfig);

        /* pick the sensor phase and desired direction */
        extensionTalon.setInverted(TalonFXInvertType.CounterClockwise);
    }

    public void robotPeriodic() {
        /* get joystick button and stick */
        boolean bPrintValues = _joy.getRawButton(2);
        boolean bFireMp = _joy.getRawButton(1);
        double axis0 = _joy.getRawAxis(0);
        double axis1 = _joy.getRawAxis(1);

        /* if button is up, just drive the motor in PercentOutput */
        if (bFireMp == false) {
            _state = 0;
        }

        switch (_state) {
            /* drive master talon normally */
            case 0:
            rotationTalon.set(TalonFXControlMode.PercentOutput, axis1 * 0.3);
            extensionTalon.set(TalonFXControlMode.PercentOutput, axis0 * 0.2);
                if (bFireMp == true) {
                    /* go to MP logic */
                    _state = 1;
                }
                break;

            /* fire the MP, and stop calling set() since that will cancel the MP */
            case 1:
                double rotationDuration = initRotationBuffer(extensionSetpoint.get(), rotationSetpoint.get());
                double extensionDuration = initExtensionBuffer(extensionSetpoint.get(), rotationSetpoint.get());

                durationDifference = rotationDuration - extensionDuration - extensionRotationProfileDelta.get();
                loopCount = 0;

                if(durationDifference > 0) {
                    // start rotation profile first
                    rotationTalon.startMotionProfile(rotationBufferedStream, 10, TalonFXControlMode.MotionProfile.toControlMode());
                }
                else {
                    // start extension profile first
                    extensionTalon.startMotionProfile(extensionBufferedStream, 10, TalonFXControlMode.MotionProfile.toControlMode());
                }
                
                _state = 2;
                Instrum.printLine("MP started");
                break;

            /* wait to start second MP */
            case 2:
                loopCount++;

                if (loopCount * 20.0/1000.0 >= Math.abs(durationDifference)) {
                    if(durationDifference > 0) {
                        // start extension profile
                        extensionTalon.startMotionProfile(extensionBufferedStream, 10, TalonFXControlMode.MotionProfile.toControlMode());
                    }
                    else {
                        // start rotation profile
                        rotationTalon.startMotionProfile(rotationBufferedStream, 10, TalonFXControlMode.MotionProfile.toControlMode());
                    }
                    _state = 3;
                }
                break;

            /* wait for MP to finish */
            case 3:
                if (rotationTalon.isMotionProfileFinished() && extensionTalon.isMotionProfileFinished()) {
                    Instrum.printLine("MP finished");
                    _state = 4;
                }
                break;

            /* MP is finished, nothing to do */
            case 4:
                break;
        }

        /* print MP values */
        Instrum.loop(bPrintValues, rotationTalon, extensionTalon);
    }

    public double initRotationBuffer(double extension, double rotation) {
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
                radiansToPigeon(Units.degreesToRadians(rotationMotionProfileExtensionCruiseVelocity.get())),
                radiansToPigeon(Units.degreesToRadians(rotationMotionProfileAcceleration.get())));
        TrapezoidProfile profile = new TrapezoidProfile(
                constraints,
                new State(
                        radiansToPigeon(Units.degreesToRadians(rotation)),
                        0),
                new State(rotationTalon.getSelectedSensorPosition(Constants.kPrimaryPIDSlot), 0));

        // based on
        // https://v5.docs.ctr-electronics.com/en/stable/ch16_ClosedLoop.html#motion-profiling-closed-loop
        TrajectoryPoint point = new TrajectoryPoint();

        /* clear the buffer, in case it was used elsewhere */
        rotationBufferedStream.Clear();

        for (double t = 0; !profile.isFinished(t - LOOP_DT_MS / 1000.0); t += LOOP_DT_MS / 1000.0) {
            double rotationPosition = profile.calculate(t).position;
            double rotationVelocity = profile.calculate(t).velocity;

            point.timeDur = LOOP_DT_MS;
            point.position = rotationPosition;
            point.velocity = rotationVelocity;
            point.auxiliaryPos = 0;
            point.auxiliaryVel = 0;
            point.profileSlotSelect0 = Constants.kPrimaryPIDSlot; /* which set of gains would you like to use [0,3]? */
            point.profileSlotSelect1 = 0; /* auxiliary PID [0,1], leave zero */
            point.zeroPos = (t == 0); /* set this to true on the first point */
            point.isLastPoint = profile.isFinished(t); /* set this to true on the last point */
            point.arbFeedFwd = calculateRotationFeedForward(extension, pigeonToRadians(rotationPosition));

            rotationBufferedStream.Write(point);
        }

        return profile.totalTime();
    }

    public double initExtensionBuffer(double extension, double rotation) {
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
                        extensionMotionProfileExtensionCruiseVelocity.get(),
                        EXTENSION_PULLEY_CIRCUMFERENCE,
                        EXTENSION_GEAR_RATIO),
                mpsToFalconMotionMagicUnits(
                        extensionMotionProfileAcceleration.get(),
                        EXTENSION_PULLEY_CIRCUMFERENCE,
                        EXTENSION_GEAR_RATIO));
        TrapezoidProfile profile = new TrapezoidProfile(
                constraints,
                new State(
                        Conversions.metersToFalcon(
                                Units.inchesToMeters(extension), EXTENSION_PULLEY_CIRCUMFERENCE, EXTENSION_GEAR_RATIO),
                        0),
                new State(extensionTalon.getSelectedSensorPosition(Constants.kPrimaryPIDSlot), 0));

        // based on
        // https://v5.docs.ctr-electronics.com/en/stable/ch16_ClosedLoop.html#motion-profiling-closed-loop
        TrajectoryPoint point = new TrajectoryPoint();

        /* clear the buffer, in case it was used elsewhere */
        extensionBufferedStream.Clear();

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
            point.arbFeedFwd = calculateExtensionFeedForward(Units.metersToInches(Conversions
                    .falconToMeters(extensionPosition, EXTENSION_PULLEY_CIRCUMFERENCE, EXTENSION_GEAR_RATIO)),
                    Units.degreesToRadians(rotation));

            extensionBufferedStream.Write(point);
        }

        return profile.totalTime();
    }

    private static double mpsToFalconMotionMagicUnits(
            double mps, double circumference, double gearRatio) {
        double pulleyRotationsPerSecond = mps / circumference;
        double motorRotationsPerSecond = pulleyRotationsPerSecond * gearRatio;
        double ticksPerSecond = motorRotationsPerSecond * 2048.0;
        return ticksPerSecond / 10.0; // per 100 ms
    }

    private double pigeonToRadians(double counts) {
        return counts / PIGEON_UNITS_PER_ROTATION * (2 * Math.PI);
      }
    
      private double radiansToPigeon(double radians) {
        return radians / (2 * Math.PI) * PIGEON_UNITS_PER_ROTATION;
      }

      private static final double D1 = 39.8;
  private static final double D2 = 40.3;
  private static final double D3 = 3.9;
  private static final double D5 = 40.5;
  private static final double H1 = 14.0;
  private static final double H2 = 49.0;
  private static final double M = 21.6;
  private static final double T_SPRING = 34.0;
  private static final double MAX_EXTENSION_BEFORE_MOVING_STAGE_ENGAGEMENT = 34.0;
  private static final double CARRIAGE_MASS = 8.682;
  private static final double MOVING_STAGE_MASS = 4.252;
  private static final double FIXED_STAGE_MASS = 9.223;
  private static final double F_COLLAPSED_ELEVATOR_AT_11_DEG = 25.712; // FIXME: update after tuning
  private static final double MIN_MOTOR_POWER_TO_EXTEND_CARRIAGE_AT_60_DEG = 0.05; // FIXME: tune
  private static final double MIN_MOTOR_POWER_TO_ROTATE_COLLAPSED_ELEVATOR_AT_11_DEG =
      0.05; // FIXME: tune

  private static double calculateRotationFeedForward(double extension, double rotation) {
    double r =
        Math.sqrt(
            Math.pow((D2 - D1 * Math.sin(rotation)), 2)
                + Math.pow((D1 * Math.cos(rotation) + D3), 2));
    double Sa =
        Math.sqrt(
            1 - Math.pow((Math.pow(r, 2) + Math.pow(D1, 2) - Math.pow(D5, 2)) / (2 * D1 * r), 2));
    double h;

    if (extension <= MAX_EXTENSION_BEFORE_MOVING_STAGE_ENGAGEMENT) {
      h = 14.0 + 0.441176 * extension;
    } else {
      h = 0.575539 * extension + 9.43165;
    }

    double F3 =
        (M * h * Math.cos(rotation) + T_SPRING * ((2 * rotation / Math.PI) + 1.0 / 3.0))
            / (D1 * Sa);

    double feedForward =
        (MIN_MOTOR_POWER_TO_ROTATE_COLLAPSED_ELEVATOR_AT_11_DEG / F_COLLAPSED_ELEVATOR_AT_11_DEG)
            * F3;
    return feedForward;
  }

  private static double calculateExtensionFeedForward(double extension, double rotation) {
    double mass;
    if (extension <= MAX_EXTENSION_BEFORE_MOVING_STAGE_ENGAGEMENT) {
      mass = CARRIAGE_MASS;
    } else {
      mass = (CARRIAGE_MASS + MOVING_STAGE_MASS) / 2.0; // two belts are now in tension
    }

    double f = mass * Math.sin(rotation);

    double feedForward =
        (MIN_MOTOR_POWER_TO_EXTEND_CARRIAGE_AT_60_DEG / (CARRIAGE_MASS * 0.866)) * f;
    return feedForward;
  }
    
}