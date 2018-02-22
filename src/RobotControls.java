import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;

public class RobotControls {
	// wall too far/ too near action counter
	private static int interventionCounter = 0;
	private static int bumpCounter = 0;

	// motors
	private static RegulatedMotor leftMotor = new EV3LargeRegulatedMotor(MotorPort.B);
	private static RegulatedMotor rightMotor = new EV3LargeRegulatedMotor(MotorPort.C);

	// sensors and sensor samplers
	private static EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(SensorPort.S1);
	private static SampleProvider usSensorSampler = RobotControls.usSensor.getDistanceMode();
	private static EV3TouchSensor touchSensor = new EV3TouchSensor(SensorPort.S4);
	private static SampleProvider touchSensorSampler = RobotControls.touchSensor.getTouchMode();

	// Constants
	public static final float TARGET_DISTANCE = (float) 0.20; // meters
	public static final float LEEWAY = (float) 0.15; // meters
	public static final int TURN_ROTATION = 90; // degrees (adjust direction)
	public static final int POST_TURN_DISTANCE = 180; // degrees (move)
	public static final int BACK_UP_DISTANCE = -180; // degrees
	public static final int BACK_UP_ROTATION = 135; // degrees
	public static final float MOTOR_SLOW_FACTOR = (float) 0.75; // dimensionless
	public static final float DISTANCE_BETWEEN_WHEELS = (float) 12; // cm

	// PID parameters
	private static final double P_GAIN = 2;
	private static final double I_GAIN = 0;
	private static final double D_GAIN = 1;
	public static final double PID_OUTPUT_LIMIT = 0.2;

	// PID controller
	private static MiniPID pid = new MiniPID(RobotControls.P_GAIN, RobotControls.I_GAIN, RobotControls.D_GAIN);
	static {
		// configure controllers
		pid.setSetpoint(RobotControls.TARGET_DISTANCE);
		pid.setOutputLimits(-RobotControls.PID_OUTPUT_LIMIT,RobotControls.PID_OUTPUT_LIMIT);
	}

	public static RegulatedMotor getLeftMotor() {
		return leftMotor;
	}
	
	public static RegulatedMotor getRightMotor() {
		return rightMotor;
	}
	
	public static MiniPID getPID() {
		return pid;
	}
	
	public static SampleProvider getUsSensorSampler() {
		return usSensorSampler;
	}
	
	public static SampleProvider getTouchSensorSampler() {
		return touchSensorSampler;
	}
	
	public static void incrementInterventionCounter () {
		RobotControls.interventionCounter++;
		LCD.drawString("Corrections: " + RobotControls.interventionCounter, 0, 3);
	}
	
	public static void incrementBumpCounter() {
		RobotControls.bumpCounter++;
		LCD.drawString("Bumps: " + RobotControls.bumpCounter, 0, 2);
	}

}
