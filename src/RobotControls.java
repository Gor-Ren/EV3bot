import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;

public class RobotControls {
	// motors
	private static RegulatedMotor leftMotor = new EV3LargeRegulatedMotor(MotorPort.B);
	private static RegulatedMotor rightMotor = new EV3LargeRegulatedMotor(MotorPort.C);
	
	// sensors and sensor samplers
	private static EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(SensorPort.S1);
	private static SampleProvider usSensorSampler = RobotControls.usSensor.getDistanceMode();
	private static EV3TouchSensor touchSensor = new EV3TouchSensor(SensorPort.S4);
	private static SampleProvider touchSensorSampler = RobotControls.touchSensor.getTouchMode();

	
	// Constants
	public static final float TARGET_DISTANCE = (float) 0.15; // meters
	public static final float LEEWAY = (float) 0.05; // meters
	public static final int TURN_ROTATION = 75; // degrees
	public static final int POST_TURN_ROTATION = 360; // degrees
	public static final int BACK_UP_DISTANCE = -180; // degrees
	public static final int BACK_UP_ROTATION = 90; // degrees
	public static final float MOTOR_SLOW_FACTOR = (float) 0.7; // dimensionless
	public static final float DISTANCE_BETWEEN_WHEELS = (float) 12; // cm
	
	public static RegulatedMotor getLeftMotor() {
		return leftMotor;
	}
	
	public static RegulatedMotor getRightMotor() {
		return rightMotor;
	}
	
	public static SampleProvider getUsSensorSampler() {
		return usSensorSampler;
	}
	
	public static SampleProvider getTouchSensorSampler() {
		return touchSensorSampler;
	}
}
