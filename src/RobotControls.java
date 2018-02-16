import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.RegulatedMotor;

public class RobotControls {
	private static RegulatedMotor leftMotor = new EV3LargeRegulatedMotor(MotorPort.B);
	private static RegulatedMotor rightMotor = new EV3LargeRegulatedMotor(MotorPort.C);
	private static EV3UltrasonicSensor uSSensor = new EV3UltrasonicSensor(SensorPort.S2);
	
	// Constants
	public static final float TARGET_DISTANCE = (float) 0.15; // meters
	public static final float LEEWAY = (float) 0.05; // meters
	public static final int TURN_ROTATION = 45; // degrees
	public static final int POST_TURN_ROTATION = 2*360; // degrees
	
	public static RegulatedMotor getLeftMotor() {
		return leftMotor;
	}
	
	public static RegulatedMotor getRightMotor() {
		return rightMotor;
	}
	
	public static EV3UltrasonicSensor getSensor() {
		return uSSensor;
	}
}
