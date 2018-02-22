import lejos.hardware.lcd.LCD;
import lejos.robotics.navigation.MoveController;
import lejos.robotics.subsumption.Behavior;
import lejos.utility.Delay;

public class CleverIdealWallDistance implements Behavior {

	private boolean suppressed;
	private float lastReading[] = new float[1];
	private float currentReading[] = new float[1];
	private float currentWheelAngVelocity, currentRobotVelocity, currentAngle;
	private float waitTime;
	
	@Override
	public boolean takeControl() {
		return true;
	}

	@Override
	public void action() {
		this.suppressed = false;
		LCD.drawString(this.getClass().getName(), 0, 4);
		RobotControls.getUsSensorSampler().fetchSample(currentReading,0); // initial sensor reading

		RobotControls.getLeftMotor().forward();
		RobotControls.getRightMotor().forward();
		
		Delay.msDelay(500);
		int leftWheelMaxAngVelocity = RobotControls.getLeftMotor().getSpeed(); // deg per second
		int rightWheelMaxAngVelocity = RobotControls.getRightMotor().getSpeed(); // deg per second
		
		while(!this.suppressed) {

			currentWheelAngVelocity = RobotControls.getLeftMotor().getSpeed(); // deg per second
			
			lastReading[0] = currentReading[0];
			RobotControls.getUsSensorSampler().fetchSample(currentReading,0);
			//Calculate angle
			currentWheelAngVelocity = RobotControls.getLeftMotor().getSpeed(); // deg per second
			currentRobotVelocity = (float) (currentWheelAngVelocity / 360 * MoveController.WHEEL_SIZE_EV3 * Math.PI); //cm per second
			currentAngle = (float) ((currentReading[0]-lastReading[0]) / (currentRobotVelocity * 0.5));
			//Control wheels
			if (currentAngle < 0) { // negative angle corresponds to pointing at wall
				RobotControls.getRightMotor().setSpeed(rightWheelMaxAngVelocity);
				RobotControls.getLeftMotor().setSpeed((int) (leftWheelMaxAngVelocity * RobotControls.MOTOR_SLOW_FACTOR)); // slow left wheel
			} else if (currentAngle >= 0) { // positive angle corresponds to pointing away from wall
				RobotControls.getRightMotor().setSpeed((int) (rightWheelMaxAngVelocity * RobotControls.MOTOR_SLOW_FACTOR)); // slow right wheel
				RobotControls.getLeftMotor().setSpeed(leftWheelMaxAngVelocity); 
			}
			waitTime = (float) ((currentAngle) / (rightWheelMaxAngVelocity * 360 / (2 * Math.PI * RobotControls.DISTANCE_BETWEEN_WHEELS)));
			Delay.msDelay((long) (waitTime * 1000));
			
			RobotControls.getRightMotor().setSpeed(rightWheelMaxAngVelocity);
			RobotControls.getLeftMotor().setSpeed(leftWheelMaxAngVelocity);
			
			Thread.yield();
		}
		
		RobotControls.getLeftMotor().stop();
		RobotControls.getRightMotor().stop();
	}

	@Override
	public void suppress() {
		this.suppressed = true;
	}
	
}
