import lejos.hardware.lcd.LCD;
import lejos.robotics.subsumption.Behavior;
import lejos.utility.Delay;

public class PIDIdealWallDistance implements Behavior {

	private boolean suppressed;
	private float currentReading[] = new float[1];
	
	@Override
	public boolean takeControl() {
		return true;
	}

	@Override
	public void action() {
		this.suppressed = false;
		LCD.drawString(this.getClass().getName(), 0, 4);
		
		// start moving forward
		RobotControls.getLeftMotor().forward();
		RobotControls.getRightMotor().forward();
		
		// get PID controller reference and restart its state
		MiniPID pid = RobotControls.getPID();
		pid.reset();

		Delay.msDelay(100);
		int leftWheelMaxAngVelocity = RobotControls.getLeftMotor().getSpeed(); // deg per second
		int rightWheelMaxAngVelocity = RobotControls.getRightMotor().getSpeed(); // deg per second

		while(!this.suppressed) {

			RobotControls.getUsSensorSampler().fetchSample(currentReading,0);
			double measuredDistance = (double) currentReading[0];
			
			// get PID controller output
			double output = pid.getOutput(measuredDistance);
			LCD.drawString("Output: " + Double.toString(output), 0, 5);
			LCD.drawString("Measured: " + Double.toString(measuredDistance), 0, 6);
			
			// normalise output
			output /= RobotControls.PID_OUTPUT_LIMIT;
			
			//Control wheels
			if (output > 0) { // positive output corresponds to pointing at wall
				RobotControls.getRightMotor().setSpeed(rightWheelMaxAngVelocity);
				RobotControls.getLeftMotor().setSpeed((int) (leftWheelMaxAngVelocity * (1 - output * RobotControls.MOTOR_SLOW_FACTOR))); // slow left wheel
			} else { // negative output corresponds to pointing away from wall
				RobotControls.getRightMotor().setSpeed((int) (rightWheelMaxAngVelocity * (1 + output * RobotControls.MOTOR_SLOW_FACTOR))); // slow right wheel
				RobotControls.getLeftMotor().setSpeed(leftWheelMaxAngVelocity); 
			}

			Delay.msDelay(100);
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
