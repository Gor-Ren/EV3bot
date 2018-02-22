import lejos.hardware.lcd.LCD;
import lejos.robotics.subsumption.Behavior;

public class WallTooFar implements Behavior {

	private boolean suppressed;
	
	@Override
	public boolean takeControl() {
		float[] sample = new float[1];
		RobotControls.getUsSensorSampler().fetchSample(sample,0);
		return sample[0] > RobotControls.TARGET_DISTANCE + RobotControls.LEEWAY;
	}

	@Override
	public void action() {
		this.suppressed = false;
		LCD.drawString(this.getClass().getName(), 0, 4);

		RobotControls.incrementCounter();

		// make an incremental turn towards wall
		RobotControls.getRightMotor().rotate(-RobotControls.TURN_ROTATION, true);
		RobotControls.getLeftMotor().rotate(RobotControls.TURN_ROTATION, false);

		// drive forwards fixed distance
		RobotControls.getLeftMotor().rotate(RobotControls.POST_TURN_DISTANCE, true);
		RobotControls.getRightMotor().rotate(RobotControls.POST_TURN_DISTANCE, true);
		
		while(!this.suppressed && RobotControls.getLeftMotor().isMoving()) {
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
