import lejos.hardware.lcd.LCD;
import lejos.robotics.subsumption.Behavior;

public class HitFrontWall implements Behavior {

	private boolean suppressed;
	
	@Override
	public boolean takeControl() {
		float sample[] = new float[1];
		RobotControls.getTouchSensorSampler().fetchSample(sample,0);
		return sample[0] > 0.1; //True if not zero.
	}

	@Override
	public void action() {
		suppressed = false;
		LCD.drawString(this.getClass().getName(), 0, 4);
		
		//Back up.
		RobotControls.getLeftMotor().rotate(RobotControls.BACK_UP_DISTANCE, true);
		RobotControls.getRightMotor().rotate(RobotControls.BACK_UP_DISTANCE, false); // Back up fully before returning.
		
		// Turn left.
		RobotControls.getLeftMotor().rotate(-RobotControls.BACK_UP_ROTATION, true);
		RobotControls.getRightMotor().rotate(RobotControls.BACK_UP_ROTATION, false);
		
		RobotControls.getLeftMotor().stop();
		RobotControls.getRightMotor().stop();	
	}

	@Override
	public void suppress() {
		// TODO Auto-generated method stub
		
	}

}
