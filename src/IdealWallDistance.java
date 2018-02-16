import lejos.hardware.lcd.LCD;
import lejos.robotics.subsumption.Behavior;

public class IdealWallDistance implements Behavior {

	private boolean suppressed;
	
	@Override
	public boolean takeControl() {
		return true;
	}

	@Override
	public void action() {
		this.suppressed = false;
		LCD.drawString(this.getClass().getName(), 0, 4);

		RobotControls.getLeftMotor().forward();
		RobotControls.getRightMotor().forward();
		
		while(!this.suppressed) {
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
