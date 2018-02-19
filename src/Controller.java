import lejos.robotics.subsumption.Arbitrator;
import lejos.robotics.subsumption.Behavior;

public class Controller {
	public static void main(String[] args) {
		Behavior[] behaviors = {
		                        new IdealWallDistance(),
		                        new WallTooFar(),
		                        new WallTooClose(),
								};
		
		Arbitrator arbitrator = new Arbitrator(behaviors);
		arbitrator.go();
	}
}
