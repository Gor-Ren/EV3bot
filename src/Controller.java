import lejos.robotics.subsumption.Arbitrator;
import lejos.robotics.subsumption.Behavior;

public class Controller {
	public static void main(String[] args) {
		Behavior goForward = new IdealWallDistance();
		Behavior adjustRight = new WallTooFar();
		Behavior adjustLeft = new WallTooClose();
		Behavior[] behaviors = {goForward, adjustRight, adjustLeft};
		
		Arbitrator arbitrator = new Arbitrator(behaviors);
		arbitrator.go();
	}
}
