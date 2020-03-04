package Map;

import Robot.RobotConstants;

public class Image {
	private String id;
	private RobotConstants.HEADING heading;
	
	public Image(String id, RobotConstants.HEADING h){
		this.id = id;
		this.heading = h;
	}
	
	public String getId() {
		return this.id;
	}
	public RobotConstants.HEADING getHeading() {
		return this.heading;
	}
}
