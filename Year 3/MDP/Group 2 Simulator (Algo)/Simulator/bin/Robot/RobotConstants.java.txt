package Robot;

public class RobotConstants {
	public static final int START_POS_X = 1;
	public static final int START_POS_Y = 18;
	public static HEADING START_HEADING = HEADING.NORTH;
	
	public static final int LONG_RANGE_VAL_MIN =3;
	public static final int LONG_RANGE_VAL_MAX =4;
	public static final int SHORT_RANGE_VAL_MAX=2;
	public static final int SHORT_RANGE_VAL_MIN=1;
	public static final int MOVE_COST =1;
	public static final int TURN_COST =2;
	public static final double OBSTACLE_COST=9999;
	public static final int END_POS_X = 13;
	public static final int END_POS_Y = 1;
	
	public enum HEADING{
		NORTH,
		WEST,
		SOUTH,
		EAST
	}
	
	public enum DIRECTION{
		FRONT,
		LEFT,
		RIGHT
	}
}
