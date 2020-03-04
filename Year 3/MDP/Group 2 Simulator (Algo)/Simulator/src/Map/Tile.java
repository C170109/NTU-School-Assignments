package Map;

import java.util.HashMap;

import Robot.RobotConstants;

public class Tile {
	private final int row;
	private final int col;
	private boolean isObstacle;
	private boolean isVirtualWall;
	private boolean isExplored;
	private boolean isStartZone;
	private boolean isGoalZone;
	private boolean isPath;
	private boolean isWaypoint;
	private boolean isVisited;
	private boolean isImage;
	private boolean hasScanned;
	private boolean hasToBeScanned;
	private boolean isMoved;
	private boolean isDiscoverable=true;
	private double cost=0;
	private boolean isDebug;

	public HashMap<String, Boolean> unknownPoints;
	private Image image = null;
	public Tile(int row, int col) {
		this.row = row;
		this.col = col;
	}
	
	public int getRow() {
		return row;
	}
	
	public int getCol() {
		return col;
	}
	public void setIsDiscoverble(boolean v) {
		this.isDiscoverable=v;
	}
	public boolean getIsDiscoverble() {
		return this.isDiscoverable;
	}
	public void setIsObstacle(boolean v) {
		isObstacle = v;
		if(unknownPoints==null)
			this.initUnknownPoints();
	}
	public void setIsVirtualWall(boolean v) {
		isVirtualWall = v;
	}
	
	public void setIsExplored(boolean v) {
		isExplored = v;
	}
	
	public void setIsStartZone(boolean v) {
		isStartZone = v;
	}
	
	public void setIsGoalZone(boolean v) {
		isGoalZone = v;
	}
	
	public boolean getIsObstacle() {
		return isObstacle;
	}
	
	public boolean getIsVirtualWall() {
		return isVirtualWall;
	}
	
	public boolean getIsExplored() {
		return isExplored;
	}
	
	public boolean getIsStartZone() {
		return isStartZone;
	}
	
	public boolean getIsGoalZone() {
		return isGoalZone;
	}
	
	public boolean getIsPath() {
		return isPath;
	}
	
	public void setIsPath(boolean v) {
		this.isPath = v;
	}
	
	public boolean getIsWaypoint() {
		return this.isWaypoint;
	}

	public void setIsWaypoint(boolean v) {
		this.isWaypoint = v;
	}
	public boolean getIsVisited() {
		return this.isVisited;
	}

	public void setIsVisited(boolean v) {
		this.isVisited = v;
	}
	public String printLoc() {
		return "Row :"+row+" Col :"+col;
	}
	
	public double getCost() {
		return this.cost;
	}
	public void setCost(double totalCost) {
		this.cost = totalCost;
	}
	public boolean getIsImage() {
		return this.isImage;
	}
	public void setIsImage(boolean v) {
		this.isImage =v;
	}
	public void setIsImage(boolean v, String id, String heading) {
		this.isImage =v;
		this.image = new Image(id,RobotConstants.HEADING.valueOf(heading));
	}
	public Image getImage() {
		return this.image;
	}
	public boolean getIsScanned() {
		return this.hasScanned;
	}
	public void setIsScanned(boolean v) {
		this.hasScanned=v;
	}
	public void initUnknownPoints() {
		this.unknownPoints = new HashMap<String,Boolean>();
		this.unknownPoints.put("NORTH", false);
		this.unknownPoints.put("SOUTH", false);
		this.unknownPoints.put("WEST", false);
		this.unknownPoints.put("EAST", false);
	}
	public boolean getIsDebug() {
		return this.isDebug;
	}
	public void setIsDebug(boolean v) {
		this.isDebug=v;
	}
	public void debugPrintUP() {
		System.out.println("NORTH :"+unknownPoints.get("NORTH"));
		System.out.println("SOUTH :"+unknownPoints.get("SOUTH"));
		System.out.println("EAST :"+unknownPoints.get("EAST"));
		System.out.println("WEST :"+unknownPoints.get("WEST"));
	}
	public void replaceValue(String dir, boolean v) {
		this.unknownPoints.replace(dir, v);
		this.checkAll4();
	}
	
	public void checkAll4() {
		if(this.unknownPoints.get("NORTH") &&this.unknownPoints.get("SOUTH") &&this.unknownPoints.get("EAST") &&this.unknownPoints.get("WEST")) {
			this.hasScanned=true;
			this.hasToBeScanned=false;
		}
		else
			this.hasToBeScanned=true;
	}
	public void checkWhich(){
		if(!this.unknownPoints.get("NORTH"))
			System.out.println("NORTH ->"+this.printLoc());
		if(!this.unknownPoints.get("SOUTH"))
			System.out.println("SOUTH ->"+this.printLoc());
		if(!this.unknownPoints.get("WEST"))
			System.out.println("WEST ->"+this.printLoc());
		if(!this.unknownPoints.get("EAST"))
			System.out.println("EAST ->"+this.printLoc());
	}
	public void cheatCode() {
		this.unknownPoints.replace("NORTH", true);
		this.unknownPoints.replace("SOUTH", true);
		this.unknownPoints.replace("WEST", true);
		this.unknownPoints.replace("EAST", true);
	}

	public boolean getHasToBeScanned() {
		// TODO Auto-generated method stub
		return this.hasToBeScanned;
	}
	public boolean getIsMoved() {
		return this.isMoved;
	}
	public void setIsMoved(boolean v) {
		this.isMoved = v;
	}
}
