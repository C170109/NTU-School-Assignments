package Image_Algorithms;

import java.util.*;
import Map.Map;
import Map.MapConstants;
import Map.Tile;
import Robot.Robot;
import Robot.RobotConstants;

public class FastestPathAlgo {
	private final List<Tile> frontier;
	private final List<Tile> closed;
	//private final List<Tile> path;
	private HashMap<Tile,Double> arenaGCost;
	private HashMap<Tile,Double> arenaTotalCost;
	private HashMap<Tile,Tile> parentTile;
	private RobotConstants.HEADING curHeading;
	private RobotConstants.HEADING nextHeading;
	
	//private final int start_row;
	//private final int start_col;
	//private final int end_row;
	//private final int end_col;
	private final Tile waypoint;
	
	private Map explored_map;
	private Tile curTile;
	private Robot robot;
	
	private final Tile[] neighborTile = new Tile[4];
	
	public FastestPathAlgo(Map explored_map, Robot robot, Tile waypoint) {
		this.explored_map = explored_map;
		//this.start_row = start_row;
		//this.start_col = start_col;
		//this.startTile = explored_map.getTile(start_row,start_col);
		this.robot = robot;		//this.end_row = end_row;
		//this.end_col = end_col;
		this.waypoint = waypoint;
		if(waypoint !=null) {
			waypoint.setIsWaypoint(true);
		}		
	
		this.frontier = new ArrayList<Tile>();
		this.closed = new ArrayList<Tile>();
		//this.path = new ArrayList<Tile>();
		
		//this.curTile = explored_map.getTile(start_row,start_col);
		this.curHeading = robot.getCurrentHeading();
		
		arenaGCost = new HashMap<Tile,Double>();
		parentTile = new HashMap<Tile,Tile>();
		arenaTotalCost = new HashMap<Tile, Double>();
		
		//init initial cost
		for(int col = 0; col<MapConstants.MAP_COLS;col++) {
			for(int row =0; row<MapConstants.MAP_ROWS;row++) {
				Tile temp = explored_map.getTile(row,col);
				if(explored_map.checkValid(row,col)) {
					arenaGCost.put(temp,0.0);
					arenaTotalCost.put(temp, 0.0);
				}
				else {
					arenaGCost.put(temp,RobotConstants.OBSTACLE_COST);
					arenaTotalCost.put(temp, RobotConstants.OBSTACLE_COST);
				}
			}
		}
		
		explored_map.setObstacleVirtualWall();
	}
	//reinit : set path to not visited and not path 
	public void setPathNotTrue() {
		for(int col = 0; col<MapConstants.MAP_COLS;col++) {
			for(int row =0; row<MapConstants.MAP_ROWS;row++) {
				Tile temp = explored_map.getTile(row,col);;
				temp.setIsPath(false);
				temp.setIsVisited(false);
			}
		}
	}
	/*
	 * Find fastest path 
	 * General Idea:
	 * -> From start tile, look at the four adjacent tiles (as neighboring tiles)
	 * -> Calculate total cost : g + h cost. Heuristic using Manhattan Distance
	 * -> Add all neighboring tiles to open list
	 * -> Choose smallest cost to explore next
	 * -> Add to hashmap Parent of tile to be explored and current tile. 
	 * -> Add current tile to closed, and next current tile to be the smallest cost tile
	 * -> Repeat
	 */
	public ArrayList<Tile> findFastestPath(int start_row, int start_col, int end_row, int end_col, RobotConstants.HEADING h) {
		//reinit
		parentTile.clear();
		frontier.clear();
		closed.clear();
		this.resetCost();
		//this.setPathNotTrue();
		
		this.curHeading = h;
		Tile goal = explored_map.getTile(end_row,  end_col);
		this.curTile = explored_map.getTile(start_row, start_col);
		/*
		 * while searched does not contain goal, explore next smallest cost
		 */
		while(!closed.contains(goal)) {
			this.addNeighbors();
			
			closed.add(curTile);
			Tile temp = this.getLowestCostF(goal);
			if(temp == null)
				return null;
			curTile = temp;
			curHeading = nextHeading;
			frontier.remove(temp);
			closed.add(temp);
			//explored_map.getTile(temp.getRow(),temp.getCol()).setIsPath(true);
			//explored_map.paintAgain();
			//this.printGMap();
		}
		explored_map.paintAgain();
		this.curTile=explored_map.getTile(start_row, start_col);
		this.curHeading = robot.getCurrentHeading();
		ArrayList<Tile> path = reversePath(goal);
		if(path.get(0)==explored_map.getTile(start_row,start_col))
			path.remove(0);
		
		this.printFastestPath(path);
		frontier.clear();
		closed.clear();
		parentTile.clear();
		return path;
	}
	
	public void addNeighbors() {
		Tile tempTile;
		int row,col;
		double g,h;

		//system.out.println("HI");
		if(this.parentTile.get(curTile) != null)
			curHeading = this.getNextHeading(this.parentTile.get(curTile), curTile.getRow(), curTile.getCol(), curHeading);
		

		//System.out.println("PASS");
		//front
		row=curTile.getRow();
		col=curTile.getCol()-1;
		
		//immediate front
		if(explored_map.checkValid(row, col) ) {
			g =  arenaGCost.get(explored_map.getTile(row,col)) + calCostG(curTile,row,col,curHeading);// + calCostH(row,col);
			//h = calCostH(row,col);
			tempTile = explored_map.getTile(row,col);
			if(frontier.contains(tempTile)==false && closed.contains(tempTile)==false) {
				//System.out.println("DEBUG :: No same node found");
				frontier.add(tempTile);
				neighborTile[0]=tempTile;
				arenaGCost.put(tempTile,g);
			}
			else {
				//System.out.println("DEBUG :: Same node found");
				double tempG = arenaGCost.get(tempTile);
				if(g<tempG)
				{
					arenaGCost.replace(tempTile,g);
				}
			}
		}
		
		
		//right
		row=curTile.getRow()+1;
		col=curTile.getCol();
		
		if(explored_map.checkValid(row, col)) {
			g = arenaGCost.get(explored_map.getTile(row,col)) + calCostG(curTile,row,col,curHeading);// + calCostH(row,col);
			//h = calCostH(row,col);
			tempTile = explored_map.getTile(row,col);
			if(frontier.contains(tempTile)==false && closed.contains(tempTile)==false) {
				//System.out.println("DEBUG :: No same node found");
				frontier.add(tempTile);
				neighborTile[1] = tempTile;
				arenaGCost.put(tempTile,g);
			}
			else {
				//System.out.println("DEBUG :: Same node found");
				double tempG = arenaGCost.get(tempTile);
				if(g<tempG)
				{
					arenaGCost.replace(tempTile,g);
				}
			}
		}
		
		//left
		row=curTile.getRow()-1;
		col=curTile.getCol();

		if(explored_map.checkValid(row, col)) {
			g = arenaGCost.get(explored_map.getTile(row,col)) + calCostG(curTile,row,col,curHeading);
			//h = calCostH(row,col);
			tempTile = explored_map.getTile(row,col);
			if(frontier.contains(tempTile)==false && closed.contains(tempTile)==false) {
				//System.out.println("DEBUG :: No same node found");
				frontier.add(tempTile);
				neighborTile[2] =tempTile;
				arenaGCost.put(tempTile,g);
			}
			else {
				//System.out.println("DEBUG :: Same node found");
				double tempG = arenaGCost.get(tempTile);
				if(g<tempG)
				{
					arenaGCost.replace(tempTile,g);
				}
			}
		}
		
		//south
		row=curTile.getRow();
		col=curTile.getCol()+1;

		if(explored_map.checkValid(row, col)) {
			g = arenaGCost.get(explored_map.getTile(row,col)) + calCostG(curTile,row,col,curHeading); //+ calCostH(row,col);
			//h = calCostH(row,col);
			tempTile = explored_map.getTile(row,col);
			if(frontier.contains(tempTile)==false && closed.contains(tempTile)==false) {
				//System.out.println("DEBUG :: No same node found");
				frontier.add(tempTile);
				neighborTile[3]=tempTile;
				arenaGCost.put(tempTile,g);
			}
			else {
				//System.out.println("DEBUG :: Same node found");
				double tempG = arenaGCost.get(tempTile);
				if(g<tempG)
				{
					arenaGCost.replace(tempTile,g);
				}
			}
		}
		addAndReinitNeighbor();
	}
	 
	private void addAndReinitNeighbor() {
		Tile temp;
		for(int i=0; i<4; i++) {
			temp=neighborTile[i];
			if(temp!=null && !closed.contains(temp)) {
				//System.out.println("DEBUG :: Added Tile :"+temp.printLoc());
				parentTile.put(temp, curTile);
			}
			neighborTile[i]=null;
			//System.out.println(i);
		}
		//System.out.println("reach here");
	}
	
	/*
	 * Returns tile for lowest cost (if any), if none returns null => can't be travelled to
	 */
	private Tile getLowestCostF(Tile goal) {
		double lowestCost =999999999;
		Tile lowestCostT = null;
		
		for(Tile temp:frontier) {
			if(temp.getRow() == goal.getRow() && temp.getCol() == goal.getCol()) {
				if(lowestCostT == null)
					lowestCostT=temp;
				nextHeading = this.getNextHeading(curTile, lowestCostT.getRow(), lowestCostT.getCol(), curHeading);
				System.out.println(nextHeading);
				return temp;
			} 
			double totalCost = arenaGCost.get(temp) + calCostH(temp,goal);	
			arenaTotalCost.replace(temp, totalCost);
			//explored_map.getTile(temp.getRow(), temp.getCol()).setCost(totalCost);
			if(totalCost<lowestCost) {// && difference == 1) {
				//System.out.println("DEBUG :: Lowest Now : "+temp.printLoc() + "cost : "+lowestCost);
				lowestCostT=temp;
				lowestCost=totalCost;
			}
		}
		if(lowestCostT==null) {
			//System.out.println("fail");
			return null;
		}

		
		nextHeading = this.getNextHeading(curTile, lowestCostT.getRow(), lowestCostT.getCol(), curHeading);
		//System.out.println(curTile.printLoc() + " , " + lowestCostT.printLoc() + " : "+nextHeading);
		return lowestCostT;
	}
	private double calCostH(Tile temp, Tile goal) {
		// TODO Auto-generated method stub
		return Math.abs(temp.getRow()- goal.getRow()) + Math.abs(temp.getCol()- goal.getCol()); 	
		//return Math.sqrt(Math.abs(((this.end_row-row)^2 + (this.end_col-col)^2)));
	}

	public double calCostG(Tile curTile, int row, int col, RobotConstants.HEADING h) {
		double gcost = RobotConstants.MOVE_COST + calTurnCost(curTile,row,col,h) + arenaGCost.get(curTile);
		return gcost;
	}
	
	public double calTurnCost(Tile curTile, int row, int col, RobotConstants.HEADING h) {		
		RobotConstants.HEADING nextH = getNextHeading(curTile, row, col, h);

		int numTurn = Math.abs(h.ordinal() - nextH.ordinal());
		
		if(numTurn >2) {
			numTurn %=2;
		}

		return numTurn * RobotConstants.TURN_COST;
	}
	public RobotConstants.HEADING getNextHeading(Tile curTile, int row, int col, RobotConstants.HEADING h){
		
		if(curTile.getCol() - col>0) {
			return RobotConstants.HEADING.NORTH;
		}
		else if(curTile.getCol()-col<0) {
			return RobotConstants.HEADING.SOUTH;
		}
		else if(curTile.getRow()-row>0) {
			return RobotConstants.HEADING.WEST;
		}
		else
			return RobotConstants.HEADING.EAST;
	}
	/*
	 * From the goal, find the parent tile to it (parent tile signifies the previous tile that the algorithm took
	 * to that said child tile)
	 * Therefore, requires to travel backwards to get fastest path.
	 */
	private ArrayList<Tile> reversePath(Tile goal) {
        Tile temp = this.explored_map.getTile(goal.getRow(), goal.getCol());
        ArrayList <Tile> tempPath = new ArrayList<Tile>();
        while (true) {
        	explored_map.getTile(temp.getRow(),temp.getCol()).setIsPath(true);
        	explored_map.getTile(temp.getRow(), temp.getCol()).setIsVisited(true);
        	tempPath.add(temp);
            temp = parentTile.get(temp);
            if (temp == null) {
                break;
            }
            //System.out.println(path.size());
        }
        Collections.reverse(tempPath);
        return tempPath;
       // printFastestPath();
	}
	public void printFastestPath(ArrayList<Tile> path) {
		for(Tile temp : path) {
			System.out.println(temp.printLoc());
		}
	}
	public void resetCost() {
		for(int col = 0; col<MapConstants.MAP_COLS;col++) {
			for(int row =0; row<MapConstants.MAP_ROWS;row++) {
				Tile temp = explored_map.getTile(row,col);;
				if(explored_map.checkValid(row,col)) {
					arenaGCost.put(temp,0.0);
				}
				else {
					arenaGCost.put(temp,RobotConstants.OBSTACLE_COST);
				}
			}
		}
	}
	public void printMap() {
		System.out.print("Y/X");
		for(int i = 0; i<15;i++) {
			System.out.print(String.format("%1$"+6+"s", String.valueOf(i))+" ");
		}
		System.out.println("");
		for(int col =0; col<MapConstants.MAP_COLS;col++) {
			System.out.print(String.format("%1$"+3+"s", String.valueOf(col) + " "));
			for(int row=0; row<MapConstants.MAP_ROWS;row++) {
				double tcost = arenaTotalCost.get(explored_map.getTile(row,col));
				//String.format("%-" + n + "s", tcost);  
				if(explored_map.getTile(row, col).getIsPath())
					System.out.print(String.format("%1$"+6+"s", "("+String.valueOf(tcost)+")")+" ");
				else
					System.out.print(String.format("%1$"+6+"s", String.valueOf(tcost))+" ");
				//System.out.print(arenaTotalCost.get(explored_map.getTile(row,col))+" ,");
			}
			System.out.println("");
		}
	}
	public void printGMap() {
		System.out.print("Y/X");
		for(int i = 0; i<15;i++) {
			System.out.print(String.format("%1$"+6+"s", String.valueOf(i))+" ");
		}
		System.out.println("");
		for(int col =0; col<MapConstants.MAP_COLS;col++) {
			System.out.print(String.format("%1$"+3+"s", String.valueOf(col) + " "));
			for(int row=0; row<MapConstants.MAP_ROWS;row++) {
				double tcost = arenaGCost.get(explored_map.getTile(row,col));
				//String.format("%-" + n + "s", tcost);  
				if(explored_map.getTile(row, col).getIsPath())
					System.out.print(String.format("%1$"+6+"s", "("+String.valueOf(tcost)+")")+" ");
				else
					System.out.print(String.format("%1$"+6+"s", String.valueOf(tcost))+" ");
				//System.out.print(arenaTotalCost.get(explored_map.getTile(row,col))+" ,");
			}
			System.out.println("");
		}
	}
	
	public void printgMap(RobotConstants.HEADING h) {
		System.out.print("Y/X");
		for(int i = 0; i<15;i++) {
			System.out.print(String.format("%1$"+6+"s", String.valueOf(i))+" ");
		}
		System.out.println("");
		for(int col =0; col<MapConstants.MAP_COLS;col++) {
			System.out.print(String.format("%1$"+3+"s", String.valueOf(col) + " "));
			for(int row=0; row<MapConstants.MAP_ROWS;row++) {
				double tcost = arenaGCost.get(explored_map.getTile(row,col));
				//String.format("%-" + n + "s", tcost);  
				if(explored_map.getTile(row, col).getIsPath())
					System.out.print(String.format("%1$"+6+"s", "("+String.valueOf(tcost)+" " +h +")")+" ");
				else
					System.out.print(String.format("%1$"+6+"s", String.valueOf(tcost))+" "+h+ " ");
				//System.out.print(arenaTotalCost.get(explored_map.getTile(row,col))+" ,");
			}
			System.out.println("");
		}
	}
	
	public void setMap(Map explored_map) {
		this.explored_map = explored_map;
	}
	
}
