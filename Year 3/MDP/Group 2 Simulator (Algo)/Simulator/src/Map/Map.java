package Map;

import Robot.Robot;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.time.LocalTime;
import java.util.ArrayList;

import javax.swing.*;

public class Map extends JPanel{
	//init variables
	private boolean masked = false;    
	private boolean initialDraw = true;
    private final Tile[][] arena;
    private Robot robot = null;
    private Tile waypoint=null;
    private DisplayTile[][] arenaTile;
    private ArrayList<Tile> possiblePoints = new ArrayList<Tile>();
    private ArrayList<Tile> lefToBeExplored = new ArrayList<Tile>();
    private String mapId;
   // private int mapcounter=1;
    
    private class DisplayTile{
    	public final int xCoor;
    	public final int yCoor;
    	public final int tileSize;
    	
    	public DisplayTile(int xCoor, int yCoor, int tileSize) {
	    	this.xCoor = xCoor+20;
	    	this.yCoor = yCoor+20;
	    	this.tileSize = tileSize-MapConstants.TILE_WEIGHT;
    	}
    }

	public Map(Robot robot, boolean masked) {
		this.robot = robot;
		this.masked = masked;
		
		//arena init
		this.arena = new Tile[MapConstants.MAP_ROWS][MapConstants.MAP_COLS];
    	for(int x = 0; x<arena.length; x++) {
    		for(int y = 0; y<arena[0].length;y++) {
    			arena[x][y] = new Tile(x,y);
    		}
    	}
    	if(masked) {
    		this.mapId = "Sim";
    	}
    	else
    		this.mapId ="Real";
    	setStartGoalZone();
    	initSetVirtualWall();
    	
		//init paint tiles
		if(initialDraw) {

	    	arenaTile = new DisplayTile[MapConstants.MAP_ROWS][MapConstants.MAP_COLS];
			for(int x=0;x<MapConstants.MAP_ROWS;x++) {
				for(int y=0;y<MapConstants.MAP_COLS;y++) {
	    			arenaTile[x][y] = new DisplayTile(x*MapConstants.TILE_SIZE,y*MapConstants.TILE_SIZE,MapConstants.TILE_SIZE);
	    		}
	    	}
			//this.paintAgain();
			initialDraw=false;
		}
	}
	
	public void setWaypoint(int xRow, int yRow) {
		if(this.waypoint==null) {
			this.waypoint = arena[xRow][yRow];
			this.waypoint.setIsWaypoint(true);
		}
		else {
			this.waypoint.setIsWaypoint(false);
			this.waypoint = arena[xRow][yRow];
			this.waypoint.setIsWaypoint(true);
		}
		
		this.paintAgain();
	}
	public Tile getWaypoint() {
		return this.waypoint;
	}
	
    public void setStartGoalZone() {
    	for(int x=0;x<MapConstants.ZONE_SIZE;x++) {
    		for(int y=0;y<MapConstants.ZONE_SIZE;y++) {
        		arena[x][y+MapConstants.START_ZONE].setIsStartZone(true);
        		arena[x][y+MapConstants.START_ZONE].setIsExplored(true);
    		}
    	}
    	
    	for(int x=0;x<MapConstants.ZONE_SIZE;x++) {
    		for(int y=0;y<MapConstants.ZONE_SIZE;y++) {
        		arena[x+MapConstants.GOAL_ZONE][y].setIsGoalZone(true);
    		}
    	}
    	/*
    	//Robot center
    	robot.setRow(1);
    	robot.setCol(START_ZONE+1);
    	//robot.setHeading(Robot.HEADING.NORTH);
    	 * */
    	 
    }
    public void initSetVirtualWall() {
    	for(int col =0; col<MapConstants.MAP_COLS;col++) {
    		arena[0][col].setIsVirtualWall(true);
    		arena[MapConstants.MAP_ROWS-1][col].setIsVirtualWall(true);
    	}
    	
    	for(int row=0; row<MapConstants.MAP_ROWS;row++) {
    		arena[row][0].setIsVirtualWall(true);
    		arena[row][MapConstants.MAP_COLS-1].setIsVirtualWall(true);
    	}
    }
    
    public Tile getTile(int row, int col) {
    	return arena[row][col];
    }

    public boolean checkValid(int row, int col) {
    	try {
    		return arena[row][col].getIsExplored() && !arena[row][col].getIsObstacle() && !arena[row][col].getIsVirtualWall();
    	}catch(ArrayIndexOutOfBoundsException e) {
    		return false;
    	}
    }
    
    //get next nearest non-explored
    public Tile getNextNotExplored(int row, int col) {
    	Tile nearest = null;
    	//double distance = -9999, tempDistance=0;
    	double distance = 9999, tempDistance=0;
    	//original
    	for(int c =MapConstants.MAP_COLS-1; c>=0;c--) {
    		for(int r =0 ; r< MapConstants.MAP_ROWS;r++) {
    			if(!arena[r][c].getIsExplored() &&arena[r][c].getIsDiscoverble()) {// && this.checkTileSurroundingLenie(r,c)) {
    				tempDistance = this.distance(row, col, r, c);
    				//if(distance<tempDistance) {
    				if(tempDistance<distance) {
    					nearest = arena[r][c];
    					distance = tempDistance;
    				}
    			}
    		}
    	}
    	return nearest;
    }
    
    //get nearest from unexplored
    public Tile getNearestExplored(Tile unexplored, int cur_row, int cur_col) {
    	Tile nearest = null;
    	double distance = 9999;
    	if (nearest == null) {
    		distance = 9999;
    		for(int col = MapConstants.MAP_COLS-2; col>=1; col--) {
        		for(int row =1 ; row< MapConstants.MAP_ROWS;row++) {
        			if(!arena[row][col].getIsVisited()) {
        			if(checkTileSurroundingLenient(row,col) && row != cur_row && col!=cur_col) {
        				if(distance>distance(row,col,unexplored.getRow(),unexplored.getCol())) {
        					nearest = arena[row][col];
        					distance = distance(row,col,unexplored.getRow(),unexplored.getCol());
        				}
        			}
        		}
        		}
        	}
    	}
    	System.out.println("Nearest Row:" + nearest.getRow() + " Col:" + nearest.getCol());
    	if(nearest!=null)
    		nearest.setIsVisited(true);
    	return nearest;
    }
    
    //get nearest from unknown point (ISLAND)
    public Tile getNearestIslandPoint(Tile unexplored, int cur_row, int cur_col) {
    	Tile nearest = null;
    	double distance = 9999;
    	if (nearest == null) {
    		distance = 9999;
    		for(int row = 1; row<MapConstants.MAP_ROWS;row++) {
        		for(int col = MapConstants.MAP_COLS-2; col>=1; col--) {
        			if(!arena[row][col].getIsVisited()) {
        			if(checkTileSurroundingLenient(row,col) && row != cur_row && col!=cur_col) {
        				if(distance>distance(row,col,unexplored.getRow(),unexplored.getCol())) {
        					nearest = arena[row][col];
        					distance = distance(row,col,unexplored.getRow(),unexplored.getCol());
        				}
        			}
        		}
        		}
    		}
    	}
    	System.out.println("Nearest Row:" + nearest.getRow() + " Col:" + nearest.getCol());
    	if(nearest!=null)
    		nearest.setIsVisited(true);
    	return nearest;
    }
    
    public double distance(int row, int col, int target_row, int target_col) {
    	//manhattan
		return Math.abs(row- target_row) + Math.abs(col- target_col); 	
    }
    
    /*
     *check surrounding tile by a 3x3 cell 
     */
    public boolean checkTileSurrounding(int row, int col) {
    	for(int c = col-1; c<=col+1; c++) {
    		for(int r= row-1; r<=row+1; r++) {
    			if(arena[r][c].getIsVirtualWall() || !arena[r][c].getIsExplored() || arena[r][c].getIsObstacle())
    				return false;
    		}
    	}
    	
    	return true;
    }
    /*
     * check surrounding tile by a 3x3 cell
     * -> Valid as long as center is not virtual wall/ obstacle (explored)
     * -> Valid as long as side cells from center is not obstacle (explored)
     * => MEANS valid tile to travel to 
     */
    public boolean checkTileSurroundingLenient(int row, int col){
    	for(int c = col-1; c<=col+1; c++) {
       		if ((c == 20) || (c==-1))
       			break;
    		for(int r= row-1; r<=row+1; r++) {
    			if ((r == 15) || (r==-1))
    				break;
    			if(r==row && c == col) 
    				if(arena[r][c].getIsVirtualWall() || arena[r][c].getIsObstacle() || !arena[r][c].getIsDiscoverble())
    					return false;
    			if(arena[r][c].getIsObstacle() || !arena[r][c].getIsExplored() || !arena[r][c].getIsDiscoverble())
    				return false;
    		}
    	}
    	return true;
    }
      
    public void paintAgain() {
    	this.paintComponent(this.getGraphics());
    }
    
    @Override
    public void paintComponent(Graphics g) {
    	//System.out.println(mapId +" -> Time Entered :"+LocalTime.now());
    	axisInit(g);
		int currentRow = robot.getRow();
		int currentCol = robot.getCol();
				//paint tiles
				for(int y=0;y<MapConstants.MAP_COLS;y++) {
					for(int x=0;x<MapConstants.MAP_ROWS;x++) {
						Color tileCol;
												
						if(!masked) {
							tileCol = Color.WHITE;
			    			if(arena[x][y].getIsObstacle()) {
			    				if(arena[x][y].getIsImage())
			    					tileCol=Color.PINK;
			    				else
			    					tileCol = MapConstants.TILE_OBSTACLE;
			    			}
			    			else if(arena[x][y].getIsWaypoint())
			    				tileCol = MapConstants.TILE_WAYPOINT;
			    			else if(arena[x][y].getIsPath())
			    				tileCol=MapConstants.TILE_PATH;
			    			else if(arena[x][y].getIsVirtualWall())
			    				tileCol = MapConstants.TILE_VIRTUALWALL;
	
						} else {
							//System.out.println("X :"+x+" Y: "+y+" Explored:"+arena[x][y].getIsExplored());
							
			    			if(arena[x][y].getIsWaypoint())
			    				tileCol = MapConstants.TILE_WAYPOINT;
			    			else if(!arena[x][y].getIsExplored())
								tileCol=MapConstants.TILE_UNEXPLORED;
			    			else if(arena[x][y].getHasToBeScanned())
			    				tileCol = Color.RED;
							else if(arena[x][y].getIsObstacle())
								tileCol=MapConstants.TILE_OBSTACLE;
			    			else if(arena[x][y].getIsWaypoint())
			    				tileCol = MapConstants.TILE_WAYPOINT;
							else if(arena[x][y].getIsPath()) //test path
								tileCol=MapConstants.TILE_PATH;
			    			else if(arena[x][y].getIsVirtualWall())
			    				tileCol = MapConstants.TILE_VIRTUALWALL;
			    			else if(!arena[x][y].getIsMoved())
			    				tileCol = Color.GREEN;
			    			else
								tileCol = Color.WHITE;
						}
						
						
						if(arena[x][y].getIsStartZone()||arena[x][y].getIsGoalZone()) 
							tileCol = MapConstants.TILE_START;
						
		    			g.setColor(tileCol);
		    			g.fillRect(arenaTile[x][y].xCoor, arenaTile[x][y].yCoor, arenaTile[x][y].tileSize, arenaTile[x][y].tileSize);
						
		    			//g.drawString(String.valueOf(arena[x][y].getCost()), arenaTile[x][y].xCoor, arenaTile[x][y].yCoor+2);
		    		}

				}
				//paint robot on tile	
				Color tileCol = Color.ORANGE;

				g.setColor(tileCol);

				
		    	for(int y=currentCol-1;y<currentCol+MapConstants.ZONE_SIZE-1;y++) {
		    		for(int x=currentRow-1;x<currentRow+MapConstants.ZONE_SIZE-1;x++) {
		    			g.fillRect(arenaTile[x][y].xCoor, arenaTile[x][y].yCoor, arenaTile[x][y].tileSize, arenaTile[x][y].tileSize);
		    		}
		    	}
		    	
				tileCol = MapConstants.ROBOT_CENTER;		
				g.setColor(tileCol);
				g.fillRect(arenaTile[robot.getRow()][robot.getCol()].xCoor, arenaTile[robot.getRow()][robot.getCol()].yCoor, arenaTile[robot.getRow()][robot.getCol()].tileSize, arenaTile[robot.getRow()][robot.getCol()].tileSize);
		    
				tileCol = MapConstants.ROBOT_HEADING;
				g.setColor(tileCol);
				g.fillRect(arenaTile[robot.getFacingRow()][robot.getFacingCol()].xCoor, 
						arenaTile[robot.getFacingRow()][robot.getFacingCol()].yCoor, 
						arenaTile[robot.getFacingRow()][robot.getFacingCol()].tileSize, 
						arenaTile[robot.getFacingRow()][robot.getFacingCol()].tileSize);
				
	    	//System.out.println(mapId +" -> Time Finished :"+LocalTime.now());
		}

    
    /*   
     * Sets virtual wall on all sides as shown below
    4,4	5,4	6,4
    4,5	5,5	6,5
    4,6	5,6	6,6
    */
    public void setObstacleVirtualWall() throws ArrayIndexOutOfBoundsException {
    	for(int col=0;col<MapConstants.MAP_COLS;col++) {
    		for(int row=0;row<MapConstants.MAP_ROWS;row++) {
    			if(arena[row][col].getIsObstacle())
    			{
    				setTileVirtualWall(row,col+1);
    				setTileVirtualWall(row,col-1);
    				setTileVirtualWall(row-1,col);
    				setTileVirtualWall(row+1,col);
    				setTileVirtualWall(row-1,col-1);
    				setTileVirtualWall(row-1,col+1);
    				setTileVirtualWall(row+1,col+1);
    				setTileVirtualWall(row+1,col-1);
    				
       				this.obstacleBlocks(row,col); //check neighbors
    				}
    			}
    		
    	}
    }
    public ArrayList<Tile> getPossiblePoints(){
    	return this.possiblePoints;
    }
    //for island hugging
    private void obstacleBlocks(int row, int col) throws ArrayIndexOutOfBoundsException{
    	int val = 2;
    	if(row>11 || row<3 || col>16 || col <3) {
    		arena[row][col].cheatCode();
    	}
    	else {
    	if(!this.checkIfSurface(row, col, row, col-1,"NORTH"))
    	{
    		if(checkValid(row,col-val) && !arena[row][col].unknownPoints.get("NORTH")) {
    			this.possiblePoints.add(arena[row][col-val]);
    			arena[row][col-val].setIsDebug(true);
    		}
    	}
    	if(!this.checkIfSurface(row, col, row, col+1, "SOUTH")) {
    		if(checkValid(row,col+val) && !arena[row][col].unknownPoints.get("SOUTH")) {
    			this.possiblePoints.add(arena[row][col+val]);
    			arena[row][col+val].setIsDebug(true);
    		}
    	}
    	if(!this.checkIfSurface(row, col, row-1, col, "WEST")) {
    		if(checkValid(row-val,col) && !arena[row][col].unknownPoints.get("WEST")) {
    			this.possiblePoints.add(arena[row-val][col]);
    			arena[row-val][col].setIsDebug(true);
    		}
    	}
    	if(!this.checkIfSurface(row, col, row+1, col, "EAST")) {
    		if(checkValid(row+val,col) && !arena[row][col].unknownPoints.get("EAST")) {
    			this.possiblePoints.add(arena[row+val][col]);
    			arena[row+val][col].setIsDebug(true);
    		}
    	}
    	}

    	//System.out.println(this.possiblePoints.get(0));
    	/*
    	if(arena[row][col+1].getIsObstacle()) {
    		arena[row][col-1].unknownPoints.replace("SOUTH", true);
    	}

    	if(arena[row-1][col].getIsObstacle()) {
    		arena[row][col-1].unknownPoints.replace("WEST", true);
    	}

    	if(arena[row+1][col-1].getIsObstacle()) {
    		arena[row][col-1].unknownPoints.replace("EAST", true);
    	}
		*/
    }
    private boolean checkIfSurface(int oriRow, int oriCol, int row, int col,String dir){
    	try {
    	if(arena[row][col].getIsObstacle()) {
    		arena[oriRow][oriCol].unknownPoints.replace(dir, true);
    		return true;
    	}
    	return false;
    	}
    	catch(ArrayIndexOutOfBoundsException e){
    		return false;
    	}
    }

	private void setTileVirtualWall(int row, int col) {
		// TODO Auto-generated method stub
    	try {
    		arena[row][col].setIsVirtualWall(true);
    	}catch(ArrayIndexOutOfBoundsException e) {
    		
    	}
	}
	
	private void axisInit(Graphics g) {
    	g.drawString("0", 5, 35);
    	g.drawString("1", 5, 55);
    	g.drawString("2", 5, 75);
    	g.drawString("3", 5, 95);
    	g.drawString("4", 5, 115);
    	g.drawString("5", 5, 135);
    	g.drawString("6", 5, 155);
    	g.drawString("7", 5, 175);
    	g.drawString("8", 5, 195);
    	g.drawString("9", 5, 215);
    	g.drawString("10", 5, 235);
    	g.drawString("11", 5, 255);
    	g.drawString("12", 5, 275);
    	g.drawString("13", 5, 295);
    	g.drawString("14", 5, 315);
    	g.drawString("15", 5, 335);
    	g.drawString("16", 5, 355);
    	g.drawString("17", 5, 375);
    	g.drawString("18", 5, 395);
    	g.drawString("19", 5, 415);
    	
    	g.drawString("0", 25, 15);
    	g.drawString("1", 45, 15);
    	g.drawString("2", 65, 15);
    	g.drawString("3", 86, 15);
    	g.drawString("4", 105, 15);
    	g.drawString("5", 125, 15);
    	g.drawString("6", 145, 15);
    	g.drawString("7", 165, 15);
    	g.drawString("8", 185, 15);
    	g.drawString("9", 205, 15);
    	g.drawString("10", 225, 15);
    	g.drawString("11", 245, 15);
    	g.drawString("12", 265, 15);
    	g.drawString("13", 285, 15);
    	g.drawString("14", 305, 15);
    }
    
	public void setRobot(Robot r) {
		this.robot = r;
	}

	public void setRobotMoved(int row, int col) {
		// TODO Auto-generated method stub
		for(int r=-1;r<=1;r++) {
			for(int c=-1;c<=1;c++) {
				if(!arena[row+r][col+c].getIsMoved())
					arena[row+r][col+c].setIsMoved(true);
			}
		}
	}
	public void setToBeExplored() {
//    	for(int col=MapConstants.MAP_COLS-4;col>=3;col--) {
//    		for(int row=3;row<MapConstants.MAP_ROWS-2;row++) {
//    			if(arena[row][col].getIsObstacle() && arena[row][col].getHasToBeScanned())
//    			{
//    				this.lefToBeExplored.add(arena[row][col]);
//    			}
//    		}
//    	}
    	
    	for(int row=3;row<MapConstants.MAP_ROWS-3;row++) {
    		for(int col=MapConstants.MAP_COLS-3;col>=3;col--) {
    			if(arena[row][col].getIsObstacle() && arena[row][col].getHasToBeScanned())
    			{
    				this.lefToBeExplored.add(arena[row][col]);
    			}
    		}
    	}
	}
	private boolean checkIfValidCP(Tile temp) {
		int x_coor=temp.getRow();
		int y_coor= temp.getCol();
		if(arena[x_coor-1][y_coor].getIsObstacle() || arena[x_coor+1][y_coor].getIsObstacle())
			return true;
		if(arena[x_coor-1][y_coor-1].getIsObstacle() || arena[x_coor+1][y_coor-1].getIsObstacle() ||
				arena[x_coor-1][y_coor+1].getIsObstacle() || arena[x_coor+1][y_coor+1].getIsObstacle())
			return false;
		return true;
	}
	public ArrayList<ArrayList<Tile>> getToBeExplored(){
		ArrayList<ArrayList<Tile>> outer = new ArrayList<ArrayList<Tile>>();
		ArrayList <Tile> inner = new ArrayList<Tile>();
		
		/*Cluster Algorithm
		 * For each point, check against any existing cluter's element for distance, add into cluster iff distance for any given point is <=2 else create new cluster
		 */
		
		for(int i =0; i<lefToBeExplored.size();i++) {
			Tile ref = lefToBeExplored.get(i);
			System.out.println(ref.printLoc());
			if(!checkIfValidCP(ref))
				continue;
			if(inner.size()==0) {
				inner.add(ref);
				outer.add(inner);
				continue;
			}
			int outerIndex;
			if((outerIndex=this.checkAgainstClusters(ref,outer))!=-1) {
				System.out.println("Cluster found " +outerIndex);
				outer.get(outerIndex).add(ref);
			}
			else {
				System.out.println("Cluster not found");
				inner = new ArrayList<Tile>();
				inner.add(ref);
				outer.add(inner);
			}
			
			
		}
//		if(lefToBeExplored.size()!=0)
//			inner.add(lefToBeExplored.get(0));
//		int counter=0;
//		for(int i =1; i<lefToBeExplored.size();i++) {
//			System.out.println(lefToBeExplored.get(i).printLoc());
//			if(inner.size()==0) {
//				inner.add(lefToBeExplored.get(i));
//				continue;
//			}
//			if(distance(inner.get(counter),lefToBeExplored.get(i))<=2) {
//				inner.add(lefToBeExplored.get(i));
//				counter++;
//			}
//			else {
//				outer.add(inner);
//				inner = new ArrayList<Tile>();
//				counter=0;
//			}
//		}
//		if(inner.size()!=0)
//			outer.add(inner);
		System.out.println("out");
		return outer;
	}
	private int checkAgainstClusters(Tile ref, ArrayList<ArrayList<Tile>> outer) {
		// TODO Auto-generated method stub
		int index=-1;
		for(int i=0; i<outer.size();i++) {
			for(int y=0; y<outer.get(i).size();y++) {
				Tile clusterRef = outer.get(i).get(y);
				if(distance(ref, clusterRef)<=4)
					index=i;
			}
		}
		return index;
		
		
	}

	private double distance(Tile temp, Tile goal) {
		// TODO Auto-generated method stub
		return Math.abs(temp.getRow()- goal.getRow()) + Math.abs(temp.getCol()- goal.getCol()); 	
		//return Math.sqrt(Math.abs(((this.end_row-row)^2 + (this.end_col-col)^2)));
	}
    
}
