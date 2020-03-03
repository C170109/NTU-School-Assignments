package Image_Algorithms;

import java.io.IOException;
import java.time.LocalTime;
import java.util.ArrayList;

import javax.swing.JOptionPane;
import javax.swing.SwingWorker;

import Main.SimulatorMDP;
import Map.Map;
import Map.MapConstants;
import Map.Tile;
import Robot.Robot;
import Robot.Sensor;
import Robot.RobotConstants;
import Robot.RobotConstants.DIRECTION;
import Utils.ConnectionManager;
import Utils.MapDescriptor;

public class Image_ExplorationAlgo {
	
	/*
	 * 1 second = 1000ms.
	 * 1 minute = 60*1000ms.
	 * 6 minutes = 6 * 60 * 1000ms.
	 */
	private boolean continueRun=true;
	private long startTime;
	private  long endTime;
	private long timeLimit;
	private  int coverageLimit;
	private int numSteps;
	private int wayX, wayY;
	private static final double one_second = 1000;
	private final int maxCount = 10;
	private String[] imageDetails = null;
	private Map real_map;
	private Map explored_map;
	private FastestPathAlgo fpa;
	private int cur_row, cur_col;
	private int cur_front;
	private ArrayList<String> imageList = new ArrayList<String>();
	private ArrayList<String> imageIdsList = new ArrayList<String>();
	private String imgToSendAnd = "";
	private Robot robot;
	private  int exploredArea=0;
	private ArrayList<Tile> unexploredPath = new ArrayList<Tile>();
	private int LoopCount = 1;
	private int debug_photoCmdCount=0;
	private int moveCount =0;
	//change to user selected
	private long ROBOT_SPEED=10; 
	public displayManagement displayMgt = null;
	private boolean processImage;
	public Image_ExplorationAlgo(Map real_map, Map explored_map, Robot robot, long timeLimit, int coverageLimit, int numSteps,boolean processImage) {
		this.real_map = real_map;
		this.explored_map = explored_map;
		this.robot = robot;
		this.timeLimit = (long) (timeLimit * one_second);
		this.numSteps = numSteps;
		this.coverageLimit = coverageLimit;
		//if processImage is false, then skip all checks for image.
		this.processImage=processImage;
		//this.fpa = new FastestPathAlgo(explored_map,robot,null);

		ROBOT_SPEED = (long) (one_second/numSteps);
		
		//ensure no negative or over coverage limit
		if(coverageLimit<=0 || coverageLimit>300)
			this.coverageLimit = 300;
		//ensure no negative or over 6 mins requirement
		if(timeLimit<=0 || timeLimit >360)
			this.timeLimit = (long) (360*one_second);
	}
	
	/*
	 * To update main display on the simulator
	 */
    public class displayManagement extends SwingWorker<Integer,Void>{
		protected Integer doInBackground() throws Exception{
		    startTime = System.currentTimeMillis();
		    endTime = startTime + timeLimit; 
			while(true) {
				//stop thread condition
				if(processImage) {
					if(System.currentTimeMillis()>=endTime || displayMgt.isCancelled()) {//|| exploredArea>=coverageLimit) {
						displayMgt.cancel(true);
						//Main.SimulatorMDP.exploreWorker.cancel(true);
						break;
					}
				}
				else {
					if(System.currentTimeMillis()>=endTime || displayMgt.isCancelled() || exploredArea>=coverageLimit) {//|| exploredArea>=coverageLimit) {
						displayMgt.cancel(true);
						//Main.SimulatorMDP.exploreWorker.cancel(true);
						break;
					}
				}
				exploredArea = calculateAreaExplored();
				//System.out.println("DEBUG :: Time ->>> "+((endTime - System.currentTimeMillis())/one_second));
				Main.SimulatorMDP.lblTimePassed.setText("Time Left :"+((endTime - System.currentTimeMillis())/one_second)+"s");//;("Time left :"+(endTime - System.currentTimeMillis()) * one_second);
				Main.SimulatorMDP.lblAreaExplored.setText("Area Explored:"+ exploredArea + "/"+ 300);
				Main.SimulatorMDP.lblRobotPos.setText(robot.printLoc());
			}
			continueRun = false;
			return 111;
		}
    }
    
	public void explore() {
		this.continueRun=true;
	    System.out.println("Starting exploration...");
	    startExploration();
	}
	/*
	 * 1. Get sensor information
	 * 2. Update information on map
	 * 2.1 Update MDF  
	 * 3. Do algorithm
	 * 3.1 If processImage is true, proceed to do image checking
	 * 4. Send movement to Arduino + coordinates and MDF string to Android
	 * 5. Repeat until reach start zone.
	 * 6. If start zone reach and exploration not fully done, proceed search algorith,.
	 */
	public void startExploration() {

	    //this.updateMain();
	    //starts seperate thread for time display management
	    displayMgt = new displayManagement();
	    displayMgt.execute();
	    /*
		 * Exit conditions
		 * 1. Coverage Limit reached
		 * 2. Time Limit reached
		 * 3. Reach back at start (basic right wall will always reach starting point (assuming))
		 */
	    
	    //initial sensor processing
	    this.processSensor(false);
	    //explored_map.paintAgain();
	    do {
			this.rightWallHug(true,null);
		}while(continueRun && (!this.checkIfStart())); //exploredArea<=coverageLimit && System.currentTimeMillis() <=endTime && !this.checkIfStart());
		//System.out.println("Sending F to ARD");
		//set up virtual wall before fpa
		explored_map.setObstacleVirtualWall();
		
		System.out.println("Current explored area :"+exploredArea);

		if(!this.checkIfStart()) {
			//remember to turn it back on
			this.goBackToStartFollowRight();
		}
		else {
			/*
			 * To find if got unexplored area
			 */
			int maxCount = this.maxCount;
			if(this.fpa==null)
				fpa = new FastestPathAlgo(explored_map,robot,null);
			
			//do FPA to path and back to start (bursting) once exploredArea = 300
			/*
			 * Search algorithm to achieve full exploration
			 * General Idea: 
			 * -> Find the nearest unexplored tile
			 * -> Find the closest explorable/ valid tile from it
			 * -> Do A* search algorithm (pathing) to it
			 * -> Move Arduino
			 * -> Repeat until no more valid unexplored tile left
			 */
			while(this.exploredArea!=300) {
				if(SimulatorMDP.exploreWorker.isCancelled()) {
					System.out.println("Exploration is cancelled");
					displayMgt.cancel(true);
					return;
				}
				
				if(!continueRun)
					break;
				//maxCount--;
				Tile unexplored_temp = explored_map.getNextNotExplored(robot.getRow(), robot.getCol());
				if(unexplored_temp!=null) {
					unexploredPath.add(unexplored_temp);
					Tile temp=explored_map.getNearestExplored(unexplored_temp, robot.getRow(), robot.getCol());
					System.out.println("Tile to go: " + temp.getRow() + "," + temp.getCol());
					ArrayList<Tile> tempPath = this.fpa.findFastestPath(robot.getRow(), robot.getCol(), temp.getRow(), temp.getCol(), robot.getCurrentHeading());
					
					if (tempPath == null || tempPath.size()==0 ||
							maxCount==0) {
						//this.goBackToStartFollowRight();
						break;
					} else {
						Tile start = explored_map.getTile(robot.getRow(), robot.getCol());
						ArrayList<RobotConstants.DIRECTION> list_direction =this.moveFPA(tempPath,start,robot.getCurrentHeading());	
						if(list_direction!=null)
							this.moveWithList(list_direction,false);
						
						fpa.printGMap();
						
						if(explored_map.getTile(unexplored_temp.getRow(), unexplored_temp.getCol()).getIsExplored())
							unexploredPath.remove(explored_map.getTile(unexplored_temp.getRow(), unexplored_temp.getCol()));
						else
							explored_map.getTile(unexplored_temp.getRow(), unexplored_temp.getCol()).setIsDiscoverble(false);
					}
					//fpa.setMap(explored_map);
					explored_map.setObstacleVirtualWall();
					explored_map.paintAgain();
					//this.updateMain();
				
				}
				else {
					int remainingCoverage = MapConstants.MAP_ROWS * MapConstants.MAP_COLS -this.exploredArea;
					if(remainingCoverage-unexploredPath.size()==0)
						break;
				}
				if(unexplored_temp==null)
				break;
			}
			System.out.println("Finding path back to start full explore");
			if(!this.checkIfStart()) {
				this.goBackToStartFollowRight();
			}
			else {
				while(this.robot.getCurrentHeading() != RobotConstants.HEADING.NORTH)
				this.turnRobot('R',true);
			}			
		}
		displayMgt.cancel(true);
		System.out.println("Command Count : "+this.debug_photoCmdCount);
		System.out.println("Possible points :"+this.explored_map.getPossiblePoints().size());
    	for(int c =MapConstants.MAP_COLS-1; c>=0;c--) {
    		for(int r =0 ; r< MapConstants.MAP_ROWS;r++) {
    			Tile temp = explored_map.getTile(r,c);
    			if(temp.getIsObstacle())
    				temp.checkAll4();
    		}
    	}
    	/*
    	 * 'Island' Hugging algorithm. FOR IR. Did not use
    	 * Basic Idea: Find clusters of obstacles in the center of the arena
    	 * Perform Right-Wall hug on all clusters
    	 */
    	/*
    	continueRun=true;
    	if(continueRun) {
    	System.out.println("OUT");
    	explored_map.paintAgain();
    	explored_map.setToBeExplored();
    	ArrayList<ArrayList<Tile>> tempList =explored_map.getToBeExplored(); 
    	for(int i =0; i<tempList.size();i++) {
    		System.out.println("ISLAND :"+i);
    		System.out.println("SIZE OF NEXT LOOP :"+tempList.get(i).size());
    		for(int y =0; y<tempList.get(i).size();y++) {
    			System.out.println(tempList.get(i).get(y).printLoc());
    		}
    		System.out.println("==========================================================");
    	}
		if(this.fpa==null)
			fpa = new FastestPathAlgo(explored_map,robot,null);
		
    	for(int i=0;i<tempList.size();i++) {
//    		if(tempList.get(i).size()<=1)
//    			continue;
			if(this.fpa==null)
				fpa = new FastestPathAlgo(explored_map,robot,null);
			
			Tile temp=explored_map.getNearestIslandPoint(tempList.get(i).get(0), robot.getRow(), robot.getCol());
			System.out.println("Tile to go: " + temp.getRow() + "," + temp.getCol());
			ArrayList<Tile> tempPath = this.fpa.findFastestPath(robot.getRow(), robot.getCol(), temp.getRow(), temp.getCol(), robot.getCurrentHeading());
			
			if (tempPath == null || tempPath.size()==0 ||
					maxCount==0) {
				//this.goBackToStartFollowRight();
				break;
			} else {
				Tile start = explored_map.getTile(robot.getRow(), robot.getCol());
				ArrayList<RobotConstants.DIRECTION> list_direction =this.moveFPA(tempPath,start,robot.getCurrentHeading());	
				if(list_direction!=null) {
					this.moveWithList(list_direction,false);
					
					//reach the point
					this.makeMeTurnCorrect(tempList.get(i).get(0));
//					if(this.checkFront()!=-1) {
//						this.turnRobot('L',false);
//						if(this.checkRObs() == 2) {
//							this.sendTakePhotoCmd();
//						}
//						if(this.checkFront()==-1)
//							this.moveRobot(false);
//					}
					
					while(continueRun) {
						this.rightWallHug(false,temp);//explored_map.getTile(robot.getRow(), robot.getCol()));
						if(robot.getRow() == temp.getRow() && robot.getCol()==temp.getCol())
							break;
						
						
					}
					if(!continueRun)
						break;
					//Break point
					//if(cur_row == ref.getRow() && cur_col==ref.getCol())
						//return;
					//}
					
					//this.rightWallHug(false,explored_map.getTile(robot.getRow(), robot.getCol()));
				}
    	
			}
    	}
    	
    	if(!this.checkIfStart())
    		this.goBackToStartFollowRight();
    	
    	displayMgt.cancel(true);
    	}
    	*/
    	
	}
	
	/*
	 * For IR island-hugging.
	 * Turns the robot to face correctly in order to perform right-wall hugging
	 */
	private void makeMeTurnCorrect(Tile ref) {
		// TODO Auto-generated method stub
		RobotConstants.HEADING h = robot.getCurrentHeading();
		switch(h) {
		case NORTH:
			if(robot.getRow()==ref.getRow()) {
				if(robot.getCol()>ref.getCol()) {
					this.turnRobot('L', false);
					if(this.checkFront()==-1)
						this.moveRobot(false);
				}
				
			}
			if(robot.getRow()>ref.getRow()) {
				if(robot.getCol()>=ref.getCol()) {
					this.turnRobot('L',false);
					this.turnRobot('L',false);
					
				}
				else {
				this.turnRobot('L', false);
				if(this.checkFront()==-1)
					this.moveRobot(false);
				}
			}
			else {
				if(this.checkR()!=-1)
					break;
				if(this.checkR()==-1) {
					while(this.checkR()==-1) {
						if(this.checkFront()==-1)
							moveRobot(false);
						else break;
					}
				}
				else {
				this.turnRobot('L', false);
				if(this.checkFront()==-1)
					this.moveRobot(false);
				}
			}
			break;
		case SOUTH:
			if(robot.getRow()<ref.getRow()) {
				this.turnRobot('L',false);
				this.turnRobot('L',false);
			}
			break;
		case EAST:
			if(robot.getCol()>=ref.getCol()) {
				this.turnRobot('L',false);
				this.turnRobot('L',false);
			}
			//else if(robot.getCol()<ref.getCol())
			break;
		case WEST:
			if(robot.getRow()<=ref.getRow()) {
				if(robot.getCol()<ref.getCol()) {
				this.turnRobot('L',false);
				this.turnRobot('L',false);
				}
			}
			break;
		}
		
	}

	private void stopRun() {
		// TODO Auto-generated method stub
		System.out.println("Exploration is cancelled");
		displayMgt.cancel(true);
		explored_map.setObstacleVirtualWall();
		explored_map.paintAgain();
	}

	/*
	 * ignoreSensors control the need to burst or just move regularly while updating the map
	 */
	private void moveWithList(ArrayList<DIRECTION> list_direction, boolean ignoreSensor) {
		// TODO Auto-generated method stub
		if(!ignoreSensor) {
		for(RobotConstants.DIRECTION e : list_direction) {
			switch(e) {
			case FRONT:
				this.moveRobot(ignoreSensor);//robot.moveRobot(robot.getCurrentHeading(),explored_map);
				break;
			case LEFT:
				this.turnRobot('L',ignoreSensor);
				//robot.turningRobot('L', explored_map);
				break;
			case RIGHT:
				this.turnRobot('R',ignoreSensor);
				//robot.turningRobot('R', explored_map);
				break;
			}
		}
		}
		else {
			for(RobotConstants.DIRECTION e : list_direction) {
				switch(e) {
				case FRONT:
					robot.moveRobot(robot.getCurrentHeading(),explored_map);
					//this.moveRobot(ignoreSensor);//robot.moveRobot(robot.getCurrentHeading(),explored_map);
					break;
				case LEFT:
					//this.turnRobot('L',ignoreSensor);
					robot.turningRobot('L', explored_map);
					break;
				case RIGHT:
					//this.turnRobot('R',ignoreSensor);
					robot.turningRobot('R', explored_map);
					break;
				}
				//this.explored_map.paintAgain();
			}
		}
	}

	/*
	 * Find fastest path back to start using A* search
	 */
	public void goBackToStartFollowRight() {
		System.out.println("Going back following path");
		if(this.fpa==null)
			fpa = new FastestPathAlgo(explored_map,robot,null);
		
		fpa.resetCost();
		ArrayList<Tile> pathToStart = fpa.findFastestPath(robot.getRow(), robot.getCol(), RobotConstants.START_POS_X, RobotConstants.START_POS_Y, robot.getCurrentHeading());
		//this.moveFPA(pathToStart, false,true);
		Tile start = explored_map.getTile(robot.getRow(), robot.getCol());
		ArrayList<RobotConstants.DIRECTION> list_direction =this.moveFPA(pathToStart,start,robot.getCurrentHeading());	
		if(list_direction!=null) {
			String movementString = this.getMovementString(list_direction);
			if(robot.getRealRun()) {
				ConnectionManager.getConnMgr().sendMessage(movementString, "ARD");	
				try {
					Thread.sleep(1000);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				//ConnectionManager.getConnMgr().sendMessage(movementString, "AND");
			}
			this.moveWithList(list_direction,true);
			String mdfString = this.updateMap();
			//ConnectionManager.getConnMgr().sendMessage(mdfString+":"+robot.getRow()+":"+(Math.abs(robot.getCol()-MapConstants.MAP_COLS)-1)+":"+"N", "AND");
			if(this.imgToSendAnd=="")
				ConnectionManager.getConnMgr().sendMessage(mdfString+":"+robot.getRow()+":"+(Math.abs(robot.getCol()-MapConstants.MAP_COLS)-1)+":"+"N", "AND");
			else
				ConnectionManager.getConnMgr().sendMessage(mdfString+":"+robot.getRow()+":"+(Math.abs(robot.getCol()-MapConstants.MAP_COLS)-1)+":"+"N"+":"+this.imgToSendAnd, "AND");
			
		}
		//delay to prevent message concatenation
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		while(true) {
			this.turnRobot('R',true);
			if(robot.getCurrentHeading() == RobotConstants.HEADING.NORTH)
				break;
		}
//		displayMgt.cancel(true);
//		System.out.println("Cancel");
		fpa.printGMap();
	}
	
	/*
	 * Algorithm for fastest path
	 * General Idea:
	 * -> If waypoint is set, find path to waypoint first.
	 * -> Find path to goal zone from current point.
	 * -> Concatenate both path (if exist).
	 * -> Generate movement string from list of paths.
	 * -> Send to Ard and And.
	 */
	public void startFastestPathAlgo() {
		System.out.println("DEBUG :: Finding Path...");
		fpa.resetCost();
		int end_row = RobotConstants.END_POS_X;
		int end_col = RobotConstants.END_POS_Y;
		int start_row = RobotConstants.START_POS_X;
		int start_col = RobotConstants.START_POS_Y;
		String movementString="";
		this.robot.setRow(start_row);
		this.robot.setCol(start_col);
		this.robot.setHeading(RobotConstants.HEADING.NORTH);
		explored_map.paintAgain();
    	robot = new Robot(RobotConstants.START_POS_X, RobotConstants.START_POS_Y, robot.getRealRun(), 0);
		explored_map.setRobot(robot);
		RobotConstants.HEADING curH = robot.getCurrentHeading();
		Tile waypoint = explored_map.getWaypoint();
		FastestPathAlgo fpa = new FastestPathAlgo(explored_map,robot,waypoint);
		fpa.setPathNotTrue();
		if(this.exploredArea == 300) {
			end_row = RobotConstants.END_POS_X;
			end_col = RobotConstants.END_POS_Y;
		}
		ArrayList<RobotConstants.DIRECTION> list_direction;
		if(waypoint==null) {
			Tile start = explored_map.getTile(start_row, start_col);
			ArrayList <Tile> path = fpa.findFastestPath(start_row, start_col, end_row, end_col, curH);
			list_direction =this.moveFPA(path,start,curH);	

		}else {	
			ArrayList<Tile> pathtowp = fpa.findFastestPath(start_row, start_col, waypoint.getRow(), waypoint.getCol(),curH);
			fpa.printGMap();
			//moveFPA(pathtowp,true,false);
			Tile start = explored_map.getTile(start_row, start_col);
			Tile temp = pathtowp.get(pathtowp.size()-2);
			RobotConstants.HEADING tempH = fpa.getNextHeading(temp, waypoint.getRow(), waypoint.getCol(), curH);
			list_direction =this.moveFPA(pathtowp,start, curH);	
			ArrayList<Tile> pathtogoal = fpa.findFastestPath(waypoint.getRow(), waypoint.getCol(), end_row, end_col, tempH);//robot.getCurrentHeading());
			//moveFPA(pathtogoal,true,false);
			ArrayList<RobotConstants.DIRECTION> list_direction2 =this.moveFPA(pathtogoal,waypoint,tempH);	
			list_direction.addAll(list_direction2);
		}
		fpa.printGMap();
		
		if(robot.getRealRun()) {
			while(true) {
				System.out.println("WAITING FOR FP FROM AND");
				String msg = ConnectionManager.getConnMgr().receiveMessage();
				if(msg!=null) {
					if(msg.equalsIgnoreCase("FP"))
						break;
				}
			}
		}
		else {
			System.out.println("FP Received");
		}
		

		if(list_direction!=null) {
			movementString = this.getMovementString(list_direction);
			if(robot.getRealRun()) {
				ConnectionManager.getConnMgr().sendMessage(movementString, "ARD");
				try {
					Thread.sleep(1000);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				ConnectionManager.getConnMgr().sendMessage(movementString, "AND");
			}
		}
		this.moveWithList(list_direction,false);
	}
	
	/*
	 * To generate movement string for fastest path/ bursting
	 * F, 0-9 : Number of forward movements 
	 * R : Right Turn
	 * L : Left Turn
	 */
	private String getMovementString(ArrayList<DIRECTION> list_direction) {
		// TODO Auto-generated method stub
		StringBuilder sb = new StringBuilder();
		int front_movement =-1;
		for(RobotConstants.DIRECTION e : list_direction) {
			switch(e) {
			case FRONT:
				front_movement++;
				//sb.append("F");
				break;
			case LEFT:
				if(front_movement==0)
					sb.append("F");
				else if(front_movement>9) {
					sb.append(9);
					sb.append((front_movement-10)==0 ? "F" : (front_movement-10));
				}
				else if(front_movement!=-1)
					sb.append(front_movement);

				sb.append("L");
				front_movement=-1;
				break;
			case RIGHT:
				if(front_movement==0)
					sb.append("F");
				else if(front_movement>9) {
					sb.append(9);
					sb.append((front_movement-10)==0 ? "F" : (front_movement-10));
				}
				else if(front_movement!=-1)
					sb.append(front_movement);

				sb.append("R");
				front_movement=-1;
				break;
			}
		}
		if(front_movement==0)
			sb.append("F");
		else if(front_movement>9) {
			sb.append(9);
			sb.append((front_movement-10)==0 ? "F" : (front_movement-10));
		}
		else if(front_movement!=-1)
			sb.append(front_movement);
		System.out.println("DEBUG :: Movement String -> "+sb.toString());
		return sb.toString();
	}
	
	
	/*
	 * For generation of "fake" sensor string values generated from a given arena map.
	 */
	private String readSimulatorMap() {
		// TODO Auto-generated method stub
		StringBuilder sb = new StringBuilder();
		//ArrayList <Sensor> sensors = robot.getSensors();
		if(real_map!=null) {
			sb.append("LLR:");
			sb.append(robot.LLR.senseReal(real_map));
			sb.append(":LSR:");
			sb.append(robot.LSR.senseReal(real_map));
			sb.append(":FSR1:");
			sb.append(robot.FSR1.senseReal(real_map));
			sb.append(":FSR2:");
			sb.append(robot.FSR2.senseReal(real_map));
			sb.append(":FSR3:");
			sb.append(robot.FSR3.senseReal(real_map));
			sb.append(":RSR:");
			sb.append(robot.RSR.senseReal(real_map));
			sb.append(":");
			if(robot.FSR1.senseImage(real_map)!=-1)
				sb.append("1:2");
			else if(robot.FSR2.senseImage(real_map)!=-1)
				sb.append("2:2");
			else if(robot.FSR3.senseImage(real_map)!=-1)
				sb.append("3:3");
			else
				sb.append("-1:-1");
			//System.out.println(sb.toString());
			
		}
		
		return sb.toString();
	}

	/*
	 * Check if start
	 */
	private boolean checkIfStart() {
    	if(robot.getRow()==robot.getStartPosX() && robot.getCol() == robot.getStartPosY())
    		return true;
    	return false;
	}
	
	/*
	 * Checks for area explored
	 */
    private int calculateAreaExplored() {
        int result = 0;
        for (int r = 0; r < MapConstants.MAP_ROWS; r++)
            for (int c = 0; c < MapConstants.MAP_COLS; c++)
                if (explored_map.getTile(r, c).getIsExplored())
                    result++;
        return result;
    }
    
    /*
     * Generates MDF String
     */
	public String updateMap() {
		//System.out.println("Still here");
		String[] mdp = MapDescriptor.getMapDescriptor().generateHexadecimalEqvl(this.explored_map);
		String mdfString = mdp[0]+":"+mdp[1];
		//System.out.println(mdfString);
		return  mdfString;
	}
	
	/*
	 * Check right to see if should turn right.
	 * Conditions:
	 * -> Turn right if right has unexplored tile/ free of obstacles.
	 */
	public int checkR() {
		switch(robot.getCurrentHeading()) {
		case NORTH:
			return robot.checkRight(explored_map) == -1 && checkSurrounding(cur_row+2,cur_col) ==-1 && checkSurrounding(cur_row+2,cur_col+1) ==-1 ? -1 : 1;
		case WEST:
			return robot.checkRight(explored_map) == -1 && checkSurrounding(cur_row+1,cur_col-2) ==-1 && checkSurrounding(cur_row,cur_col-2) ==-1 ? -1 : 1;
		case EAST:
			return robot.checkRight(explored_map) == -1 && checkSurrounding(cur_row,cur_col+2) ==-1 && checkSurrounding(cur_row-1,cur_col+2) ==-1 ? -1 : 1;
		case SOUTH:
			return robot.checkRight(explored_map) == -1 && checkSurrounding(cur_row-2,cur_col) ==-1 && checkSurrounding(cur_row-2,cur_col-1) ==-1 ? -1 : 1;	
		default:
			break;
		}
		return 1;
	}
	
	/*
	 * Checks right if can calibrate
	 * Returns -1 when none of the function detects obstacle/ Wall
	 * Requires 3 lined obstacle/ wall for calibration
	 */
	public int checkRCalibrate() {
		switch(robot.getCurrentHeading()) {
		case NORTH:
			return robot.checkRight(explored_map) != -1 && (checkSurrounding(cur_row+2,cur_col) !=-1 ) && (checkSurrounding(cur_row+2,cur_col+1) !=-1 ) ? -1 : 1;
		case WEST:
			return robot.checkRight(explored_map) != -1 && (checkSurrounding(cur_row+1,cur_col-2) !=-1) && checkSurrounding(cur_row,cur_col-2) !=-1 ? -1 : 1;
		case EAST:
			return robot.checkRight(explored_map) != -1 && checkSurrounding(cur_row,cur_col+2) != -1 && checkSurrounding(cur_row-1,cur_col+2) != -1 ? -1 : 1;
		case SOUTH:
			return robot.checkRight(explored_map) != -1 && checkSurrounding(cur_row-2,cur_col) != -1 && checkSurrounding(cur_row-2,cur_col-1) != -1 ? -1 : 1;	
		default:
			break;
		}
		return 1;
	}
	/*
	 * Checks for obstacles only on right only (returns 2)
	 */
	public int checkRObs() {
		int distance_center=0;
		switch(robot.getCurrentHeading()) {
		case NORTH:
			return this.checkSurrounding(cur_row+2, cur_col+1);//robot.checkRight(explored_map) != -1 && (checkSurrounding(cur_row+2,cur_col) !=-1 ) && (checkSurrounding(cur_row+2,cur_col+1) !=-1 ) ? -1 : 1;
		case WEST:
			return this.checkSurrounding(cur_row+1, cur_col-2);//obot.checkRight(explored_map) != -1 && (checkSurrounding(cur_row+1,cur_col-2) !=-1) && checkSurrounding(cur_row,cur_col-2) !=-1 ? -1 : 1;
		case EAST:
			return this.checkSurrounding(cur_row-1, cur_col+2);  //robot.checkRight(explored_map) != -1 && checkSurrounding(cur_row,cur_col+2) != -1 && checkSurrounding(cur_row-1,cur_col+2) != -1 ? -1 : 1;
		case SOUTH:
			return this.checkSurrounding(cur_row-2,  cur_col-1);//robot.checkRight(explored_map) != -1 && checkSurrounding(cur_row-2,cur_col) != -1 && checkSurrounding(cur_row-2,cur_col-1) != -1 ? -1 : 1;	
		default:
			break;
		}
		return 1;
	}
	
	/*
	 * Same as Right but on left
	 */
	public int checkL() {
		switch(robot.getCurrentHeading()) {
		case NORTH:
			return robot.checkLeft(explored_map) == -1 && checkSurrounding(cur_row-2,cur_col) ==-1 && checkSurrounding(cur_row-2,cur_col+1) ==-1 ? -1 : 1;
		case WEST:
			return robot.checkLeft(explored_map) == -1 && checkSurrounding(cur_row,cur_col+2) ==-1 && checkSurrounding(cur_row+1,cur_col+2) ==-1 ? -1 : 1;
		case EAST:
			return robot.checkLeft(explored_map) == -1 && checkSurrounding(cur_row,cur_col-2) ==-1 && checkSurrounding(cur_row-1,cur_col-2) ==-1 ? -1 : 1;
		case SOUTH:
			return robot.checkLeft(explored_map) == -1 && checkSurrounding(cur_row+2,cur_col) ==-1 && checkSurrounding(cur_row+2,cur_col-1) ==-1 ? -1 : 1;	
		default:
			break;
		}
		return 1;
	}
	
	/*
	 * Checks if all 3 front sensor detects free/ obstacle
	 */
	public int checkFront() {
		//return checkTrap(robot.checkFront(explored_map));
		return robot.checkFront(explored_map); //checkTrap(robot.checkFront(explored_map));
	}
	
	/*
	 * If returned value is 1 or 2, then it is valid for calibration
	 * 1 => Wall
	 * 2 => Obstacle only
	 */
	public int checkSurrounding(int row, int col){
		try {
		Tile tile = explored_map.getTile(row,col);
		//System.out.println(row+" "+col);
		if(!tile.getIsExplored())
			return -1;
		else if(tile.getIsExplored() && !tile.getIsObstacle())
			return -1;
		else
			return 2;
		}
		catch(ArrayIndexOutOfBoundsException e)
		{
			return 1;
		}
	}
	public void moveRobot(boolean ignoreSensor) {
		//map descriptor
		String mdfString = this.updateMap();
		//ConnectionManager.getConnMgr().sendMessage(mdfString, "AND");
		
		if(!robot.getRealRun()) {
		try {
			Thread.sleep(ROBOT_SPEED);
		}catch(InterruptedException ex) {
			Thread.currentThread().interrupt();
		}		
		//this.updateMain();
		}
		System.out.println("Sending F to ARD");
		
		//String msg = "";
		if(robot.getRealRun()) {
			ConnectionManager.getConnMgr().sendMessage("F", "ARD");
			//ConnectionManager.getConnMgr().sendMessage(mdfString+":"+"F", "AND");
			int tempRow = robot.getRow();
			int tempCol = robot.getCol();
			RobotConstants.HEADING tempH = robot.getCurrentHeading();
			if(this.imgToSendAnd=="")
				ConnectionManager.getConnMgr().sendMessage(mdfString+":"+tempRow+":"+(Math.abs(tempCol-MapConstants.MAP_COLS)-1)+":"+tempH.toString().charAt(0), "AND");
			else
				ConnectionManager.getConnMgr().sendMessage(mdfString+":"+tempRow+":"+(Math.abs(tempCol-MapConstants.MAP_COLS)-1)+":"+tempH.toString().charAt(0)+":"+this.imgToSendAnd, "AND");
			
		}
		robot.moveRobot(robot.getCurrentHeading(),explored_map);
		this.cur_row = robot.getRow();
		this.cur_col = robot.getCol();
		//else
			//System.out.println("Sending "+mdfString+":F to AND");
		this.processSensor(ignoreSensor);
		this.moveCount++;
		if(moveCount>=3 && this.checkRCalibrate()==-1) {
			System.out.println("Starting to CALIBRATE RIGHT");
			//this.turnRobot('R');
			if(robot.getRealRun()) {
				ConnectionManager.getConnMgr().sendMessage("A", "ARD");
				this.moveCount=0;
			}
			else {
				this.turnRobot('R',false);
				this.turnRobot('L',false);
				this.moveCount=0;
			}
		}
		else if(moveCount>=3 && this.checkLCalibrate()==-1) {
			System.out.println("Starting to CALIBRATE LEFT");
			//this.turnRobot('R');
			if(robot.getRealRun()) {
				ConnectionManager.getConnMgr().sendMessage("B", "ARD");
				this.moveCount=0;
			}
			else {
				this.turnRobot('L',false);
				this.turnRobot('R',false);
				this.moveCount=0;
			}
		}
		
		if(this.checkFront()!=-1) {
			moveCount=0;
		}	
	}

	public void turnRobot(char c, boolean ignoreSensor) {	
		//map descriptor
		String mdfString = this.updateMap();
		//ConnectionManager.getConnMgr().sendMessage(mdfString, "AND");
		
		if(!robot.getRealRun())
			try {
				Thread.sleep(ROBOT_SPEED);
			}catch(InterruptedException ex) {
				Thread.currentThread().interrupt();
		}
		

		if(c=='R') {
			robot.turningRobot(c,explored_map);
		}
		else if(c=='L') {
			robot.turningRobot(c,explored_map);
		}
		else
			System.out.println("Error");
		
		//this.updateMain();
		
		//System.out.println("Sending "+c+" to ARD");
		
		if(robot.getRealRun()) {
			//System.out.println("Sending "+String.valueOf(c)+":"+mdfString+" to AND");
			//ConnectionManager.getConnMgr().sendMessage(mdfString+":"+String.valueOf(c), "AND");	
			ConnectionManager.getConnMgr().sendMessage(String.valueOf(c), "ARD");
			//ConnectionManager.getConnMgr().sendMessage(mdfString+":"+String.valueOf(c), "AND");
			if(!ignoreSensor) {
			int tempRow = robot.getRow();
			int tempCol = robot.getCol();
			RobotConstants.HEADING tempH = robot.getCurrentHeading();
			if(this.imgToSendAnd=="")
			ConnectionManager.getConnMgr().sendMessage(mdfString+":"+tempRow+":"+(Math.abs(tempCol-MapConstants.MAP_COLS)-1)+":"+tempH.toString().charAt(0), "AND");
			else
				ConnectionManager.getConnMgr().sendMessage(mdfString+":"+tempRow+":"+(Math.abs(tempCol-MapConstants.MAP_COLS)-1)+":"+tempH.toString().charAt(0)+":"+this.imgToSendAnd, "AND");
			
			}
			
		}//else
			//System.out.println("Sending "+mdfString+":"+String.valueOf(c)+" to AND");
		this.processSensor(ignoreSensor);

	}
	
	/*
	 * For when camera is front, gets coordinate of front image
	 */
	private String getCoordinate(int image_distance,int sensor_type) {
		// TODO Auto-generated method stub
		int x_coor=0, y_coor=0, difference=0;
		switch(robot.getCurrentHeading()) {
		case NORTH:
			/*
			 * if sensor_type = FSR1:
			 * 		x_coor = robot.getRow()-1;
			 * 		y_coor = robot.getCol()-image_distance-1;
			 * if sensor_type = FSR3:
			 * 		x_coor = robot.getRow()+1;
			 * 		y_coor = robot.getCol()-image_distance-1;
			 * if sensor_type = FSR2:
			 * 		do below
			 */
			switch(sensor_type) {
			case 1: difference=-1;
			break;
			case 2: difference=0;
			break;
			case 3: difference=1;
			break;
			}
			x_coor = robot.getRow()+difference;
			y_coor = robot.getCol()-image_distance-1;
			break;
		case EAST:
			/*
			 * if sensor_type = FSR1:
			 * 		x_coor = robot.getRow()+image_distance+1
			 * 		y_coor = robot.getCol()-1;
			 * if sensor_type = FSR3:
			 * 		x_coor = robot.getRow()+image_distance+1;
			 * 		y_coor = robot.getCol()+1;
			 * if sensor_type = FSR2:
			 * 		do below
			 */
			switch(sensor_type) {
			case 1: difference=-1;
			break;
			case 2: difference=0;
			break;
			case 3: difference=1;
			break;
			}
			x_coor = robot.getRow()+image_distance+1;
			y_coor = robot.getCol()+difference;
			break;
		case WEST:
			/*
			 * if sensor_type = FSR1:
			 * 		x_coor = robot.getRow()-image_distance-1
			 * 		y_coor = robot.getCol()+1;
			 * if sensor_type = FSR3:
			 * 		x_coor = robot.getRow()-image_distance-1;
			 * 		y_coor = robot.getCol()-1;
			 * if sensor_type = FSR2:
			 * 		do below
			 */
			switch(sensor_type) {
			case 1: difference=1;
			break;
			case 2: difference=0;
			break;
			case 3: difference=-1;
			break;
			}
			x_coor = robot.getRow()-image_distance-1;
			y_coor = robot.getCol()+difference;
			break;
		case SOUTH:
			/*
			 * if sensor_type = FSR1:
			 * 		x_coor = robot.getRow()+1;
			 * 		y_coor = robot.getCol()+image_distance+1;
			 * if sensor_type = FSR3:
			 * 		x_coor = robot.getRow()-1;
			 * 		y_coor = robot.getCol()+image_distance+1;
			 * if sensor_type = FSR2:
			 * 		do below
			 */
			switch(sensor_type) {
			case 1: difference=1;
			break;
			case 2: difference=0;
			break;
			case 3: difference=-1;
			break;
			}
			x_coor = robot.getRow()+difference;
			y_coor = robot.getCol()+image_distance+1;
			break;
		}
		//y_coor = Math.abs(y_coor-MapConstants.MAP_COLS);
		String msg = x_coor+":"+y_coor;
		return msg;
	}

	/*
	 * Gets coordinate of right camera with offset
	 */
	private String getCoordinateRight() {
		// TODO Auto-generated method stub
		int x_coor=0, y_coor=0, difference=1;
		switch(robot.getCurrentHeading()) {
		case NORTH:
			x_coor = robot.getRow()+2;
			y_coor = robot.getCol()+difference;
			break;
		case EAST:
			x_coor = robot.getRow()-difference;
			y_coor = robot.getCol()+2;
			break;
		case WEST:
			x_coor = robot.getRow()+difference;
			y_coor = robot.getCol()-2;
			break;
		case SOUTH:
			x_coor = robot.getRow()-2;
			y_coor = robot.getCol()-difference;
			break;
		}
		if(robot.getRealRun()) {
			x_coor= x_coor+1;
			y_coor = Math.abs(y_coor-MapConstants.MAP_COLS);
		}
		String msg = x_coor+":"+y_coor;
		System.out.println("DEBUG :: Image coordinates "+msg);
		return msg;
	}
	
	/*
	 * Get list of directs from list of path
	 */
	public ArrayList<RobotConstants.DIRECTION> moveFPA(ArrayList<Tile> path, Tile tempTile, RobotConstants.HEADING h) {
		ArrayList<RobotConstants.DIRECTION> list_direction = new ArrayList<>();;
		if(path == null) {
		       JOptionPane.showMessageDialog(null,
	    		          "Error: No path found!", "Error Message",
	    		          JOptionPane.ERROR_MESSAGE);
		       return null;
		}
		Tile curTile = tempTile;
		RobotConstants.HEADING curHeading = h;
		//RobotConstants.HEADING curHeading = robot.getCurrentHeading();
		for(Tile temp : path) {
			/*
			if(!ignoreCancel && SimulatorMDP.exploreWorker.isCancelled()) {
				System.out.println("Exploration is cancelled");
				displayMgt.cancel(true);
				return null;
			}*/
//			if(!fullCoverage) {
//				//this.processSensor();
//				//map descriptor
//				String mdfString = this.updateMap();
//				//ConnectionManager.getConnMgr().sendMessage(mdfString, "AND");
//			}
			
			RobotConstants.HEADING nextHeading = fpa.getNextHeading(curTile,temp.getRow(),temp.getCol(),curHeading);
			int turn = Math.abs(curHeading.ordinal() - nextHeading.ordinal());
			if(turn ==0) {
					//moveRobot();
					list_direction.add(RobotConstants.DIRECTION.FRONT);
			}
			else {
				if(turn==2) {
					//turnRobot('R');
					//turnRobot('R');
					//moveRobot();
					list_direction.add(RobotConstants.DIRECTION.RIGHT);
					list_direction.add(RobotConstants.DIRECTION.RIGHT);
					list_direction.add(RobotConstants.DIRECTION.FRONT);
				}
				else {
					switch(curHeading) {
					case NORTH: {
						if(curTile.getRow()-temp.getRow()>0) {
							//turnRobot('L');
							//moveRobot();
							list_direction.add(RobotConstants.DIRECTION.LEFT);
							list_direction.add(RobotConstants.DIRECTION.FRONT);
						}

						else {
							//turnRobot('R');
							//moveRobot();
							list_direction.add(RobotConstants.DIRECTION.RIGHT);
							list_direction.add(RobotConstants.DIRECTION.FRONT);
						}
						break;
					}
					case WEST: {
						if(curTile.getCol()-temp.getCol()<0) {
							//turnRobot('L');
							//moveRobot();
							list_direction.add(RobotConstants.DIRECTION.LEFT);
							list_direction.add(RobotConstants.DIRECTION.FRONT);
							
						}

						else {
							//turnRobot('R');
							//moveRobot();
							
							list_direction.add(RobotConstants.DIRECTION.RIGHT);
							list_direction.add(RobotConstants.DIRECTION.FRONT);
						}
						break;
					}
					case EAST: {
						if(curTile.getCol()-temp.getCol()>0) {
							//turnRobot('L');
							//moveRobot();
							
							list_direction.add(RobotConstants.DIRECTION.LEFT);
							list_direction.add(RobotConstants.DIRECTION.FRONT);
						}

						else {
							//turnRobot('R');
							//moveRobot();
							
							list_direction.add(RobotConstants.DIRECTION.RIGHT);
							list_direction.add(RobotConstants.DIRECTION.FRONT);
						}
						break;
					}
					case SOUTH: {
						if(curTile.getRow()-temp.getRow()<0) {
							//turnRobot('L');
							//moveRobot();
							
							list_direction.add(RobotConstants.DIRECTION.LEFT);
							list_direction.add(RobotConstants.DIRECTION.FRONT);
						}

						else {
							//turnRobot('R');
							//moveRobot();
							
							list_direction.add(RobotConstants.DIRECTION.RIGHT);
							list_direction.add(RobotConstants.DIRECTION.FRONT);
						}
						break;
					}
					}
				}
			}
			curHeading = nextHeading; //robot.getCurrentHeading();
			curTile = temp;
		}
		return list_direction;
	}
	
	public void debug_moveRobot() {
		String 	msg = this.readSimulatorMap();
		this.robot.sense(explored_map, msg);
		moveRobot(false);
		msg = this.readSimulatorMap();
		this.robot.sense(explored_map, msg);
	}
	
	public void debug_turnRobotRight() {
		String 	msg = this.readSimulatorMap();
		this.robot.sense(explored_map, msg);
		turnRobot('R',false);
		msg = this.readSimulatorMap();
		this.robot.sense(explored_map, msg);
	}
	
	/*
	 * Prorcesses sensor information gotten from Arduino
	 */
	private void processSensor(boolean ignoreSensor) {
		while(true) {
			String msg;
			if(robot.getRealRun()) 
				msg = ConnectionManager.getConnMgr().receiveMessage();
			else
				msg = this.readSimulatorMap();
			if(msg!=null) {
				if(!ignoreSensor) {
					int sensorIdTemp, imageIdTemp;
				System.out.println("("+LocalTime.now()+") Waiting for Sensor Information from ARD .....");
				robot.sense(explored_map,msg);
				System.out.println("("+LocalTime.now()+") Done processing Sensor Information .....");
				String updateMsg = null;

					updateMsg = LoopCount+". "+robot.getCurrentHeading()+" (X."+robot.getRow()+" Y."+robot.getCol()+") --- "+msg+"\n";
				String originalMsg = Main.SimulatorMDP.lblSensorInfo.getText();
				Main.SimulatorMDP.lblSensorInfo.setText(updateMsg + originalMsg);//LoopCount+". "+robot.getCurrentHeading()+" (X."+robot.getRow()+" Y."+robot.getCol()+") --- "+msg+"\n"+originalMsg);
				LoopCount++;
				//Main.SimulatorMDP.lblSensorInfo.setText(msg);
				}
				break;
			}
		}
		
		/*
		 * Finish sensor processing, ->check right for obstacle. If obstacle detected, send command to take photo!
		 */
		if(this.processImage) {
			if(this.checkRObs()==2) {
				System.out.println("Current Postion -> X:"+robot.getRow() +" Y:"+robot.getCol());
				System.out.println("Obstacle Detected On Right! ");
				this.processObstacle();
				this.sendTakePhotoCmd(); //send command to take photo
			}
		}
		
	}
	/*
	 * Process obstacle surface, depending on sides if already have seen that surface. 
	 * e.g. South of an obstacle block. Mainly for island points
	 */
	private void processObstacle() {
		// TODO Auto-generated method stub
		String msg = this.getCoordinateRight();
		String[] sb = msg.split(":");
		int r = Integer.parseInt(sb[0]);
		int c = Integer.parseInt(sb[1]);
		if(!(r >14 || r<0 || c>19 ||c<0)) {
			Tile temp = explored_map.getTile(r, c);
			if(temp.getIsObstacle()) {
				int nextOrdinal = Math.abs(robot.getCurrentHeading().ordinal()+1);
				nextOrdinal = nextOrdinal%4;
				RobotConstants.HEADING h = RobotConstants.HEADING.values()[nextOrdinal];
				temp.replaceValue(h.toString(), true);
			}
		}
	}
	
	/*
	 * Set and Replacing Waypoints set
	 */
	public void setAndReplaceWP(int xRow, int yCol) {
		if(!robot.getRealRun())
			real_map.setWaypoint(xRow, yCol);
		explored_map.setWaypoint(xRow, yCol);
		
	}
	
	public int checkTrap(int frontValue) {
		if(frontValue == -1)
			return -1;
		else {
			if(frontValue == -2) {
				if(this.checkR()!=-1 && this.checkL()!=-1) {
					return 1;
				}
				else
					return -1;
			}
		}
		return 1;
	}
	/* Traps Detection
	public int checkIfTrap() {
		//if(this.lookFront2IfWall()==1)
			//return this.check2IfTrap();
		if(lookFrontIfWall()==-1)
			return -1;

		
		return this.trap1();// || this.trap2()==1 ? 1 : -1;
	}
	
	private int trap2() {
		switch(robot.getCurrentHeading()) {
		case EAST:
		return (checkSurrounding(cur_row,cur_col-3) == 1 && checkSurrounding(cur_row+1,cur_col-3) == 1 && checkSurrounding(cur_row-1, cur_col-3) == 1) && 
		 (checkSurrounding(cur_row,cur_col+2) == 1 && checkSurrounding(cur_row+1,cur_col+2)==1 && checkSurrounding(cur_row-1, cur_col+2)==1) ? 1:-1;
		case WEST:
		return (checkSurrounding(cur_row,cur_col-3) == 1 && checkSurrounding(cur_row+1,cur_col-3) == 1 && checkSurrounding(cur_row-1, cur_col-3) == 1) && 
		 (checkSurrounding(cur_row,cur_col+2) == 1 && checkSurrounding(cur_row+1,cur_col+2)==1 && checkSurrounding(cur_row-1, cur_col+2)==1) ? 1:-1;
		case NORTH:
		return (checkSurrounding(cur_row-3,cur_col) == 1 && checkSurrounding(cur_row-3,cur_col+1) == 1 && checkSurrounding(cur_row-3, cur_col-1) == 1) && 
		 (checkSurrounding(cur_row+2,cur_col) == 1 && checkSurrounding(cur_row+2,cur_col+1)==1 && checkSurrounding(cur_row+2, cur_col-1)==1) ? 1:-1;
		case SOUTH:
			return (checkSurrounding(cur_row+3,cur_col) == 1 && checkSurrounding(cur_row+3,cur_col+1) == 1 && checkSurrounding(cur_row+3, cur_col-1) == 1) && 
					 (checkSurrounding(cur_row-2,cur_col) == 1 && checkSurrounding(cur_row-2,cur_col+1)==1 && checkSurrounding(cur_row-2, cur_col-1)==1) ? 1:-1;
	
	}
	return -1;
	}

	public int trap1() {
		switch(robot.getCurrentHeading()) {
		case EAST:
		return (checkSurrounding(cur_row,cur_col-3) == 2 && checkSurrounding(cur_row+1,cur_col-3) == 2 && checkSurrounding(cur_row-1, cur_col-3) == 2) && 
		 (checkSurrounding(cur_row,cur_col+3) == 2 && checkSurrounding(cur_row+1,cur_col+3)==2 && checkSurrounding(cur_row-1, cur_col+3)==2) ? 1:-1;
		case WEST:
		return (checkSurrounding(cur_row,cur_col-3) == 2 && checkSurrounding(cur_row+1,cur_col-3) == 2 && checkSurrounding(cur_row-1, cur_col-3) == 2) && 
		 (checkSurrounding(cur_row,cur_col+3) == 2 && checkSurrounding(cur_row+1,cur_col+3)==2 && checkSurrounding(cur_row-1, cur_col+3)==2) ? 1:-1;
		case NORTH:
		return (checkSurrounding(cur_row-3,cur_col) == 2 && checkSurrounding(cur_row-3,cur_col+1) == 2 && checkSurrounding(cur_row-3, cur_col-1) == 2) && 
		 (checkSurrounding(cur_row+3,cur_col) == 2 && checkSurrounding(cur_row+3,cur_col+1)==2 && checkSurrounding(cur_row+3, cur_col-1)==2) ? 1:-1;
		case SOUTH:
			return (checkSurrounding(cur_row+3,cur_col) == 2 && checkSurrounding(cur_row+3,cur_col+1) == 2 && checkSurrounding(cur_row+3, cur_col-1) == 2) && 
					 (checkSurrounding(cur_row-3,cur_col) == 2 && checkSurrounding(cur_row-3,cur_col+1)==2 && checkSurrounding(cur_row-3, cur_col-1)==2) ? 1:-1;
	
	}
	return -1;
	}
	public int check2IfTrap() {
		switch(robot.getCurrentHeading()) {
			case EAST:
			return (checkSurrounding(cur_row,cur_col-3) == 1 && checkSurrounding(cur_row+1,cur_col-3) == 1 && checkSurrounding(cur_row-1, cur_col-3) == 1) && 
			 (checkSurrounding(cur_row,cur_col+3) == 1 && checkSurrounding(cur_row+1,cur_col+3)==1 && checkSurrounding(cur_row-1, cur_col+3)==1) ? 2:-1;
			case WEST:
			return (checkSurrounding(cur_row,cur_col-3) == 1 && checkSurrounding(cur_row+1,cur_col-3) == 1 && checkSurrounding(cur_row-1, cur_col-3) == 1) && 
			 (checkSurrounding(cur_row,cur_col+3) == 1 && checkSurrounding(cur_row+1,cur_col+3)==1 && checkSurrounding(cur_row-1, cur_col+3)==1) ? 2:-1;
			case NORTH:
			return (checkSurrounding(cur_row-3,cur_col) == 1 && checkSurrounding(cur_row-3,cur_col+1) == 1 && checkSurrounding(cur_row-3, cur_col-1) == 1) && 
			 (checkSurrounding(cur_row+3,cur_col) == 1 && checkSurrounding(cur_row+3,cur_col+1)==1 && checkSurrounding(cur_row+3, cur_col-1)==1) ? 2:-1;
			case SOUTH:
				return (checkSurrounding(cur_row+3,cur_col) == 1 && checkSurrounding(cur_row+3,cur_col+1) == 1 && checkSurrounding(cur_row+3, cur_col-1) == 1) && 
						 (checkSurrounding(cur_row-3,cur_col) == 1 && checkSurrounding(cur_row-3,cur_col+1)==1 && checkSurrounding(cur_row-3, cur_col-1)==1) ? 2:-1;
		
		}
		return -1;
	}
	public int lookFrontIfWall() {
		switch(robot.getCurrentHeading()) {
		case NORTH:
			if((cur_col-2)<0)
				return 1;
			break;
		case SOUTH:
			if((cur_col+2)>19)
				return 1;
			break;
		case WEST:
			if((cur_row-2)<0)
				return 1;
			break;
		case EAST:
			if((cur_row+2)>14)
				return 1;
			break;
		}
		return -1;
	}
	public int lookFront2IfWall() {
		switch(robot.getCurrentHeading()) {
		case NORTH:
			if((cur_col-3)==-2)
				return 1;
			break;
		case SOUTH:
			if((cur_col+3)==20)
				return 1;
			break;
		case WEST:
			if((cur_row-3)==-2)
				return 1;
			break;
		case EAST:
			if((cur_row+3)==15)
				return 1;
			break;
		}
		return -1;
	}

	public int lookFrontAhead() {
		switch(robot.getCurrentHeading()) {
		case NORTH:
			return checkSurrounding(cur_row,cur_col-3) ==1 && checkSurrounding(cur_row-1,cur_col-3)==1 && checkSurrounding(cur_row+1, cur_col-3)==1 ? 1:-1;
		case EAST:
			return checkSurrounding(cur_row+3,cur_col) ==1 && checkSurrounding(cur_row+3,cur_col-1)==1 && checkSurrounding(cur_row+3, cur_col+1)==1 ? 1:-1;
		case WEST:
			return checkSurrounding(cur_row-3,cur_col) ==1 && checkSurrounding(cur_row-3,cur_col-1)==1 && checkSurrounding(cur_row-3, cur_col+1)==1 ? 1:-1;
		case SOUTH:
			return checkSurrounding(cur_row,cur_col+3) ==1 && checkSurrounding(cur_row-1,cur_col+3)==1 && checkSurrounding(cur_row+1, cur_col+3)==1 ? 1:-1;
				
		default:
			break;
		}
		return -1;
	}
	*/
	
	/*
	 * Right-Wall Hug Algorithm
	 * 1. Prioritize Right Turn
	 * 2. Move Forward if avail.
	 * 3. Turn left if 1 and 2 fails.
	 * 4. Else double turn
	 */
	public void rightWallHug(boolean v, Tile ref) {
		//init sensors
		//this.processSensor();
		//map descriptor
		//this.updateMap();
		
		//algorithm
		if(v) {
			if(this.checkR()==-1) {
				this.turnRobot('R',false);
				//System.out.println("RIGHT CLEAR CHECK");
				if(!continueRun)
					return;
				if(this.checkFront()==-1) {
					this.moveRobot(false);
				}
				else {
					this.turnRobot('L',false);
					if(this.checkFront()==-1) {
						this.moveRobot(false);
					}
				}
			}
			else if(this.checkFront()==-1) {
				this.moveRobot(false);
			}
			else if(this.checkL()==-1) {
				this.turnRobot('L',false);
			}
			else{
				this.turnRobot('L',false);
				if(!continueRun)
					return;
				this.turnRobot('L',false);
			}
			
			this.cur_row = robot.getRow();
			this.cur_col = robot.getCol();
	
			explored_map.paintAgain();
			
			if(SimulatorMDP.exploreWorker.isCancelled()) {
				this.stopRun();
				return;
			}
		}
		else { //for island hugging obsolete. 
			if(this.checkR()==-1) {
				this.turnRobot('R',false);
				//System.out.println("RIGHT CLEAR CHECK");
				if(!continueRun)
					return;
				if(this.checkFront()==-1) {
					this.moveRobot(false);
				}
			}
			else if(this.checkFront()==-1) {
					this.moveRobot(false);
			}
			else if(this.checkL()==-1) {
				this.turnRobot('L',false);
			}
			else{
				this.turnRobot('L',false);
				if(!continueRun)
					return;
				this.turnRobot('L',false);
				if (this.checkRObs() != 2)
					this.leftWallHug(false, ref);
			}
			
			this.cur_row = robot.getRow();
			this.cur_col = robot.getCol();
	
			explored_map.paintAgain();
			
		}
	}

	/*
	 * Algorithm for left wall hug; same as RWH, just opposite.
	 */
	public void leftWallHug(boolean v, Tile ref) {
		//init sensors
		//this.processSensor();
		//map descriptor
		//this.updateMap();
		
		//algorithm
		while(true) {
		if(this.checkL()==-1) {
			this.turnRobot('L',false);
			//System.out.println("RIGHT CLEAR CHECK");
			if(!continueRun)
				return;
			if(this.checkFront()==-1) {
				this.moveRobot(false);
			}
		}
		else if(this.checkFront()==-1) {
			this.moveRobot(false);
		}
		else if(this.checkR()==-1) {
			this.turnRobot('R',false);
		}
		else {
			this.turnRobot('R',false);
			if(!continueRun)
				return;
			this.turnRobot('R',false);
		}
		this.cur_row = robot.getRow();
		this.cur_col = robot.getCol();

		explored_map.paintAgain();
		
		if(cur_row == ref.getRow() && cur_col==ref.getCol())
		return;
		}
	}
	
	/*
	 * Algorithm for sending take photo command
	 * If Right has obstacle -> chance for image -> send message to RPI to take photo
	 */
	private void sendTakePhotoCmd() {
		String msg="";
		debug_photoCmdCount++;
		System.out.println("Sending Photo Command to RPI");
		if(robot.getRealRun()) {
		ConnectionManager.getConnMgr().sendMessage("P", "RPI");
			
		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
		// TODO Auto-generated catch block
			e.printStackTrace();
		}
		while(true) {
			msg = ConnectionManager.getConnMgr().receiveMessage();
			if(msg.equalsIgnoreCase("None") || msg!=""){
				break;
			}
		}
		}
		else {
			String sb = this.getCoordinateRight();
			String[] coor = sb.split(":");
			int y_coor = Integer.parseInt(coor[1]);
			int x_coor = Integer.parseInt(coor[0]);
			Tile temp;
			if((temp=explored_map.getTile(x_coor, y_coor)).getIsImage()) {
				msg=temp.getImage().getId();
			}
			else
				msg="None";
		}
		if(!msg.equalsIgnoreCase("None"))
			this.processImageReceive(msg);
	}
	
	/*
	 * Process image received, if any, from RPI with respective coordinates and the Photo ID.
	 */
	private void processImageReceive(String imageMsg) {
		// TODO Auto-generated method stub
		System.out.println("Debug :: Processing Image Received!");
		//imageDetails = imageMsg.split(":");
		String imgId = imageMsg;
		//String sensorPos = imageDetails[1];
		//int sensorPosI = Integer.parseInt(sensorPos.substring(sensorPos.length()-1));
//		int roguePos =-1;
//		switch(sensorPosI) {
//		case 1: roguePos=-1;
//		break;
//		case 2: roguePos=0;
//		break;
//		case 3: roguePos=1;
//		break;
//		
//		}
//		System.out.println("DEBUG :: Sensor Position ->"+sensorPosI);
//		System.out.println("DEBUG :: Rogue Position ->"+roguePos);
//		int image_distance=  this.findDistance(roguePos);
//		System.out.println("DEBUG :: IMAGE DISTANCE ->"+image_distance);
//		if(image_distance==-1)
//			return;
		String imagemsg = this.getCoordinateRight() + ":"+imgId; //this.getCoordinate(image_distance, sensorPosI) + ":" +imgId;
		//if(!imageIdsList.contains(imgId)) {
			System.out.println("DEBUG MSG :: FOUND IMAGE - >" +imagemsg);
			imageList.add(imagemsg);
			imageIdsList.add(imgId);
			this.imgToSendAnd = imagemsg; //sets image to send message (wait for next round of communication)
		//}
	}
	
	/*
	 * Checks for Left is obstacle only for calibration
	 */
	public int checkLCalibrate() {
		switch(robot.getCurrentHeading()) {
		case NORTH:
			return checkSurrounding(cur_row-2,cur_col-1) ==2 && checkSurrounding(cur_row-2,cur_col) ==2 && checkSurrounding(cur_row-2,cur_col+1) ==2 ? -1 : 1;
		case WEST:
			return checkSurrounding(cur_row-1,cur_col+2) ==2 && checkSurrounding(cur_row,cur_col+2) ==2 && checkSurrounding(cur_row+1,cur_col+2) ==2 ? -1 : 1;
		case EAST:
			return checkSurrounding(cur_row+1,cur_col-2) ==2 && checkSurrounding(cur_row,cur_col-2) ==2 && checkSurrounding(cur_row-1,cur_col-2) ==2 ? -1 : 1;
		case SOUTH:
			return checkSurrounding(cur_row+2,cur_col+1) ==2 && checkSurrounding(cur_row+2,cur_col) ==2 && checkSurrounding(cur_row+2,cur_col-1) ==2 ? -1 : 1;	
		default:
			break;
		}
		return 1;
	} 
}
