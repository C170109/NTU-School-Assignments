package Robot;

import java.util.ArrayList;

import Map.Map;
import Robot.RobotConstants.HEADING;

public class Robot {
	private int row;
	private int col;
	private boolean realRun;
	private long robotSpeed;
	
	//heading of robot
	private int facing_row;
	private int facing_col;
	
	//sensors
	public Sensor FSR2=null;
	public Sensor FSR1=null;
	public Sensor FSR3=null;
	public Sensor LSR=null;
	public Sensor RSR=null;
	public Sensor LLR=null;
	
	private ArrayList<Sensor> sensors = new ArrayList<Sensor>();
	
	private RobotConstants.HEADING currentHeading;
	private RobotConstants.HEADING leftSensorHeading;
	private RobotConstants.HEADING rightSensorHeading;
	
	public Robot(int row, int col, boolean realRun, long robotSpeed) {
		this.row = row;
		this.col = col;
		this.realRun = realRun;
		this.robotSpeed = robotSpeed;
		
		//init center sensor
		this.facing_row = row;
		this.facing_col = col-1;
		
//		this.facing_row = row+1;
//		this.facing_col = col;
		//setHeading(RobotConstants.START_POS_X,RobotConstants.START_POS_Y-1);
		
		//set sensor
		currentHeading = RobotConstants.HEADING.NORTH;
		leftSensorHeading = RobotConstants.HEADING.WEST;
		rightSensorHeading = RobotConstants.HEADING.EAST;
		
//		currentHeading = RobotConstants.HEADING.EAST;
//		leftSensorHeading = RobotConstants.HEADING.NORTH;
//		rightSensorHeading = RobotConstants.HEADING.SOUTH;
		
		FSR2 = new Sensor(RobotConstants.SHORT_RANGE_VAL_MIN, RobotConstants.SHORT_RANGE_VAL_MAX,facing_row,facing_col,currentHeading,"FSR2");
		FSR1 = new Sensor(RobotConstants.SHORT_RANGE_VAL_MIN, RobotConstants.SHORT_RANGE_VAL_MAX,facing_row-1,facing_col,currentHeading,"FSR1");
		FSR3 = new Sensor(RobotConstants.SHORT_RANGE_VAL_MIN, RobotConstants.SHORT_RANGE_VAL_MAX,facing_row+1,facing_col,currentHeading,"FSR3");
		LSR = new Sensor(RobotConstants.SHORT_RANGE_VAL_MIN, RobotConstants.SHORT_RANGE_VAL_MAX, facing_row-1,facing_col,leftSensorHeading,"LSR");
		LLR = new Sensor(RobotConstants.LONG_RANGE_VAL_MIN,RobotConstants.LONG_RANGE_VAL_MAX, facing_row-1, facing_col+1, leftSensorHeading, "LLR");
		RSR = new Sensor(RobotConstants.SHORT_RANGE_VAL_MIN, RobotConstants.SHORT_RANGE_VAL_MAX, facing_row+1, facing_col,rightSensorHeading,"RSR");
		
//		FSR2 = new Sensor(RobotConstants.SHORT_RANGE_VAL_MIN, RobotConstants.SHORT_RANGE_VAL_MAX,facing_row,facing_col,currentHeading,"FSR2");
//		FSR1 = new Sensor(RobotConstants.SHORT_RANGE_VAL_MIN, RobotConstants.SHORT_RANGE_VAL_MAX,facing_row,facing_col-1,currentHeading,"FSR1");
//		FSR3 = new Sensor(RobotConstants.SHORT_RANGE_VAL_MIN, RobotConstants.SHORT_RANGE_VAL_MAX,facing_row,facing_col+1,currentHeading,"FSR3");
//		LSR = new Sensor(RobotConstants.SHORT_RANGE_VAL_MIN, RobotConstants.SHORT_RANGE_VAL_MAX, facing_row,facing_col-1,leftSensorHeading,"LSR");
//		LLR = new Sensor(RobotConstants.LONG_RANGE_VAL_MIN,RobotConstants.LONG_RANGE_VAL_MAX, facing_row-1, facing_col-1, leftSensorHeading, "LLR");
//		RSR = new Sensor(RobotConstants.SHORT_RANGE_VAL_MIN, RobotConstants.SHORT_RANGE_VAL_MAX, facing_row-1, facing_col+1,rightSensorHeading,"RSR");
//		
		sensors.add(LLR);
		sensors.add(LSR);
		sensors.add(FSR1);
		sensors.add(FSR2);
		sensors.add(FSR3);
		sensors.add(RSR);
	}
	public ArrayList<Sensor> getSensors(){
		return this.sensors;
	}
	public void setRow(int row) {
		this.row = row;
	}
	public void setCol(int col) {
		this.col=col;
	}
	public int getRow() {
		return this.row;
	}
	public int getCol() {
		return this.col;
	}
	public int getFacingRow() {
		return this.facing_row;
	}
	public int getFacingCol() {
		return this.facing_col;
	}
	public boolean getRealRun() {
		return this.realRun;
	}

	public int getStartPosX() {
		// TODO Auto-generated method stub
		return RobotConstants.START_POS_X;
	}
	public int getStartPosY() {
		return RobotConstants.START_POS_Y;
	}
	public String[] processSensorMessage(String s) {
		String[] message = s.split(":");
		//System.out.println("Done splitting");
		return message;
	}	
	public RobotConstants.HEADING getCurrentHeading(){
		return this.currentHeading;
	}
	public int checkRight(Map explored_map) {
		return RSR.sense(explored_map);
	}
	public int checkLeft(Map explored_map) {
		int debugvalue = LSR.sense(explored_map);// + LLR.sense(explored_map);
		//System.out.println("Debug Value :"+debugvalue);
		//System.out.println(debugvalue==-2 ? -1 : 1);
		return debugvalue==-1 ? -1 : 1;
	}
	public int checkFront(Map explored_map) {
		int debugvalue_center = FSR2.sense(explored_map);
		int debugvalue_left = FSR1.sense(explored_map);
		int debugvalue_right = FSR3.sense(explored_map);
		int debugvalue = debugvalue_center+debugvalue_left+debugvalue_right;
		System.out.println("Debug Value :"+debugvalue);
		//System.out.println(debugvalue==-3 ? -1 : 1);
		//return center_sensor.sense(m) + sensor2.sense(m) + sensor3.sense(m);
		//return debugvalue==-3 ? -1 : 1;
		
		if(debugvalue==-3)
			return -1;
		else
			return 1;
		
		
	}
	
	public void sense(Map explored_map, String s) {
		String[] message = processSensorMessage(s);
		//do all the shit
		int LLR_value = (int)Double.parseDouble((message[1]));
		if(LLR_value ==3 || LLR_value ==4 || LLR_value==-1)
			LLR.sense(explored_map, (int)Double.parseDouble((message[1])));//Integer.parseInt(message[1]));
		LSR.sense(explored_map, (int)Double.parseDouble((message[3])));//(explored_map, Integer.parseInt(message[3]));
		int FSR1_val =FSR1.sense(explored_map, (int)Double.parseDouble((message[5]))); //(explored_map, Integer.parseInt(message[5]));
		int FSR2_val =FSR2.sense(explored_map, (int)Double.parseDouble((message[7])));//(explored_map, Integer.parseInt(message[7]));
		int FSR3_val =FSR3.sense(explored_map, (int)Double.parseDouble((message[9])));//(explored_map, Integer.parseInt(message[9]));
		RSR.sense(explored_map, (int)Double.parseDouble((message[11])));//(explored_map, Integer.parseInt(message[11]));
//		if(message.length>12) {
//		switch(Integer.parseInt(message[13])) {
//		case -1: return message[12]+":"+message[13];
//		case 1: return message[12]+":"+message[13]+","+FSR1_val;
//		case 2:return message[12]+":"+message[13]+","+FSR2_val;
//		case 3:return message[12]+":"+message[13]+","+FSR3_val;
//			default:System.out.println("No Image");
//			break;
//				
//		}
//		}
//		return "-1:-1";
		//System.out.println("Sense done");
		//explored_map.paintAgain();
		//return (int)Double.parseDouble((message[7]))+":"+message[12];
	}
	 public void moveRobot(RobotConstants.HEADING h, Map explored_map) {
	int newRow =this.row;
	int newCol =this.col;
	int oldRow = this.facing_row;
	int oldCol = this.facing_col;
	
	if(facing_col==19 || facing_col==0 || facing_row==0 || facing_row == 14) {
		System.out.println("Robot cannot move, wall ahead!");
		return;
	}
	
	switch(h) {
		case NORTH: 
    		newRow=this.row;
	    	newCol=this.col-1;
			facing_row=newRow;
			facing_col=newCol-1;
			currentHeading=RobotConstants.HEADING.NORTH;
	    	break;
			case EAST:
				newRow=this.getRow()+1;
				newCol=this.getCol();
   	    		facing_row=newRow+1;
   	    		facing_col=newCol;
   	    		currentHeading=RobotConstants.HEADING.EAST;

		break;
			case SOUTH:
				newRow=this.getRow();
				newCol=this.getCol()+1;
   				facing_row=newRow;
   				facing_col=newCol+1;
   				currentHeading=RobotConstants.HEADING.SOUTH;

   		break;
			case WEST:
				newRow=this.getRow()-1;
				newCol=this.getCol();
    			facing_row=newRow-1;
    			facing_col=newCol;
    			currentHeading=RobotConstants.HEADING.WEST;

	    	break;
	}
	explored_map.setRobotMoved(this.row,this.col);
	this.setRow(newRow);
	this.setCol(newCol);
	
	//this.setCurrentHeading(currHeading);
	//to clean for debugging
	//System.out.println("Robot's current coordinate - Row :"+ this.getRow()+"    Col :"+ this.getCol());

	updateAndPaintMap(explored_map);
}

	public void turningRobot(char c, Map explored_map) {
		//System.out.println("TURNING");
		if (c == 'R')
			checkHeadingR(currentHeading);
		else if (c == 'L')
			checkHeadingL(currentHeading);	
		
		updateAndPaintMap(explored_map);
	}
	
	//set Sensor position
	public void updateAndPaintMap(Map explored_map) {
		FSR2.setPosition(facing_row,facing_col,currentHeading);
		RobotConstants.HEADING h = FSR2.getCurrentHeading();
		switch(h) {		
		case NORTH:		
			FSR1.setPosition(facing_row-1,facing_col,h);
			FSR3.setPosition(facing_row+1,facing_col,h);
			LSR.setPosition(facing_row-1, facing_col, leftSensorHeading);
			LLR.setPosition(facing_row-1, facing_col+1, leftSensorHeading);
			RSR.setPosition(facing_row+1, facing_col, rightSensorHeading);
			break;
		case WEST:		
			FSR1.setPosition(facing_row,facing_col+1,h);
			FSR3.setPosition(facing_row,facing_col-1,h);
			LSR.setPosition(facing_row, facing_col+1, leftSensorHeading);
			LLR.setPosition(facing_row+1, facing_col+1, leftSensorHeading);
			RSR.setPosition(facing_row, facing_col-1, rightSensorHeading);
			break;
		case EAST:
			FSR1.setPosition(facing_row,facing_col-1,h);
			FSR3.setPosition(facing_row,facing_col+1,h);
			LSR.setPosition(facing_row, facing_col-1, leftSensorHeading);
			LLR.setPosition(facing_row-1, facing_col-1, leftSensorHeading);
			RSR.setPosition(facing_row, facing_col+1, rightSensorHeading);
			break;
		case SOUTH:		
			FSR1.setPosition(facing_row+1,facing_col,h);
			FSR3.setPosition(facing_row-1,facing_col,h);
			LSR.setPosition(facing_row+1, facing_col, leftSensorHeading);
			LLR.setPosition(facing_row+1, facing_col-1, leftSensorHeading);
			RSR.setPosition(facing_row-1, facing_col, rightSensorHeading);
			break;
		default:
			System.out.println("Error in heading");
			break;
		}
		explored_map.paintAgain();
		
		//debug message
		//System.out.println("Center Camera - X: "+facing_row+" Y:"+facing_col+" Direction :"+currentHeading);
		//System.out.println("Left Camera - X: "+facing_row+" Y:"+facing_col+" Direction :"+leftSensorHeading);
		//System.out.println("Right Camera - X: "+facing_row+" Y:"+facing_col+" Direction :"+rightSensorHeading);
		//this.sense(explored_map);
		//m.repaint();
	}
	
	public void checkHeadingR(RobotConstants.HEADING h) {
		switch(h) {
		case NORTH:		
			this.facing_row=row+1;
			this.facing_col=col;
			this.currentHeading = RobotConstants.HEADING.EAST;
			this.leftSensorHeading= RobotConstants.HEADING.NORTH;
			this.rightSensorHeading = RobotConstants.HEADING.SOUTH;
			break;
		case WEST:		
			this.facing_row=row;
			this.facing_col=col-1;
			this.currentHeading = RobotConstants.HEADING.NORTH;
			this.leftSensorHeading = RobotConstants.HEADING.WEST;
			this.rightSensorHeading = RobotConstants.HEADING.EAST;
			break;
		case EAST:		
			this.facing_row=row;
			this.facing_col=col+1;
			this.currentHeading = RobotConstants.HEADING.SOUTH;
			this.leftSensorHeading = RobotConstants.HEADING.EAST;
			this.rightSensorHeading = RobotConstants.HEADING.WEST;
			break;
		case SOUTH:		
			this.facing_row=row-1;
			this.facing_col=col;
			this.currentHeading = RobotConstants.HEADING.WEST;
			this.leftSensorHeading = RobotConstants.HEADING.SOUTH;
			this.rightSensorHeading = RobotConstants.HEADING.NORTH;
			break;
		default:
			System.out.println("Error in heading");
			break;
		}

	}
	
	public void checkHeadingL(RobotConstants.HEADING h) {
		switch(h) {
		case NORTH:		
			this.facing_row=row-1;
			this.facing_col=col;
			this.currentHeading = RobotConstants.HEADING.WEST;
			this.leftSensorHeading = RobotConstants.HEADING.SOUTH;
			this.rightSensorHeading = RobotConstants.HEADING.NORTH;
			break;
		case WEST:		
			this.facing_row=row;
			this.facing_col=col+1;
			this.currentHeading = RobotConstants.HEADING.SOUTH;
			this.leftSensorHeading = RobotConstants.HEADING.EAST;
			this.rightSensorHeading = RobotConstants.HEADING.WEST;
			break;
		case EAST:		
			this.facing_row=row;
			this.facing_col=col-1;
			this.currentHeading = RobotConstants.HEADING.NORTH;
			this.leftSensorHeading = RobotConstants.HEADING.WEST;
			this.rightSensorHeading = RobotConstants.HEADING.EAST;
			break;
		case SOUTH:		
			this.facing_row=row+1;
			this.facing_col=col;
			this.currentHeading = RobotConstants.HEADING.EAST;
			this.leftSensorHeading = RobotConstants.HEADING.NORTH;
			this.rightSensorHeading = RobotConstants.HEADING.SOUTH;
			break;
		default:
			System.out.println("Error in heading");
			break;
		}
	}
	public String printLoc() {
		return "I am at -> X :"+this.row+" Y :"+this.col+ " HEADING :"+currentHeading;
	}
	public void setHeading(HEADING north) {
		// TODO Auto-generated method stub
		this.currentHeading = north;
		
	}
	private void initRobot() {
		
	}
}
