package Robot;

import Map.Map;

public class Sensor {
	private int long_range;
	private int short_range;
	private int row;
	private int col;
	private RobotConstants.HEADING currentHeading;
	private String id;
	
	public Sensor(int short_range, int long_range, int row, int col, RobotConstants.HEADING h, String id) {
		this.short_range = short_range;
		this.long_range = long_range;
		this.row = row;
		this.col = col;
		this.currentHeading = h;
		this.id = id;
	}
	
	public RobotConstants.HEADING getCurrentHeading(){return this.currentHeading;}
	
	public void setPosition(int row, int col, RobotConstants.HEADING h) {
		this.row = row;
		this.col = col;
		this.currentHeading = h;
	}
	
	
	public int senseReal(Map real_map) {
		switch(currentHeading) {
		case NORTH:
			return checkRealMap(real_map,0,-1);
		case EAST:
			return checkRealMap(real_map,1,0);
		case SOUTH:
			return checkRealMap(real_map,0,1);
		case WEST:
			return checkRealMap(real_map,-1,0);
		default:
			break;
		}
		return -1;
	}
	
	private int checkRealMap(Map real_map, int rowVal, int colVal) {
		for(int i = this.short_range; i<= long_range; i++) {
			try {
				if(real_map.getTile(row+(rowVal*i),col+(colVal*i)).getIsObstacle()) {
						return i;
				}			
			}catch(IndexOutOfBoundsException e) {
				if(i==1)
					return i;
				else
					return -1;
			}
			
		}
		return -1;
	}
	
	public int senseImage(Map real_map) {
		switch(currentHeading) {
		case NORTH:
			return checkRealMapImage(real_map,0,-1);
		case EAST:
			return checkRealMapImage(real_map,1,0);
		case SOUTH:
			return checkRealMapImage(real_map,0,1);
		case WEST:
			return checkRealMapImage(real_map,-1,0);
		default:
			break;
		}
		return -1;
	}
	
	private int checkRealMapImage(Map real_map, int rowVal, int colVal) {
		for(int i = this.short_range; i<= long_range; i++) {
			try {
				if(real_map.getTile(row+(rowVal*i),col+(colVal*i)).getIsImage()) {
						return Integer.parseInt(this.id.substring(id.length()-1));
				}			
			}catch(IndexOutOfBoundsException e) {
					return -1;
			}
			
		}
		return -1;
	}
	
	//below is for real run
	/*
	 * val => sensor detect obstacle
	 */
	public int sense(Map explored_map) {
		switch(currentHeading) {
		case NORTH:
			return checkMap(explored_map,0,-1);
		case EAST:
			return checkMap(explored_map,1,0);
		case SOUTH:
			return checkMap(explored_map,0,1);
		case WEST:
			return checkMap(explored_map,-1,0);
		default:
			break;
		}
		return -1;
	}
	private int checkMap(Map explored_map, int rowVal, int colVal) {
		for(int i = this.short_range; i<= long_range; i++) {
			try {
				//if(!explored_map.getTile(row+(rowVal*i),col+(colVal*i)).getIsExplored())
					//explored_map.getTile(row+(rowVal*i),col+(colVal*i)).setIsExplored(true);
				
				if(explored_map.getTile(row+(rowVal*i),col+(colVal*i)).getIsObstacle()) {
					if(i==1)
						return i;
				}			
			}catch(IndexOutOfBoundsException e) {
				if(i==1)
					return 1;
				else
					return -1;
			}
			
		}
		return -1;
	}	
	
	
	
	public int sense(Map explored_map, int val) {
		if(val==0)
			return 1;
		switch(currentHeading) {
		case NORTH:
			return updateMap(explored_map,val,0,-1);
		case EAST:
			return updateMap(explored_map,val,1,0);
		case SOUTH:
			return updateMap(explored_map,val,0,1);
		case WEST:
			return updateMap(explored_map,val,-1,0);
		default:
			break;
		}
		return -1;
	}
	
	private int updateMap(Map explored_map, int val, int rowVal, int colVal) {
		for(int i =this.short_range; i<=long_range;i++) {
			try {
				if(!explored_map.getTile(row+(rowVal*i),col+(colVal*i)).getIsExplored() || this.id.contains("FSR")
						|| ((this.id.contains("RSR") || this.id.contains("LSR")) && i ==1)) {
					//continue;
				explored_map.getTile(row+(rowVal*i),col+(colVal*i)).setIsExplored(true);
				//testing
				if(val == i) {
					//System.out.println("Obstacle detected");
					//if(!explored_map.getTile(row+(rowVal*i),col+(colVal*i)).getIsExplored()) {
						explored_map.getTile(row+(rowVal*i),col+(colVal*i)).setIsObstacle(true);
						//explored_map.getTile(row+(rowVal*i),col+(colVal*i)).setIsExplored(true);
						
					//if(i == 1)
						return i;
					//}
				}
				if(explored_map.getTile(row+(rowVal*i),col+(colVal*i)).getIsObstacle())
					explored_map.getTile(row+(rowVal*i),col+(colVal*i)).setIsObstacle(false);
				}
				//explored_map.getTile(row+(rowVal*i),col+(colVal*i)).setIsExplored(true);
				
				if(explored_map.getTile(row+(rowVal*i),col+(colVal*i)).getIsObstacle())
					return i;
				
				
			}catch(IndexOutOfBoundsException e) {
				if(i==1)
					return i;
				else
					return -1;
			}
			
		}
		return -1;
	}	
}
