package Utils;

import Map.Map;
import Map.MapConstants;

public class MapDescriptor {
	static MapDescriptor mdp = null;
	public MapDescriptor() {
	}
	
	public static MapDescriptor getMapDescriptor() {
		if(mdp==null)
			mdp = new MapDescriptor();
		
		return mdp;
	}
	//part 1
	public String[] generateHexadecimalEqvl(Map m) {
		///System.out.println("DEBUG :: Generating Hexadecimal");
		String[] mdp= new String[2];
		StringBuilder hexa = new StringBuilder();
		StringBuilder hexaFinal = new StringBuilder();
		
		int row = MapConstants.MAP_ROWS;
		int col = MapConstants.MAP_COLS;
		//padding start
		hexa.append("11");
		
		//start from bottom
		for(int c = col-1; c>=0;c--) {
			for(int r = 0; r<row;r++) {
				if(m.getTile(r,c).getIsExplored())
					hexa.append("1");
				else
					hexa.append("0");
				
				if(hexa.length()==4) {
					hexaFinal.append(Integer.toHexString(Integer.parseInt(hexa.toString(),2)));
					hexa.setLength(0);
				}				
			}
		}
		//padding end
		hexa.append("11");
		hexaFinal.append(Integer.toHexString(Integer.parseInt(hexa.toString(),2)));
		//System.out.println(hexaFinal.toString());
		//part 1 add
		mdp[0] = hexaFinal.toString();
		
		StringBuilder hexa2 = new StringBuilder();
		StringBuilder hexaFinal2 = new StringBuilder();
		//part2
		for(int c = col-1; c>=0;c--) {
			for(int r =0; r<row; r++) {
				if(m.getTile(r,c).getIsExplored()) {
					if(m.getTile(r,c).getIsObstacle())
						hexa2.append("1");
					else
						hexa2.append("0");
					
					if(hexa2.length()==4) {
						hexaFinal2.append(Integer.toHexString(Integer.parseInt(hexa2.toString(),2)));
						hexa2.setLength(0);
					}
				}
			}
		}
			
		//if got leftover binaries
		if(hexa2.length()>0)
			hexaFinal2.append(Integer.toHexString(Integer.parseInt(hexa2.toString(),2)));
		mdp[1]=hexaFinal2.toString();
		
		return mdp;
	}
	
	
}
