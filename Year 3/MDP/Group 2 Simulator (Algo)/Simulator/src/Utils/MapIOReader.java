package Utils;

import java.io.*;

import Map.Map;
import Map.MapConstants;

public class MapIOReader {
	public MapIOReader() {
	}
	
	public void loadMapFile(Map map, String fileName) {
		try {
			InputStream in = new FileInputStream("src/Utils/GridMaps/"+fileName+".txt");
			BufferedReader reader = new BufferedReader(new InputStreamReader(in));
			StringBuilder out = new StringBuilder();
			String line;
			
			while(((line=reader.readLine())!=null)) {
				out.append(line+"\n");
			}
			
			reader.close();
			
			String mapStream = out.toString().replace("\n","");
			int ptr=0;
			
			///build map object	
			for(int y=0;y<MapConstants.MAP_COLS;y++) {
				for(int x=0;x<MapConstants.MAP_ROWS;x++) {
					if(mapStream.charAt(ptr)=='1') {
						map.getTile(x,y).setIsObstacle(true);
						map.getTile(x,y).setIsVirtualWall(true); //setTileVirtualWall(x,y);
					}
					
					ptr++;
				}
				System.out.println("");
			}
			
		}catch(Exception e) {
			System.out.println(e.getMessage());
		}
	}	
}
