package Map;

import java.awt.Color;

public class MapConstants {
    public static final int TILE_WEIGHT = 1;
    public static final int TILE_SIZE = 20;
    
    public static final Color TILE_EXPLORED = Color.WHITE;
    public static final Color TILE_UNEXPLORED = Color.GRAY;
    public static final Color TILE_OBSTACLE = Color.BLACK;
    public static final Color TILE_START = Color.GREEN;
    public static final Color ROBOT_CENTER = Color.BLUE;
    public static final Color ROBOT_HEADING = Color.RED;
    public static final Color TILE_PATH = Color.MAGENTA;
    public static final Color TILE_WAYPOINT = Color.orange;
    public static final Color TILE_VIRTUALWALL = Color.cyan;
    
    public static final int MAP_ROWS = 15;
    public static final int MAP_COLS = 20;
    
    public static final int ZONE_SIZE =3;
    
    public static final int START_ZONE = MAP_COLS-ZONE_SIZE;
    public static final int GOAL_ZONE = MAP_ROWS-ZONE_SIZE;
    
}
