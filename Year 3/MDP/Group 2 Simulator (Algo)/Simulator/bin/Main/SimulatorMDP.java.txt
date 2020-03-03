package Main;

import java.awt.BorderLayout;
import java.awt.CardLayout;
import java.awt.Color;
import java.awt.ComponentOrientation;
import java.awt.Dimension;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.Hashtable;

import javax.swing.*;
import javax.swing.border.EmptyBorder;
import javax.swing.event.DocumentEvent;
import javax.swing.event.DocumentListener;

import Algorithms.ExplorationAlgo;
import Image_Algorithms.Image_ExplorationAlgo;

import Map.Map;
import Map.MapConstants;
import Robot.Robot;
import Robot.RobotConstants;
import Utils.ConnectionManager;
import Utils.MapIOReader;

public class SimulatorMDP {
	//constant final variables
	public static final int TILE_WEIGHT =1;
	public static final int TILE_SIZE = 30;
	
	public static final int ARENA_WIDTH = 1600;
	public static final int ARENA_HEIGHT = 400;
	
	//GUI Frame init
	private static JFrame mainFrame = null;
	private static JPanel mainPanel = null;
	private static JPanel mapPanel = null;
	private static JPanel mapPanel2 = null;
	private static JPanel cardMap_real = null;
	private static JPanel cardMap_simulated = null; 
    private static JPanel sidePanel = null;
    private static JPanel buttonPanel = null;
    private static JPanel settingsPanel=null;
    private static JPanel debugPanel = null;
    private static GroupLayout layout= null;
    
    //variables init
    private static Map simulationMap = null;
    private static Map masked_simulationMap = null;
    //private static Map explored_map = null;
    private static Robot robot =null;
    private static long robotSpeed = 100;
    
    private static Image_ExplorationAlgo exploration = null;
	private static JTextField txtCoverage;
	private static JTextField txtTime;
	private static JTextField txtNumSteps;
	private static JTextField txtXWaypoint;
	private static JTextField txtYWaypoint;
	private static JSlider slider = new JSlider();
	
	public static JLabel lblTimePassed;
	public static JLabel lblAreaExplored;
	public static JLabel lblRobotPos;
	public static JTextArea lblSensorInfo;
    private static final boolean realRun = false;
    private static final boolean imageRun = false;
    
    //threads
    public static ExplorationBG exploreWorker = null; //new ExplorationBG();
    
    public static void main(String[] args) {
        SwingUtilities.invokeLater(new Runnable() {
            public void run() {
    	MapIOReader io = new MapIOReader();    	
    	//init 
    	robot = new Robot(RobotConstants.START_POS_X, RobotConstants.START_POS_Y, realRun, robotSpeed);
    	simulationMap = new Map(robot,false);    	
    	masked_simulationMap = new Map(robot,true);		
		io.loadMapFile(simulationMap, "BlankMap");
		displayEverything();
            }
        });
    }
    
    //multithread
    static class FastestPathBG extends SwingWorker<Integer,String>{
    	protected Integer doInBackground() throws Exception{
    		exploration.startFastestPathAlgo();
    		return 555;
    	}
    }
    
    //multithread
    public static class ExplorationBG extends SwingWorker<Integer,String>{    		
    	protected Integer doInBackground() throws Exception{
    		//	public ExplorationAlgo(Map real_map, Map explored_map, Robot robot, long timeLimit, int coverageLimit) {    	
    		robot.setRow(RobotConstants.START_POS_X);
    		robot.setCol(RobotConstants.START_POS_Y);
    		if(exploration != null) {
    			exploration = null;
    			masked_simulationMap =null;
    			//robotSpeed = 1000;
    	    	robot = new Robot(RobotConstants.START_POS_X, RobotConstants.START_POS_Y, realRun, robotSpeed);
    			masked_simulationMap = new Map(robot,true);
    			initMap();
    		}
    		processExploration();
		    System.out.println("Done setting exploration");
		    processWP();
		    System.out.println("Done setting WP");
    		exploration.explore();
			System.out.println("Finish exploration!");
			if(robot.getRealRun()) {
				System.out.println("Sending Explored to AND");
				ConnectionManager.getConnMgr().sendMessage("Explored", "AND");
				new FastestPathBG().execute();
			}
    		return 555;
    	}    	
    }
    public static void processWP() {

		if(!realRun) {
			if(Integer.parseInt(txtXWaypoint.getText()) == -1 || Integer.parseInt(txtYWaypoint.getText())==-1)
				return;
			if(exploration!=null) {
				exploration.setAndReplaceWP(Math.abs(Integer.parseInt(txtXWaypoint.getText())), Math.abs(Integer.parseInt(txtYWaypoint.getText())-MapConstants.MAP_COLS));
			}
			else {
				simulationMap.setWaypoint(Math.abs(Integer.parseInt(txtXWaypoint.getText())), Math.abs(Integer.parseInt(txtYWaypoint.getText())-MapConstants.MAP_COLS));
				masked_simulationMap.setWaypoint(Math.abs(Integer.parseInt(txtXWaypoint.getText())), Math.abs(Integer.parseInt(txtYWaypoint.getText())-MapConstants.MAP_COLS));
    			
			}		
		}
		else {
			String[] instruction;
			while(true) {
				System.out.println("WAITING FOR WAYPOINT FROM ANDROID");
				String msg = ConnectionManager.getConnMgr().receiveMessage();
				if(msg!=null) {
					instruction = msg.split(":");
					if(instruction[0].equalsIgnoreCase("WP"))
						break;
				}
			}
			//String[] waypoint = instruction[1].split(",");
			int and_x = Integer.parseInt(instruction[1])-1;
			int and_y = Math.abs(Integer.parseInt(instruction[2])-MapConstants.MAP_COLS);
			if(and_x != -2) {
			int wp_x = and_x;
			int wp_y = and_y;
			
			System.out.println("Setting waypoint -> X:"+wp_x+ " Y:"+wp_y);
			//exploration.setAndReplaceWP(Math.abs(Integer.parseInt(waypoint[0])-MapConstants.MAP_ROWS), Math.abs(Integer.parseInt(txtYWaypoint.getText())-MapConstants.MAP_COLS)-1);
			//masked_simulationMap.setWaypoint(Math.abs(Integer.parseInt(txtXWaypoint.getText())), Math.abs(Integer.parseInt(txtYWaypoint.getText())-MapConstants.MAP_COLS));
			exploration.setAndReplaceWP(wp_x, wp_y);
			//exploration.startFastestPathAlgo();
			}
		}
		
	
    }
	public static void processExploration() {
		// TODO Auto-generated method stub
		if(realRun) {
			if(!imageRun)
				exploration = new Image_ExplorationAlgo(simulationMap, masked_simulationMap, robot, Integer.parseInt(txtTime.getText()), (int)(((Double.parseDouble(txtCoverage.getText())/100))*300), slider.getValue()*5,false);
			else			
				exploration = new Image_ExplorationAlgo(simulationMap, masked_simulationMap, robot, Integer.parseInt(txtTime.getText()), (int)(((Double.parseDouble(txtCoverage.getText())/100))*300), slider.getValue()*5,true);
			}
		else {
			exploration = new Image_ExplorationAlgo(simulationMap, masked_simulationMap, robot, Integer.parseInt(txtTime.getText()), (int)(((Double.parseDouble(txtCoverage.getText())/100))*300), slider.getValue()*5,true);
		}
	}
	public static void cancelThreads() {
		exploreWorker.cancel(true);
		exploration.displayMgt.cancel(true);
	}
    private static void displayEverything() {
	    // Initialise main frame for display
	    mainFrame = new JFrame();
	    mainFrame.setTitle("Group 2 Simulation MDP");
	    mainFrame.setSize(new Dimension(ARENA_WIDTH,ARENA_HEIGHT));
	    mainFrame.setResizable(true);

	    mainPanel = new JPanel(); 
	    //Map Panel
	    mapPanel = new JPanel(new CardLayout());
	    mapPanel.setPreferredSize(new Dimension(350,480));
	    mapPanel.setBackground(Color.LIGHT_GRAY);
	    mapPanel.setBorder(new EmptyBorder(10,10,10,10)); 
	    
	    mapPanel2 = new JPanel(new CardLayout());
	    mapPanel2.setPreferredSize(new Dimension(350,480));
	    mapPanel2.setBackground(Color.LIGHT_GRAY);
	    mapPanel2.setBorder(new EmptyBorder(10,10,10,10)); 
	    
	    
	    sidePanel = new JPanel(new GridLayout(2,0,10,20));
	    sidePanel.setBorder(new EmptyBorder(1,1,1,1));
	    
	    buttonPanel = new JPanel(new GridLayout(0,2,5,5));
	    buttonPanel.setBorder(new EmptyBorder(5,5,5,5));
	    
	    settingsPanel = new JPanel(new GridBagLayout());
	    settingsPanel.setBorder(new EmptyBorder(20,5,5,5));
	    
	    debugPanel = new JPanel(new GridLayout(0,2,5,5));
	    debugPanel.setBorder(new EmptyBorder(5,5,5,5));
	    
	    layout = new GroupLayout(mainPanel);
	    mainPanel.setLayout(layout);
	     
	    initMainLayout();
	    mainFrame.pack();
	    mainFrame.setVisible(true);
		mainFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
    }
    
    private static void initMainLayout() {
    	if(simulationMap!=null) {
	    	cardMap_real = new JPanel(new BorderLayout());
	    	cardMap_real.add(simulationMap,BorderLayout.CENTER);
	    	cardMap_real.setBackground(Color.LIGHT_GRAY);
	    	mapPanel.add(cardMap_real,"Shown map");
    	}
    	
    	cardMap_simulated = new JPanel(new BorderLayout());
    	cardMap_simulated.add(masked_simulationMap,BorderLayout.CENTER);
    	cardMap_simulated.setBackground(Color.LIGHT_GRAY);
    	mapPanel2.add(cardMap_simulated,"Masked map");
    	
    	JButton connectBtn = new JButton("Connect");
    	buttonPanel.add(connectBtn);
    	connectBtn.addActionListener(new ActionListener(){
    		public void actionPerformed(ActionEvent e) {
    			ConnectionManager.getConnMgr().openConnection();
    		}
    	});
    	
    	JButton disconnectBtn = new JButton("Disconnect");
    	buttonPanel.add(disconnectBtn);
    	disconnectBtn.addActionListener(new ActionListener(){
    		public void actionPerformed(ActionEvent e) {
    			ConnectionManager.getConnMgr().closeConnection();
    			
    		}
    	});
    	
    	JButton btn1 = new JButton("Load Map");
    	//btn1.setPreferredSize(new Dimension(50,50));
    	buttonPanel.add(btn1);
    	btn1.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				// TODO Auto-generated method stub
				showOptionPane();
			}
    	});
    	
    	JButton mdfBtn = new JButton("MDF String");
    	buttonPanel.add(mdfBtn);
    	mdfBtn.addActionListener(new ActionListener() {
    		public void actionPerformed(ActionEvent e) {
    			if(exploration==null)
    				exploration = new Image_ExplorationAlgo(simulationMap, masked_simulationMap, robot, Integer.parseInt(txtTime.getText()), (int)(((Double.parseDouble(txtCoverage.getText())/100))*300), slider.getValue(),false);//Integer.parseInt(txtNumSteps.getText()));
    		    
    			exploration.updateMap();
    		}
    	});
    	
    	JButton btn2 = new JButton("Exploration");
    	buttonPanel.add(btn2);
    	btn2.addActionListener(new ActionListener(){
			@Override
			public void actionPerformed(ActionEvent e) {
				exploreWorker = new ExplorationBG();
				// TODO Auto-generated method stub
				//new ExplorationBG().execute();
				exploreWorker.execute();//exploreWorker.execute();
			}
    		
    	});
    	
    	JButton stopExploration = new JButton("Stop Explr");
    	buttonPanel.add(stopExploration);
    	stopExploration.addActionListener(new ActionListener() {
    		@Override
    		public void actionPerformed(ActionEvent e) {
    			System.out.println("STOPPING EXPLORATION!!!!");
    			if(realRun) {
    				ConnectionManager.getConnMgr().sendMessage("Explored", "AND");
    			}
    			cancelThreads();
    		}
    	});
    	
    	JButton btn6 = new JButton("Fastest Path");
    	buttonPanel.add(btn6);
    	btn6.addActionListener(new ActionListener(){
			@Override
			public void actionPerformed(ActionEvent e) {
				// TODO Auto-generated method stub
				new FastestPathBG().execute();
				//masked_simulationMap.repaint();
				//_appFrame.repaint();
			}
    		
    	});
    	
       	JButton btn4 = new JButton("Stop FPA");
    	buttonPanel.add(btn4);
    	btn4.addActionListener(new ActionListener(){
			@Override
			public void actionPerformed(ActionEvent e) {
				// TODO Auto-generated method stub
				/*
				exploration = new Exploration(simulationMap, masked_simulationMap, robot, 100);
				int test = exploration.checkR();
				System.out.println(test);*/
				
				//exploration.fastestPathAlgorithm();
				//masked_simulationMap.repaint();
				//_appFrame.repaint();
			}
    		
    	});
    	
    	JLabel lblCoverage = new JLabel("Coverage Limit (%): ");
    	txtCoverage = new JTextField("100");
    	txtCoverage.setPreferredSize(new Dimension(120,18));
    	txtCoverage.getDocument().addDocumentListener(new DocumentListener() {
    		  public void changedUpdate(DocumentEvent e) {
    		    warn();
    		  }
    		  public void removeUpdate(DocumentEvent e) {
    		    warn();
    		  }
    		  public void insertUpdate(DocumentEvent e) {
    		    warn();
    		  }

    		  public void warn() {
    			  try {
    		     if (Integer.parseInt(txtCoverage.getText())<=0){
    		       JOptionPane.showMessageDialog(null,
    		          "Error: Please enter number bigger than 0", "Error Message",
    		          JOptionPane.ERROR_MESSAGE);
    		     }
    		     else if (Integer.parseInt(txtCoverage.getText())>100){
      		       JOptionPane.showMessageDialog(null,
      		          "Error: Please enter number smaller than 100", "Error Message",
      		          JOptionPane.ERROR_MESSAGE);
      		     }
    		      }catch(NumberFormatException e)
    			  {
    		    	  System.out.println("Invalid format");
    				  return;
    			  }
    			  }
    		});
    	
    	JLabel lblTime = new JLabel("Time Limit (Seconds): ");
    	txtTime = new JTextField("360");
    	txtTime.setPreferredSize(new Dimension(120,18));
    	txtTime.getDocument().addDocumentListener(new DocumentListener() {
  		  public void changedUpdate(DocumentEvent e) {
  		    warn();
  		  }
  		  public void removeUpdate(DocumentEvent e) {
  		    warn();
  		  }
  		  public void insertUpdate(DocumentEvent e) {
  		    warn();
  		  }

  		  public void warn() {
  			  try {
  		     if (Integer.parseInt(txtTime.getText())<=0){
  		       JOptionPane.showMessageDialog(null,
  		          "Error: Please enter number bigger than 0", "Error Message",
  		          JOptionPane.ERROR_MESSAGE);
  		     }
  		     else if (Integer.parseInt(txtTime.getText())>360){
    		       JOptionPane.showMessageDialog(null,
    		          "Error: Please enter number smaller than 360 (Seconds)", "Error Message",
    		          JOptionPane.ERROR_MESSAGE);
    		     }
  		      }catch(NumberFormatException e)
  			  {
  		    	  System.out.println("Invalid format");
  				  return;
  			  }
  			  }
  		});
    	
    	JLabel lblSteps = new JLabel("Number of Steps: ");
    	txtNumSteps = new JTextField("100");
    	txtNumSteps.setPreferredSize(new Dimension(120,18));
    	
        // Set major or minor ticks for the slider
    	slider = new JSlider(0,20,10);
        //slider.setMajorTickSpacing(5);
        slider.setMinorTickSpacing(1);
        slider.setPaintTicks(true);
         
        // Set the labels to be painted on the slider
        slider.setPaintLabels(true);
         
        // Add positions label in the slider
        Hashtable position = new Hashtable();
        position.put(0, new JLabel("0"));

        position.put(5, new JLabel("5"));

        position.put(10, new JLabel("10"));

        position.put(15, new JLabel("15"));

        position.put(20, new JLabel("20"));

         
        // Set the label to be drawn
        slider.setLabelTable(position);
         
        // Add the slider to the panel

    	JLabel lblStats = new JLabel("Stats: ");
    	lblTimePassed = new JLabel("Time Passed: ");
    	lblAreaExplored = new JLabel("Area Explored: ");
    	lblRobotPos = new JLabel("I am at -> X : - Y : -");

    	lblSensorInfo = new JTextArea();
    	lblSensorInfo.setColumns(10);
    	lblSensorInfo.setRows(3);
    	//lblSensorInfo.setLineWrap(true);
    	lblSensorInfo.setWrapStyleWord(true);
    	lblSensorInfo.setEditable(false);
    	lblSensorInfo.setBorder(null);

    	JScrollPane jsp = new JScrollPane(lblSensorInfo);
    	jsp.setPreferredSize(new Dimension(600,70));
    	
    	JLabel lblXWaypoint = new JLabel("Waypoint (X): ");
    	JLabel lblYWaypoint = new JLabel("Waypoint (Y): ");
    	txtXWaypoint = new JTextField("-1");
    	txtYWaypoint = new JTextField("-1");    	
    	txtXWaypoint.setPreferredSize(new Dimension(120,18));
    	txtYWaypoint.setPreferredSize(new Dimension(120,18));
    	    	
	    GridBagConstraints c =new GridBagConstraints();
	    c.anchor = GridBagConstraints.NORTHWEST;
	    c.weighty=1;
	    c.weightx=1;
	    c.gridx=0;
	    c.gridy=0;
    	settingsPanel.add(lblCoverage,c);
    	c.gridx=1;
    	c.gridy=0;
    	c.gridwidth=3;
    	settingsPanel.add(txtCoverage,c);
	    //c.weighty=1;
	    //c.weightx=1;
	    c.gridx=0;
	    c.gridy=1;
    	settingsPanel.add(lblTime,c);
    	c.gridx=1;
    	c.gridy=1;
    	c.gridwidth=3;
    	settingsPanel.add(txtTime,c);
	    c.gridx=0;
	    c.gridy=2;
    	settingsPanel.add(lblSteps,c);
    	c.gridx=0;
    	c.gridy=3;
    	c.gridwidth=4;
    	settingsPanel.add(slider,c);
	    c.gridx=0;
	    c.gridy=4;
    	settingsPanel.add(lblXWaypoint,c);
    	c.gridx=1;
    	c.gridy=4;
    	c.gridwidth=3;
    	settingsPanel.add(txtXWaypoint,c);
	    c.gridx=0;
	    c.gridy=5;
    	settingsPanel.add(lblYWaypoint,c);
    	c.gridx=1;
    	c.gridy=5;
    	c.gridwidth=3;
    	settingsPanel.add(txtYWaypoint,c);
	    c.gridx=0;
	    c.gridy=7;
    	settingsPanel.add(lblStats,c);
    	c.gridx=0;
    	c.gridy=8;
    	c.gridwidth=3;
    	settingsPanel.add(lblAreaExplored,c);
    	c.gridx=0;
    	c.gridy=9;
    	c.gridwidth=3;
    	settingsPanel.add(lblTimePassed,c);
    	c.gridx=0;
    	c.gridy=10;
    	c.gridwidth=3;
    	settingsPanel.add(lblRobotPos,c);
    	c.gridx=0;
    	c.gridy=11;
    	c.gridwidth=10;
    	settingsPanel.add(jsp,c);
    	
    	/*
    	JTextArea txtCoverage = new JTextArea("Coverage Limit :");
    	txtCoverage.setColumns(5);
    	txtCoverage.setWrapStyleWord(true);
    	txtCoverage.setEditable(false);
    	txtCoverage.setBorder(null);
    	settingsPanel.add(txtCoverage);
    	*/
    	
 
    	
    	JButton setNewWP = new JButton("Set Waypoint");
    	buttonPanel.add(setNewWP);
    	setNewWP.addActionListener(new ActionListener() {
    		@Override
    		public void actionPerformed(ActionEvent e) {
    			
    			if(!realRun) {
	    			if(Integer.parseInt(txtXWaypoint.getText()) == -1 || Integer.parseInt(txtYWaypoint.getText())==-1)
	    				return;
	    			if(exploration!=null) {
	    				exploration.setAndReplaceWP(Math.abs(Integer.parseInt(txtXWaypoint.getText())), Math.abs(Integer.parseInt(txtYWaypoint.getText())-MapConstants.MAP_COLS));
	    			}
	    			else {
	    				simulationMap.setWaypoint(Math.abs(Integer.parseInt(txtXWaypoint.getText())), Math.abs(Integer.parseInt(txtYWaypoint.getText())-MapConstants.MAP_COLS));
	    				masked_simulationMap.setWaypoint(Math.abs(Integer.parseInt(txtXWaypoint.getText())), Math.abs(Integer.parseInt(txtYWaypoint.getText())-MapConstants.MAP_COLS));
		    			
	    			}		
    			}
    			else {
    				String[] instruction;
    				while(true) {
    					System.out.println("WAITING FOR WAYPOINT FROM ANDROID");
    					String msg = ConnectionManager.getConnMgr().receiveMessage();
    					if(msg!=null) {
    						instruction = msg.split(":");
    						if(instruction[0].equalsIgnoreCase("WP"))
    							break;
    					}
    				}
    				//String[] waypoint = instruction[1].split(",");
    				int and_x = Integer.parseInt(instruction[1])-1;
    				int and_y = Math.abs(Integer.parseInt(instruction[2])-MapConstants.MAP_COLS);
    				if(and_x != -2) {
    				int wp_x = and_x;
    				int wp_y = and_y;
    				
					System.out.println("Setting waypoint -> X:"+wp_x+ " Y:"+wp_y);
					//exploration.setAndReplaceWP(Math.abs(Integer.parseInt(waypoint[0])-MapConstants.MAP_ROWS), Math.abs(Integer.parseInt(txtYWaypoint.getText())-MapConstants.MAP_COLS)-1);
					masked_simulationMap.setWaypoint(Math.abs(Integer.parseInt(txtXWaypoint.getText())), Math.abs(Integer.parseInt(txtYWaypoint.getText())-MapConstants.MAP_COLS));
					exploration.setAndReplaceWP(wp_x, wp_y);
					//exploration.startFastestPathAlgo();
    				}
    			}
    			
    		}
    	});
    	
    	JButton goBackH = new JButton("Go back home");
    	buttonPanel.add(goBackH);
    	goBackH.addActionListener(new ActionListener() {
    		@Override
    		public void actionPerformed(ActionEvent e) {
    			if(exploration!=null)
    				exploration.goBackToStartFollowRight();    			
    		}
    	});
    	
    	JButton secondDebug = new JButton("2nd debug");
    	buttonPanel.add(secondDebug);
    	secondDebug.addActionListener(new ActionListener() {
    		@Override
    		public void actionPerformed(ActionEvent e) {
    			setImage();
    			}
    		}
    	);
    	JButton btnNE = new JButton("Unexplored");
    	debugPanel.add(btnNE);
    	btnNE.addActionListener(new ActionListener() {
    		@Override
    		public void actionPerformed(ActionEvent e) {
    			//exploration.checkNextNonExplored();
    		}
    	});
    	
    	sidePanel.add(settingsPanel); 
    	sidePanel.add(buttonPanel);
    	if(!realRun) {
    	layout.setHorizontalGroup(
    			layout.createSequentialGroup()
    			.addComponent(mapPanel,GroupLayout.PREFERRED_SIZE, GroupLayout.DEFAULT_SIZE,
    			          GroupLayout.PREFERRED_SIZE)
    			.addComponent(mapPanel2,GroupLayout.PREFERRED_SIZE, GroupLayout.DEFAULT_SIZE,
    			          GroupLayout.PREFERRED_SIZE)
    			.addComponent(sidePanel,GroupLayout.PREFERRED_SIZE, GroupLayout.DEFAULT_SIZE,
    			          GroupLayout.PREFERRED_SIZE)
		);
    	layout.setVerticalGroup(
    			layout.createSequentialGroup()
    			.addGroup(layout.createParallelGroup(GroupLayout.Alignment.BASELINE)
    			.addComponent(mapPanel,GroupLayout.PREFERRED_SIZE, GroupLayout.DEFAULT_SIZE,
    			          GroupLayout.PREFERRED_SIZE)
    			.addComponent(mapPanel2,GroupLayout.PREFERRED_SIZE, GroupLayout.DEFAULT_SIZE,
    			          GroupLayout.PREFERRED_SIZE)
    			.addComponent(sidePanel,GroupLayout.PREFERRED_SIZE, GroupLayout.DEFAULT_SIZE,
    			          GroupLayout.PREFERRED_SIZE)
		)); 
    	}
    	else {
        	layout.setHorizontalGroup(
        			layout.createSequentialGroup()
        			.addComponent(mapPanel2,GroupLayout.PREFERRED_SIZE, GroupLayout.DEFAULT_SIZE,
        			          GroupLayout.PREFERRED_SIZE)
        			.addComponent(sidePanel,GroupLayout.PREFERRED_SIZE, GroupLayout.DEFAULT_SIZE,
        			          GroupLayout.PREFERRED_SIZE)
    		);
        	layout.setVerticalGroup(
        			layout.createSequentialGroup()
        			.addGroup(layout.createParallelGroup(GroupLayout.Alignment.BASELINE)
        			.addComponent(mapPanel2,GroupLayout.PREFERRED_SIZE, GroupLayout.DEFAULT_SIZE,
        			          GroupLayout.PREFERRED_SIZE)
        			.addComponent(sidePanel,GroupLayout.PREFERRED_SIZE, GroupLayout.DEFAULT_SIZE,
        			          GroupLayout.PREFERRED_SIZE)
    		)); 
    	}

    	
    mainFrame.add(mainPanel);
    	
    }
    //init/ reinit map
    public static void initMap() {
    	mapPanel.remove(cardMap_real);
    	cardMap_real = new JPanel(new BorderLayout());
    	cardMap_real.add(simulationMap,BorderLayout.CENTER);
    	mapPanel.add(cardMap_real,"Shown map");
    	mapPanel.revalidate();
    	mapPanel.repaint();  
    	
    	
    	mapPanel2.remove(cardMap_simulated);
    	cardMap_simulated = new JPanel(new BorderLayout());
    	cardMap_simulated.add(masked_simulationMap, BorderLayout.CENTER); 	
    	mapPanel2.add(cardMap_simulated,"Masked map");
    	mapPanel2.validate();
    	mapPanel2.repaint();
    	//masked_simulationMap.paintAgain();
    }
    
    public static void showOptionPane() {	    	
		robot = new Robot(RobotConstants.START_POS_X,RobotConstants.START_POS_Y,realRun,robotSpeed); //0 = filler value
		
		simulationMap = new Map(robot, false);
		
		masked_simulationMap = new Map(robot, true);
		
		//initiailize robot sensor
		//robot.setMap(simulationMap,masked_simulationMap);
    	
    	String mapInput = JOptionPane.showInputDialog("Load map?","SampleArena4");
    	//System.out.println(mapInput);
    	
		MapIOReader io = new MapIOReader();
		io.loadMapFile(simulationMap, mapInput);
		exploration = new Image_ExplorationAlgo(simulationMap, masked_simulationMap, robot, Integer.parseInt(txtTime.getText()), (int)(((Double.parseDouble(txtCoverage.getText())/100))*300), slider.getValue()*5,false);//Integer.parseInt(txtNumSteps.getText()));
	        //io.loadMapFile(masked_simulationMap, mapInput);
		
		initMap();
    }
    
    public static void setImage() {
        JTextField xField = new JTextField(5);
        JTextField yField = new JTextField(5);
        JTextField id = new JTextField(5);
        JTextField heading = new JTextField(5);
        
        JPanel myPanel = new JPanel();
        myPanel.add(new JLabel("x:"));
        myPanel.add(xField);
        myPanel.add(Box.createHorizontalStrut(15)); // a spacer
        myPanel.add(new JLabel("y:"));
        myPanel.add(yField);
        myPanel.add(Box.createHorizontalStrut(15)); // a spacer
        myPanel.add(new JLabel("id:"));
        myPanel.add(id);
        myPanel.add(Box.createHorizontalStrut(15)); // a spacer
        myPanel.add(new JLabel("Heading:"));
        myPanel.add(heading);

        int result = JOptionPane.showConfirmDialog(null, myPanel, 
                 "Please Enter X and Y Values", JOptionPane.OK_CANCEL_OPTION);
        if (result == JOptionPane.OK_OPTION) {
        	simulationMap.getTile(Integer.parseInt(xField.getText()),Integer.parseInt(yField.getText())).setIsImage(true,id.getText(),heading.getText());
        	masked_simulationMap.getTile(Integer.parseInt(xField.getText()),Integer.parseInt(yField.getText())).setIsImage(true,id.getText(),heading.getText());
        	simulationMap.paintAgain();
        }
    }
}
