package Utils;

import java.net.Socket;
import java.time.LocalDateTime;
import java.time.LocalTime;
import java.time.format.DateTimeFormatter;
import java.net.*;
import java.io.*;

public class ConnectionManager {
	private static ConnectionManager connMgr;
	private final String SSID = "MDPGrp2";
	private final String Static_IP = "192.168.2.2";
    //PrintStream streamToServer;
   // BufferedReader streamFromServer;
    private static Socket toServer = null;
    private BufferedWriter streamToServer;
    private BufferedReader streamFromServer;
    
	public ConnectionManager() {		
	}
	
	//singleton
	public static ConnectionManager getConnMgr() {
		if(connMgr == null){
			connMgr = new ConnectionManager();
		}
		return connMgr;
	}
	
	public void openConnection() {	
		try {
			System.out.println("Connecting ..");
			int PORT = 1273;
	
			toServer = new Socket(Static_IP,PORT);
			//System.out.println("test");
	        streamToServer = new BufferedWriter(new OutputStreamWriter(new BufferedOutputStream(toServer.getOutputStream())));
	        streamFromServer = new BufferedReader(new InputStreamReader(toServer.getInputStream()));
	        System.out.println("Connection established successfully!");

        } catch (IOException e) {
            System.out.println("DEBUG :: Connection failed -> IOException");
        } catch (Exception e) {
            System.out.println("DEBUG :: Connection failed -> Exception");
            System.out.println(e.toString());
        }

	}
	
	public void closeConnection() {		
        try {
            streamFromServer.close();

            if (toServer != null) {
            	toServer.close();
            	toServer = null;
            }
            System.out.println("Connection closed!");
        } catch (Exception e) {
            System.out.println("DEBUG :: Close Connection failed -> Exception");
            System.out.println(e.toString());
        }
	}
	
	public void sendMessage(String msg, String msgType) {	
        System.out.println("Sending a message!");
        try {
            String outputMsg = "Error in setting message";
            if(msgType.equalsIgnoreCase("AND"))
            	outputMsg = "AN,PC,"+msg;
            else if(msgType.equalsIgnoreCase("ARD"))
            	outputMsg = "AR,PC,"+msg;
            else if(msgType.equalsIgnoreCase("RPI"))
            	outputMsg = "RPI,PC,"+msg;
            DateTimeFormatter dtf = DateTimeFormatter.ofPattern("yyyy/MM/dd HH:mm::ss");
            LocalDateTime now = LocalDateTime.now();
            System.out.println("("+dtf.format(now) + ") Sending out message:\n" + outputMsg);
            streamToServer.write(outputMsg);
            streamToServer.flush();
        } catch (Exception e) {
            System.out.println("DEBUG :: Sending Message failed -> Exception");
            System.out.println(e.toString());
        }
	}
	
	public String receiveMessage() {		
        try {
            StringBuilder sb = new StringBuilder();
            String input = streamFromServer.readLine();
            while(input == null || input.isEmpty()) {
            	input = streamFromServer.readLine();
            }
           // while(reader.ready() && (input = reader.readLine())!=null)
            //	break;
        
            sb.append(input);
            System.out.println(" Receiving message :"+sb.toString());
            
            return sb.toString();
        } catch (Exception e) {
            System.out.println("DEBUG :: Receiving Message failed -> IOException");
            System.out.println(e.toString());
        }

        return null;
    }
	
}
