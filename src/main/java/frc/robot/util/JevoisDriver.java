package frc.robot.util;

import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;

import java.io.BufferedReader;
import java.io.InputStreamReader;

import frc.robot.Constants;
import frc.robot.util.library.simple.*;
import frc.robot.util.library.simple.parser.*;


/* 
 * This Code is heavily based on the work of Team 1736.
 * https://github.com/RobotCasserole1736/JeVoisTester/blob/master/JeVoisTest/src/org/usfirst/frc/team1736/robot/JeVoisInterface.java
 * 
 * Those code is based on Spectrum's jevois driver, with modifications for deprications, personal prefrence and naming
 * Pitt Pirates are authors of 50%+ of this code, the other is pulled from Spectrum and Eagle Force
 * 
 * Thank you to both of these teams/people for their work developing the JeVois foundations for FRC.
 * 
 * 
 */

public class JevoisDriver { 

	
	// Serial port used for getting target data from JeVois (listed as null to be changed laster
	//in the try catch for the port)
	SerialPort visionPort = null; 
	
	// USBCam and server used for broadcasting a webstream of what is seen (try catch)
	UsbCamera visionCam = null;
	MjpegServer camServer = null;
	
	
	public void PirateJevois() {
		//ConnectJeVois and Check if we are connected
		connectJeVois();
		checkJeVoisConnection();
        
        //Start listening for packets
        System.out.println("Starting JeVois Listener Threads");
		jevoisListenerThread.start();
	} 
    
	/*
	 * Connect to the JeVois Camera
	 */
	public void connectJeVois(){
		int retry_counter = 0;
		//Retry strategy to get this serial port open or switch to the other usb ports
		while(visionPort == null && retry_counter++ < 10){
			try {
				System.out.print("Creating JeVois SerialPort...");
				visionPort = new SerialPort(Constants.kBaudRate,SerialPort.Port.kUSB);
				System.out.println("SUCCESS!!");
			} catch (Exception e) {
				try {
					System.out.print("Creating JeVois SerialPort...");
					visionPort = new SerialPort(Constants.kBaudRate,SerialPort.Port.kUSB1);
					System.out.println("SUCCESS!!");
				} catch (Exception f) {
					try {
						System.out.print("Creating JeVois SerialPort...");
						visionPort = new SerialPort(Constants.kBaudRate,SerialPort.Port.kUSB2);
						System.out.println("SUCCESS!!");
					} catch (Exception g) {
						System.out.println("FAILED!!");
						System.out.println("Failed to connect to JeVois on any port!");
			            sleep(500);
			            System.out.println("Retry " + Integer.toString(retry_counter));
					}
				}
			}
		}
	}
	
	/*
	 * Check if we are able to communicate with the JeVois Camera
	 */
	public boolean checkJeVoisConnection(){
		//Test to make sure we are actually talking to the JeVois
		if(sendPing() != 0){
			DriverStation.reportError("JeVois ping test failed. Not starting vision system.", false);
			return false;
		}
		System.out.println("JeVois Connection Good!");
		return true;
	}
	
    /**
     * Open an Mjpeg streamer from the JeVois camera
     */
	public void startCameraStream1(){
		//stopCameraStream();
		Constants.serOutEnable = false; //Disables the full serOut so that we can parse the packets and use the tracking documents
		try{
			System.out.print("Starting JeVois Cam Stream 1...");
			visionCam = new UsbCamera("VisionProcCam", Constants.kJevoisCamNumber);
			visionCam.setVideoMode(PixelFormat.kMJPEG, Constants.kPixleWidth1, Constants.kPixleHeight1, Constants.kFPS1); 
			camServer = new MjpegServer("VisionCamServer", Constants.kMjpgStreamPort);
			camServer.setSource(visionCam);
			Constants.camStreamRunning = true;
			Constants.trackingEnable = true;
			System.out.println("Vision Cam Stream 1 Opened!!");
		} catch (Exception e) {
			DriverStation.reportError("Cannot start camera stream 1 from JeVois", false);
            e.printStackTrace();
		}
	}
	
    /**
     * Open an Mjpeg streamer from the JeVois camera
     */
	public void startCameraStream2(){
	//	stopCameraStream();
        Constants.serOutEnable = true; //Turns on the serout so that we can see the full serial output of the stream
		try{
			System.out.print("Starting JeVois Cam Stream 2...");
			visionCam = new UsbCamera("VisionProcCam", Constants.kJevoisCamNumber);
			visionCam.setVideoMode(PixelFormat.kMJPEG, Constants.kPixleWidth2, Constants.kPixleHeight2, Constants.kFPS2); 
			camServer = new MjpegServer("VisionCamServer", Constants.kMjpgStreamPort);
			camServer.setSource(visionCam);
			Constants.camStreamRunning = true;
			Constants.trackingEnable = false;
			System.out.println("Vision Cam Stream 2 Opened!!");
		} catch (Exception e) {
			DriverStation.reportError("Cannot start camera stream 2 from JeVois", false);
            e.printStackTrace();
		}
	}
	

	/*
	 * Set serOutEnable
	 */
	public void setSerOutEnable(boolean enable){
		Constants.serOutEnable = enable;
	}
	
	/**
	 * This is the main perodic update function for the Listener. It is intended
	 * to be run in a background task, as it will block until it gets packets. 
	 */
	private void backgroundUpdate(){
		
		//Debug - just print whatever we get on the serial port
		if (Constants.serOutEnable){
			blockAndPrintAllSerial();
		}
		
		//Real code - Grab packets and parse them.
		String packet;
		packet = blockAndGetPacket(10);
		
		//Check if we are interested in tracking data and parse it if we are.
		if (Constants.trackingEnable){
			if(packet != null){
				Constants.trackingOnline = true;
				parsePacket(packet);
			} else {
				Constants.trackingOnline = false;
				DriverStation.reportWarning("Cannot get packet from JeVois Vision Processor", false);
			}
		} else{
			Constants.trackingOnline = false;
		}
		
	}
	
	/**
	 * Send the ping command to the JeVois to verify it is connected
	 * @return 0 on success, -1 on unexpected response, -2 on timeout
	 */
    public int sendPing() {
    	int retval = -1;
        if (visionPort != null){
            retval = sendCmdAndCheck("ping");
        }
        return retval;
    }
	
    /**
     * Send commands to the JeVois to configure it for image-processing friendly parameters
     */
    public void setCamVisionProcMode() {
        if (visionPort != null){
            sendCmdAndCheck("setcam autoexp 1"); //Disable auto exposure
            sendCmdAndCheck("setcam absexp 50"); //Force exposure to a low value for vision processing
        }
    }
	
    /**
     * Send parameters to the camera to configure it for a human-readable image
     */
    public void setCamHumanDriverMode() {
        if (visionPort != null){
            sendCmdAndCheck("setcam autoexp 0"); //Enable AutoExposure
        }
    }
	
	/**
	 * Sends a command over serial to JeVois and returns immediately.
     * @param cmd String of the command to send (ex: "ping")
	 * @return number of bytes written
	 */
    private int sendCmd(String cmd){
	    int bytes;
        bytes = visionPort.writeString(cmd + "\n");
        System.out.println("wrote " +  bytes + "/" + (cmd.length()+1) + " bytes, cmd: " + cmd);
	    return bytes;
    };
    
    /**
     * Sends a command over serial to the JeVois, waits for a response, and checks that response
     * Automatically ends the line termination character.
     * @param cmd String of the command to send (ex: "ping")
     * @return 0 if OK detected, -1 if ERR detected, -2 if timeout waiting for response
     */
    public int sendCmdAndCheck(String cmd){
    	int retval = 0;
	    sendCmd(cmd);
	    retval = blockAndCheckForOK(1.0);
	    if(retval == -1){
	    	System.out.println(cmd + " Produced an error");
	    } else if (retval == -2) {
	    	System.out.println(cmd + " timed out");
	    }
	    return retval;
    };

    //Persistent but "local" variables for getBytesPeriodic()
    private String getBytesWork = "";
	private int loopCount = 0;
    /**
     * Read bytes from the serial port in a non-blocking fashion
     * Will return the whole thing once the first "OK" or "ERR" is seen in the stream.
     * Returns null if no string read back yet.
     */
    public String getCmdResponseNonBlock() {
    	String retval =  null;
        if (visionPort != null){
            if (visionPort.getBytesReceived() > 0) {
            	String rxString = visionPort.readString();
                System.out.println("Waited: " + loopCount + " loops, Rcv'd: " + rxString);
                getBytesWork += rxString;
                if(getBytesWork.contains("OK") || getBytesWork.contains("ERR")){
                	retval = getBytesWork;
                	getBytesWork = "";
                	System.out.println(retval);
                }
                loopCount = 0;
            } else {
                ++loopCount;
            }
        }
		return retval;
    }
    
    /** 
     * Blocks thread execution till we get a response from the serial line
     * or timeout. 
     * Return values:
     *  0 = OK in response
     * -1 = ERR in response
     * -2 = No token found before timeout_s
     */
    public int blockAndCheckForOK(double timeout_s){
    	int retval = -2;
    	double startTime = Timer.getFPGATimestamp();
    	String testStr = "";
        if (visionPort != null){
            while(Timer.getFPGATimestamp() - startTime < timeout_s){
                if (visionPort.getBytesReceived() > 0) {
                	testStr += visionPort.readString();
                	if(testStr.contains("OK")){
                		retval = 0;
                		break;
                	}else if(testStr.contains("ERR")){
                		retval = -1;
                		break;
                	}

                } else {
                	sleep(10);
                }
            }
        }
        return retval;
    }
    
    
    // buffer to contain data from the port while we gather full packets 
    private String packetBuffer = "";
    /** 
     * Blocks thread execution till we get a valid packet from the serial line
     * or timeout. 
     * Return values:
     *  String = the packet 
     *  null = No full packet found before timeout_s
     */
    public String blockAndGetPacket(double timeout_s){
    	String retval = null;
    	double startTime = Timer.getFPGATimestamp();
        if (visionPort != null){
            while(Timer.getFPGATimestamp() - startTime < timeout_s){
                // Keep trying to get bytes from the serial port until the timeout expires.
                
                if (visionPort.getBytesReceived() > 0) {
                    // If there are any bytes available, read them in and 
                    //  append them to the buffer.
                	packetBuffer += visionPort.readString();
                    
                    // Attempt to detect if the buffer currently contains a complete packet
                	if(packetBuffer.contains(Constants.kPacketStart)){
				        if(packetBuffer.contains(Constants.kPacketEnd)){
                            // Buffer also contains at least one start & end character.
				        	// But we don't know if they're in the right order yet.
                            // Start by getting the most-recent packet end character's index
                            int endIdx = packetBuffer.lastIndexOf(Constants.kPacketEnd);
                            
                            // Look for the index of the start character for the packet
                            //  described by endIdx. Note this line of code assumes the 
                            //  start character for the packet must come _before_ the
                            //  end character.
                            int startIdx = packetBuffer.lastIndexOf(Constants.kPacketStart, endIdx);
                            
                            if(startIdx == -1){
                                // If there was no start character before the end character,
                                //  we can assume that we have something a bit wacky in our
                                //  buffer. For example: ",abc}garbage{1,2".
                                // Since we've started to receive a good packet, discard 
                                //  everything prior to the start character.
                                startIdx = packetBuffer.lastIndexOf(Constants.kPacketStart);
                                packetBuffer = packetBuffer.substring(startIdx);
                            } else {
                                // Buffer contains a full packet. Extract it and clean up buffer
                                retval = packetBuffer.substring(startIdx, endIdx+1);
                                packetBuffer = packetBuffer.substring(endIdx+1);
                                break;
                            } 
                        } else {
                          // In this case, we have a start character, but no end to the buffer yet. 
                          //  Do nothing, just wait for more characters to come in.
                        }
                    } else {
                        // Buffer contains no start characters. None of the current buffer contents can 
                        //  be meaningful. Discard the whole thing.
                        packetBuffer = "";
                    }
                } else {
                	sleep(10);
                }
            }
        }
        return retval;
    }
    
    /**
     * Private wrapper around the Thread.sleep method, to catch that interrupted error.
     * @param time_ms
     */
    private void sleep(int time_ms){
    	try {
			Thread.sleep(time_ms);
		} catch (InterruptedException e) {
			System.out.println("DO NOT WAKE THE SLEEPY BEAST");
			e.printStackTrace();
		}
    }
    
    /**
     * Mostly for debugging. Blocks execution while serOutEnable and just prints all serial 
     * characters to the console. It might print a different message too if nothing
     * comes in.
     */
    public void blockAndPrintAllSerial(){
        if (visionPort != null){
        	System.out.println("Starting Serial Output Dipslay");
            while(Constants.serOutEnable){
                if (visionPort.getBytesReceived() > 0) {
                	System.out.print(visionPort.readString());
            	} else {
            		//System.out.println("Nothing Rx'ed");
                	sleep(1000);
                }
            }
        }

    }
    
    /**
     * Parse individual numbers from a packet
     * @param pkt
     */
    public void parsePacket(String packet){
    	JSONParser parser = new JSONParser();
    	//System.out.println("Testing Parser: " + packet);
    	 try {

             Object obj = parser.parse(packet);

             JSONObject jsonObject = (JSONObject) obj;
             //System.out.println(jsonObject);
             Constants.trk = (double) jsonObject.get("trk");
			 System.out.println(Constants.trk);
			 
             Constants.xCntr = (double) jsonObject.get("tx");
             System.out.println(Constants.xCntr);
             
             Constants.yCntr = (double) jsonObject.get("ty");
             System.out.println(Constants.yCntr);
             
        
            

         } catch (Exception e){
        	 System.out.println("Parse Exception, probably need to check you JeVois output that it's matching what you are parsing");
        	 e.printStackTrace();
         }
    }
    
    /*
     * Main getters/setters
     */
    public double getxCntr() {
		return Constants.xCntr;
	}

	public double getyCntr() {
		return Constants.yCntr;
	}

	public double gettrk() {
		return Constants.trk;
	}

	public double getCamMode(){
		return Constants.camMode;
	}
	
	public boolean isVisionOnline() {
		return Constants.trackingOnline;
	}

	public static String executeCommand(String command) {

		StringBuffer output = new StringBuffer();

		Process p;
		try {
			p = Runtime.getRuntime().exec(command);
			p.waitFor();
			BufferedReader reader = 
                            new BufferedReader(new InputStreamReader(p.getInputStream()));

                        String line = "";			
			while ((line = reader.readLine())!= null) {
				output.append(line + "\n");
			}

		} catch (Exception e) {
			e.printStackTrace();
		}

		return output.toString();

	}
    
    /**
     * This thread runs a periodic task in the background to listen for vision camera packets.
     */
    Thread jevoisListenerThread = new Thread(new Runnable(){
    	public void run(){
    		while(true){
    			backgroundUpdate();		
    		}
    	}
    });
}