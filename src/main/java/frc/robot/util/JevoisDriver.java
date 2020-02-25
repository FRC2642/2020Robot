package frc.robot.util;



	
	import edu.wpi.cscore.UsbCamera;
	import edu.wpi.cscore.VideoMode;
	import edu.wpi.first.cameraserver.CameraServer;
	import edu.wpi.first.wpilibj.DriverStation;
	import edu.wpi.first.wpilibj.SerialPort;
	import edu.wpi.first.wpilibj.Timer;
	
	
	public class JevoisDriver implements Runnable {
	
		private static final String TapePrefix = "TAPE";
	
		private UsbCamera _jevoisCam;
		private Thread _stream;
		private SerialPort _jevoisPort;
	
		private String[] parts;
		private boolean detectsTape;
				
		public String id;
		public int w;
		public int h;
		public double hlist;
		public double rvecs;
		public int tvecs;
	

	
		public void JevoisTracker(SerialPort.Port port, int baud) {
			_stream = new Thread(this);
			try {
				_jevoisPort = new SerialPort(baud, port);
				_stream.start();
			} catch (Exception e) {
				DriverStation.reportError("Jevois Cam Serial port missing!", e.getStackTrace());
			}
		}
	
		public void startCameraStream() {
			if (_stream.isAlive()) {
				try {
					_jevoisCam = CameraServer.getInstance().startAutomaticCapture();
					_jevoisCam.setVideoMode(VideoMode.PixelFormat.kYUYV, 640, 480, 30);
				} catch (Exception e) {
					DriverStation.reportError("Jevois Cam failed to connect!", e.getStackTrace());
				}
			}
		}
	
		public boolean detectsTape() {
			return detectsTape;
		}
	
		@Override
		public void run() {
			while (_stream.isAlive()) {
				Timer.delay(0.01);
				try {
					if (_jevoisPort.getBytesReceived() > 0) {
						String read = _jevoisPort.readString();
						if (read.startsWith(TapePrefix)) {
							detectsTape = true;
							parts = dataParse(read);
						} else {
							detectsTape = false;
						}
					}
				} catch (Exception e) {
					DriverStation.reportError("Jevois Cam Serial failed to connect!", e.getStackTrace());
					detectsTape = false;
				}
			}
		}

		private String[] dataParse(String input) {
			input = input.substring(TapePrefix.length() + 1); // strip prefix from front (including single whitespace
			input = input.stripLeading(); // strip whitespace at front
			return input.split("\\s*,\\s*");
		}

		public String[] getParts() {
			return parts;
		}

		//self.sendAllSerial(w, h, hlist, rvecs, tvecs)

		public void VisionTarget(String str) {
			String[] tokens = str.split(",");
			id = tokens[0];
			try {
				w = Integer.parseInt(tokens[1]);
				h = Integer.parseInt(tokens[2]);
				hlist = Double.parseDouble(tokens[3]);
				rvecs = Double.parseDouble(tokens[4]);
				tvecs = Integer.parseInt(tokens[5]);

			} catch(NumberFormatException ex) {
			}
		}
	
		public void VisionTarget(String ida,
							int wa,
							int ha,
							double hlista,
							double rvecsa,
							int tvecsa) {
			id = ida;
			w = wa;
			h = ha;
			hlist = hlista;
			rvecs = rvecsa;
			tvecs = tvecsa;

		}
	
		public void printSystemOut() {
			System.out.println("Id = " + id);
			System.out.println("w = " + w);
			System.out.println("h = " + h);
			System.out.println("hlist = " + hlist);
			System.out.println("rvecs = " + rvecs);
			System.out.println("tvecs = " + tvecs);

			
		}

		public double getDistFromTarget(){
			return hlist;
		}
	}