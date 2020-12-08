package frc.robot.util;



	
	import edu.wpi.cscore.UsbCamera;
	import edu.wpi.cscore.VideoMode;
	import edu.wpi.first.cameraserver.CameraServer;
	import edu.wpi.first.wpilibj.DriverStation;
	import edu.wpi.first.wpilibj.SerialPort;
	import edu.wpi.first.wpilibj.Timer;
	
	
	public class JevoisDriver implements Runnable {
	
		private static final String TapePrefix = "D3";
	
		private UsbCamera _jevoisCam;
		private Thread _stream;
		private SerialPort _jevoisPort;
	
		private String[] parts;
		private boolean detectsTape;
				
		public String id; // chang 
		public double xpos;
		public double ypos;
		public double zpos;
		public double x;
		public double y;
		public double rot;
	

	
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
					_jevoisCam.setVideoMode(VideoMode.PixelFormat.kYUYV, 640, 480, 20);
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
				xpos = Double.parseDouble(tokens[1]);
				ypos = Double.parseDouble(tokens[2]);
				zpos = Double.parseDouble(tokens[3]);
				x = Double.parseDouble(tokens[4]);
				y = Double.parseDouble(tokens[5]);
				rot = Double.parseDouble(tokens[6]);

			} catch(NumberFormatException ex) {
			}
		}
	
		public void VisionTarget(String ida,
							int xposa,
							int yposa,
							double zposa,
							double xa,
							double ya,
							double rota) {
			id = ida;
			xpos = xposa;
			ypos = yposa;
			zpos = zposa;
			x = xa;
			y = ya;
			rot = rota;

		}
	
		public void printSystemOut() {
			System.out.println("Id = " + id);
			System.out.println("xpos = " + xpos);
			System.out.println("ypos = " + ypos);
			System.out.println("zpos = " + zpos);
			System.out.println("x = " + x);
			System.out.println("y = " + y);
			System.out.println("rot = " + rot);
			
			
		}

		public double getCenterOfVisionTarget(){
			return x;
		}

	}