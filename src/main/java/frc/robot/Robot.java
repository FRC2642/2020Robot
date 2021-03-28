/* Official 2642 Prayer to the FIRST Robotics Gods (the ones that remain anyway):

  We pray to Dean and Don,
 May Woodie forever rest in peace,
 to help us succeed in our matches
 and for the spirit of FIRST to fill us
 with Gracious Professionalism(TM) and Coopertition(TM).

 We pray to the Robonauts, Cheesy Poofs, Highrollers,
 Simbotics, Beach Bots, and Robowranglers
 for the power of vision tracking,
 and the ability to comprehend their code.
 We can't read too good.

 We once again pray to the previously mentioned teams,
 for the strength of our robot as a whole.
 Please don't break.

public void homageToDepricatedSubsystems() {

 For ekatni (intake backwards)
 for its identity has been lost
 never to be seen again.
 I love you <3

 For Z-Target (our auto-aiming system)
 because somebody thought aimbot was better.
 despite obvious inferiority
 
 Into true egress
 for hanger prayed.
 Lost to new ages
 to dust it lay.

return "We love you <3";
}
 

 We pray to the control systems, and National Instruments,
 for if we do not we will surely perish
 as the RoboRio may not work.
 Please have mercy.

 We pray to the Robot Inspectors and the scale,
 as even though our robot is small,
 it will probably still be too heavy.
 By the weight sensor may we succeed
 so that we may compete another day.

 We pray to the FTA
 to ensure swift connections
 and accurate cameras for the drivers.
 They need them.

 And finally to the power of stupid ideas,
 because if you spout enough nonsense,
 a good idea is bound to appear eventually.
 We love you H.A.N.G.E.R. system.

 In the name of our founder,
 and in the honor of a deceased god,
 we now say
 Kamen, and Farewell...
*/

package frc.robot;

import static frc.robot.util.GeneralUtil.generateAuto;

import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.cscore.VideoSource;
import edu.wpi.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.vision.VisionPipeline;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.JevoisDriver;
import frc.robot.RobotContainer;
import frc.robot.commands.autoCommands.AutoCommandGroup;
import frc.robot.subsystems.SwerveDriveSubsystem;



/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public Command m_autonomousCommand;
  public RobotContainer robotContainer;

/*   public VideoSource camera;
  public VisionPipeline cameraPipeline;
  
  public VisionThread visionThread;
  
  public final Object visionLock = new Object();

   private Object particleReports; */
  
  NetworkTable table;

  // Jevois driver
  JevoisDriver jevoisCam;
 
  public PowerDistributionPanel pdp;
  
  public static Command autoCommand;
  SwerveDriveSubsystem drive = new SwerveDriveSubsystem();


  UsbCamera intakeCam;
  UsbCamera shooterCam;

  VideoSink camServer;
  
  @Override
  public void robotInit() {
    
    robotContainer = new RobotContainer();
    jevoisCam = new JevoisDriver();
    pdp = new PowerDistributionPanel();

    autoCommand = generateAuto();

    intakeCam = CameraServer.getInstance().startAutomaticCapture(0);
    shooterCam = CameraServer.getInstance().startAutomaticCapture(1);

    intakeCam.setFPS(10);
    shooterCam.setFPS(15);

    intakeCam.setResolution(320, 240);
    shooterCam.setResolution(320, 240);

    intakeCam.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    shooterCam.setConnectionStrategy(ConnectionStrategy.kAutoManage);

    camServer = CameraServer.getInstance().getServer();

    //table = NetworkTableInstance.getDefault().getTable("GRIP/mycontoursReport");

  }
 
 /*  public void setUpCamera() {
    camera = CameraServer.getInstance().startAutomaticCapture(1);
   // camera.setResolution(Constants.IMG_WIDTH, Constants.IMG_HEIGHT);
   // camera.setExposureManual(Constants.exposure);
    
    visionThread = new VisionThread(camera, cameraPipeline, pipeline -> {
        if (!pipeline.filterContoursOutput().isEmpty()) {
            Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
            synchronized (visionLock) {
                int centerX = r.x + (r.width / 2);
            }

    ParticleReport[] reports = new ParticleReport[contours.size()];

    for (int i = 0; i < reports.length; i++)
        {
        reports[i] = new ParticleReport();
        Rect r = Imgproc.boundingRect(contours.get(i));
        reports[i].area = r.area();
        reports[i].center = new Point(r.x + (r.width / 2),
                r.y + (r.height / 2));
        reports[i].boundingRect = r;
        }

    this.particleReports = reports();
        } 
    });    
  }  

  private Object reports() {
    return reports();
  }

  private void visionThread(VideoSource camera2, VisionPipeline visionPipeline2) {
  }
  */

  @Override
  public void robotPeriodic() {
    
    CommandScheduler.getInstance().run();


    

    //jevoisCam.printSystemOut();

    /**
     * place any SmartDashboard methods that should be running even when the robot is disabled here
     */

    SmartDashboard.putNumber("arm pot", RobotContainer.arm.getPot());

    //SmartDashboard.putNumber("mag vel", RobotContainer.magazine.getVelocity());
    
   /*  SmartDashboard.putNumber("fl", RobotContainer.drive.frontLeftModule.getModulePosition());
    SmartDashboard.putNumber("fr", RobotContainer.drive.frontRightModule.getModulePosition());
    SmartDashboard.putNumber("bl", RobotContainer.drive.backLeftModule.getModulePosition());
    SmartDashboard.putNumber("br", RobotContainer.drive.backRightModule.getModulePosition());  */

    //SmartDashboard.putBoolean("isShoot", RobotContainer.shooter.isAtTargetVelocity());
    //SmartDashboard.putBoolean("isArm", RobotContainer.arm.isArmAtGoal());
    SmartDashboard.putBoolean("isMagReady", RobotContainer.magazine.isMagReadyToShoot());
   
    SmartDashboard.putNumber("blangle", RobotContainer.drive.backLeftModule.getAbsoluteAngleEncoder());
    SmartDashboard.putNumber("brangle", RobotContainer.drive.backRightModule.getAbsoluteAngleEncoder());
    SmartDashboard.putNumber("flangle", RobotContainer.drive.frontLeftModule.getAbsoluteAngleEncoder());
    SmartDashboard.putNumber("frangle", RobotContainer.drive.frontRightModule.getAbsoluteAngleEncoder());

    SmartDashboard.putNumber("relblangle", RobotContainer.drive.backLeftModule.getRelativeAngleEncoder());
    SmartDashboard.putNumber("relbrangle", RobotContainer.drive.backRightModule.getRelativeAngleEncoder());
    SmartDashboard.putNumber("relflangle", RobotContainer.drive.frontLeftModule.getRelativeAngleEncoder());
    SmartDashboard.putNumber("relfrangle", RobotContainer.drive.frontRightModule.getRelativeAngleEncoder());

    /*SmartDashboard.putNumber("blvelocity", RobotContainer.drive.backLeftModule.getDriveVelocity());
    SmartDashboard.putNumber("brvelocity", RobotContainer.drive.backRightModule.getDriveVelocity());
    SmartDashboard.putNumber("flvelocity", RobotContainer.drive.frontLeftModule.getDriveVelocity());
    SmartDashboard.putNumber("frvelocity", RobotContainer.drive.frontRightModule.getDriveVelocity());*/

    SmartDashboard.putNumber("driveEncoder", RobotContainer.drive.getDrivePosition());


    //SmartDashboard.putString("targetColor", value)
  }

  @Override
  public void disabledInit() {
    drive.driveEncoder.setPosition(0.0);

  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    
    m_autonomousCommand = robotContainer.getAutonomousCommand();
    //m_autonomousCommand = new AutoCommandGroup();

    //m_autonomousCommand =  robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {



    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

  }

  @Override
  public void teleopPeriodic() {

    if(RobotContainer.driveController.getTriggerAxis(Hand.kLeft) > .5){
      camServer.setSource(intakeCam);
    } else {
      camServer.setSource(shooterCam);
    }


    /* double[] defaultValue = new double[0];

    double[] areas = table.getEntry("areas").getDoubleArray(defaultValue);

    System.out.print("areas: ");
    
    for(double area: areas){
      System.out.print(area + " ");
    }

    System.out.println(); */
    /**
     * DO NOT PLACE SMARTDASHBOARD DIAGNOSTICS HERE
     * Place any teleop-only SmartDashboard diagnostics in the appropriate subsystem's periodic() method
     */
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }
}
