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

import edu.wpi.cscore.VideoSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.JevoisDriver;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot<MyFindTapePipeline> extends TimedRobot {

  public Command m_autonomousCommand;
  public RobotContainer robotContainer;
  public PowerDistributionPanel pdp;
    
  public VideoSource usbCamera;
  public VideoSource _jevoisMjpegServer;
  
  //Jevois driver
  public static JevoisDriver jevoisCam;

  public static Command autoCommand;

  @Override
  public void robotInit() {

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
    //takes a picture with the camera
    //sets resolution of camera
    jevoisCam = new JevoisDriver();
    pdp = new PowerDistributionPanel();

    autoCommand = generateAuto();

    CameraServer.getInstance().startAutomaticCapture();
    CameraServer.getInstance().startAutomaticCapture(1);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    
    CommandScheduler.getInstance().run();

    //jevoisCam.printSystemOut();

    /**
     * place any SmartDashboard methods that should be running even when the robot is disabled here
     */

    SmartDashboard.putNumber("shooter vel", RobotContainer.shooter.getAverageVelocity());
    SmartDashboard.putNumber("arm pot", RobotContainer.arm.getPot());
    SmartDashboard.putNumber("mag vel", RobotContainer.magazine.getVelocity());

    
    /* SmartDashboard.putNumber("fl", RobotContainer.drive.frontLeftModule.getModulePosition());
    SmartDashboard.putNumber("fr", RobotContainer.drive.frontRightModule.getModulePosition());
    SmartDashboard.putNumber("bl", RobotContainer.drive.backLeftModule.getModulePosition());
    SmartDashboard.putNumber("br", RobotContainer.drive.backRightModule.getModulePosition()); */

    //SmartDashboard.putString("targetColor", value)
  }

  /**
   * This function is called once each time the robot enters Disabled mode.s
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    
    m_autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    } 
    
  }
  
  /**
   * This function is called periodically during autonomous.
   * 
   */

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {
    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    /**
     * DO NOT PLACE SMARTDASHBOARD DIAGNOSTICS HERE
     * Place any teleop-only SmartDashboard diagnostics in the appropriate subsystem's periodic() method
     */
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
