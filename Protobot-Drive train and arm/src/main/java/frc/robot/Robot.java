// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);

DoubleSolenoid WristClose = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1,6);

DoubleSolenoid ArmExtend = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 7);

private Victor Leftmotor1 = new Victor(0);
private Victor Leftmotor2 = new Victor(1);
private Victor Rightmotor1 = new Victor(4);
private Victor Rightmotor2 = new Victor(5);

private Spark LeftArm = new Spark(7);
private Spark RightArm = new Spark(6);

private Spark Wrist = new Spark(8);

double BalanceToggle;


Encoder LeftEncoder = new Encoder(0, 1);
Encoder RightEncoder = new Encoder(4, 5);

AHRS ahrs = new AHRS();

//Controller scheme

private Joystick joy1 = new Joystick(0);
private Joystick joy2 = new Joystick(1);
private Joystick joy3 = new Joystick(2);

private double autoStartTime;

//camera initialization
UsbCamera Camera1;
UsbCamera Camera2;

  @Override
  public void robotInit() {

// Creates UsbCamera and MjpegServer [1] and connects them
Camera1 = CameraServer.startAutomaticCapture(0);
Camera2 = CameraServer.startAutomaticCapture(1);



pcmCompressor.enableDigital();

WristClose.set(Value.kReverse);

ArmExtend.set(Value.kForward);

RightEncoder.reset();
LeftEncoder.reset();



    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();


//LeftEncoder.setDistancePerPulse(1./240.);
//RightEncoder.setDistancePerPulse(1./240.);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */


  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

var LeftFeet = (LeftEncoder.getDistance() * .0783);
var RightFeet = (RightEncoder.getDistance()* .0783);


SmartDashboard.putNumber("Left Encoder Value", LeftFeet);
SmartDashboard.putNumber("Right Encoder Value", RightFeet);


AHRS.BoardYawAxis yaw_axis = ahrs.getBoardYawAxis();
SmartDashboard.putString(   "YawAxisDirection",     yaw_axis.up ? "Up" : "Down" );
SmartDashboard.putNumber(   "YawAxis",              yaw_axis.board_axis.getValue() );


SmartDashboard.putNumber(   "Tilt",          ahrs.getYaw());


SmartDashboard.putNumber(   "IMU_Accel_X",          ahrs.getWorldLinearAccelX());
          SmartDashboard.putNumber(   "IMU_Accel_Y",          ahrs.getWorldLinearAccelY());
          SmartDashboard.putBoolean(  "IMU_IsMoving",         ahrs.isMoving());
          SmartDashboard.putBoolean(  "IMU_IsRotating",       ahrs.isRotating());
        
SmartDashboard.putNumber("Not balance = 1", BalanceToggle);

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
  autoStartTime = Timer.getFPGATimestamp();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    RightEncoder.reset();
    LeftEncoder.reset();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }


  /** This function is called periodically during autonomous. 
   * @param RightFeet */
  @Override
  public void autonomousPeriodic() {
 
    double currTime = Timer.getFPGATimestamp();
    double timeElapsed = currTime - autoStartTime; 
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    BalanceToggle = 1;

    RightEncoder.reset();
    LeftEncoder.reset();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

//Joystick variables

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {



//Joystick variables
double Speed = -joy1.getRawAxis(1) * 1;
double Turn = -joy1.getRawAxis(2) * 0.6;
double Armspeed = joy2.getRawAxis(1);
double Wristspeed = joy3.getRawAxis(1);
//Turn control math
double Left = Speed - Turn;
double Right =  Speed + Turn;

//deadband
if (Math.abs(Speed) < 0.03){
  Speed = 0;
}

if (Math.abs(Turn) < 0.05){
  Turn = 0;
}

//Arm input
LeftArm.set(-Armspeed);
RightArm.set(Armspeed);
Wrist.set(-Wristspeed);

//Motor input
Leftmotor1.set(Left);
Leftmotor2.set(Left);
Rightmotor1.set(-Right);
Rightmotor2.set(-Right);


//button input for wrist
if (joy3.getRawButtonPressed(1)){
WristClose.toggle();

}

if (joy2.getRawButtonPressed(1)){
ArmExtend.toggle();

}


//Balance train toggles
if (joy1.getRawButtonPressed(3)) { //Not balanced
  BalanceToggle = 1;
  }
  else { 
  if (joy1.getRawButtonPressed(4)) { // Balancing
    BalanceToggle = -1;
    }
  }

if (BalanceToggle < 0) {


}

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
