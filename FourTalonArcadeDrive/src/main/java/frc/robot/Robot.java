/**
 * Phoenix Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and 
 * Phoenix Software API Libraries ONLY when in use with CTR Electronics hardware products
 * as well as the FRC roboRIO when in use in FRC Competition.
 * 
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * CROSS THE ROAD ELECTRONICS BE LIABLE FOR ANY INCIDENTAL, SPECIAL, 
 * INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE
 */

/**
 * Description:
 * The SixTalonArcadeDrive example demonstrates the ability to create WPI Talons/Victors
 * to be used with WPI's drivetrain classes. WPI Talons/Victors contain all the functionality
 * of normally created Talons/Victors (Phoenix) with the remaining SpeedController functions
 * to be supported by WPI's classes. 
 * 
 * The example uses two master motor controllers passed into WPI's DifferentialDrive Class 
 * to control the remaining 4 Talons (Follower Mode) to provide a simple Tank Arcade Drive 
 * configuration.
 */

package frc.robot;

//imports, duh
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.cscore.AxisCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;

public class Robot extends TimedRobot {
	

//_______________Declarations_______________

	//talon declaration
	WPI_VictorSPX frontLeftMotor = new WPI_VictorSPX(2);
	WPI_VictorSPX frontRightMotor = new WPI_VictorSPX(6);
	WPI_VictorSPX leftSlave1 = new WPI_VictorSPX(1);
	WPI_VictorSPX rightSlave1 = new WPI_VictorSPX(5);
	WPI_VictorSPX elevator = new WPI_VictorSPX(4);
	WPI_VictorSPX swiffer = new WPI_VictorSPX(8);
	WPI_VictorSPX swifferupdown = new WPI_VictorSPX(3);
	WPI_VictorSPX swifferupdownSlave = new WPI_VictorSPX(7);
	WPI_VictorSPX ballthingy = new WPI_VictorSPX(9);
	Spark hatchCollector = new Spark(0);

	//pneumatic delarations
	DoubleSolenoid solenoidFront = new DoubleSolenoid(10, 4, 5);
	DoubleSolenoid solenoidBack = new DoubleSolenoid(10, 6, 7);
	Compressor compressor;

    // Construct drivetrain by providing master motor controllers
	DifferentialDrive drive = new DifferentialDrive(frontLeftMotor, frontRightMotor);

	//Joystick declaration
	Joystick joyE = new Joystick(0);
	Joystick joyL = new Joystick(1);
	Joystick joyR = new Joystick(2);

	//Joystick button declarations
	boolean joyLTrigger;
	boolean joyRTrigger;
	boolean joyETRigger;
	boolean joyESwifferIn;
	boolean joyESwifferOut;
	boolean joyEFrontpneu;
	boolean joyEBackpneu;
	boolean joyEallpneu;
	boolean joyRspikeup;
	boolean joyRspikedown;
	boolean joyRballshoot;

	//debugging variables
	int spikedebug;
	int ballshootdebug;
	int robotSlowDriveDebug;

	//Variable declarations regarding the joysticks
	double leftVal;
	double rightVal;
	double otherVal;
	int joyEPOV;
	int joyRPOV;
	double elevatorVal = 0;
	double swifferVal = 0;
	double ballshoot = 0;

	//toggle variables
	boolean frontpneuToggle;
	boolean backpneuToggle;

	//limit switch
	DigitalInput limitSwitch;
	boolean limitVal;

	//camera declarations
	private String[] hosts = {"10.13.17.11", "10.13.17.12"};
    private AxisCamera cams = CameraServer.getInstance().addAxisCamera("cams", hosts);

// This function is called once at the beginning during operator control
	public void robotInit() {
		// Factory Default all hardware to prevent unexpected behaviour (elevator commented out for testing)
		frontLeftMotor.setNeutralMode(NeutralMode.Brake);
		frontRightMotor.setNeutralMode(NeutralMode.Brake);
		leftSlave1.setNeutralMode(NeutralMode.Brake);
		rightSlave1.setNeutralMode(NeutralMode.Brake);
		swiffer.configFactoryDefault();
		swifferupdown.configFactoryDefault();

		//limit switch
		limitSwitch = new DigitalInput(1);

		//toggle the functions below to make sure that the wheels are turning the correct way
		frontLeftMotor.setInverted(false); // <<<<<< Adjust this until robot drives forward when stick is forward
		frontRightMotor.setInverted(false); // <<<<<< Adjust this until robot drives forward when stick is forward
		leftSlave1.setInverted(InvertType.FollowMaster);
		rightSlave1.setInverted(InvertType.FollowMaster);

		/* diff drive assumes (by default) that 
			right side must be negative to move forward.
			Change to 'false' so positive/green-LEDs moves robot forward  */
		drive.setRightSideInverted(false); // do not change this
	}

	// This function is called periodically during operator control
	public void robotPeriodic() {

		//get joystick values and buttons and such
		leftVal = -1.0 * joyL.getY();
		rightVal = 1.0 * joyR.getY();
		otherVal = joyE.getY();

		//is the switch pushed in or nah?
		limitVal = limitSwitch.get();

		//Declare and obtain button inputs
		joyLTrigger = joyL.getRawButton(1);
		joyRTrigger = joyR.getRawButton(1);
		joyETRigger = joyE.getRawButton(1);
		joyESwifferIn = joyE.getRawButton(2);
		joyESwifferOut = joyE.getRawButton(1);
		joyEFrontpneu = joyE.getRawButtonPressed(5);
		joyEBackpneu = joyE.getRawButtonPressed(3);
		joyEallpneu = joyE.getRawButtonPressed(6);
		joyRspikeup = joyR.getRawButton(5);
		joyRspikedown = joyR.getRawButton(3);
		joyRballshoot = joyR.getRawButton(2);
		joyRPOV = joyR.getPOV();
		joyEPOV = joyE.getPOV();

		//motor+pnuematic control
		//swiffer up/down
		if (joyETRigger == false) {
			swifferupdown.set(otherVal/2);
			swifferupdownSlave.set(otherVal/-2);
			//this just sets the swiffer values for it going up and down to the value of the extra joystick
		}
		else {
			swifferupdown.set(0);
			swifferupdownSlave.set(0);
		}

		//Driving
		// Deadband - within 5% joystick, make it zero
		if (Math.abs(leftVal) < 0.05) {
			leftVal = 0;
		}
		if (Math.abs(rightVal) < 0.05) {
			rightVal = 0;
		}

		//slow the robot whilst driving
		if(joyLTrigger && joyRTrigger) {
			leftVal = leftVal/2;
			rightVal = rightVal/2;
			robotSlowDriveDebug = 1;
		}
		else {
			robotSlowDriveDebug = 0;
		}

		//shoot the ball
		if(joyRballshoot) {
			ballshoot = -1;
			ballshootdebug = 1;
		}
		else if(joyRPOV == 180 && limitVal==false) {
			ballshoot = 0.25;
			ballshootdebug = -1;
		}
		else {
			ballshoot = 0;
			ballshootdebug = 0;
		}
		ballthingy.set(ballshoot);

		//testing limitswitch
		System.out.println("limitVal " + limitVal);


		//System.out.println("leftVal = " + leftVal + " rightval = " + rightVal);

		//drive the diggity dang robit
		//drive.tankDrive(leftVal, rightVal);
		frontLeftMotor.set(leftVal);
		leftSlave1.set(leftVal);
		frontRightMotor.set(rightVal);
		rightSlave1.set(rightVal);

		//elevator up/down control (reformatted for the sake of testing, just about all of the code for the elevator is here)
		//should it be necessary for debugging, just have otherVal output
		if (joyETRigger == true) {
			elevator.set(otherVal);
		}
		else {
			elevator.set(0);
		}

		//spike hatch control
		if (joyEPOV == 0) {
			hatchCollector.set(-0.5);
			spikedebug = 1;
		}
		else if (joyEPOV == 180) {
			hatchCollector.set(0.5);
			spikedebug = -1;
		}
		else {
			hatchCollector.set(0);
			spikedebug = 0;
		}
		//System.out.println("spikeHatchCollector = " + spiketest);

		//swiffer in/out control w/ limit switch
		//once again, if I need it for debugging, just use swifferVal
		if(joyRPOV == 0) {
			swifferVal = 0.5;
		}
		else if(joyRPOV == 180) {
			swifferVal = -0.5;
		}
		else {
			swifferVal = 0;
		}
		swiffer.set(swifferVal);

		//everything pneumatic
		//button based pneumatic control
		//uncomment the printIn's for debugging purposes
		 if(joyEFrontpneu) {
			 frontpneuToggle = !frontpneuToggle;
		 }
		 if(joyEBackpneu) {
			 backpneuToggle = !backpneuToggle;
		}

		 if(joyEallpneu) {
		 	backpneuToggle = !backpneuToggle;
			 frontpneuToggle = !frontpneuToggle;
		 }

		//uncomment the print ins if debugging is necessary
		 if(frontpneuToggle) {
			solenoidFront.set(DoubleSolenoid.Value.kForward);
			//System.out.println("front solenoids going out");
		 }
		 else {
			solenoidFront.set(DoubleSolenoid.Value.kReverse);
			//System.out.println("front solenoids going back in");
		 }
		 if(backpneuToggle) {
			solenoidBack.set(DoubleSolenoid.Value.kForward);
			//System.out.println("back solenoids going out");
		 }
		 else {
			solenoidBack.set(DoubleSolenoid.Value.kReverse);
			//System.out.println("back solenoids going back in");
		 }	
		

		//print the values for different variables for bugtesting
		//System.out.println("JoyL:" + leftVal + "  joyR:" + rightVal + " joy3: " + otherVal + "elevatorVal: " + elevatorVal + "swifferVal: " + swifferVal);
		
	}
}
