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
	Relay spikeHatchCollector;

	//pneumatic delarations
	DoubleSolenoid solenoidFront = new DoubleSolenoid(10, 4, 5);
	DoubleSolenoid solenoidBack = new DoubleSolenoid(10, 6, 7);

    // Construct drivetrain by providing master motor controllers
	DifferentialDrive drive = new DifferentialDrive(frontLeftMotor, frontRightMotor);

	//Joystick declaration
	Joystick joyE = new Joystick(0);
	Joystick joyL = new Joystick(1);
	Joystick joyR = new Joystick(2);

	//Joystick button declarations
	boolean joyLTrigger;
	boolean joyESwifferIn;
	boolean joyESwifferOut;
	boolean joyEFrontpneu;
	boolean joyEBackpneu;
	boolean joyEallpneu;
	boolean joyRspikeup;
	boolean joyRspikedown;
	boolean joyLballshoot;

	Compressor compressor;
	

	//Joystick button toggles
	boolean speedToggle;
	boolean frontpneuToggle;
	boolean backpneuToggle;

	//Variable declarations regarding the joysticks
	double leftVal;
	double rightVal;
	double otherVal;
	int joyEPOV;
	double elevatorVal = 0;
	double swifferVal = 0;
	double ballshoot = 0;

	//limit switch
	DigitalInput limitSwitch = new DigitalInput(1);
	boolean limitVal;

	//camera declarations
	private String[] hosts = {"10.13.17.11", "10.13.17.12"};
    private AxisCamera cams = CameraServer.getInstance().addAxisCamera("cams", hosts);


	
// This function is called once at the beginning during operator control
	public void robotInit() {
		// Factory Default all hardware to prevent unexpected behaviour (elevator commented out for testing)
		frontLeftMotor.configFactoryDefault();
		frontRightMotor.configFactoryDefault();
		leftSlave1.configFactoryDefault();
		rightSlave1.configFactoryDefault();
		//elevator.configFactoryDefault();
		swiffer.configFactoryDefault();
		swifferupdown.configFactoryDefault();

		//set the slaves to follow the mains
		leftSlave1.follow(frontLeftMotor);
		rightSlave1.follow(frontRightMotor);
		//swifferupdownSlave.follow(swifferupdown);


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
		joyLTrigger = joyL.getTriggerPressed();
		joyESwifferIn = joyE.getRawButton(2);
		joyESwifferOut = joyE.getRawButton(1);
		joyEFrontpneu = joyE.getRawButtonPressed(5);
		joyEBackpneu = joyE.getRawButtonPressed(3);
		joyEallpneu = joyE.getRawButtonPressed(6);
		joyRspikeup = joyR.getRawButton(5);
		joyRspikedown = joyR.getRawButton(3);
		joyLballshoot = joyL.getRawButton(6);

		//_____Motor and pneumatic control below_______
		//any simple .set code (elevator commented out for testing)
		//elevator.set(elevatorVal);
		swiffer.set(swifferVal);
		swifferupdown.set(otherVal*-0.5);
		swifferupdownSlave.set(otherVal/2);
		ballthingy.set(ballshoot);

		//Driving
		// Deadband - within 5% joystick, make it zero
		if (Math.abs(leftVal) < 0.05) {
			leftVal = 0;
		}
		if (Math.abs(rightVal) < 0.05) {
			rightVal = 0;
		}

		//slow the robot whilst driving
        if(joyLTrigger) {
			speedToggle = !speedToggle;
		}
		if(speedToggle) {
			leftVal = leftVal/2;
			rightVal = rightVal/2;
		}

		//shoot the ball
		if(joyLballshoot) {
			ballshoot = 0.5;
		}
		else {
			ballshoot = 0;
		}

		//System.out.println("leftVal = " + leftVal + " rightval = " + rightVal);

		//drive the diggity dang robit
		drive.tankDrive(leftVal, rightVal);	

		//elevator up/down control (reformatted for the sake of testing, just about all of the code for the elevator is here)
		joyEPOV = joyE.getPOV();
		if (joyEPOV == 0) {
			elevatorVal = 0.75;
		}
		else if (joyEPOV == 180) {
			elevatorVal = -0.75;
		}
		else {
			elevatorVal = 0;
		}

		System.out.println("elevatorVal = " + elevatorVal);
		elevator.set(elevatorVal);

		//swiffer in/out control w/ limit switch
		if(joyESwifferIn/* && limitVal==false*/) {
			swifferVal = 0.25;
		}
		else if(joyESwifferOut) {
			swifferVal = -0.01;
		}
		else {
			swifferVal = 0;
		}

		//everything pneumatic
		//button based pneumatic control

		
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

		 if(frontpneuToggle) {
			 solenoidFront.set(DoubleSolenoid.Value.kForward);
		 }
		 else {
			 solenoidFront.set(DoubleSolenoid.Value.kReverse);
		 }
		 if(backpneuToggle) {
			 solenoidBack.set(DoubleSolenoid.Value.kForward);
		 }
		 else {
			 solenoidBack.set(DoubleSolenoid.Value.kReverse);
		 }	
		
		
		//spike controller for hatches
		// if(joyRspikeup) {
		// 	spikeHatchCollector.set(Relay.Value.kForward);
		// }
		// else if(joyRspikedown) {
		// 	spikeHatchCollector.set(Relay.Value.kReverse);
		// }
		// else {
		// 	spikeHatchCollector.set(Relay.Value.kOff);
		// }
		

		//print the values for different variables for bugtesting
		//System.out.println("JoyL:" + leftVal + "  joyR:" + rightVal + " joy3: " + otherVal + "elevatorVal: " + elevatorVal + "swifferVal: " + swifferVal);
		
	}
}
