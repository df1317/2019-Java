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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.cscore.AxisCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.CameraServer;
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
	/*
	ID 0 = power distribution panel
	ID 1 = left 1
	ID 2 = left 2
	ID 3 = right 1
	ID 4 = right 2
	ID 5 = swiffer actuactor
	ID 6 = swiffer 1
	ID 7 = swiffer 2
	ID 8 = elevator
	ID 9 = ball shooter
	ID 10 = pcm (pneu control moduel)
	spike does hatch collector
	*/


//_______________Declarations_______________

	//talon declaration
	WPI_VictorSPX frontLeftMotor = new WPI_VictorSPX(1);
	WPI_VictorSPX frontRightMotor = new WPI_VictorSPX(3);
	WPI_VictorSPX leftSlave1 = new WPI_VictorSPX(2);
	WPI_VictorSPX rightSlave1 = new WPI_VictorSPX(4);
	WPI_VictorSPX elevator = new WPI_VictorSPX(8);
	WPI_VictorSPX swiffer = new WPI_VictorSPX(5);
	WPI_VictorSPX swifferupdown = new WPI_VictorSPX(6);
	WPI_VictorSPX swifferupdownSlave = new WPI_VictorSPX(7);
	WPI_VictorSPX ballShooter = new WPI_VictorSPX(9);
	Relay spikeHatchCollector;

	//pneumatic delarations
	DoubleSolenoid solenoidFront = new DoubleSolenoid(4, 5);
	DoubleSolenoid solenoidBack = new DoubleSolenoid(6, 7);

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
	boolean joyRspikeF;
	boolean joyRspikeB;
	int joyEPOV = joyE.getPOV();

	//Joystick button toggles
	boolean speedToggle;
	boolean frontpneuToggle;
	boolean backpneuToggle;

	//Variable declarations regarding the joysticks
	double leftVal = 1.0 * joyL.getY();	// Sign this so forward is positive
	double rightVal = -1.0 * joyR.getY(); // Sign this so right is positive
	double otherVal = joyE.getY();
	double elevatorVal = 0;
	double swifferVal = 0;

	//limit switch
	DigitalInput limitSwitch = new DigitalInput(1);
	boolean limitVal;

	//camera declarations (commented out for now)
	//private AxisCamera cam1 = CameraServer.getInstance().addAxisCamera("10.13.17.11");
	//private AxisCamera cam2 = CameraServer.getInstance().addAxisCamera("10.13.17.12");
	private String[] hosts = {"10.13.17.11", "10.13.17.12"};
    private AxisCamera cams = CameraServer.getInstance().addAxisCamera("cams", hosts);


	
	// This function is called once at the beginning during operator control
	public void teleopInit() {

		// Factory Default all hardware to prevent unexpected behaviour
		frontLeftMotor.configFactoryDefault();
		frontRightMotor.configFactoryDefault();
		leftSlave1.configFactoryDefault();
		rightSlave1.configFactoryDefault();
		elevator.configFactoryDefault();
		swiffer.configFactoryDefault();
		swifferupdown.configFactoryDefault();

		//set the slave talons to follow the main talons
		leftSlave1.follow(frontLeftMotor);
		rightSlave1.follow(frontRightMotor);
		swifferupdownSlave.follow(swifferupdown);

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
	public void teleopPeriodic() {

		//Put info on the smartdashboard
		/*
		SmartDashboard.putNumber("Left Joystick Y value", joyL.getY());
		SmartDashboard.putNumber("Right Joystick Y value", joyR.getY());
		SmartDashboard.putNumber("Extra Joystick Y value", joyE.getY());
		SmartDashboard.putBoolean("Front Pnuematics", joyEFrontpneu == true);
		SmartDashboard.putBoolean("Back Pnuematics", joyEBackpneu == true);
		*/

		//is the switch pushed in or nah?
		limitVal = limitSwitch.get();

		//Declare and obtain button inputs
		joyLTrigger = joyL.getTriggerPressed();
		joyESwifferIn = joyE.getRawButton(2);
		joyESwifferOut = joyE.getRawButton(1);
		joyEFrontpneu = joyE.getRawButtonPressed(5);
		joyEBackpneu = joyE.getRawButtonPressed(3);
		joyRspikeF = joyR.getRawButton(6);
		joyRspikeB = joyR.getRawButton(4);

		//_____Motor and pneumatic control below_______
		//any simple .set code
		elevator.set(elevatorVal);
		swiffer.set(swifferVal);
		swifferupdown.set(otherVal);

		//Driving
		// Deadband - within 10% joystick, make it zero


		if (Math.abs(leftVal) < 0.10) {
			leftVal = 0;
		}
		if (Math.abs(rightVal) < 0.10) {
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

		//drive the diggity dang robit
		drive.tankDrive(leftVal, rightVal);	

		//elevator/swiffer control
		//elevator up/down control
		if (joyEPOV == 0) {
			elevatorVal = 0.5;
		}
		if (joyEPOV == 4) {
			elevatorVal = -0.5;
		}

		//swiffer in/out control w/ limit switch
		if(joyESwifferIn==true && limitVal==false) {
			swifferVal = 0.5;
		}
		if(joyESwifferOut) {
			swifferVal = -0.5;
		}

		//everything pneumatic
		//button based pneumatic control
		if(joyEFrontpneu) {
			frontpneuToggle = !frontpneuToggle;
		}
		if(joyEBackpneu) {
			backpneuToggle = !backpneuToggle;
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
		if(joyRspikeF) {
			spikeHatchCollector.set(Relay.Value.kForward);
		}
		if(joyRspikeB) {
			spikeHatchCollector.set(Relay.Value.kReverse);
		}
		
		

		//print the values for different variables for bugtesting
		System.out.println("JoyL:" + leftVal + "  joyR:" + rightVal + " joy3: " + otherVal + "elevatorVal: " + elevatorVal + "swifferVal: " + swifferVal);
		
	}
}
