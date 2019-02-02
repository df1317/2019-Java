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
 *
 * Controls:
 * Left Joystick Y-Axis: Drive robot in forward and reverse direction
 * Right Joystick X-Axis: Turn robot in right and left direction
 */
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.*;

public class Robot extends TimedRobot {

	//talon declaration
	WPI_TalonSRX frontLeftMotor = new WPI_TalonSRX(1);
	WPI_TalonSRX frontRightMotor = new WPI_TalonSRX(3);
	WPI_TalonSRX leftSlave1 = new WPI_TalonSRX(2);
	WPI_TalonSRX rightSlave1 = new WPI_TalonSRX(4);
	WPI_TalonSRX elevator = new WPI_TalonSRX(5);
	WPI_TalonSRX swiffer = new WPI_TalonSRX(6);
	WPI_TalonSRX swifferupdown = new WPI_TalonSRX(7);
	WPI_TalonSRX swifferupdownSlave = new WPI_TalonSRX(8);
	//when switching these over to victors, just remember that it's WPI_VictorSPX

	//pneumatic delarations
	DoubleSolenoid solenoidFront1 = new DoubleSolenoid(1, 2);
	DoubleSolenoid solenoidFront2 = new DoubleSolenoid(3, 4);
	DoubleSolenoid solenoidBack1 = new DoubleSolenoid(5, 6);
	DoubleSolenoid solenoidBack2 = new DoubleSolenoid(7, 8);

    /* Construct drivetrain by providing master motor controllers */
	DifferentialDrive drive = new DifferentialDrive(frontLeftMotor, frontRightMotor);

	//Joystick declaration
	Joystick joy1 = new Joystick(1);
	Joystick joy2 = new Joystick(2);
	Joystick joy3 = new Joystick(3);

	//Joystick button declarations
	boolean joy2Trigger;
	boolean joy3SwifferIn;
	boolean joy3SwifferOut;
	int joy3POV = joy3.getPOV();
	int joy1POV = joy1.getPOV();
	

	// This function is called once at the beginning during operator control
	public void teleopInit() {

		// Factory Default all hardware to prevent unexpected behaviour
		frontLeftMotor.configFactoryDefault();
		frontRightMotor.configFactoryDefault();
		leftSlave1.configFactoryDefault();
		rightSlave1.configFactoryDefault();

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

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
        //Aquisition of Joystick values
		double leftVal = 1.0 * joy2.getY();	// Sign this so forward is posiPtive
		double rightVal = -1.0 * joy1.getY();       // Sign this so right is positive
		double otherVal = joy3.getY();
		double elevatorVal = 0;
		double swifferVal = 0;

		//obtain button inputs
		joy2Trigger = joy2.getRawButton(1);
		joy3SwifferIn = joy3.getRawButton(2);
		joy3SwifferOut = joy3.getRawButton(1);


		//additional motor/pneumatic functions
		elevator.set(elevatorVal);
		swiffer.set(swifferVal);
		swifferupdown.set(otherVal);

        // Deadband - within 10% joystick, make it zero
		if (Math.abs(leftVal) < 0.10) {
			leftVal = 0;
		}
		if (Math.abs(rightVal) < 0.10) {
			rightVal = 0;
		}
		//slow the robot whilst driving
        if(joy2Trigger) {
			leftVal = leftVal/2;
			rightVal = rightVal/2;
		}
		//elevator up/down control
		if (joy3POV == 0) {
			elevatorVal = 0.5;
		}
		if (joy3POV == 4) {
			elevatorVal = -0.5;
		}
		//swiffer in/out control
		if(joy3SwifferIn) {
			swifferVal = 0.5;
		}
		if(joy3SwifferOut) {
			swifferVal = -0.5;
		}
		

		//print the values for different variables
		System.out.println("JoyL:" + leftVal + "  joyR:" + rightVal + " joy3: " + otherVal + "elevatorVal: " + elevatorVal + "swifferVal: " + swifferVal);
		
        
		//drive the diggity dang robit
		drive.tankDrive(leftVal, rightVal);
	}
}
