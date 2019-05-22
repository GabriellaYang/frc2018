package org.usfirst.frc.team6662.robot.subsystems;
/* This code is from the 2017-2018 First robotics year. The drivetrain code was a file that I primarly worked on and includes
soley imports from the First Robotics library. This is a earliar version of the code that does not include imports
from the CTRX Pheonix library of which we later used their sensor classes. Later versions include code designed for a CAN Bus that
included all sensors incuding the encoders and gryoscope but this version uses only PWM motor controllers. 
 The code below uses primary simple if and else statements and draws from many superclasses and methods defined by the first library. I included it 
as my academic example as it was a file I worked heavily on and also demonstrates most of the code concepts that I've dealt with.
While I do not often work directly with the stream in most of my robotics projects, it is often that I must work with theoretical
values and concepts. In my teams first year of robotics most of us were novices to coding and so understanding basic principles of coding
such as encapsulation and the hierarchy became especially important ind developing our programs. Having to understand often contextless values
from the First java docs and other resources while also learning how to utlize those values in our code in a way that worked with the First
Robotics framework was an specifically trying challenge. In the next file I will show a later version of this code as we moved furthur difficult
challenges when we introduced a new library with new hardware in the middle of the build season.
*/
import org.usfirst.frc.team6662.robot.OI;
import org.usfirst.frc.team6662.robot.Robot;
import org.usfirst.frc.team6662.robot.RobotMap;
import org.usfirst.frc.team6662.robot.commands.TankDriveWithJoystick;
//Imports other classes from team 6662 robot code including a Oi which interfaces with the FRC hardware, RobotMap
//- a class that encapsulates values used by the mechanical team, and TankDriveWithJoystick which interfaces
//with our remote controller

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
//imports first classes for moter controllers, sensors, and miscellanious classes required to interface with the robot
public class Drivetrain extends Subsystem {
	public static final boolean HIGH_GEAR = true;
	public static final boolean LOW_GEAR = false;
	//boolean values that pertain to the state of the gear shifters on the robot (pneumatic)
	
	private Spark frontLeft = new Spark(RobotMap.FRONT_LEFT_MOTOR);
	private Spark rearLeft = new Spark(RobotMap.REAR_LEFT_MOTOR);
	private SpeedControllerGroup leftSide = new SpeedControllerGroup(frontLeft, rearLeft);
	//motor controller group that matches values on the motor controllers on the sides of the drive train
	
	private Spark frontRight = new Spark(RobotMap.FRONT_RIGHT_MOTOR);
	private Spark rearRight = new Spark(RobotMap.REAR_RIGHT_MOTOR);
	private SpeedControllerGroup rightSide = new SpeedControllerGroup(frontRight, rearRight);
	
	private DifferentialDrive drivetrain = new DifferentialDrive(leftSide, rightSide);
	//A class that combines motor controllers into a preset drive train class
	//Used tank drivetrain that turned using a matching motors on each side of a rectangular drive train(like a tank)
	
	private Compressor compressor = new Compressor();
	private DoubleSolenoid shifter = new DoubleSolenoid(RobotMap.SHIFTER_FORWARD_PORT, 
			RobotMap.SHIFTER_REVERSE_PORT);
	//created compressor and solenoids for drivetrain shifting
	
	private Encoder leftEncoder = new Encoder(RobotMap.ENCODER_INPUT_LEFT_A, RobotMap.ENCODER_INPUT_LEFT_B);
	private Encoder rightEncoder = new Encoder(RobotMap.ENCODER_INPUT_RIGHT_A, RobotMap.ENCODER_INPUT_RIGHT_B);
	//Created class for encoderes that monitor the speed of the motor controllers
	  
	private boolean shiftState = LOW_GEAR;
	
	private Gyro gyroscope = new ADXRS450_Gyro();
	//Creates gyroscope sensor to measure angle of robot
	
	public Drivetrain() {
		super("Drivetrain");
		compressor.setClosedLoopControl(true);
		//calles to super drive train class and sets compressor value to true
		//Defined by first library
	}
	
	public Gyro getGyro() {
		return gyroscope;
		//calles method from gryo class that gives back "gyroscope" a value that encapsulates
		//the yaw, pitch, roll, and other values from gyro
	}
	
	public Encoder getLeftEncoder() {
		return leftEncoder;
	}
	
	public Encoder getRightEncoder() {
		return rightEncoder;
	}
	//returns encoder values to be checked later
	
	public void tankDrive(double leftSpeed, double rightSpeed) {
		drivetrain.tankDrive(leftSpeed, rightSpeed);
	}
	//enables tank drive from class DifferentialDrive defined above
	
	public void shiftGear() {
		if (shiftState == LOW_GEAR) {
			shiftToHighGear();
		}
		else {
			shiftToLowGear();
		}
		//boolean value pertaining to shift state of pneumatics
	}
	
	public void shiftToHighGear() {
		shifter.set(DoubleSolenoid.Value.kForward);
		shiftState = HIGH_GEAR;
	}
	
	public void shiftToLowGear() {
		shifter.set(DoubleSolenoid.Value.kReverse);
		shiftState = LOW_GEAR;
	}

	@Override
	protected void initDefaultCommand() {
		setDefaultCommand(new TankDriveWithJoystick(Robot.oi.getJoystick(), OI.LEFT_Y_AXIS, OI.RIGHT_Y_AXIS));
		//sets default action of the robot when using class to run motors on drive train using values from joystick
	}
}
