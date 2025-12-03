/*
Copyright 2025 FIRST Tech Challenge Team 31065

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import android.util.Size;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import java.text.SimpleDateFormat;
import java.util.Date;


@TeleOp()


public class BlueTeleOp extends LinearOpMode {
		private static final boolean USE_WEBCAM = true;	// true for webcam, false for phone camera
		private AprilTagProcessor aprilTag;
		private VisionPortal visionPortal;
		View relativeLayout;
		private DistanceSensor sensorDistance;
		private DistanceSensor sensorDistance2;
		TouchSensor touchSensor;	// Touch sensor Object
		private ElapsedTime runtime = new ElapsedTime();
		private ElapsedTime cTimer = new ElapsedTime();
		private ElapsedTime cTimer2 = new ElapsedTime();
		public ElapsedTime redundancy = new ElapsedTime();
		DcMotor frontLeftDrive = null;
		DcMotor backLeftDrive = null;
		DcMotor frontRightDrive = null;
		DcMotor backRightDrive = null;
		private DcMotor catapult = null;
		private DcMotor catapult2 = null;
		private DcMotor muncher = null;
		private DcMotor muncher2 = null;
		private Servo release1 = null;
		private Servo release2 = null;
		private Servo release3 = null;
		private Servo guideL = null;
		private Servo guideR = null;
		IMU imu;
		SparkFunOTOS myOtos;

		//Variables, will be used in code, not for setting
		public double otosHeading = 0;
		public double currentAngle = 0;
		private int cState = 1;
		public boolean detected = false;
		private double launchPower = 0;
		private double launchPowerClamped = 0; // makes sure it is in between 0 and 1
		private double TargetA = 0;
		private double autoTurnSpeed = 0;
		private boolean cat1Launched = true;
		private boolean cat2Launched = true;
		private boolean cat3Launched = true;
		
		private boolean cPosOverride = false;
		
		private boolean lessThan = false;
		private boolean greaterThan = false;
		
		private boolean runOnce = false;
	

		public double yaw = 0;
		private double axial = 0;
		private double lateral = 0;
		private int cPos = 0;
		private int cPosCorrected = 0;
		public int aprilId = 0;
		public double aprilAngle = 0;

		private boolean shotAll = false;
		private boolean cBack = false;
		private boolean autoAiming = false;
		private double otosAngleDegrees = 0;

		private double driveSpeed = 1;
		private double hyp = 0;
		public double rotate = 0;
		public double headingOffset = 0;
		public double correctedHeading = 0;
		private double angleDifference = 0;

		//variables but could be changed
		private double savedX = -117.0;
		private double savedY = -23.0;
		private double savedH = 0.0;
		private double savedC = 0;
		public static final String FINAL_X_KEY = "final_x";
		public static final String FINAL_Y_KEY = "final_y";
		public static final String FINAL_H_KEY = "final_h";
		public static final String FINAL_C_KEY = "final_c";
		private int catOffset = 0;
		public double xOffset = 0;
		public double yOffset = 0;
		public double rotationOffset = 0;

		//These are the offset position values
		public double yAdjusted = 0;
		public double xAdjusted = 0;
		private double hypOffset = 0;
		private double driverHypOffset = 0;
		private boolean slowModeBoolean = false;
		private boolean lastLeftBumper = false;
		public String[] motif = {"green", "purple", "purple"};
		public String[] catColors = {"blank", "blank", "blank"};
		public int i = 0;
		public int j = 0;
		public double aprilX = 0;
		public double aprilY = 0;
		public int catapultPositionDifference = 0;



		//variables for changing
		public double closeLaunchRatio = 1; // increase this variable if you want to have more power in the close zone, and decrease for less. Default is 1, launch power would be linear, increasing as you get further from the goal.
		public double catapultPositionMultiplier = 1.4;






		public void runOpMode() {


				// you can use this as a regular DistanceSensor.
				sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");
				sensorDistance2 = hardwareMap.get(DistanceSensor.class, "sensor_distance2");
				touchSensor = hardwareMap.get(TouchSensor.class, "sensor_touch");


				// you can also cast this to a Rev2mDistanceSensor if you want to use added
				// methods associated with the Rev2mDistanceSensor class.
				Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;
				Rev2mDistanceSensor sensorTimeOfFlight2 = (Rev2mDistanceSensor) sensorDistance2;



				// Get a reference to the RelativeLayout so we can later change the background
				// color of the Robot Controller app to match the hue detected by the RGB sensor.
				int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
				relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);



				catapult = hardwareMap.get(DcMotor.class, "C");
				catapult2 = hardwareMap.get(DcMotor.class, "C2");
				muncher = hardwareMap.get(DcMotor.class, "M");
				muncher2 = hardwareMap.get(DcMotor.class, "M2");
				release1 = hardwareMap.get(Servo.class, "r1");
				release2 = hardwareMap.get(Servo.class, "r2");
				release3 = hardwareMap.get(Servo.class, "r3");
				guideL = hardwareMap.get(Servo.class, "gl");
				guideR = hardwareMap.get(Servo.class, "gr");


				initAprilTag();


				// Get a reference to the sensor
				myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
				// All the configuration for the OTOS is done in this helper method, check it out!
				configureOtos();


				cPos = 0;
				hypOffset = 0;







				frontLeftDrive = hardwareMap.get(DcMotor.class, "LF");
				frontRightDrive = hardwareMap.get(DcMotor.class, "RF");
				backLeftDrive = hardwareMap.get(DcMotor.class, "LB");
				backRightDrive = hardwareMap.get(DcMotor.class, "RB");


				// We set the left motors in reverse which is needed for drive trains where the left
				// motors are opposite to the right ones.
				backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
				frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);


				frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
				frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
				backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
				backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


				imu = hardwareMap.get(IMU.class, "imu");
				// This needs to be changed to match the orientation on your robot
				RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
								RevHubOrientationOnRobot.LogoFacingDirection.UP;
				RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
								RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

				RevHubOrientationOnRobot orientationOnRobot = new
								RevHubOrientationOnRobot(logoDirection, usbDirection);
				imu.initialize(new IMU.Parameters(orientationOnRobot));


				// This uses RUN_USING_ENCODER to be more accurate.	 If you don't have the encoder
				// wires, you should remove these
				frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
				frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
				backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
				backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

				aprilTagDetectionMethod();



				// Wait for the game to start (driver presses START)
				telemetry.addData("Status", "Initialized");
				telemetry.addLine("If the auto messed up press start");






				telemetry.update();


				waitForStart(); // code underneath will run ONCE when we press play


				runtime.reset();


				catapult.setDirection(DcMotor.Direction.REVERSE);
				catapult2.setDirection(DcMotor.Direction.FORWARD);
				muncher.setDirection(DcMotor.Direction.FORWARD);
				muncher.setDirection(DcMotor.Direction.FORWARD);
				catapult.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
				catapult.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
				catapult2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
				catapult2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
				catapult.setPower(1);
				catapult2.setPower(1);
				release1.setPosition(0.3);
				release2.setPosition(0.67);
				release3.setPosition(0.75);


				// run until the end of the match (driver presses STOP)
				while (opModeIsActive()) {
						aprilTagDetectionMethod();
						//BLUE SPECIFIC
						SparkFunOTOS.Pose2D pos = myOtos.getPosition();	 //needs to be before the otosHeading=pos.h
						otosHeading = pos.h + 1.57079632679; // the plus 1.57079632679 is to adjust field centric driving so that it works when the robot is turned 90 degrees starting teleop
						//This code is for offsetting the pos.x and pos.y values (not really, just gets them into variables?)
						yAdjusted = pos.y + yOffset;
						xAdjusted = pos.x + xOffset;


					 


						telemetry.addLine("Press B to reset robot position and angle at starting position");
						telemetry.addLine("");
						telemetry.addLine("Hold left bumper to drive in robot relative");
						telemetry.addLine("If the auto messed up press start on the driver");
						telemetry.addLine("");
						telemetry.addLine("driver press right bumper to toggle slow mode");
						telemetry.addLine("");
						telemetry.addLine("driver press A to auto aim");
						telemetry.addLine("");
						telemetry.addLine("");
						telemetry.addData("aprilX", aprilX);
						telemetry.addLine("");
						telemetry.addData("aprilY", aprilY);
						telemetry.addLine("");
						telemetry.addData("aprilAngle", aprilAngle);
						telemetry.addLine("");
						telemetry.addLine("");

						/*
						if(gamepad2.start){ // in the case of an auto faliure
						SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(savedX, savedY, savedH);
						}
						*/

						// press B to reset the robot position to where you ideally end auto
						if (gamepad2.b) {
								configureOtos();
						}
						

					 

						catapultControl();

						//toggle slow mode
						boolean currentLeftBumper = gamepad2.right_bumper;
						if (currentLeftBumper && !lastLeftBumper) {
								slowModeBoolean = !slowModeBoolean; }
						lastLeftBumper = currentLeftBumper;


						if(slowModeBoolean == true || gamepad1.dpad_down){ // for final base park
								driveSpeed = 0.4;
						}else{
								driveSpeed = 1;
						}


						catapult.setTargetPosition(cPosCorrected);
						catapult.setMode(DcMotor.RunMode.RUN_TO_POSITION);

						catapult2.setTargetPosition(cPosCorrected-catapultPositionDifference);
						catapult2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

						cPosCorrected = cPos + 1870;
						
						catapultPositionDifference = catapult.getCurrentPosition()-catapult2.getCurrentPosition();
						
						
						double distance1 = sensorDistance.getDistance(DistanceUnit.MM);
						double distance2 = sensorDistance2.getDistance(DistanceUnit.MM);


						if(/*touchSensor.isPressed() || */distance2 < 100){
								cBack = true;
						}


				

					
						if(gamepad1.x){
								launchLeft();
								cTimer.reset();}
						if(gamepad1.y){
								launchMiddle();
								cTimer.reset();}
						if(gamepad1.b){
								launchRight();
								cTimer.reset();} 
						if(gamepad1.left_bumper){ //THE BIG BANG - Shoot all three
								bigBang(); //release all
						}


						if(gamepad1.dpad_up){
								launchMotif(catColors);
						}




						//reset all offsets to 0
						if(gamepad2.back){
								catOffset=0;
								driverHypOffset=0;
								hypOffset=0;
								rotationOffset=0;
								xOffset=0;
								yOffset=0;
						}

						intakeAndGuideServos();
						
						if(aprilId == 20 && detected == true){
								SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(pos.x, pos.y, aprilAngle);
								myOtos.setPosition(currentPosition);
								//angleDifference = (90 - (aprilAngle + 60)) - otosAngleDegrees;
							//	currentAngle = 90 - (aprilAngle + 60);
							
						}/*else{
								currentAngle = correctedHeading;
						}*/
						
						otosAngleDegrees = pos.h * (180/3.14); // convert the otos angle to degrees
						correctedHeading = otosAngleDegrees + angleDifference;
						
						currentAngle = otosAngleDegrees /*correctedHeading*/;
						
					
						
						TargetA = (90 - ((180/3.14) * (Math.abs(Math.atan((xAdjusted)/(yAdjusted)))))) + rotationOffset; // calculate target angle
						autoTurnSpeed = Math.abs(Math.abs((currentAngle - TargetA)) / 65 + .05);

						 if(gamepad2.a){ //AUTO AIM
								autoAim();
						}else if(gamepad1.right_bumper){
								constantAngleAim(pos.x, pos.y, pos.h);
						} else { //drive normally
								if (gamepad2.left_bumper) {
										// If you press the left bumper, you get a drive from the point of view of the robot
										// (much like driving an RC vehicle)
										drive((-gamepad2.left_stick_y*driveSpeed), (gamepad2.left_stick_x*driveSpeed), (0.8*gamepad2.right_stick_x*driveSpeed));
								} else {
										driveFieldRelative((-gamepad2.left_stick_y*driveSpeed), (gamepad2.left_stick_x*driveSpeed),(0.8*gamepad2.right_stick_x*driveSpeed));
								}
						}

						// Show the elapsed game time and wheel power.
						telemetry.addData("Status", "Run Time: " + runtime.toString());

						telemetry.addData("c1current pos", catapult.getCurrentPosition());
						telemetry.addData("c2current pos", catapult2.getCurrentPosition());
						telemetry.addData("catapultPositionDifference", catapultPositionDifference);
						telemetry.addLine("");
						telemetry.addData("aprilX", aprilX);
						telemetry.addLine("");
						telemetry.addData("aprilY", aprilY);
						telemetry.addLine("");
						telemetry.addData("aprilAngle", aprilAngle);
						telemetry.addLine("");
						telemetry.addData("angleDiffernce", angleDifference);
						telemetry.addLine("");
						telemetry.addData("correctedHeading", correctedHeading);
						telemetry.addLine("");
						telemetry.addData("otosAngleDegrees", otosAngleDegrees);
						telemetry.addLine("");

						telemetry.addData("currentAngle", currentAngle);
						telemetry.addLine("");
						telemetry.addData("autoTurnSpeed", autoTurnSpeed);
						
						telemetry.addData("greaterThan", greaterThan);
						telemetry.addData("lessThan", lessThan);
						telemetry.addData("detected", detected);
						telemetry.addLine("");
						telemetry.addData("pos.x", pos.x);
						telemetry.addLine("");
						telemetry.addData("pos.y", pos.y);
						telemetry.addLine("");
						telemetry.addData("hyp", hyp);
						telemetry.addLine("");
						telemetry.addData("launchPower", launchPower);
						telemetry.addLine("");
						telemetry.addData("launchPowerClamped", launchPowerClamped);
						telemetry.addLine("");
						telemetry.addData("aprilId", aprilId);
						telemetry.addLine("");
						telemetry.addLine("");
						telemetry.addData("TargetA", TargetA);
						telemetry.addData("autoTurnSpeed", autoTurnSpeed);

						telemetry.addData("range", String.format("%.01f mm", sensorDistance.getDistance(DistanceUnit.MM)));
						telemetry.addData("range", String.format("%.01f mm", sensorDistance2.getDistance(DistanceUnit.MM)));


						telemetry.addLine("");
						telemetry.addData("OTOS pos.h", pos.h);
						telemetry.addData("cPos",cPos);
						telemetry.addData("catapult.getCurrentPosition()", catapult.getCurrentPosition());
						telemetry.addData("catapult2.getCurrentPosition()", catapult2.getCurrentPosition());
						telemetry.addLine("");
						telemetry.addData("catapult1getTargetPosition", catapult.getTargetPosition());
						telemetry.addData("catapult2getTargetPosition", catapult2.getTargetPosition());
						telemetry.addLine("");
						telemetry.addData("cState",cState);
						telemetry.addData("yaw (auto turn power)", yaw);
						telemetry.addData("rotationOffset", rotationOffset);
						telemetry.addData("driveSpeed", driveSpeed);
						telemetry.addData("autoTurnSpeed", autoTurnSpeed);
						telemetry.addData("otosHeading", otosHeading);
						telemetry.addData("OTOS pos.h", pos.h);
						telemetry.addData("cBack",cBack);



						telemetry.update();
				}
		}

		private void driveFieldRelative(double forward, double right, double rotate) {
				// First, convert direction being asked to drive to polar coordinates


				SparkFunOTOS.Pose2D pos = myOtos.getPosition();


				double theta = Math.atan2(forward, right);
				double r = Math.hypot(right, forward);


				// Second, rotate angle by the angle the robot is pointing
				theta = AngleUnit.normalizeRadians(theta -
								otosHeading); //imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));


				// Third, convert back to cartesian
				double newForward = r * Math.sin(theta);
				double newRight = r * Math.cos(theta);


				// Finally, call the drive method with robot relative forward and right amounts
				drive(newForward, newRight, rotate);
				telemetry.addData("IMU angle", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
		}
		// ROBOT CENTRIC driving (non field centric) when the driver holds left bumper
		public void drive(double forward, double right, double rotate) {
				// This calculates the power needed for each wheel based on the amount of forward,
				// strafe right, and rotate
				double frontLeftPower = forward + right + rotate;
				double frontRightPower = forward - right - rotate;
				double backRightPower = forward + right - rotate;
				double backLeftPower = forward - right + rotate;


				double maxPower = 1.0;
				double maxSpeed = 1.0;	// make this slower for outreaches


				// This is needed to make sure we don't pass > 1.0 to any wheel
				// It allows us to keep all of the motors in proportion to what they should
				// be and not get clipped
				maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
				maxPower = Math.max(maxPower, Math.abs(frontRightPower));
				maxPower = Math.max(maxPower, Math.abs(backRightPower));
				maxPower = Math.max(maxPower, Math.abs(backLeftPower));


				// We multiply by maxSpeed so that it can be set lower for outreaches
				// When a young child is driving the robot, we may not want to allow full
				// speed.
				frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
				frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
				backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
				backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
		}
		public void catapultControl(){

				//allow auto pull back if all three are lauched or if the operator overrides by pressing a
				if((gamepad1.a || (cat1Launched && cat2Launched && cat3Launched)) && cTimer.seconds() > 0.250){
						cState = 1;
				}

				//auto pullback
				if(cState == 1){


						//open up the ones that were launched
						if (cat1Launched){
								release1.setPosition(0.3);}
						if (cat2Launched){
								release2.setPosition(0.67);}
						if (cat3Launched){
								release3.setPosition(0.75);}
						cat1Launched = false;
						cat2Launched = false;
						cat3Launched = false;
						//pull it back
						cPos = -200000;
						cState = 2;
						redundancy.reset();
					


				}	if(cState == 2 && (cBack == true || redundancy.seconds() > 5)){
						//lock in place
					 
							
								cPos = catapult.getCurrentPosition() - 1870;
								redundancy.reset();
						

						release1.setPosition(0.7);
						release2.setPosition(0.22);
						release3.setPosition(0.35);
						cState = 3;


				}
				if(cState == 3 && redundancy.seconds() > 0.4){
						catapult.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
						catapult2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
						cPos = -1870;// * (int) catapultPositionMultiplier;
						cState = 4;
						cTimer2.reset();
						redundancy.reset();
				}
				


				if(cState == 4){
						cBack = false;
						if(cTimer2.seconds() > 0.5){
								cPos = (int) Math.round( (-1860*catapultPositionMultiplier) + ((1860*catapultPositionMultiplier) * launchPowerClamped));
						}


						hyp = Math.sqrt(Math.abs(((xAdjusted * xAdjusted)) + ((yAdjusted * yAdjusted)))) + hypOffset + driverHypOffset; // calculate the hypotenuse (distance to the goal)


						if(hyp > 95){
								//launchPower will create a decimal ranging from 0 to 1, which corresponds to the amount	the catapults pull back.
								launchPower = (1.5);
						}else{
								//launchPower will create a decimal ranging from 0 to 1, which corresponds to the amount	the catapults pull back.
								launchPower = ((hyp/140) * closeLaunchRatio) ;
						}
						launchPowerClamped = Math.max(0.0, Math.min(1.5, launchPower)); //.min picks the smaller, meaning no values more than 1. .max picks bigger meaning none less than 0.


						if (gamepad2.dpad_up || gamepad1.dpad_up){
								catOffset -= 25;
						}
						if (gamepad2.dpad_down){
								catOffset += 25;
						}
						if(gamepad2.right_bumper){
								if(gamepad2.dpad_up){
										driverHypOffset -= 1;
								}
								if(gamepad2.dpad_down){
										driverHypOffset += 1;
								}


						}
				}


		}
		public void autoAim(){

				/*if(aprilId == 20){
						if(otosAngleDegrees >	90 - (aprilAngle + 45)){
								headingOffset = otosAngleDegrees - ( (aprilAngle + 45));
						}
				}*/
				
				correctedHeading = otosAngleDegrees - headingOffset;


			 //else if (aprilId != 20 || detected == false){
					 // currentAngle = otosAngleDegrees;
		//		}
						
			 /* TargetA = (90 - ((180/3.14) * (Math.abs(Math.atan((xAdjusted)/(xAdjusted)))))) + rotationOffset; // calculate target angle
				autoTurnSpeed = Math.abs(Math.abs((otosAngleDegrees - TargetA)) / 45 + .1);
*/
				driveFieldRelative((-gamepad2.left_stick_y*driveSpeed), (gamepad2.left_stick_x*driveSpeed), yaw);

				telemetry.addData("autoTurnSpeed", autoTurnSpeed);
				telemetry.addData("TargetA", TargetA);
				telemetry.addData("greaterThan", greaterThan);
				telemetry.addData("lessThan", lessThan);
				telemetry.update();


				if(currentAngle < TargetA - 3){
						yaw = -.5 * autoTurnSpeed;
						lessThan = true;
						greaterThan = false;
				}else if (currentAngle > TargetA + 3){
						yaw = 0.5 * autoTurnSpeed;
						lessThan = false;
						greaterThan = true;
				}else{
						yaw = 0;
						lessThan = false;
						greaterThan = false;
				}


				if(gamepad2.dpad_left || gamepad1.left_stick_x > 0.1){
						rotationOffset += 1;
				}
				if(gamepad2.dpad_right || gamepad1.left_stick_x<-0.1){
						rotationOffset -=1;
				}



		} // End AutoAim void
		private void constantAngleAim(double xPos, double yPos, double hPos){


				autoTurnSpeed = Math.abs(Math.abs((otosAngleDegrees - TargetA)) / 65 + .1);
				if(xPos < -70){
						TargetA = 23 * -1;
				} else if(xPos >= -70){
						TargetA = 45 * -1;
				}

				if(currentAngle < TargetA - 3){
						yaw = -.67 * autoTurnSpeed;
				}else if (currentAngle > TargetA + 3){
						yaw = 0.67 * autoTurnSpeed;
				}else{
						yaw = 0;
				}
				driveFieldRelative((-gamepad2.left_stick_y*driveSpeed), (gamepad2.left_stick_x*driveSpeed), yaw);

		}
		private void pullBackCatapults(){


		}
		private void launchLeft(){
				release1.setPosition(0.3);
				cat1Launched = true;
		}
		private void launchMiddle(){
				release2.setPosition(0.67);
				cat2Launched = true;
		}
		private void launchRight(){
				release3.setPosition(0.75);
				cat3Launched = true;
		}
		private void bigBang(){
				release1.setPosition(0.3);
				release2.setPosition(0.67);
				release3.setPosition(0.75);
				cat1Launched = true;
				cat2Launched = true;
				cat3Launched = true;
		}
		private void launchMotif(String[] cColors) {
				telemetry.addData("cColors", cColors[0]);
				telemetry.addData("cColors", cColors[1]);
				telemetry.addData("cColors", cColors[2]);


				telemetry.addData("motif", motif[0]);
				telemetry.addData("motif", motif[1]);
				telemetry.addData("motif", motif[2]);


				telemetry.update();
				// sleep (200000);
				for (i = 0; i < 3; i++){
						for (j = 0; j < 3; j++){
								if (cColors[j] == motif[i]){
										if (j == 0){
												//fire catapult 1
												release1.setPosition(0.3);
												cColors[j] = "empty";
												sleep(700);
										}if (j == 1){
												//fire catapult 2
												release2.setPosition(0.67);
												cColors[j] = "empty";
												sleep(700);
										}if(j == 2){
												//fire catapult 3
												release3.setPosition(0.75);
												cColors[j] = "empty";
												sleep(700);
										}


										break;
								}
						}
				}
				for (i = 0; i < 3; i++){
						if (cColors[i] != "empty"){
								if (i == 0){
										//fire catapult 1
										release1.setPosition(0.3);
								} else if (i == 1){
										//fire catapult 2
										release2.setPosition(0.67);
								} else if(i == 2){
										//fire catapult 3
										release3.setPosition(0.75);
								}
						}
				}
		}
		public void intakeAndGuideServos(){

				//intake and spit
				if(gamepad1.right_trigger > 0.8 || gamepad2.right_trigger > 0.8){
						//intake
						muncher.setPower(1);
						muncher2.setPower(1);
				}else if ((gamepad1.right_trigger > 0.2 && gamepad1.right_trigger <= 0.8 ) || (gamepad2.right_trigger > 0.2 && gamepad2.right_trigger <= 0.8 )){
						muncher.setPower(0.7);
						muncher2.setPower(0.7);
				}
				else if (gamepad1.left_trigger > 0.2 || gamepad2.left_trigger > 0.2){
						//spit
						muncher.setPower(-1);
						muncher2.setPower(-1);
						//whack by closing
						guideL.setPosition(0.45);
						guideR.setPosition(0.6);
				} else {
						//no power, stop
						muncher.setPower(0);
						muncher2.setPower(0);
				}



				if(otosHeading > -0.3 ){
						//intake guide servo code for when the intake is faced towards the drive team
						if (gamepad1.left_trigger > 0.2 || gamepad2.left_trigger > 0.2){
								//spit
								muncher.setPower(-1);
								muncher2.setPower(-1);
								//whack by closing
								guideL.setPosition(0.45);
								guideR.setPosition(0.6);
						} else if (gamepad1.right_stick_y<-0.4){
								//Middle (eject)
								guideL.setPosition(0.3);
								guideR.setPosition(0.742);
						} else if(gamepad1.right_stick_x>-0.4 && gamepad1.right_stick_x<0.4 ){
								//open
								guideL.setPosition(0.097);
								guideR.setPosition(0.93);
						} else if (gamepad1.right_stick_x<-0.4){
								//side
								guideL.setPosition(0.097);
								guideR.setPosition(0.65);
						} else if (gamepad1.right_stick_x>0.4){
								//side
								guideL.setPosition(0.4);
								guideR.setPosition(0.93);
						}
				} else {
						if (gamepad1.left_trigger > 0.2 || gamepad2.left_trigger > 0.2) {
								//spit
								muncher.setPower(-1);
								muncher2.setPower(-1);
								//whack by closing
								guideL.setPosition(0.45);
								guideR.setPosition(0.6);
						} else if (gamepad1.right_stick_y>0.4){
								guideL.setPosition(0.35);
								guideR.setPosition(0.692);
						} else if(gamepad1.right_stick_x>-0.4 && gamepad1.right_stick_x<0.4 ){
								//open
								guideL.setPosition(0.097);
								guideR.setPosition(0.93);
						} else if (gamepad1.right_stick_x>0.4){
								guideL.setPosition(0.097);
								guideR.setPosition(0.65);
						} else if (gamepad1.right_stick_x<-0.4){
								guideL.setPosition(0.4);
								guideR.setPosition(0.93);
						}
				}
		}//end of void intakeAndGuideServoStuff



		private void initAprilTag() {

				// Create the AprilTag processor.
				aprilTag = new AprilTagProcessor.Builder()

								// The following default settings are available to un-comment and edit as needed.
								//.setDrawAxes(false)
								//.setDrawCubeProjection(false)
								//.setDrawTagOutline(true)
								//.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
								//.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
								//.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

								// == CAMERA CALIBRATION ==
								// If you do not manually specify calibration parameters, the SDK will attempt
								// to load a predefined calibration for your camera.
								//.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
								// ... these parameters are fx, fy, cx, cy.

								.build();

				// Adjust Image Decimation to trade-off detection-range for detection-rate.
				// eg: Some typical detection data using a Logitech C920 WebCam
				// Decimation = 1 ..	Detect 2" Tag from 10 feet away at 10 Frames per second
				// Decimation = 2 ..	Detect 2" Tag from 6	feet away at 22 Frames per second
				// Decimation = 3 ..	Detect 2" Tag from 4	feet away at 30 Frames Per Second (default)
				// Decimation = 3 ..	Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
				// Note: Decimation can be changed on-the-fly to adapt during a match.
				//aprilTag.setDecimation(3);

				// Create the vision portal by using a builder.
				VisionPortal.Builder builder = new VisionPortal.Builder();

				// Set the camera (webcam vs. built-in RC phone camera).
				if (USE_WEBCAM) {
						builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
				} else {
						builder.setCamera(BuiltinCameraDirection.BACK);
				}

				// Choose a camera resolution. Not all cameras support all resolutions.
				//builder.setCameraResolution(new Size(640, 480));

				// Enable the RC preview (LiveView).	Set "false" to omit camera monitoring.
				//builder.enableLiveView(true);

				// Set the stream format; MJPEG uses less bandwidth than default YUY2.
				//builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

				// Choose whether or not LiveView stops if no processors are enabled.
				// If set "true", monitor shows solid orange screen if no processors enabled.
				// If set "false", monitor shows camera view without annotations.
				//builder.setAutoStopLiveView(false);

				// Set and enable the processor.
				builder.addProcessor(aprilTag);

				// Build the Vision Portal, using the above settings.
				visionPortal = builder.build();

				// Disable or re-enable the aprilTag processor at any time.
				//visionPortal.setProcessorEnabled(aprilTag, true);

		}	 // end method initAprilTag()
		public void configureOtos() {
				telemetry.addLine("Configuring OTOS...");
				telemetry.update();


				// Set the desired units for linear and angular measurements. Can be either
				// meters or inches for linear, and radians or degrees for angular. If not
				// set, the default is inches and degrees. Note that this setting is not
				// persisted in the sensor, so you need to set at the start of all your
				// OpModes if using the non-default value.
				// myOtos.setLinearUnit(DistanceUnit.METER);
				myOtos.setLinearUnit(DistanceUnit.INCH);
				myOtos.setAngularUnit(AngleUnit.RADIANS);
				//myOtos.setAngularUnit(AngleUnit.DEGREES);


				// Assuming you've mounted your sensor to a robot and it's not centered,
				// you can specify the offset for the sensor relative to the center of the
				// robot. The units default to inches and degrees, but if you want to use
				// different units, specify them before setting the offset! Note that as of
				// firmware version 1.0, these values will be lost after a power cycle, so
				// you will need to set them each time you power up the sensor. For example, if
				// the sensor is mounted 5 inches to the left (negative X) and 10 inches
				// forward (positive Y) of the center of the robot, and mounted 90 degrees
				// clockwise (negative rotation) from the robot's orientation, the offset
				// would be {-5, 10, -90}. These can be any value, even the angle can be
				// tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
				SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
				myOtos.setOffset(offset);


				// Here we can set the linear and angular scalars, which can compensate for
				// scaling issues with the sensor measurements. Note that as of firmware
				// version 1.0, these values will be lost after a power cycle, so you will
				// need to set them each time you power up the sensor. They can be any value
				// from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
				// first set both scalars to 1.0, then calibrate the angular scalar, then
				// the linear scalar. To calibrate the angular scalar, spin the robot by
				// multiple rotations (eg. 10) to get a precise error, then set the scalar
				// to the inverse of the error. Remember that the angle wraps from -180 to
				// 180 degrees, so for example, if after 10 rotations counterclockwise
				// (positive rotation), the sensor reports -15 degrees, the required scalar
				// would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
				// robot a known distance and measure the error; do this multiple times at
				// multiple speeds to get an average, then set the linear scalar to the
				// inverse of the error. For example, if you move the robot 100 inches and
				// the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
				myOtos.setLinearScalar(1.0);
				myOtos.setAngularScalar(1.0);


				// The IMU on the OTOS includes a gyroscope and accelerometer, which could
				// have an offset. Note that as of firmware version 1.0, the calibration
				// will be lost after a power cycle; the OTOS performs a quick calibration
				// when it powers up, but it is recommended to perform a more thorough
				// calibration at the start of all your OpModes. Note that the sensor must
				// be completely stationary and flat during calibration! When calling
				// calibrateImu(), you can specify the number of samples to take and whether
				// to wait until the calibration is complete. If no parameters are provided,
				// it will take 255 samples and wait until done; each sample takes about
				// 2.4ms, so about 612ms total
				myOtos.calibrateImu();


				// Reset the tracking algorithm - this resets the position to the origin,
				// but can also be used to recover from some rare tracking errors
				myOtos.resetTracking();


				// After resetting the tracking, the OTOS will report that the robot is at
				// the origin. If your robot does not start at the origin, or you have
				// another source of location information (eg. vision odometry), you can set
				// the OTOS location to match and it will continue to track from there.
				// Load position from autonomous (or use defaults if auto didn't run)
				// savedX = (double) blackboard.getOrDefault(FINAL_X_KEY, -117.0);
				// savedY = (double) blackboard.getOrDefault(FINAL_Y_KEY, -23.0);
				//savedH = (double) blackboard.getOrDefault(FINAL_H_KEY, 0.0);
				SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(-117, -23, 0);
				myOtos.setPosition(currentPosition);


				// Get the hardware and firmware version
				SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
				SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
				myOtos.getVersionInfo(hwVersion, fwVersion);


				telemetry.addLine("OTOS configured! Press start to get position data!");
				telemetry.addLine();
				telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
				telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
				telemetry.update();
		}//end of configureOtos()
		private void aprilTagDetectionMethod() {

				List<AprilTagDetection> currentDetections = aprilTag.getDetections();
				// telemetry.addData("# AprilTags Detected", currentDetections.size());

				// Step through the list of detections and display info for each one.
				for (AprilTagDetection detection : currentDetections) {
						if (detection.metadata != null) {
								// telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
							 // telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f	(inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
						//		telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f	(deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
								//		telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f	(inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
								aprilId = detection.id;
							 // detected = true;
								aprilAngle =	detection.ftcPose.yaw;
								aprilY = detection.ftcPose.x;
								aprilX = detection.ftcPose.y;
						} else {
								aprilId = detection.id;
								//	telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
								//telemetry.addLine(String.format("Center %6.0f %6.0f	 (pixels)", detection.center.x, detection.center.y));
						}
						
						
				}	 // end for() loop
						if(currentDetections.size() > 0){
								detected = true;
						}else if(currentDetections.size() <= 0){
								detected = false;
						}

				// Add "key" information to telemetry
	//			telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
		//		telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
			//	telemetry.addLine("RBE = Range, Bearing & Elevation");

		}	 // end method aprilTagDetectionMethod()

}//End of class



















