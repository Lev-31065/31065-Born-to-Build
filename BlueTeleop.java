package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp()

public class BlueTeleop extends LinearOpMode {

    SparkFunOTOS myOtos;
    public double otosHeading = 0;

    // This declares the IMU needed to get the current direction the robot is facing
    IMU imu;

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime cTimer = new ElapsedTime();
    private ElapsedTime cTimer2 = new ElapsedTime();

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

    private int catOffset = 0;
    private int cState = 1;

    private int angle = 45;
    private double launchPower = 0;
    private double TargetA = 0;
    private double autoTurnSpeed = 0;
    private boolean cat1Launched = false;
    private boolean cat2Launched = false;
    private boolean cat3Launched = false;

    public double yaw = 0;
    private double axial = 0;
    private double lateral = 0;
    private int cPos = 0;

    private boolean shotAll = false;
    private boolean cBack = false;
    private boolean autoAiming = false;
    private double otosAngleDegrees = 0;

    private double driveSpeed = 1;
    private double hyp = 0;
    public double rotate = 0;
    public double xOffset = 0;
    public double yOffset = 0;
    public double rotationOffset = 0;


    //These are the offset position values
    public double yAdjusted = 0;
    public double xAdjusted = 0;
    private double hypOffset = 0;
    private double driverHypOffset = 0;

    public double closeLaunchRatio = 1; // increase this variable if you want to have more power in the close zone, and decrease for less. Default is 1, launch power would be linear, increasing as you get further from the goal.



    public String[] motif = {"green", "purple", "purple"};
    public String[] catColors = {"blank", "blank", "blank"};

    public int i = 0;
    public int j = 0;

    public double launch1 = 0.3;
    public double launch2 = 0.67;
    public double launch3 = 0.75;



    public void runOpMode() {


        catapult = hardwareMap.get(DcMotor.class, "C");
        catapult2 = hardwareMap.get(DcMotor.class, "C2");
        muncher = hardwareMap.get(DcMotor.class, "M");
        muncher2 = hardwareMap.get(DcMotor.class, "M2");
        release1 = hardwareMap.get(Servo.class, "r1");
        release2 = hardwareMap.get(Servo.class, "r2");
        release3 = hardwareMap.get(Servo.class, "r3");
        guideL = hardwareMap.get(Servo.class, "gl");
        guideR = hardwareMap.get(Servo.class, "gr");


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

        // This uses RUN_USING_ENCODER to be more accurate.   If you don't have the encoder
        // wires, you should remove these
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
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

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

        SparkFunOTOS.Pose2D pos = myOtos.getPosition();   //needs to be before the otosHeading=pos.h
        otosHeading = pos.h + 1.57079632679; // the plus 1.57079632679 is to adjust field centric driving so that it works when the robot is turned 90 degrees starting teleop

        telemetry.addLine("Press B to reset Yaw");
        telemetry.addLine("Hold left bumper to drive in robot relative");
        telemetry.addLine("The left joystick sets the robot direction");
        telemetry.addLine("Moving the right joystick left and right turns the robot");

        // If you press the A button, then you reset the Yaw to be zero from the way
        // the robot is currently pointing
        if (gamepad2.b) {
            imu.resetYaw();
            myOtos.calibrateImu();

        } // Reset tracking if the user requests it
            if (gamepad2.x) {
                myOtos.resetTracking();
            }

        // If you press the left bumper, you get a drive from the point of view of the robot
        // (much like driving an RC vehicle)
        if (autoAiming == false){
            if (gamepad2.left_bumper) {
                drive((-gamepad2.left_stick_y*driveSpeed), (gamepad2.left_stick_x*driveSpeed), ((gamepad2.right_stick_x*driveSpeed)+(0.2*gamepad1.left_stick_x*driveSpeed)));
            } else {
                driveFieldRelative((-gamepad2.left_stick_y*driveSpeed), (gamepad2.left_stick_x*driveSpeed),((gamepad2.right_stick_x*driveSpeed)+(0.2*gamepad1.left_stick_x*driveSpeed)));
            }
        }
        //This code is for offsetting the pos.x and pos.y values
        yAdjusted = pos.y + yOffset;
        xAdjusted = pos.x + xOffset;

        if(gamepad2.dpad_right){
            rotationOffset -= 1;
        }
        if(gamepad2.dpad_left){
            rotationOffset +=1;
        }

        if(gamepad2.dpad_up){
            driverHypOffset -= 1;
        }
        if(gamepad2.dpad_down){
            driverHypOffset += 1;
        }


            if(gamepad1.back){
                driveSpeed = 0.4;
            }else{
                driveSpeed = 1;
            }

            if(pos.h > -0.3 || pos.h > 2.61799387799 ){
                    //intake guide servo code for when the intake is faced towards the drive team
            if (gamepad1.right_stick_y<-0.6){
               //Middle (eject)
                guideL.setPosition(0.35);
                guideR.setPosition(0.692);
            }else if(gamepad1.right_stick_x>-0.4 && gamepad1.right_stick_x<0.4 ){
            //open
                guideL.setPosition(0.097);
                guideR.setPosition(0.93);
            } else if (gamepad1.right_stick_x<-0.6){
               //side
                guideL.setPosition(0.097);
                guideR.setPosition(0.65);
            } else if (gamepad1.right_stick_x>0.6){
                //side
                guideL.setPosition(0.4);
                guideR.setPosition(0.93);
            }
            } else {
                if (gamepad1.right_stick_y>0.6){
                guideL.setPosition(0.35);
                guideR.setPosition(0.692);
            }else if(gamepad1.right_stick_x>-0.4 && gamepad1.right_stick_x<0.4 ){
                guideL.setPosition(0.097);
                guideR.setPosition(0.93);
            } else if (gamepad1.right_stick_x>0.6){
                guideL.setPosition(0.097);
                guideR.setPosition(0.65);
            } else if (gamepad1.right_stick_x<-0.6){
                guideL.setPosition(0.4);
                guideR.setPosition(0.93);
            }
            }



            catapult.setTargetPosition(cPos - catOffset);
            catapult.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            catapult2.setTargetPosition(cPos - catOffset);
            catapult2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if(catapult.getCurrentPosition()<-1850 + catOffset && catapult.getCurrentPosition()>-1890){
                cBack = true;
            }

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
                cPos = -1870;
                cState = 2;

                } if(cState == 2 && cBack == true){
                //lock in place
                release1.setPosition(0.7);
                release2.setPosition(0.22);
                release3.setPosition(0.35);
                cTimer2.reset();
                cState = 3;

                }

                if(cState == 3){
                    cBack = false;
                    if(cTimer2.seconds() > 0.5){
                        cPos = (int) Math.round( -1870 + (1870 * launchPower));
                    }

                    if(gamepad1.left_bumper){
                       if (gamepad1.dpad_down){
                            catOffset += 25;
                        }
                        if (gamepad1.dpad_up){
                            catOffset -= 25;
                        }

                    }
                }

            if(gamepad1.x){
                launchLeft();
                cTimer.reset();
            }

            if(gamepad1.y){
                launchMiddle();
                cTimer.reset();
            }

            if(gamepad1.b){
                launchRight();
                cTimer.reset();
            }

            if(gamepad1.left_bumper){ //THE BIG BANG - Shoot all three
                bigBang(); //release all
            }

            if(gamepad1.dpad_up){
                launchMotif(catColors);
            }

            //intake and spit
            if(gamepad1.right_trigger > 0.2 || gamepad2.right_trigger > 0.2){
                muncher.setPower(1);
                muncher2.setPower(1);
            } else if (gamepad1.left_trigger > 0.2 || gamepad2.left_trigger > 0.2){
                muncher.setPower(-1);
                muncher2.setPower(-1);
            } else {
                muncher.setPower(0);
                muncher2.setPower(0);
            }

            if(pos.x < -100){
                //launchPower will create a decimal ranging from 0 to 1, which corresponds to the amount  the catapults pull back.
                launchPower = (hyp/119);
            }else{
                //launchPower will create a decimal ranging from 0 to 1, which corresponds to the amount  the catapults pull back.
                launchPower = ((hyp/119) * closeLaunchRatio) ;
            }

            otosAngleDegrees = pos.h * (180/3.14); // convert the otos angle to degrees
            hyp = Math.sqrt(Math.abs(((xAdjusted * xAdjusted)) + ((yAdjusted * yAdjusted)))) + hypOffset + driverHypOffset; // calculate the hypotenuse (distance to the goal)



            autoTurnSpeed = Math.abs(Math.abs((otosAngleDegrees - TargetA)) / 45 + .1);

            if(gamepad2.a){ //AUTO AIM
                TargetA = (90 - ((180/3.14) * (Math.abs(Math.atan((xAdjusted)/(yAdjusted)))))) - rotationOffset; // calculate target angle

                autoAiming = true;
                driveFieldRelative((-gamepad2.left_stick_y*driveSpeed), (gamepad2.left_stick_x*driveSpeed), yaw);
                autoAiming = true;

                if(otosAngleDegrees < TargetA - 3){
                    yaw = -.67 * autoTurnSpeed;
                }else if (otosAngleDegrees > TargetA + 3){
                    yaw = 0.67 * autoTurnSpeed;
                }else{
                    yaw = 0;
                }
            }else{
                autoAiming = false;
            }




            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Catapult Motor Pos", catapult.getCurrentPosition());
            telemetry.addData("otos X coordinate", pos.x);
            telemetry.addData("otos Y coordinate", pos.y);
            telemetry.addData("TargetA", TargetA);
            telemetry.addData("hyp", hyp);
            telemetry.addData("cPos",cPos);
            telemetry.addData("cState",cState);
            telemetry.addData("cBack",cBack);
            telemetry.addData("yaw (auto turn power)", yaw);
            telemetry.addData("Rotation Offset", rotationOffset);
            telemetry.addData("launch power", launchPower);
            telemetry.addData("driveSpeed", driveSpeed);
            telemetry.addData("auto turning speed", autoTurnSpeed);
            telemetry.addData("otosHeading", otosHeading);
            telemetry.addData("OTOS pos.h", pos.h);
            telemetry.addData("OTOS Heading degrees", pos.h * (180/3.14));
            
            telemetry.update();
        }
    }




    // This routine drives the robot field relative
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
        double maxSpeed = 1.0;  // make this slower for outreaches

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
        launchLeft();
        launchMiddle();
        launchRight();
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
                                release1.setPosition(launch1);
                                cColors[j] = "empty";
                                sleep(700);
                            }if (j == 1){
                                //fire catapult 2
                                 release2.setPosition(launch2);
                                cColors[j] = "empty";
                                sleep(700);
                            }if(j == 2){
                                //fire catapult 3
                                release3.setPosition(launch3);
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
                        release1.setPosition(launch1);
                    } else if (i == 1){
                        //fire catapult 2
                        release2.setPosition(launch2);
                    } else if(i == 2){
                        //fire catapult 3
                        release3.setPosition(launch3);
                    }
                }
            }
}
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
    }
}








