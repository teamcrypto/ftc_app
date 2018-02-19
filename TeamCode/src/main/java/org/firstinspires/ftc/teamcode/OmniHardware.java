/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.VuMarkTargetResult;

import org.firstinspires.ftc.robotcontroller.internal.UserInput;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import java.util.ArrayList;

/*
  This class defines all the hardware and functions needed for the competition.
  This is NOT an opmode

  The robot this class is used for is a omnibot. This means it can move in all directions without rotation.

  Motor channel:  Left  drive motor:        "left_drive"
  Motor channel:  Right drive motor:        "right_drive"
  Motor channel:  Top drive motor:          "up_drive"
  Motor channel:  Bottom drive motor:       "down_drive"
  Motor channel:  Arm drive motor:          "arm"
  Servo channel:  Servo to open left claw:  "left_hand"
  Servo channel:  Servo to open right claw: "right_hand"
 */

public class OmniHardware
{
    /* Public OpMode members. */
    DcMotor upDrive = null;
    DcMotor rightDrive = null;
    DcMotor leftDrive = null;
    DcMotor downDrive = null;
    DcMotor arm = null;
    Servo hand_left = null;
    Servo hand_right = null;

    // get user input
    private UserInput userInput;

    // servo positions
    double right_hand_open = 0.6;
    double right_hand_closed = 0.25;
    double left_hand_closed = 0.4;
    double left_hand_open = 0.6;

    double arm_up = 2800;
    double arm_down = 0;



    private double hand_open = 0.0;

    private double hand_closed = 0.25;


    private double afwijking = 0.0;

    // are the servo's and drive motors initiated ?
    private boolean isServoInit = false;
    private boolean isDriveInit = false;

    // variables for using encoders
    private static final double     COUNTS_PER_MOTOR_REV    = 32176/30 *1.04;  // the example variables didn't works so well for us


    /* local OpMode members. */
    private HardwareMap hwMap           =   null;
    Telemetry telemetry         =   null;
    private LinearOpMode opMode         =   null;
    public ElapsedTime period  = new ElapsedTime();
    private ArrayList<DcMotor> motors = new ArrayList<>(0);  // motors ready to move

    // variables needed for VuMark
    private VuforiaLocalizer vuforia;
    private VuforiaTrackables relicTrackables;
    private VuforiaTrackable relicTemplate;
    private RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;


    /* Constructor */
    public OmniHardware(OpMode opMode){ init(opMode.hardwareMap, opMode.telemetry); }
    public OmniHardware(LinearOpMode lOpMode) {
        opMode = lOpMode;
        init(lOpMode.hardwareMap, lOpMode.telemetry);
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, Telemetry _telemetry) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        userInput = UserInput.getInstance();
        userInput.setup();

        telemetry = _telemetry;

        initDriveMotors();
        initArm();
    }

    public void initDriveMotors(){
        upDrive = hwMap.get(DcMotor.class, "up_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        leftDrive = hwMap.get(DcMotor.class, "left_drive");
        downDrive = hwMap.get(DcMotor.class, "down_drive");

        // reset the encoders
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        downDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        upDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // turn on the encoders
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        downDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        upDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set the direction of the motors so that if the motors rotate forward the robot turns clockwise
        downDrive.setDirection(DcMotor.Direction.FORWARD);
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        upDrive.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        downDrive.setPower(0);
        upDrive.setPower(0);

        // drive motors have been initiated
        isDriveInit = true;
    }

    public void initArm(){
        // servo's voor de hand
        hand_left = hwMap.get(Servo.class, "left_hand");
        hand_right = hwMap.get(Servo.class, "right_hand");

        arm = hwMap.get(DcMotor.class, "arm");

        // reset the encoders
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // turn on the encoders
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm.setPower(0);

        // servo's have been initiated
        isServoInit = true;
    }

    public RelicRecoveryVuMark recognizeTarget(){
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "Ac8kPcD/////AAAAmbjJEPHIc0ROkLMyrtU5YfYeF2HqcJNbo3TFfD0uG5fvI6mxgsqq9FA+4/BRd9qTGEgQ7Kae0UvNHb+2xcrpF7wLweLO17Be04HSBWONX/6tAZ9Uzjzx/Av/BWIYo4/0/c+BfIdbKmUGm9jEXJgcsJj7wwkmTGYa+gSiQZCGr3k9/MWdX3Y/jM0PyxFUgHyh3YT0MBelWc2LaSlitQ68L3As/QGfnpvTTBdOs8YsKkNYNMB9ALmIfwKcOq2E5NnA2Cf/N1nX3efZEB3XWm0ql8WHmpjBD/ThvoqOn+qNasTYn1x9Hg2NMXwiJZqktJMSKRBdRfJqdyomV1iKXyVlVxpHFUYSTiiTvXkIIEYB4FC4";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        //waitForStart();
        relicTrackables.activate();

        RelicRecoveryVuMark result = RelicRecoveryVuMark.UNKNOWN;
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.milliseconds() < 2000) {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                result = vuMark;
                telemetry.addData("VuMark", "%s visible", vuMark);
            }
            else {
                telemetry.addData("VuMark", "not visible");
            }

            telemetry.update();
        }
        return result;
    }


    public void initVuMark(){
        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        //parameters.vuforiaLicenseKey = "Ac8kPcD/////AAAAmbjJEPHIc0ROkLMyrtU5YfYeF2HqcJNbo3TFfD0uG5fvI6mxgsqq9FA+4/BRd9qTGEgQ7Kae0UvNHb+2xcrpF7wLweLO17Be04HSBWONX/6tAZ9Uzjzx/Av/BWIYo4/0/c+BfIdbKmUGm9jEXJgcsJj7wwkmTGYa+gSiQZCGr3k9/MWdX3Y/jM0PyxFUgHyh3YT0MBelWc2LaSlitQ68L3As/QGfnpvTTBdOs8YsKkNYNMB9ALmIfwKcOq2E5NnA2Cf/N1nX3efZEB3XWm0ql8WHmpjBD/ThvoqOn+qNasTYn1x9Hg2NMXwiJZqktJMSKRBdRfJqdyomV1iKXyVlVxpHFUYSTiiTvXkIIEYB4FC4";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        //parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        //this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         **/
        /*relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary*/
    }

    public void printPatternId(){
        vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
            telemetry.addData("VuMark", "%s visible", vuMark);

        }
        else {
            telemetry.addData("VuMark", "not visible");
        }
        relicTrackables.activate();
        telemetry.update();
    }

    public RelicRecoveryVuMark getPatternId(){
        return vuMark;
    }

    public void startDrive(){
        if(checkDriveInit()) {
            boolean isBusy = false;
            do {
                for (DcMotor motor :
                        motors) {
                    telemetry.addData("motor position", motor.getCurrentPosition());
                    telemetry.addData("target position", motor.getTargetPosition());
                    isBusy = isBusy || motor.isBusy();
                }
            } while (opMode.opModeIsActive() && isBusy);
            telemetry.update();

            for (DcMotor motor :
                    motors) {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }

    public void encoderDrive(double power, int ticks, ArrayList<DcMotor> _motors) {
        if(checkDriveInit()) {
            for (DcMotor motor :
                    _motors) {
                int newPos = motor.getCurrentPosition() + ticks;
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setTargetPosition(newPos);
                motor.setPower(power);
            }
            motors.addAll(_motors);
        }
    }

    public void driveForward(double power, int ticks){
        if(checkDriveInit()) {
            ArrayList<DcMotor> _motors = new ArrayList<>(2);
            _motors.add(leftDrive);
            _motors.add(rightDrive);
            encoderDrive(power, ticks, _motors);
            startDrive();
        }
    }

    public void testEncoders(){
        driveForward(0.5, 1200);
    }



    public void addVar(int var, String name, int range_min, int range_max){
        userInput.addVariable(var, name, range_min, range_max);

        boolean isUIVisible = false;
        if(!isUIVisible){
            userInput.showUI();
        }
    }

    public void open_hand(){
        setLeftHandPosition(left_hand_open);
        setRightHandPosition(right_hand_open);
    }

    public void close_hand(){
        setLeftHandPosition(left_hand_closed);
        setRightHandPosition(right_hand_closed);
    }

    public void sleep(long ms){
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public int get(String name){
        return ((int) userInput.get(name));
    }

    public double getdouble(String name, double maxVal){
        return userInput.getDouble(name) * maxVal;
    }

    public double getdouble(String name){
        return userInput.getDouble(name);
    }

    boolean checkDriveInit(){
        if(!isDriveInit){
            telemetry.addLine("Drive motors have not been initialized");
        }
        return isDriveInit;
    }

    public void setHandPosition(double position){
        if(isServoInit) {
            hand_left.setPosition(position);
            hand_right.setPosition(1 - position + afwijking);
        }else telemetry.addLine("servo's have not been initialized");
    }

    public void setLeftHandPosition(double position){
        if(isServoInit) {
            hand_left.setPosition(position);
        }else telemetry.addLine("servo's have not been initialized");
    }
    public void setRightHandPosition(double position){
        if(isServoInit) {
            hand_right.setPosition(position);
        }else telemetry.addLine("servo's have not been initialized");
    }
 }

