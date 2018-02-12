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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.internal.UserInput;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
  This class defines all the hardware and functions needed for the competition.

  Motor channel:  Left  drive motor:        "left_drive"
  Motor channel:  Right drive motor:        "right_drive"
  Motor channel:  Top drive motor:          "up_drive"
  Motor channel:  Bottom drive motor:       "down_drive"
  Motor channel:  Manipulator drive motor:  "arm"
  Servo channel:  Servo to open left claw:  "left_hand"
  Servo channel:  Servo to open right claw: "right_hand"
 */

/**
 * TODO
 * - moveWithEncoder(Motor, distance)
 * - driveUp(distance) can also be used to drive down
 * using encoders
 * - driveRight(distance) can also be used to drive left
 *
 *
 */

public class OmniHardware
{
    /* Public OpMode members. */
    private DcMotor upDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor leftDrive = null;
    private DcMotor downDrive = null;
    private DcMotor arm = null;
    private Servo hand_left = null;
    private Servo hand_right = null;

    // get user input
    private UserInput userInput;

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
    private ArrayList<DcMotor> motors;  // motors ready to move

    /* Constructor */
    public OmniHardware(OpMode opMode){
        init(opMode.hardwareMap, opMode.telemetry);
    }
    public OmniHardware(LinearOpMode lOpMode) {
        opMode = lOpMode;
        init(lOpMode.hardwareMap, lOpMode.telemetry);
    }

    public void initArm(){
        // servo's voor de hand
        hand_left = hwMap.get(Servo.class, "left_hand");
        hand_right = hwMap.get(Servo.class, "right_hand");

        arm = hwMap.get(DcMotor.class, "arm");

        // servo's have been initiated
        isServoInit = true;

        open_hand();

    }

    public void startDrive(){
        boolean isBusy = false;
        do {
            for (DcMotor motor :
                    motors) {
                telemetry.addData("motor position", motor.getCurrentPosition());
                telemetry.addData("target position", motor.getTargetPosition());
                isBusy = isBusy || motor.isBusy();
            }
        }while(opMode.opModeIsActive() && isBusy);
        telemetry.update();

        for (DcMotor motor :
                motors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void encoderDrive(double power, int ticks, ArrayList<DcMotor> _motors){
        for (DcMotor motor:
             _motors) {
            int newPos = motor.getCurrentPosition() + ticks;
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setTargetPosition(newPos);
            motor.setPower(power);
        }
        motors.addAll(_motors);

    }

    public void driveForward(double power, int ticks){
        ArrayList<DcMotor> motors = new ArrayList<>(2);
        motors.add(leftDrive);
        motors.add(rightDrive);
        encoderDrive(power, ticks, motors);
        startDrive();
    }

    public void testEncoders(){
        telemetry.addData("Status:", "resetting encoders");
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("Status:", "setting mode");
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status:", "Initialized");
        telemetry.update();

        opMode.waitForStart();

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive.setPower(0.2);


        int counts = (int) (COUNTS_PER_MOTOR_REV * 2.0);
        telemetry.addData("counst", counts);
        telemetry.update();
        sleep(1000);
        leftDrive.setTargetPosition(leftDrive.getCurrentPosition()+counts);
        while(opMode.opModeIsActive() && leftDrive.isBusy()) {
            telemetry.addData("pos", leftDrive.getCurrentPosition());
            telemetry.update();
        }

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void initDriveMotors(){
        upDrive = hwMap.get(DcMotor.class, "up_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        leftDrive = hwMap.get(DcMotor.class, "left_drive");
        downDrive = hwMap.get(DcMotor.class, "down_drive");
        //arm = hwMap.get(DcMotor.class, "arm");

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

        // Set the direction of the motors so that if the rotate forward the robot  turns clockwise
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

    public void addVar(int var, String name, int range_min, int range_max){
        userInput.addVariable(var, name, range_min, range_max);

        boolean isUIVisible = false;
        if(!isUIVisible){
            userInput.showUI();
        }
    }

    public void open_hand(){
        setHandPosition(hand_open);
    }

    public void close_hand(){
        setHandPosition(hand_closed);
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

