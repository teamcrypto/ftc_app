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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="omnibot 2")
//@Disabled
public class Omnibot2 extends OpMode
{

    OmniHardware bot = null;
    @Override
    public void init() {

        bot = new OmniHardware(this);
        //bot.initArm();
        bot.initDriveMotors();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry

        double xPower = gamepad1.left_stick_x;
        double yPower = gamepad1.left_stick_y;

        double rotation = Math.atan2(yPower, xPower);
        double lenght = Math.sqrt(Math.pow(xPower, 2) + Math.pow(yPower, 2));  // pythagoras

        telemetry.addData("rotation: ", rotation);
        telemetry.addData("lenght: ", lenght);

        rotation += (2* Math.PI) / 8;

        double newXPower = lenght * Math.cos(rotation);
        double newYPower = lenght * Math.sin(rotation);

        telemetry.addData("old x: ", xPower);
        telemetry.addData("new x: ", newXPower);
        telemetry.addData("old y: ", yPower);
        telemetry.addData("new y: ", newYPower);

        xPower = newXPower;
        yPower = newYPower;
        double turnpPower = gamepad1.right_stick_x;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        // Send calculated power to wheels
        bot.upDrive.setPower(xPower + turnpPower);
        bot.downDrive.setPower(xPower - turnpPower);
        bot.leftDrive.setPower(yPower + turnpPower);
        bot.rightDrive.setPower(yPower - turnpPower);

        if(gamepad1.x){
            //bot.open_hand();
        }

        if(gamepad1.b){
            //bot.close_hand();
        }

        if(gamepad1.y){
            bot.upDrive.setPower(1);
            telemetry.addData("up ", 1);
        }
        if(gamepad1.b){
            bot.rightDrive.setPower(1);
            telemetry.addData("right ", 1);
        }
        if(gamepad1.a) {
            bot.downDrive.setPower(1);
            telemetry.addData("down ", 1);
        }
        if(gamepad1.dpad_up){
            bot.leftDrive.setPower(1);
            telemetry.addData("left ", 1);
        }
        /*
        if(gamepad1.dpad_down){
            bot.leftDrive.setPower(-1);
            telemetry.addData("left backward", 1);
        }*/

        // Show the elapsed game time and wheel power.
        //telemetry.addData("Status", "Run Time: " + bot.period.toString());
        //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.addData("servo Pos", bot.hand_left.getPosition());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
