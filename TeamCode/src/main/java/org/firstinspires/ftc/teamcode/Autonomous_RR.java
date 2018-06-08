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


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="RR")
//\|@Disabled
public class Autonomous_RR extends LinearOpMode
{
    OmniHardware bot = null;

    @Override
    public void runOpMode(){
        bot = new OmniHardware(this);

        waitForStart();

        bot.close_hand();
        sleep(2000);
        bot.arm.setPower(0.3);
        bot.sleep(1500);
        bot.arm.setPower(0);
        bot.upDrive.setPower(1);
        bot.downDrive.setPower(1);
        bot.sleep(2000);
        bot.upDrive.setPower(0);
        bot.downDrive.setPower(0);
        bot.sleep(1000);
        double turnpPower = -0.5;
        bot.upDrive.setPower( turnpPower);
        bot.downDrive.setPower(turnpPower);
        bot.leftDrive.setPower(turnpPower);
        bot.rightDrive.setPower(turnpPower);
        bot.sleep(1000);
        bot.upDrive.setPower(0);
        bot.downDrive.setPower(0);
        bot.rightDrive.setPower(0);
        bot.leftDrive.setPower(0);

        bot.upDrive.setPower(1);
        bot.downDrive.setPower(1);
        bot.sleep(2000);
        bot.upDrive.setPower(0);
        bot.downDrive.setPower(0);
        bot.sleep(2000);
    }



}
