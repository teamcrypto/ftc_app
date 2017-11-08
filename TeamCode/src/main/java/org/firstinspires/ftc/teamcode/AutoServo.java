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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.internal.UserInput;

/**
 * Demonstrates empty OpMode
 */
@Autonomous(name = "Auto servo", group = "Concept")
//@Disabled
public class AutoServo extends OpMode {

  static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
  static final int    CYCLE_MS    =   50;     // period of each cycle
  static final double MAX_POS     =  0.4;     // Maximum rotational position
  static final double MIN_POS     =  0.0;     // Minimum rotational position
  int speed = 10;     // speed of the servo, determined by user input

  // Define class members
  Servo servo;
  Servo servo2;
  double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
  boolean rampUp = true;

  private UserInput userInput;

  private ElapsedTime runtime = new ElapsedTime();

  @Override
  public void init() {
    servo = hardwareMap.get(Servo.class, "left_hand");
    servo2 = hardwareMap.get(Servo.class, "right_hand");
    userInput = UserInput.getInstance();
    userInput.setup();

    userInput.setMinValue(0);
    userInput.setMaxValue(100);

    userInput.addVariable(speed, "servo speed");
    telemetry.addData("Status", "Initialized");
  }

  /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
  @Override
  public void init_loop() {
    telemetry.addData("servo speed ", speed);
    speed = userInput.getValue();
  }

  /*
   * This method will be called ONCE when start is pressed
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   */
  @Override
  public void start() {
    runtime.reset();
  }

  /*
   * This method will be called repeatedly in a loop
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   */
  @Override
  public void loop() {
    // slew the servo, according to the rampUp (direction) variable.
    if (rampUp) {
      // Keep stepping up until we hit the max value.
      position += INCREMENT ;
      if (position >= MAX_POS ) {
        position = MAX_POS;
        rampUp = !rampUp;   // Switch ramp direction
      }
    }
    else {
      // Keep stepping down until we hit the min value.
      position -= INCREMENT ;
      if (position <= MIN_POS ) {
        position = MIN_POS;
        rampUp = !rampUp;  // Switch ramp direction
      }
    }



    // Set the servo to the new position and pause;
    servo.setPosition(position);
    servo2.setPosition(1-position);
    // Display the current value
    telemetry.addData("Servo Position", "%5.2f", position);
    telemetry.update();
    try {
      Thread.sleep((long) (CYCLE_MS * (10.0 / (((double) speed)))));
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
    }
  }
}
