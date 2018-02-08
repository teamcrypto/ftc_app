package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by kabinet_1 on 26-1-2018.
 */
@TeleOp(name="IterativeBasic")
public class Mark extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftSideways;
    private DcMotor rightSideways;
    private DcMotor leftForward;
    private DcMotor rightForward;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftSideways = hardwareMap.get(DcMotor.class, "left-sideways");
        leftForward = hardwareMap.get(DcMotor.class, "left-forward");
        rightSideways = hardwareMap.get(DcMotor.class, "right-sideways");
        rightForward = hardwareMap.get(DcMotor.class, "right-forward");

        leftSideways.setDirection(DcMotor.Direction.FORWARD);
        rightSideways.setDirection(DcMotor.Direction.REVERSE);
        leftForward.setDirection(DcMotor.Direction.FORWARD);
        rightForward.setDirection(DcMotor.Direction.REVERSE);


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
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        // Send calculated power to wheels
        leftSideways.setPower(gamepad1.left_stick_x);
        rightSideways.setPower(gamepad1.left_stick_x);
        leftForward.setPower(gamepad1.left_stick_y);
        rightForward.setPower(gamepad1.left_stick_y);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());

        while (gamepad1.right_trigger != 0) {
            leftForward.setPower(gamepad1.right_trigger);
            rightForward.setPower(-gamepad1.right_trigger);

        }
        while (gamepad1.left_trigger != 0) {
            leftForward.setPower(-gamepad1.left_trigger);
            rightForward.setPower(gamepad1.left_trigger);

        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
