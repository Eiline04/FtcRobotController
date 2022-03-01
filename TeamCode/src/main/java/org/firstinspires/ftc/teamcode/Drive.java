package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.wrappers.ClawWrapper;
import org.firstinspires.ftc.teamcode.wrappers.LifterWrapper;

@TeleOp(group = "Main")
public class Drive extends LinearOpMode {
    ControllerInput controller1;
    ControllerInput controller2;

    Hardware robot;
    LifterWrapper lifter;
    ClawWrapper claw;

    double drive, turn;

    private boolean newButtonState = false;
    private boolean oldButtonState = false;

    private double lifterOldPower = 0;
    private double lifterNewPower = 0;

    private boolean oldLeftBumper;
    private boolean oldRightBumper;
    private boolean newLeftBumper;
    private boolean newRightBumper;

    @Override
    public void runOpMode() throws InterruptedException {
        initSubsystems();
        controller1 = new ControllerInput(gamepad1);
        controller2 = new ControllerInput(gamepad2);

        waitForStart();

        while (opModeIsActive()) {
            controller1.update();
            controller2.update();

            handleDriving();
            handleLifter();
            handleSlider();
            handleClaw();

            telemetry.update();
        }
    }

    void handleSlider() {
        newLeftBumper = controller2.leftBumper();
        newRightBumper = controller2.rightBumper();
        if ((newLeftBumper != oldLeftBumper) || (newRightBumper != oldRightBumper)) {
            if (robot.slider.getCurrentPosition() < 40) {
                newLeftBumper = false;
            }
            if (!newLeftBumper && !newRightBumper)
                robot.slider.setPower(0);
            else if (newLeftBumper && !newRightBumper)
                robot.slider.setPower(-0.5);
            else if (!newLeftBumper)
                robot.slider.setPower(1);
        }
        oldLeftBumper = newLeftBumper;
        oldRightBumper = newRightBumper;
    }

    void handleLifter() {
        newButtonState = robot.button.isPressed();
        telemetry.addData("Pressed", newButtonState);
        if (newButtonState && !oldButtonState) {
            //kill the motors when the button is pressed
            lifter.stop();
            lifter.setRunMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            lifter.setRunMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
        oldButtonState = newButtonState;

        double left_trigger = controller1.left_trigger;
        double right_trigger = controller1.right_trigger;

        if (left_trigger > 0) {
            lifterNewPower = -left_trigger;
        } else if (right_trigger > 0) {
            lifterNewPower = right_trigger;
        } else {
            lifterNewPower = 0;
        }
        if (lifterNewPower != lifterOldPower) {
            if (newButtonState && lifterNewPower < 0) {
                //if button is pressed do not allow downwards movement
                lifterNewPower = 0;
            }
            lifter.setLifterPower(lifterNewPower);
        }
        lifterOldPower = lifterNewPower;
    }

    void handleClaw() {
        if (controller1.AOnce()) {
            claw.attach();
        }
        if (controller1.BOnce()) {
            claw.detach();
        }
    }

    void handleDriving() {
        drive = controller1.left_stick_y;
        turn = -controller1.right_stick_x;

        double leftPower = Range.clip(drive + turn, -0.6, 0.6);
        double rightPower = Range.clip(drive - turn, -0.6, 0.6);

        robot.leftWheel.setPower(leftPower);
        robot.rightWheel.setPower(rightPower);

        telemetry.addData("drive",drive);
        telemetry.addData("turn", turn);
    }

    void initSubsystems() {
        robot = new Hardware();
        robot.init(hardwareMap);
        lifter = new LifterWrapper(robot.leftLifter, robot.rightLifter, robot.button);
        claw = new ClawWrapper(robot.gripperFront, robot.gripperBack);
        claw.initial();
    }
}