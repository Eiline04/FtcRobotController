package org.firstinspires.ftc.teamcode.wrappers;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class LifterWrapper {
    private final DcMotorEx rightLifter;
    private final DcMotorEx leftLifter;
    private final RevTouchSensor button;

    public static double MAX_VEL = 650.0;
    public static double MAX_ACC = 200.0;
    public static double MAX_JERK = 0.0;

    public static double TICKS_PER_CM = 0;

    public LifterWrapper(DcMotorEx leftLifter, DcMotorEx rightLifter, RevTouchSensor button) {
        this.leftLifter = leftLifter;
        this.rightLifter = rightLifter;
        this.button = button;
    }

    public void setRunMode(DcMotor.RunMode runMode) {
        this.leftLifter.setMode(runMode);
    }

    public void setTargetPosition(int position) {
        leftLifter.setTargetPosition(position);
    }

    //bad idea to use this
    public void setTargetTolerance(int tolerance) {
        leftLifter.setTargetPositionTolerance(tolerance);
    }

    public void setPIDFCoeffs(DcMotor.RunMode runMode, PIDFCoefficients pidfCoeffs) {
        leftLifter.setPIDFCoefficients(runMode, pidfCoeffs);
    }

    public int getLifterPosition() {
        return leftLifter.getCurrentPosition();
    }

    public boolean isBusy() {
        return leftLifter.isBusy();
    }

    public double getLifterVelocity() {
        return leftLifter.getVelocity(AngleUnit.DEGREES);
    }

    public void setLifterPower(double power) {
        if (Math.abs(power) < 0.05) stop();
        this.rightLifter.setPower(power);
        this.leftLifter.setPower(power);
    }

    public void stop() {
        this.rightLifter.setPower(0.0);
        this.leftLifter.setPower(0.0);
    }

    public PIDFCoefficients getPIDFCoeffs(DcMotorEx.RunMode runMode) {
        return leftLifter.getPIDFCoefficients(runMode);
    }
}