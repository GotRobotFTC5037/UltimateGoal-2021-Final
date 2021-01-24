package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

<<<<<<< Updated upstream
import org.firstinspires.ftc.robotcore.external.Telemetry;
=======
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
>>>>>>> Stashed changes
import org.firstinspires.ftc.teamcode.HardwarePerseverence;

@TeleOp(name = "Motor Test", group = "Pushbot")
public class PerseverenceMotorTest extends LinearOpMode {
    HardwarePerseverence robot = new HardwarePerseverence();
    private final ElapsedTime runtime = new ElapsedTime();
<<<<<<< Updated upstream
=======
    private BNO055IMU imu;
>>>>>>> Stashed changes

    public void waitMilis(double timeOutMs) {

        runtime.reset();
        while (runtime.milliseconds() < timeOutMs) ;
    }

    @Override

    public void runOpMode() {

        robot.init(hardwareMap);
        waitForStart();
<<<<<<< Updated upstream
        telemetry.addData("Ready?", "Press A");
        telemetry.update();
        while (!gamepad1.a) {
            waitMilis(50);
        }
        telemetry.addData("Rollers", "Press A for positive 25%");
        telemetry.update();
        while (!gamepad1.a) {
            waitMilis(50);
        }
        robot.rollers.setPower(.25);
        waitMilis(5000);
        robot.rollers.setPower(0);
        telemetry.addData("Tendrails", "Press A for positive 25%");
        telemetry.update();
        while (!gamepad1.a) {
            waitMilis(50);
        }
        robot.tendrails.setPower(.25);
        waitMilis(5000);
        robot.tendrails.setPower(0);
        telemetry.addData("LookingGlass", "Press A for positive 25%");
        telemetry.update();
        while (!gamepad1.a) {
            waitMilis(50);
        }
        robot.lookingGlass.setPower(.25);
        waitMilis(5000);
        robot.lookingGlass.setPower(0);
        telemetry.addData("Flywheel", "Press A for positive 25%");
        telemetry.update();
        while (!gamepad1.a) {
            waitMilis(50);
        }
        robot.flyWheel.setPower(.25);
        waitMilis(5000);
        robot.flyWheel.setPower(0);
    }
=======
        while (opModeIsActive()) {
//            double leftPower = robot.leftDrive.getPower();
//            if (gamepad1.dpad_up) {
//                robot.leftDrive.setPower(leftPower+0.05);
//            } else if (gamepad1.dpad_down) {
//                robot.leftDrive.setPower(leftPower-0.05);
//            }
//            telemetry.addData("Motor Power", leftPower);
//            sleep(15);
            telemetry.addData("Z", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.addData("Y", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle);
            telemetry.addData("X", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle);
            telemetry.update();
        }
//            telemetry.addData("Switch", robot.chokerSwitch.getVoltage());
//            telemetry.update();
//            waitMilis(500);
    }
//            telemetry.addData("Ready?", "Press A");
//            telemetry.update();
//            while (!gamepad1.a) {
//                waitMilis(50);
//            }
//            waitMilis(500);
//            robot.finalEscapeServo.setPosition(0);
//
//        telemetry.addData("Choker", "Press A for positive 25%");
//        telemetry.update();
//        while (!gamepad1.a) {
//            waitMilis(50);
//        }
//        //robot.rollers.setPower(.25);
//        robot.choker.setPosition(0);
//        waitMilis(5000);
//        robot.rollers.setPower(0);
//        telemetry.addData("Arm", "Press A for positive 25%");
//        telemetry.update();
//        while (!gamepad1.a) {
//            waitMilis(50);
//        }
//        //robot.tendrails.setPower(.25);
//        robot.arm.setPosition(0);
//        waitMilis(5000);
//        robot.tendrails.setPower(0);
//        telemetry.addData("Aim", "Press A for positive 25%");
//        telemetry.update();
//        while (!gamepad1.a) {
//            waitMilis(50);
//        }
//        //robot.lookingGlass.setPower(.25);
//        robot.aimbot.setPosition(0);
//        waitMilis(5000);
//        robot.lookingGlass.setPower(0);
//        telemetry.addData("Final Escapement", "Press A for positive 25%");
//        telemetry.update();
//        while (!gamepad1.a) {
//            waitMilis(50);
//        }
//        //robot.flyWheel.setPower(.25);
//        robot.finalEscapeServo.setPosition(0);
//        waitMilis(5000);
//        robot.flyWheel.setPower(0);
>>>>>>> Stashed changes
}

