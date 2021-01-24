package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwarePerseverence;

//import com.spartronics4915.lib.T265Camera;
@TeleOp(name = "Motor Test", group = "Pushbot")
public class PerseverenceMotorTest extends LinearOpMode {
    HardwarePerseverence robot = new HardwarePerseverence();
    private final ElapsedTime runtime = new ElapsedTime();
    public void waitMilis(double timeOutMs) {

        runtime.reset();
        while (runtime.milliseconds() < timeOutMs) ;
    }

    @Override

    public void runOpMode() {
        robot.init(hardwareMap);
        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        while (opModeIsActive()) {
<<<<<<< Updated upstream
//            double leftPower = robot.leftDrive.getPower();
//            if (gamepad1.dpad_up) {
//                robot.leftDrive.setPower(leftPower+0.05);
//            } else if (gamepad1.dpad_down) {
//                robot.leftDrive.setPower(leftPower-0.05);
//            }
//            telemetry.addData("Motor Power", leftPower);
//            sleep(15);
            robot.arm.setTargetPosition(5);
            robot.arm.setPower(.2);
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            waitMilis(4000);
            robot.arm.setTargetPosition(0);
            robot.arm.setPower(.2);
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
=======
            telemetry.addData("Switch", robot.chokerSwitch.getVoltage());
            telemetry.update();
            waitMilis(500);
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
}
