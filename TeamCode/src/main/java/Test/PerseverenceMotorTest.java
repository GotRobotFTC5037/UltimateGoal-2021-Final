package Test;

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
    }
}
