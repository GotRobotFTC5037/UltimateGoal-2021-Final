package Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwarePerseverence;

//import com.spartronics4915.lib.T265Camera;
@TeleOp(name = "Motor Test", group = "Pushbot")
public class PerseverenceMotorTest extends LinearOpMode {
    HardwarePerseverence robot = new HardwarePerseverence();

    @Override

    public void runOpMode() {
        robot.init(hardwareMap);
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
            double redU = (double) robot.bottomColor.red() / (double) robot.bottomColor.alpha();
            double greenU = (double) robot.bottomColor.green() / (double) robot.bottomColor.alpha();
            double blueU = (double) robot.bottomColor.blue() / (double) robot.bottomColor.alpha();
            telemetry.addData("Red",  (double) robot.bottomColor.red());
            telemetry.addData("Blue",  (double) robot.bottomColor.blue());
            telemetry.addData("Green",  (double) robot.bottomColor.green());
            telemetry.update();
        }
    }
}
