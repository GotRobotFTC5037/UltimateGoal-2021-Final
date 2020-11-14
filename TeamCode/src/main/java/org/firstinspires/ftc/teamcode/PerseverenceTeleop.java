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

//import com.arcrobotics.ftclib.geometry.Pose2d;
//import com.arcrobotics.ftclib.geometry.Rotation2d;
//import com.arcrobotics.ftclib.geometry.Transform2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.spartronics4915.lib.T265Camera;


/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Tele", group="Pushbot")

public class PerseverenceTeleop extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    /* Declare OpMode members. */
    HardwarePerseverence robot           = new HardwarePerseverence();   // Use a Pushbot's hardware
//    Transform2d cameraToRobot = new Transform2d();
    public void waitMilis(double timeOutMs) {

    runtime.reset();
    while (runtime.milliseconds() < timeOutMs) ;
    }


    @Override
    public void runOpMode() {
        double leftFront;
        double rightFront;
        double leftBack;
        double rightBack;
        double drive;
        double turn;
        double max;
        double r;
        double robotAngle;
        double driveSpeed = 1;
        // Increase this value to trust encoder odometry less when fusing encoder measurements with VSLAM
        double encoderMeasurementCovariance = 0.8;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
      //  Pose2d startingPose = new Pose2d(1, 1, new Rotation2d());
      //  T265Camera slamra = new T265Camera(cameraToRobot, encoderMeasurementCovariance);
//
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Pushbot:", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
          //  slamra.getLastReceivedCameraUpdate();
            double armCurr = robot.arm.getCurrentPosition();
            double forkCurr = robot.fork.getPosition();
            double rightX = gamepad1.right_stick_x;
            r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;
            robot.leftDrive.setPower(v1 * driveSpeed);
            robot.rightDrive.setPower(v2 * driveSpeed);
            robot.leftBackDrive.setPower(v3 * driveSpeed);
            robot.rightBackDrive.setPower(v4 * driveSpeed);

            // Forks
            if (gamepad2.b) {
                robot.fork.setPosition(forkCurr+.1);
                waitMilis(20);
            } else if (gamepad2.a) {
                robot.fork.setPosition(forkCurr-1);
                waitMilis(20);
            }
            // Arm
            robot.arm.setPower(gamepad2.left_stick_y);




            // Send telemetry message to signify robot running;
            telemetry.addLine("Front Power");
            telemetry.addData("Left Power", robot.leftDrive.getPower());
            telemetry.addData("Right Power", robot.rightDrive.getPower());
            telemetry.addLine("Back Power");
            telemetry.addData("Left Power", robot.leftBackDrive.getPower());
            telemetry.addData("Right Power", robot.rightBackDrive.getPower());;
            telemetry.addLine("Servo Values");
            telemetry.addData("Grip", robot.fork.getPosition());
      //      telemetry.addData("Arm", robot.arm.getCurrentPosition());
            telemetry.update();
        }
    }
}
