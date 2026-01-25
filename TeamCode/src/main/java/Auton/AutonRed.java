package Auton;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import Systems.Robot;

@Autonomous(name = "AutoRed", group = "Auton")
public class AutonRed extends LinearOpMode {

    // Robot Instance
    private final Robot robot = new Robot();

    // Drive Constants
    private static final double drivePower = 0.50;
    private static final double rotatePower = 0.25;

    // Non-Blocking Sleep
    private void waitSec(double sec) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive() && timer.seconds() < sec) {
            idle();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        // Active If Using LimeLight
        robot.vision.limeLight.setPollRateHz(30);
        robot.vision.limeLight.pipelineSwitch(0);

        telemetry.addLine("Status: Initialized");
        telemetry.addLine("PreLoad Order: L - 2 Purple | R - 1 Green");
        telemetry.addLine("Scanning Motif...");
        telemetry.update();

        waitForStart();
        // Motif Detection
        robot.vision.updateMotif();

        telemetry.addLine("=== Motif Detection ===");
        telemetry.addData("Has Motif", robot.vision.hasMotif());
        telemetry.addData("Motif Tag ID", robot.vision.motifTagId);
        telemetry.addData("Motif Pattern", robot.vision.motifPattern);
        telemetry.update();

        sleep(100);

        // Motif Detected or BackUp
        String motif = robot.vision.hasMotif() ? robot.vision.motifPattern : "GPP";

        // Red PipeLine
        robot.vision.setPipeline(robot.vision.RED);

        // TODO: Sequence


        // Shutdown
        robot.scoringMechanisms.flyWheel1.setPower(0.0);
        robot.scoringMechanisms.flyWheel2.setPower(0.0);
        robot.scoringMechanisms.rollerIntake.setPower(0.0);
        robot.scoringMechanisms.turretRotation.setPower(0.0);
        robot.driveTrain.tankDrive(0.0, 0.0);
        robot.driveTrain.brake();
        try {
            robot.vision.limeLight.stop();
        } catch (Exception ignored) {
        }
    }
}