package TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import Systems.Robot;

@TeleOp(name = "TestTank", group = "TeleOp")
//@Disabled
public class TeleOpBasic extends LinearOpMode {

    // Robot Instance
    private final Robot robot = new Robot();

    // Telemetry Variables
    long lastTelemetryUpdate = 0;
    final long telemetryInterval = 250;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        robot.vision.limeLight.setPollRateHz(15);

        telemetry.addLine("Status: Initialized. Ready to start.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            robot.driveTrain.pinPoint.update();

            // Drive
            double drive = -gamepad1.left_stick_y;
            double rotate = gamepad1.right_stick_x;
            robot.driveTrain.tankDrive(drive, rotate);

            // Brake
            if (Math.abs(drive) <= 1e-2 && Math.abs(rotate) <= 1e-2) {
                robot.driveTrain.brake();
            }

            // Vision
            LLResult result = robot.vision.limeLight.getLatestResult();
            boolean hasTag = false;
            double txDeg = 0.0;
            double tagArea = 0.0;

            if (result != null && result.isValid()) {
                java.util.List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                if (tags != null && !tags.isEmpty()) {
                    txDeg = result.getTx();
                    tagArea = result.getTa();
                    hasTag = Math.abs(txDeg) <= Robot.ScoringMechanisms.SwitchDeadband;
                }
            }

            // Turret
            double robotHeadingDeg = robot.driveTrain.pinPoint.getPosition().getHeading(AngleUnit.DEGREES);
            double turretDeg = robot.scoringMechanisms.getTurretDeg();
            double goalHeadingDeg = Robot.wrapDeg(robotHeadingDeg + turretDeg);

            if (hasTag) {
                robot.scoringMechanisms.autoAimTurret(robotHeadingDeg, goalHeadingDeg, txDeg, true);
            } else {
                robot.scoringMechanisms.turretRotation.setPower(0.0);
            }

            // Intake
            if (gamepad1.right_bumper) {
                robot.scoringMechanisms.rollerIntake.setPower(1.0);
            } else if (gamepad1.left_bumper) {
                robot.scoringMechanisms.rollerIntake.setPower(-1.0);
            } else {
                robot.scoringMechanisms.rollerIntake.setPower(0.0);
            }

            // FlyWheel
            final double FAR_RPS = 85.0;
            final double CLOSE_RPS = 70.0;
            double targetRps = FAR_RPS;
            double idleRPS = 35.0;

            if (hasTag && tagArea >= robot.vision.tagAreaThreshold) {
                targetRps = CLOSE_RPS;
            }

            if (gamepad1.right_trigger >= 0.05) {
                robot.scoringMechanisms.setFlywheelRPS(targetRps);
            } else {
                robot.scoringMechanisms.setFlywheelRPS(idleRPS);
            }

            // Artifact Release
            if (gamepad1.x) {
                robot.scoringMechanisms.artifactRelease.setPosition(0.0);
            } else {
                robot.scoringMechanisms.artifactRelease.setPosition(1.0);
            }

            // GearShift / Elevator
            if (gamepad1.share && gamepad1.options) {
                robot.driveTrain.gearShift.setPosition(0.55);
                robot.driveTrain.elevatorLeft.setPosition(1.0);
                robot.driveTrain.elevatorRight.setPosition(0.0);
            }

            // Telemetry
            long now = System.currentTimeMillis();
            if (now - lastTelemetryUpdate >= telemetryInterval) {
                telemetry.addLine("=== Drive + PinPoint ===");
                telemetry.addData("X Pos", "%5.2f", robot.driveTrain.pinPoint.getPosition().getX(DistanceUnit.INCH));
                telemetry.addData("Y Pos", "%5.2f", robot.driveTrain.pinPoint.getPosition().getY(DistanceUnit.INCH));
                telemetry.addData("Robot Heading", "%5.2f", robotHeadingDeg);
                telemetry.addLine();

                telemetry.addLine("=== Vision + Turret ===");
                telemetry.addData("Has Tag (in window)", hasTag);
                telemetry.addData("tx (deg)", "%5.2f", txDeg);
                telemetry.addData("ta", "%5.2f", tagArea);
                telemetry.addData("Turret Deg", "%5.2f", turretDeg);
                telemetry.addLine();

                telemetry.addLine("=== Shooter ===");
                telemetry.addData("Target RPS", "%5.1f", targetRps);
                telemetry.addData("Flywheel1 vel", "%7.1f", robot.scoringMechanisms.flyWheel1.getVelocity());
                telemetry.addData("Flywheel2 vel", "%7.1f", robot.scoringMechanisms.flyWheel2.getVelocity());

                telemetry.update();
                lastTelemetryUpdate = now;
            }

            idle();
        }
        robot.vision.limeLight.stop();
    }
}