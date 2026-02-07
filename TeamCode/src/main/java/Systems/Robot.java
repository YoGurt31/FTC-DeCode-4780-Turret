package Systems;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

public class Robot {

    public static class DriveTrain {
        // Hardware Devices
        public DcMotorEx frontLeft, frontRight, backLeft, backRight;
        public GoBildaPinpointDriver pinPoint;
        public Servo gearShift, elevatorLeft, elevatorRight;

        public void init(HardwareMap hardwareMap) {

            // Initialize DcMotors - Name in " " should match Driver Station Configuration
            frontLeft = hardwareMap.get(DcMotorEx.class, "fL");
            frontRight = hardwareMap.get(DcMotorEx.class, "fR");
            backLeft = hardwareMap.get(DcMotorEx.class, "bL");
            backRight = hardwareMap.get(DcMotorEx.class, "bR");

            // Set Motor Directions - Positive Power should Drives Forward
            frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
            backLeft.setDirection(DcMotorEx.Direction.FORWARD);
            frontRight.setDirection(DcMotorEx.Direction.REVERSE);
            backRight.setDirection(DcMotorEx.Direction.REVERSE);

            // Brake when Power = 0 (Helps Negate Momentum)
            frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

            // Stops Motors and Resets Encoders - Motors will NOT Run unless Encoder Mode is Defined
            frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            // Encoder Mode Definition - Run With or Without Encoders
            frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

            // PinPoint Localizer
            pinPoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            pinPoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
            pinPoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pinPoint.setOffsets(-176, -66, DistanceUnit.MM);

            // GearShift SetUp
            gearShift = hardwareMap.get(Servo.class, "gS");
            elevatorLeft = hardwareMap.get(Servo.class, "eL");
            elevatorRight = hardwareMap.get(Servo.class, "eR");
            gearShift.setDirection(Servo.Direction.FORWARD);
            elevatorLeft.setDirection(Servo.Direction.FORWARD);
            elevatorRight.setDirection(Servo.Direction.FORWARD);
            gearShift.setPosition(0.5);
            elevatorLeft.setPosition(0.5);
            elevatorRight.setPosition(0.5);
        }

        public void PinPointReset() {
            if (pinPoint != null) {
                pinPoint.resetPosAndIMU();
                pinPoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
            }
        }

        public void tankDrive(double Drive, double Rotate) {
            double leftPower = Drive + Rotate;
            double rightPower = Drive - Rotate;

            // Prevents Motors from Exceeding 100% Power
            double maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));

            if (maxPower > 1.0) {
                leftPower /= maxPower;
                rightPower /= maxPower;
            }

            frontLeft.setPower(leftPower);
            frontRight.setPower(rightPower);
            backLeft.setPower(leftPower);
            backRight.setPower(rightPower);
        }

        public void brake() {
            frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }

        // Motion Functions
        public void driveDistance(LinearOpMode opMode, double distanceInches, double power) {
            if (opMode == null) return;
            if (pinPoint == null) return;

            double dir = Math.signum(distanceInches);
            double drivePower = Math.abs(power) * dir;

            pinPoint.update();
            Pose2D start = pinPoint.getPosition();
            double startX = start.getX(DistanceUnit.INCH);
            double startY = start.getY(DistanceUnit.INCH);
            double targetHeadingDeg = start.getHeading(AngleUnit.DEGREES);

            final double headingKp = 0.025;
            final double maxCorrection = 0.50;
            final double threshold = 0.5;

            while (opMode.opModeIsActive()) {
                pinPoint.update();
                Pose2D cur = pinPoint.getPosition();

                double x = cur.getX(DistanceUnit.INCH);
                double y = cur.getY(DistanceUnit.INCH);
                double headingDeg = cur.getHeading(AngleUnit.DEGREES);

                double dx = x - startX;
                double dy = y - startY;
                double traveled = Math.hypot(dx, dy);
                double remaining = Math.abs(distanceInches) - traveled;

                double headingErr = wrapDeg(targetHeadingDeg - headingDeg);
                double correction = Range.clip(-headingErr * headingKp, -maxCorrection, maxCorrection);

                if (remaining <= threshold) break;

                tankDrive(drivePower, correction);
                opMode.idle();
            }

            tankDrive(0.0, 0.0);
        }


        public void turnTo(LinearOpMode opMode, double targetHeadingDeg) {
            if (opMode == null) return;
            if (pinPoint == null) return;

            while (targetHeadingDeg > 180.0) targetHeadingDeg -= 360.0;
            while (targetHeadingDeg <= -180.0) targetHeadingDeg += 360.0;

            final double headingKp = 0.030;
            final double maxRotatePower = Math.abs(0.8);
            final double minRotatePower = 0.08;
            final double threshold = 1.0;
            final long settleMs = 200;
            long settleStart = -1;

            while (opMode.opModeIsActive()) {
                pinPoint.update();
                Pose2D cur = pinPoint.getPosition();
                double headingDeg = cur.getHeading(AngleUnit.DEGREES);

                double err = wrapDeg(targetHeadingDeg - headingDeg);

                if (Math.abs(err) <= threshold) {
                    if (settleStart < 0) settleStart = System.currentTimeMillis();
                    if (System.currentTimeMillis() - settleStart >= settleMs) {
                        break;
                    }
                } else {
                    settleStart = -1;
                }

                double rotate = Range.clip(-err * headingKp, -maxRotatePower, maxRotatePower);

                double absErr = Math.abs(err);
                double scaledMax = maxRotatePower;
                if (absErr < 20.0) scaledMax = Math.min(scaledMax, 0.25);
                if (absErr < 8.0) scaledMax = Math.min(scaledMax, 0.15);
                rotate = Range.clip(rotate, -scaledMax, scaledMax);

                if (absErr > 3.0 && Math.abs(rotate) < minRotatePower) {
                    rotate = minRotatePower * Math.signum(rotate);
                }

                if (absErr <= threshold) {
                    rotate = 0.0;
                }

                tankDrive(0.0, rotate);
                opMode.idle();
            }

            tankDrive(0.0, 0.0);
        }
    }

    public static double wrapDeg(double deg) {
        while (deg > 180.0) deg -= 360.0;
        while (deg <= -180.0) deg += 360.0;
        return deg;
    }

    public static class ScoringMechanisms {
        // Hardware Devices
        public DcMotorEx rollerIntake, turretRotation, flyWheel1, flyWheel2;
        public Servo artifactRelease;

        // FlyWheel Variables
        public static final double FlyWheelTicksPerRev = 28.0;
        public static final double F = 20.0;
        public static final double P = 2.5 * F;
        public static final double I = 0.0;
        public static final double D = 0.0;

        public void init(HardwareMap hardwareMap) {

            rollerIntake = hardwareMap.get(DcMotorEx.class, "rI");
            rollerIntake.setDirection(DcMotorEx.Direction.FORWARD);
            rollerIntake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            rollerIntake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            rollerIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            turretRotation = hardwareMap.get(DcMotorEx.class, "tR");
            turretRotation.setDirection(DcMotorEx.Direction.FORWARD);
            turretRotation.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            turretRotation.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            turretRotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            flyWheel1 = hardwareMap.get(DcMotorEx.class, "fW1");
            flyWheel1.setDirection(DcMotorEx.Direction.FORWARD);
            flyWheel1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            flyWheel1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            flyWheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            flyWheel1.setVelocityPIDFCoefficients(P, I, D, F);

            flyWheel2 = hardwareMap.get(DcMotorEx.class, "fW2");
            flyWheel2.setDirection(DcMotorEx.Direction.REVERSE);
            flyWheel2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            flyWheel2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            flyWheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            flyWheel2.setVelocityPIDFCoefficients(P, I, D, F);

            artifactRelease = hardwareMap.get(Servo.class, "aR");

            turretZeroOffsetTicks = 0.0;
            aimMode = TurretAimMode.Quick;
            lastTargetSeenTimeS = 0.0;
            turretAimTimer.reset();
        }

        public void setFlywheelRPS(double rps) {
            if (flyWheel1 == null || flyWheel2 == null) return;
            double tps = rps * FlyWheelTicksPerRev;
            flyWheel1.setVelocity(tps);
            flyWheel2.setVelocity(tps);
        }

        public void stopFlywheel() {
            if (flyWheel1 == null || flyWheel2 == null) return;
            flyWheel1.setPower(0.0);
            flyWheel2.setPower(0.0);
        }

        // Turret Variables
        private double turretZeroOffsetTicks = 0.0;
        private final ElapsedTime turretAimTimer = new ElapsedTime();

        private enum TurretAimMode {Quick, Precise}

        private TurretAimMode aimMode = TurretAimMode.Quick;
        private double lastTargetSeenTimeS = 0.0;

        public static final double TurretTicksPerRev = 145.1;
        public static final double TurretDriver = 24.0;
        public static final double TurretDriven = 100.0;
        public static final double TurretOutputToTurretRatio = TurretDriver / TurretDriven;
        public static final double TurretTicksPerTurretRev = TurretTicksPerRev / TurretOutputToTurretRatio;
        public static final double TurretDegPerTick = 360.0 / TurretTicksPerTurretRev;

        public static final double TurretMinDeg = -180.0;
        public static final double TurretMaxDeg = 180.0;

        public static final double LimitGuard = 0.5;

        public static final double QuickKp = 0.020;
        public static final double QuickMaxPower = 1.0;
        public static final double QuickDeadband = 5.0;

        public static final double PreciseKp = 0.020;
        public static final double PreciseMaxPower = 0.35;
        public static final double PreciseDeadband = 1.0;

        public static final double SwitchDeadband = 15.0;
        public static final double LostTargetTimeout = 0.20;

        public void zeroTurretHere() {
            if (turretRotation == null) return;
            turretZeroOffsetTicks = turretRotation.getCurrentPosition();
        }

        public double getTurretDeg() {
            if (turretRotation == null) return 0.0;
            double ticks = turretRotation.getCurrentPosition() - turretZeroOffsetTicks;
            return ticks * TurretDegPerTick;
        }

        private static double clamp(double v, double lo, double hi) {
            return Math.max(lo, Math.min(hi, v));
        }

        private boolean atMinLimit() {
            return getTurretDeg() <= (TurretMinDeg + LimitGuard);
        }

        private boolean atMaxLimit() {
            return getTurretDeg() >= (TurretMaxDeg - LimitGuard);
        }

        private double applyTurretLimitsToPower(double requestedPower) {
            if (requestedPower < 0 && atMinLimit()) return 0.0;
            if (requestedPower > 0 && atMaxLimit()) return 0.0;
            return requestedPower;
        }

        private double turretErrDeg(double desiredDeg, double currentDeg) {
            double errShort = wrapDeg(desiredDeg - currentDeg);
            double errLong;

            if (errShort >= 0) {
                errLong = errShort - 360.0;
            } else {
                errLong = errShort + 360.0;
            }

            double targetShort = currentDeg + errShort;
            double targetLong = currentDeg + errLong;

            boolean shortOk = (targetShort >= TurretMinDeg) && (targetShort <= TurretMaxDeg);
            boolean longOk = (targetLong >= TurretMinDeg) && (targetLong <= TurretMaxDeg);

            if (shortOk && longOk) {
                return (Math.abs(errShort) <= Math.abs(errLong)) ? errShort : errLong;
            }
            if (shortOk) return errShort;
            if (longOk) return errLong;

            double clampedTarget = clamp(desiredDeg, TurretMinDeg, TurretMaxDeg);
            return clampedTarget - currentDeg;
        }

        private void setTurretPower(double pwr) {
            if (turretRotation == null) return;
            turretRotation.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            turretRotation.setPower(Range.clip(pwr, -1.0, 1.0));
        }

        private void stopTurret() {
            if (turretRotation == null) return;
            turretRotation.setPower(0.0);
        }

        public boolean autoAimTurret(double robotHeadingDeg, double goalHeadingDeg) {
            return autoAimTurret(robotHeadingDeg, goalHeadingDeg, false, 0.0, false);
        }

        public boolean autoAimTurret(double robotHeadingDeg, double goalHeadingDeg, double txDeg, boolean hasTarget) {
            return autoAimTurret(robotHeadingDeg, goalHeadingDeg, true, txDeg, hasTarget);
        }

        public boolean autoAimTurret(double robotHeadingDeg, double goalHeadingDeg, boolean useVisionFineTune, double txDeg, boolean hasTarget) {
            if (turretRotation == null) return false;

            double nowS = turretAimTimer.seconds();
            if (hasTarget) lastTargetSeenTimeS = nowS;

            double desiredTurretDeg = wrapDeg(goalHeadingDeg - robotHeadingDeg);
            double currentTurretDeg = getTurretDeg();
            double turretErrDeg = turretErrDeg(desiredTurretDeg, currentTurretDeg);

            if (aimMode == TurretAimMode.Quick) {
                if (useVisionFineTune && hasTarget && Math.abs(turretErrDeg) <= SwitchDeadband) {
                    aimMode = TurretAimMode.Precise;
                }
            } else {
                if (!useVisionFineTune) {
                    aimMode = TurretAimMode.Quick;
                } else {
                    boolean targetRecentlyLost = (!hasTarget) && ((nowS - lastTargetSeenTimeS) > LostTargetTimeout);
                    if (targetRecentlyLost) {
                        aimMode = TurretAimMode.Quick;
                    }
                }
            }

            if (aimMode == TurretAimMode.Precise && useVisionFineTune && hasTarget) {
                if (Math.abs(txDeg) <= PreciseDeadband) {
                    stopTurret();
                    return true;
                }
                double cmd = Range.clip(txDeg * PreciseKp, -PreciseMaxPower, PreciseMaxPower);
                cmd = applyTurretLimitsToPower(cmd);
                setTurretPower(cmd);
                return false;
            }

            if (Math.abs(turretErrDeg) <= QuickDeadband) {
                stopTurret();
                return false;
            }

            double cmd = Range.clip(turretErrDeg * QuickKp, -QuickMaxPower, QuickMaxPower);
            cmd = applyTurretLimitsToPower(cmd);
            setTurretPower(cmd);
            return false;
        }
    }

    public static class Vision {
        public Limelight3A limeLight;
        public int artifactPipeline = 0;

        private VisionPortal turretPortal;
        private AprilTagProcessor aprilTag;
        public static final int BLUE = 20;
        public static final int RED = 24;

        private int desiredTagId = -1;
        private AprilTagDetection desiredDetection = null;

        public void init(HardwareMap hardwareMap) {
            limeLight = hardwareMap.get(Limelight3A.class, "limelight");
            limeLight.pipelineSwitch(0);
            limeLight.start();

            WebcamName turretCam = hardwareMap.get(WebcamName.class, "TurretCam");
            aprilTag = new AprilTagProcessor.Builder().build();
            turretPortal = new VisionPortal.Builder()
                    .setCamera(turretCam)
                    .addProcessor(aprilTag)
                    .build();

            desiredDetection = null;
        }

        public void setPipeline(int pipelineIndex) {
            artifactPipeline = pipelineIndex;
            if (limeLight != null) {
                try {
                    limeLight.pipelineSwitch(artifactPipeline);
                } catch (Exception ignored) {
                }
            }
        }

        public void update() {
            desiredDetection = null;
            if (aprilTag == null) return;

            List<AprilTagDetection> detections = aprilTag.getDetections();
            if (detections == null) return;

            for (AprilTagDetection d : detections) {
                if (d != null && d.id == desiredTagId) {
                    desiredDetection = d;
                    break;
                }
            }
        }

        public void setTrackedTag(int tagId) {
            desiredTagId = tagId;
        }

        public int getTrackedTag() {
            return desiredTagId;
        }

        public boolean hasTrackedTag() {
            return desiredDetection != null;
        }

        public double getTrackedYawDeg() {
            return desiredDetection == null ? 0.0 : desiredDetection.ftcPose.yaw;
        }

        public double getTrackedRange() {
            return desiredDetection == null ? 0.0 : desiredDetection.ftcPose.range;
        }

        public AprilTagDetection getTrackedDetection() {
            return desiredDetection;
        }

        public void closeTurretCam() {
            if (turretPortal != null) {
                try {
                    turretPortal.close();
                } catch (Exception ignored) {
                }
            }
        }
    }

    // Created Instances of Subsystems
    public DriveTrain driveTrain = new DriveTrain();
    public ScoringMechanisms scoringMechanisms = new ScoringMechanisms();
    public Vision vision = new Vision();

    // Initialize Hardware
    public void init(HardwareMap hwMap) {
        driveTrain.init(hwMap);
        scoringMechanisms.init(hwMap);
        vision.init(hwMap);
    }
}