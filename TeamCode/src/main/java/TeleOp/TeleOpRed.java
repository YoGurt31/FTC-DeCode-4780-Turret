package TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import Systems.Robot;

@TeleOp(name = "RED", group = "TeleOp")
public class TeleOpRed extends LinearOpMode {

    // Robot Instance
    private final Robot robot = new Robot();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        robot.vision.limeLight.setPollRateHz(15);
        robot.vision.setPipeline(1);

        waitForStart();

        while (opModeIsActive()) {

        }
        robot.vision.limeLight.stop();
    }
}