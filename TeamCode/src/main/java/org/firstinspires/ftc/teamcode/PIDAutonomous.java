package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="test:PIDControl", group="PID")
@Disabled
public class PIDAutonomous extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();

    DcMotor leftDrive;
    DcMotor rightDrive;

    @Override
    public void runOpMode() {

        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        double leftPower;
        double rightPower;
        int speed = 50;
        int Kp = 1;
        int Ki = 1;
        int Kd = 1;
        int target = 0;
        int error;
        int PreviousError = 0;
        int integral = 0;
        int derivative;
        int IterationTime = 1;

        waitForStart();
        runtime.reset();

        while(opModeIsActive()){
        int heading = gyro.getHeading();
        error = target - heading;
        integral = integral + (IterationTime*error);
        derivative = (error - PreviousError);

        int output = (Kp*error)+(Ki*integral)+(Kd*derivative);
        PreviousError = error;

        if(output<0) {
            leftPower = Range.clip(((output * (speed*0.02))+speed) / 100, -1.0, 1.0);
            rightPower = Range.clip(1.0, -1.0, 1.0);
        } else{
            leftPower = Range.clip(1.0, -1.0, 1.0);
            rightPower = Range.clip(((output * (speed*0.02))+speed) / 100, -1.0, 1.0);
        }
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

        }

    }

}
