
package org.firstinspires.ftc.teamcode.IDL;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name = "TunePID")
public class TunePID extends OpMode {
    private PIDController controller;
    public static double p=0, i=0, d=0;
    public static double f=0;
    public static int liftTarget=0;
    private DcMotorEx liftMotor;

    @Override
    public void init(){
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotorName");
    }

    public void loop(){
        controller.setPID(p,i,d);
        int liftPos = liftMotor.getCurrentPosition();
        double pid = controller.calculate(liftPos, liftTarget);
        double ff = f;
        double power = pid + ff;
        liftMotor.setPower(power);
        telemetry.addData("pos ", liftPos);
        telemetry.addData("liftTarget ", liftTarget);
        telemetry.addData("power ", power);
        telemetry.update();
    }
}