package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
//Random Comment
public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
        .mass(12.791)
  .headingPIDFCoefficients(new PIDFCoefficients(1.5, 0, 0.1, 0))
           .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.1, 0.06722866772459533

                   , 0.0025071988797622014))
//            .centripetalScaling(0)

            ;


    public static PathConstraints pathConstraints = new PathConstraints(0.95, 100, 1, 1);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
           .xVelocity(159.00131561278187)
            .yVelocity(85.77954954567335)

            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
          .forwardTicksToInches(4.11177079048003E-4)
          .strafeTicksToInches(4.143462866743475E-4)
            .turnTicksToInches( 5.80794655059454E-4)

//            .forwardTicksToInches(0)
//            .strafeTicksToInches(0)
//            .turnTicksToInches(5.684901257258713E-4)


            .leftPodY(5.801181)
            .rightPodY(-5.781)
            .strafePodX(-5.25)
            .leftEncoder_HardwareMapName("frontLeft")
            .rightEncoder_HardwareMapName("backRight")
            .strafeEncoder_HardwareMapName("frontRight")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.REVERSE)
            //.forwardTicksToInches(multiplier)
            ;

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .threeWheelLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
