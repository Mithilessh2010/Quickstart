package org.firstinspires.ftc.teamcode.pedroPathing.procedures;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.tuning.autotune.DisplayName;
import com.pedropathing.tuning.autotune.Inputs;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.TuningOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.Function;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

public class Tests extends Procedure {
    enum Test {
        @DisplayName("Hold Test")
        HOLD,
        @DisplayName("Line Test")
        LINE,
        @DisplayName("Curve Test")
        CURVED,
        @DisplayName("Interpolation Test")
        INTERPOLATION_CURVED,
    }
    Function<HardwareMap, Follower> followerFunction;

    public Tests(Function<HardwareMap, Follower> followerFunction) {
        super("Tests", "A procedure for testing the Follower.");
        this.followerFunction = followerFunction;
    }

    @Override
    public void run() throws InterruptedException {
        boolean completed = false;

        Inputs inputs = inputs("Select", "Select");
        Inputs.Field<Test> selectedTest = inputs.e("Test", Test.class).withDefault(Test.LINE);
        awaitInputs(inputs);

        switch (selectedTest.get()) {
            case HOLD:
                completed = runOpMode(new TestsHold(followerFunction));
                break;
            case LINE:
                completed = runOpMode(new TestsLine(followerFunction));
                break;
            case CURVED:
                completed = runOpMode(new TestsCurve(followerFunction));
                break;
            case INTERPOLATION_CURVED:
                completed = runOpMode(new TestsInterpolation(followerFunction));
                break;
        }

        result("Completed", completed);
    }
}

class TestsHold extends TuningOpMode<Boolean> {
    Function<HardwareMap, Follower> followerFunction;

    public TestsHold(Function<HardwareMap, Follower> followerFunction) {
        super("Hold Test", "Tests the Follower's ability to hold a position.", true);
        this.followerFunction = followerFunction;
    }

    @Override
    public Boolean runTuningOpMode() throws InterruptedException {
        Follower follower = followerFunction.apply(hardwareMap);
        follower.setPose(Pose.zero());
        waitForStart();
        follower.hold(Pose.zero());
        while (opModeIsActive()) {
            follower.update();
        }
        return true;
    }
}

class TestsLine extends TuningOpMode<Boolean> {
    Function<HardwareMap, Follower> followerFunction;

    public TestsLine(Function<HardwareMap, Follower> followerFunction) {
        super("Line Test", "Tests the Follower's ability to follow a line.", true);
        this.followerFunction = followerFunction;
    }

    @Override
    public Boolean runTuningOpMode() throws InterruptedException {
        Follower follower = followerFunction.apply(hardwareMap);
        follower.setPose(Pose.zero());

        double distance = 48;
        boolean forward = true;

        Path path1 = line(Pose.zero(), new Pose(distance,0, 0)).constant(0);
        Path path2 = line(new Pose(distance,0, 0), Pose.zero()).constant(0);

        waitForStart();
        follower.follow(path1);

        while (opModeIsActive()) {
            follower.update();
            if (follower.atParametricEnd()) {
                if (forward) {
                    follower.follow(path2);
                } else {
                    follower.follow(path1);
                }
                forward = !forward;
            }
        }
        return true;
    }
}

class TestsCurve extends TuningOpMode<Boolean> {
    Function<HardwareMap, Follower> followerFunction;

    public TestsCurve(Function<HardwareMap, Follower> followerFunction) {
        super("Curve Test", "Tests the Follower's ability to follow a curve.", true);
        this.followerFunction = followerFunction;
    }

    @Override
    public Boolean runTuningOpMode() throws InterruptedException {
        Follower follower = followerFunction.apply(hardwareMap);
        follower.setPose(Pose.zero());

        double distance = 48;
        boolean forward = true;

        Path path1 = curve(Pose.zero(), new Pose(distance + 0,0), new Pose(distance,distance)).tangent();
        Path path2 = curve(new Pose(distance,distance), new Pose(distance,0), Pose.zero()).tangent();

        waitForStart();
        follower.follow(path1);

        while (opModeIsActive()) {
            follower.update();
            if (follower.atParametricEnd()) {
                if (forward) {
                    follower.follow(path2);
                } else {
                    follower.follow(path1);
                }
                forward = !forward;
            }
        }
        return true;
    }
}

class TestsInterpolation extends TuningOpMode<Boolean> {
    Function<HardwareMap, Follower> followerFunction;

    public TestsInterpolation(Function<HardwareMap, Follower> followerFunction) {
        super("Interpolation Curve Test", "Tests the Follower's ability to follow a curve with several interpolations.", true);
        this.followerFunction = followerFunction;
    }

    @Override
    public Boolean runTuningOpMode() throws InterruptedException {
        Follower follower = followerFunction.apply(hardwareMap);
        follower.setPose(Pose.zero());

        double distance = 48;
        boolean forward = true;

        Path path1 = curve(Pose.zero(), new Pose(distance + 0,0), new Pose(distance,distance)).heading((curve, t) -> Math.PI);
        Path path2 = curve(new Pose(distance,distance), new Pose(distance,0), Pose.zero()).heading(Interpolator.piecewise().until(0.5, Interpolator.tangent).until(1.0, Interpolator.constant(0)));

        waitForStart();
        follower.follow(path1);

        while (opModeIsActive()) {
            follower.update();
            if (follower.atParametricEnd()) {
                if (forward) {
                    follower.follow(path2);
                } else {
                    follower.follow(path1);
                }
                forward = !forward;
            }
        }
        return true;
    }
}
