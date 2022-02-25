package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Red Left Remote", group = "Remote")
public class AutoRemoteRed extends AutoTeleBase {
    FiniteStateManager manager;

    float levelDistanceOffsetInch;

    float hyperSpaceHeadingIn = -98;
    float carouselHeadingIn = -26;
    float carouselHeadingOut = -145;

    // paths
    List<VectorF> listWaypoints = new ArrayList<>();

    long runningTime;

    int elementLevelIndex = 0;
    int longMoveLevel = 2;

    int turnTime = 3000;

    float carouselSpinFactor = -0.5f;
    float headingDegrees = -90;  // all Red start heading is -90, red is +90
    int autoDepositCount = 0;

    DetectElement detectElement;

    enum AutoLoopTransits {
        DetectElement,
        DepositFreight,
        MoveToShippingHub,
        MoveToCarousel,
        TurnCarousel,
        MoveToWarehouse,
        MoveToStorageZone,
        MoveArm,
        AutoDeposit,
        FinishState
    }

    @Override
    public void init() {
        super.init();
        waitState = 0;
        objectFinder.start();
        objectFinder.setMode(0);
        setTargets();
        autonomousTime = 30000; // 30 seconds

        robotStartHeadingInRad = (float) (headingDegrees * (Math.PI / 180)); // heading -14 degree, clockwise is negative, view from top

        setUpStateMachine();
        teleoptMode = false;

    }


    @Override
    public void init_loop() {
        super.init_loop();
        detectElement.lastElement = objectFinder.elementZone;
        telemetry.addData("Refresh time:", "%.2f seconds", (System.currentTimeMillis() - lastTime) * 0.001);
        telemetry.addData("Element: ", objectFinder.elementZone);
        lastTime = System.currentTimeMillis();
        telemetry.update();
    }

    @Override
    public void start() {
        super.start();
        intake.shovel();

        chassis.pidHeadingDeltaPower.reset();
        chassis.useOdometryTurn();
        chassis.setTurnPID();

        chassis.setPosition((float) robotStart.x, (float) robotStart.y, (float) 0, robotStartHeadingInRad);
        levelDistanceOffsetInch = 0;
        autoDepositCount = 0;
        trackPosition = true;

        lastTime = System.currentTimeMillis();
        startTime = System.currentTimeMillis();
    }

    @Override
    public void autoLoop() {
        // telemetry.addData("slider position", intake.motorArm.getCurrentPosition());
        if (trackPosition) {
            chassis.updateLocation();
        }
        if (manager.canAct()) {
            manager.act();
        }
        // printTelemetry();
    }

    protected void setWayPoints() {
        purePursuit.setWaypoints(listWaypoints);
        stanleyControl.setWaypoints(listWaypoints);
    }

    @Override
    public void stop() {
        super.stop();
    }

    public void setTargets() {
        headingDegrees = 90;
        carouselSpinFactor = -0.50f;
        hyperSpaceHeadingIn = 85;
        robotStart = competitionField.robotRedLeft;
        shippingHub = competitionField.redShippingHubLeftDeposit;
        shippingHub2 = competitionField.redShippingHubLeftDeposit2;
        warehouse = competitionField.redWarehouse;
        carousel = competitionField.redCarouselStop;
        sharedShippingHub = competitionField.sharedShippingHub;
        storageUnit = competitionField.redStorageUnit;
        hyperSpace = competitionField.redLeftB;
        intakePos = competitionField.redIntake;
        parkPos = competitionField.redPark;
        shippingHubAuto = competitionField.redAutoDeposit;
        hyperSpace2 = competitionField.redHyperSpaceOut;
        hyperSpace3 = competitionField.robotRedLeft;
        carouselHeadingIn = 25;
        carouselHeadingOut = 145;
        hyperSpaceMotor = chassis.motorLeftFrontWheel;
        longMoveLevel = 2;
        hyperSpaceDeltaPowerFactor = 1.0f;
        hyperSpaceBiasInches = 0;
    }

    public void setUpStateMachine() {
        manager = new FiniteStateManager();

        Begin1 begin1;

        DepositState depositState;
        TurnCarouselState turnCarouselState;
        StanleyMoveState moveState1;
        PurePursuitMoveState moveState2;
        StanleyMoveState moveState3;
        AutoDepositHub autoDepositHub;
        //FastDeposit autoDepositHub;
        MoveArm moveArm;
        FinishState finishState;

        // detect rings and move
        manager.entry(begin1 = new Begin1())
                .on(AutoLoopTransits.DetectElement,
                        (detectElement = new DetectElement(AutoLoopTransits.DetectElement,
                                AutoLoopTransits.DepositFreight)));

        // move to shipping hub and deposit freight
        detectElement.on(AutoLoopTransits.DepositFreight,
                (depositState = new DepositState(AutoLoopTransits.DepositFreight,
                        AutoLoopTransits.TurnCarousel)));

        // move to carousel
        depositState.on(AutoLoopTransits.TurnCarousel,
                (turnCarouselState = new TurnCarouselState(turnTime,
                        (carouselSpinner.spinSpeed * carouselSpinFactor),
                        AutoLoopTransits.TurnCarousel,
                        AutoLoopTransits.MoveToWarehouse)));

        // move to hyper space
        List<VectorF> warehouseWayPoints = new ArrayList<>();
//        Position midway
//                = new Position(DistanceUnit.INCH,
//                hyperSpace2.x - 24, hyperSpace2.y, 0, 0);
//        warehouseWayPoints.add(position2VectorF(midway));
//        Position midway2
//                = new Position(DistanceUnit.INCH,
//                hyperSpace2.x - 12, hyperSpace2.y, 0, 0);
//        warehouseWayPoints.add(position2VectorF(midway2));
        warehouseWayPoints.add(position2VectorF(hyperSpace2));
        turnCarouselState.on(AutoLoopTransits.MoveToWarehouse,
                (moveState3 = new StanleyMoveState(warehouseWayPoints,
                        AutoLoopTransits.MoveToWarehouse,
                        AutoLoopTransits.AutoDeposit, 4f,true)));

        // do more collections
        moveState3.on(AutoLoopTransits.AutoDeposit,
                autoDepositHub = new AutoDepositHub(AutoLoopTransits.AutoDeposit,
                        AutoLoopTransits.FinishState));
//        moveState3.on(AutoLoopTransits.AutoDeposit,
//                autoDepositHub = new FastDeposit(chassis, intake,
//                        AutoLoopTransits.AutoDeposit,
//                        AutoLoopTransits.FinishState));

        // end
        autoDepositHub.on(AutoLoopTransits.FinishState, finishState = new FinishState());
        finishState.on(AutoLoopTransits.FinishState, finishState);
        lastTime = System.currentTimeMillis();
    }


    //scan for marker
    class Begin1 extends FiniteState<AutoLoopTransits> {

        @Override
        public void onEnter() {

        }

        @Override
        public void act() {
            emit(AutoLoopTransits.DetectElement);

        }

        @Override
        public void onLeave() {

        }
    }

    class DetectElement extends FiniteState<AutoLoopTransits> {
        int count = 0;
        int lastElement = 2;
        long exitTS = 0;
        int maxCount = 0;
        int maxElement = 3;
        AutoLoopTransits currentState;
        AutoLoopTransits nextState;
        int internalState = 0;

        int[] levelHeights;

        public DetectElement(AutoLoopTransits currState,
                             AutoLoopTransits nextS) {
            super();
            currentState = currState;
            nextState = nextS;
            levelHeights = new int[3];
            levelHeights[0] = intake.botDepositPosition;
            levelHeights[1] = intake.midDepositPosition;
            levelHeights[2] = intake.topDepositPosition;
        }

        @Override
        public void onEnter() {
            objectFinder.setMode(0);
            count = 0;
            intake.motorArm.setTargetPosition(intake.botDepositPosition);
            intake.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intake.motorArm.setPower(intake.armPower);
            internalState = 0;
            maxElement = 3;
            intake.shovel();
            exitTS = System.currentTimeMillis();
        }

        @Override
        public void act() {
            if (lastElement == objectFinder.elementZone) {
                count++;
            } else {
                count = 0;
            }
            lastElement = objectFinder.elementZone;

            // track the largest count
            if (count >= maxCount) {
                maxCount = count;
                maxElement = objectFinder.elementZone;
            }

            // use the convergence count or maximum count if time out
            if (count > 5 || System.currentTimeMillis() - exitTS > 1500) {
                switch (maxElement) {
                    case 1:
                        elementLevelIndex = 0;
                        levelDistanceOffsetInch = 16;
                        break;
                    case 2:
                        elementLevelIndex = 1;
                        levelDistanceOffsetInch = 18;
                        break;
                    case 3:
                    default:
                        elementLevelIndex = 2;
                        levelDistanceOffsetInch = 18;
                        break;
                }
                //intake.intake(0.3);
                intake.motorArm.setTargetPosition(levelHeights[elementLevelIndex]);
                intake.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                intake.motorArm.setPower(intake.armPower);
                emit(nextState);
            }
        }

        @Override
        public void onLeave() {
            intake.motorArm.setTargetPosition(levelHeights[elementLevelIndex]);
            intake.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intake.motorArm.setPower(intake.armPower);
            objectFinder.pause();
        }
    }

    class FinishState extends FiniteState<AutoLoopTransits> {

        @Override
        public void onEnter() {
            System.out.println("Stanley to Carousel");
        }

        @Override
        public void act() {
            //intake.stop();
            chassis.moveLeftRight(0, 0);
            chassis.headingTolerance = 1.0f;
            telemetry.addData("done", "done");
        }

        @Override
        public void onLeave() {
        }
    }

    class StanleyMoveState extends FiniteState<AutoLoopTransits> {
        AutoLoopTransits currentState;
        AutoLoopTransits nextState;
        float distanceTolerance;
        List<VectorF> wayPoints;
        long exitTS;
        boolean moveArm = true;
        boolean backwardDriving;

        public StanleyMoveState(List<VectorF> wayPoint,
                                AutoLoopTransits currState,
                                AutoLoopTransits nextS,
                                float distTol, boolean bd) {
            super();
            currentState = currState;
            nextState = nextS;
            distanceTolerance = distTol;
            wayPoints = wayPoint;
            backwardDriving = bd;
        }

        @Override
        public void onEnter() {
            if (moveArm && intake.motorArm.getCurrentPosition() < minDrivingLevelHeight) {
                intake.motorArm.setTargetPosition(minDrivingLevelHeight);
                intake.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                intake.motorArm.setPower(intake.armPower);
            }
            stanleyControl.setWaypoints(wayPoints);
            stanleyControl.backwardDriving = backwardDriving;
            trackPosition = true;
        }

        @Override
        public void act() {
            if (stanleyControl.followWayPoints(distanceTolerance) == 0) {
                emit(nextState);
            }
        }

        @Override
        public void onLeave() {
            chassis.moveLeftRight(0, 0);
            stanleyControl.backwardDriving = false;
            exitTS = System.currentTimeMillis();
        }
    }

    class PurePursuitMoveState extends FiniteState<AutoLoopTransits> {
        AutoLoopTransits currentState;
        AutoLoopTransits nextState;
        float distanceTolerance;
        List<VectorF> listWayPoints;
        long exitTS;
        boolean moveArm = true;
        double powerBoost = 1.0;

        public PurePursuitMoveState(List<VectorF> wayPoints,
                                    AutoLoopTransits currState,
                                    AutoLoopTransits nextS, double pb, float distTol) {
            super();
            currentState = currState;
            nextState = nextS;
            distanceTolerance = distTol;
            listWayPoints = wayPoints;
            powerBoost = pb;
        }

        @Override
        public void onEnter() {
            if (moveArm && intake.motorArm.getCurrentPosition() < minDrivingLevelHeight) {
                intake.motorArm.setTargetPosition(minDrivingLevelHeight);
                intake.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                intake.motorArm.setPower(intake.armPower);
            }
            purePursuit.setWaypoints(listWayPoints);
            purePursuit.powerBoost = powerBoost;
            trackPosition = true;
        }

        @Override
        public void act() {
            if (purePursuit.followWayPoints(distanceTolerance) == 0) {
                emit(nextState);
            }
        }

        @Override
        public void onLeave() {
            chassis.moveLeftRight(0, 0);
            purePursuit.powerBoost = 1.0;
            exitTS = System.currentTimeMillis();
        }
    }

    class MoveArm extends FiniteState<AutoLoopTransits> {
        AutoLoopTransits currentState;
        AutoLoopTransits nextState;
        double armPower;
        int targetPosition;
        long enterTS;
        long exitTS;

        public MoveArm(int targetPos, double pow, AutoLoopTransits currState,
                       AutoLoopTransits nextS) {
            super();
            armPower = pow;
            targetPosition = targetPos;
            currentState = currState;
            nextState = nextS;

        }

        @Override
        public void onEnter() {
            enterTS = System.currentTimeMillis();
        }

        @Override
        public void act() {
            intake.motorArm.setTargetPosition(targetPosition);
            intake.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intake.motorArm.setPower(intake.armPower);
            if (intake.isTopLimitActive()
                    || (Math.abs(intake.motorArm.getCurrentPosition() - targetPosition) < 20)
                    || System.currentTimeMillis() - enterTS > 5000) {

                emit(nextState);
            }
        }

        @Override
        public void onLeave() {
            intake.hold();
            exitTS = System.currentTimeMillis();
        }
    }

    class TurnCarouselState extends FiniteState<AutoLoopTransits> {
        AutoLoopTransits currentState;
        AutoLoopTransits nextState;
        double spinningSpeed;
        long turnTime;
        long enterTS;
        long exitTS;
        long spintTS;
        int internalState = 0;

        public TurnCarouselState(long turnTS, double spinP,
                                 AutoLoopTransits currState,
                                 AutoLoopTransits nextS) {
            super();
            spinningSpeed = spinP;
            turnTime = turnTS;
            currentState = currState;
            nextState = nextS;
        }

        @Override
        public void onEnter() {
            //  move the slider to safe position to avoid jamming the carousel wheel
            // intake.intake(-0.3);
            intake.motorArm.setTargetPosition(minDrivingLevelHeight);
            intake.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intake.motorArm.setPower(intake.armPower);
            chassis.setCargoOffset(-3, -1, 0);
            List<VectorF> carouselWayPoints = new ArrayList<>();
            VectorF midPoint = new VectorF((float) robotStart.x, (float) carousel.y, (float) robotStart.z );
            carouselWayPoints.add(midPoint);
            carouselWayPoints.add(position2VectorF(carousel));
            purePursuit.setWaypoints(carouselWayPoints);
            enterTS = System.currentTimeMillis();
            spintTS = enterTS;
            internalState = 0;
        }

        @Override
        public void act() {
            switch (internalState) {
                case 0:
                    if (0 == purePursuit.followWayPoints(7f)
                            || System.currentTimeMillis() - enterTS> 4000){
                        internalState ++;
                        chassis.setCargoOffset(0, 0, 0);
                        carouselSpinner.spinBySpeed(spinningSpeed);
                        enterTS = System.currentTimeMillis();
                    }
                    break;
                case 1:
                    if (Math.abs(carouselSpinner.motorCarousel.getVelocity()) > Math.abs(spinningSpeed * 0.9)
                            || System.currentTimeMillis() - enterTS > turnTime) {
                        internalState++;
                        spintTS = System.currentTimeMillis();
                    }
                    break;
                case 2:
                    if (System.currentTimeMillis() - enterTS > turnTime
                            || System.currentTimeMillis() - spintTS > 500) {
                        enterTS = System.currentTimeMillis();
                        internalState++;
                    }
                    break;
                case 3:
                    if (0 == chassis.maintainHeadingAndMove(carouselHeadingOut, 0.3)
                            || System.currentTimeMillis() - enterTS > 1000) {
                        internalState++;
                    }
                    break;
                default:
                    emit(nextState);
            }
        }

        @Override
        public void onLeave() {
            carouselSpinner.stop();
            chassis.moveLeftRight(0, 0);
            intake.motorArm.setTargetPosition(minDrivingLevelHeight);
            intake.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intake.motorArm.setPower(intake.armPower);
            chassis.setCargoOffset(0, 0, 0);
            exitTS = System.currentTimeMillis();
        }
    }

    class DepositState extends FiniteState<AutoLoopTransits> {
        AutoLoopTransits currentState;
        AutoLoopTransits nextState;
        int internalState;
        long enterTS;
        long exitTS;
        int level;
        int[] levelHeights;

        public DepositState(AutoLoopTransits currState,
                            AutoLoopTransits nextS) {
            super();
            currentState = currState;
            nextState = nextS;
            levelHeights = new int[3];
            levelHeights[0] = intake.botDepositPosition;
            levelHeights[1] = intake.midDepositPosition;
            levelHeights[2] = intake.topDepositPosition;
            internalState = 0;
        }

        @Override
        public void onEnter() {
            enterTS = System.currentTimeMillis();
            level = Math.min(elementLevelIndex, levelHeights.length);
            intake.closeDepositHalf();
            chassis.setCargoOffset(0, 0, 0);
            internalState = 0;
        }

        @Override
        public void act() {
            switch (internalState) {
                case 0:
                    // Raise slider to correct level
                    intake.motorArm.setTargetPosition(levelHeights[level]);
                    intake.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    intake.motorArm.setPower(intake.armPower);
                    exitTS = System.currentTimeMillis();
                    List<VectorF> shippingHubWayPoints = new ArrayList<>();
                    if (longMoveLevel == level) {
                        shippingHubWayPoints.add(position2VectorF(hyperSpace));
                    } else {
                        shippingHubWayPoints.add(position2VectorF(shippingHub));
                    }
                    shippingHubWayPoints.add(position2VectorF(shippingHub2));
                    stanleyControl.setWaypoints(shippingHubWayPoints);
                    chassis.setCargoOffset(levelDistanceOffsetInch, 0, 0);
                    internalState++;
                    break;
                case 1:
                    if (stanleyControl.followWayPoints(9) == 0) {
                        intake.shovel();
                        chassis.moveLeftRight(0, 0);
                        chassis.setCargoOffset(0, 0, 0);
                        internalState++;
                    }
                    break;
                case 2:
                    // Wait for arm movements
                    if (intake.isTopLimitActive()
                            || (Math.abs(intake.motorArm.getCurrentPosition() - levelHeights[level]) < 35)
                            || System.currentTimeMillis() - exitTS > 3000) {
                        intake.motorArm.setTargetPosition(levelHeights[level] + 150);
                        intake.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        intake.motorArm.setPower(intake.armPower);
                        intake.openDeposit();
                        chassis.moveLeftRight(0, 0);
                        exitTS = System.currentTimeMillis();
                        // Open claw and spin in case
                        intake.intake(0.0);
                        internalState++;
                    }
                    break;
                case 3:
                    if (System.currentTimeMillis() - exitTS > 600) {
                        intake.openDepositHalf();
                        exitTS = System.currentTimeMillis();
                        internalState++;
                    }
                    break;
                case 4:
                    if (System.currentTimeMillis() - enterTS > 200) {
                        intake.shovel();
                        intake.motorArm.setTargetPosition(levelHeights[level]);
                        intake.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        intake.motorArm.setPower(intake.armPower);
                        internalState ++;
                    }
                    break;
//                case 4:
//                    // prepare to back up
//                    if (0 == chassis.maintainHeadingAndMove(carouselHeadingIn, -0.1)
//                            || System.currentTimeMillis() - exitTS > 500) {
//                        exitTS = System.currentTimeMillis();
//                        intake.shovel();
//                        internalState++;
//                    }
//                    break;
                default:
                    exitTS = System.currentTimeMillis();
                    emit(nextState);
            }
        }

        @Override
        public void onLeave() {
            chassis.setCargoOffset(0, 0, 0);
            exitTS = System.currentTimeMillis();
        }
    }

    class AutoDepositHub extends FiniteState<AutoLoopTransits> {
        AutoLoopTransits currentState;
        AutoLoopTransits nextState;
        long enterTS;
        long exitTS;
        int internalState;

        public AutoDepositHub(AutoLoopTransits currState,
                              AutoLoopTransits nextS) {
            super();
            currentState = currState;
            nextState = nextS;
            internalState = 0;
        }

        @Override
        public void onEnter() {
            enterTS = System.currentTimeMillis();
            chassis.moveLeftRight(0, 0);
            chassis.headingTolerance = 1;
            chassis.distanceTolerance = 2;
            intake.closeDepositHalf();
            autoDepositCount = 0;
            //chassis.useGyroTurn(); // use gyroscope
            chassis.useOdometryTurn();
            hyperSpaceHeadingIn = chassis.getCurrentHeading();
            chassis.setCargoOffset(0, 0, 0);
            internalState = 0;
        }

        @Override
        public void act() {
            switch (internalState) {
                case 0:
                    //intake.motorArm.setTargetPosition(minDrivingLevelHeight);
                    intake.motorArm.setTargetPosition(minDrivingLevelHeight);
                    intake.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    intake.motorArm.setPower(intake.armPower);
                    intake.intake(-0.3);
                    enterTS = System.currentTimeMillis();
                    if (autoDepositCount<1){
                        internalState+=2;
                    } else {
                        internalState++;
                    }
                    break;
                case 1:
                    // readjust heading after hyper space
                    if (chassis.maintainHeading(hyperSpaceHeadingIn) == 0
                            || System.currentTimeMillis() - enterTS > 500) {
                        chassis.moveLeftRight(0, 0);
                        internalState++;
                    }
                    break;
                case 2:
                    // enter hyper space
                    trackPosition = false;
                    intake.intake(0.8);
                    hyperSpaceLandMark = hyperSpaceMotor.getCurrentPosition() / chassis.leftOdometryCountPerInch;
                    internalState++;
                    enterTS = System.currentTimeMillis();
                    break;
                case 3: {
                    // move in with one wheel encoder
                    if (0 == chassis.moveByPIDwithOneEncoder(hyperSpaceMotor,
                            hyperSpaceLandMark + hyperSpaceDistance * (1 + 0.05 * autoDepositCount),
                            hyperSpaceDeltaPowerFactor)
                            || System.currentTimeMillis() - enterTS > 2000) {
                        //chassis.moveLeftRight(0, 0);
                        // raise the slider
                        intake.motorArm.setTargetPosition(0);
                        intake.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        intake.motorArm.setPower(intake.armPower);
                        enterTS = System.currentTimeMillis();
                        internalState++;
                    }

                    double delta = hyperSpaceDistance - hyperSpaceMotor.getCurrentPosition() / chassis.leftOdometryCountPerInch + hyperSpaceLandMark;
                    double ratio = Math.abs(delta / hyperSpaceDistance);
                    if (ratio < 0.5) {
                        intake.motorArm.setTargetPosition(0);
                        intake.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        intake.motorArm.setPower(intake.armPower);
                    }
                }
                break;
                case 4:
                    if (System.currentTimeMillis() - startTime < minCycleTimeInMs) {
                        internalState = 100;
                        enterTS = System.currentTimeMillis();
                    } else if (System.currentTimeMillis() - enterTS > 800) {
                        intake.motorArm.setTargetPosition(intake.midDepositPosition);
                        intake.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        intake.motorArm.setPower(intake.armPower);
                        intake.intake(-0.6);
                        enterTS = System.currentTimeMillis();
                        chassis.distanceTolerance = 6;
                        internalState++;
                    }
                    break;
                case 5:
                    // reverse back
                    if (0 == chassis.moveByPIDwithOneEncoder(hyperSpaceMotor,
                            hyperSpaceLandMark, hyperSpaceDeltaPowerFactor)
                             || System.currentTimeMillis() - enterTS > 2000
                    ) {
                        chassis.moveLeftRight(0, 0);
                        // turn off the hyper space
                        trackPosition = true;
                        chassis.getLandmarks();
                        chassis.getLandmarks();
                        chassis.updateLocation();
                        chassis.getLandmarks();
                        chassis.getLandmarks();
                        chassis.updateLocation();
//                        printTelemetry();
//                        telemetry.addData("hyper space distance:", "%.1f, %.1f",
//                                hyperSpaceLandMark, hyperSpaceMotor.getCurrentPosition() / chassis.leftOdometryCountPerInch);
                        enterTS = System.currentTimeMillis();
                        internalState++;
                    }
                    break;
                case 6: {
                    intake.intake(0);
                    // set deposit location
                    List<VectorF> shippingHubWayPoints = new ArrayList<>();
                    shippingHubWayPoints.add(position2VectorF(shippingHubAuto));
                    stanleyControl.setWaypoints(shippingHubWayPoints);
                    // set arm
                    intake.motorArm.setTargetPosition(intake.topDepositPosition);
                    intake.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    intake.motorArm.setPower(intake.armPower);
                    chassis.setCargoOffset(16, 0, 0);
                    //printTelemetry();
                    internalState++;
                }
                break;
                case 7:
                    // move to shipping hub
                    if (stanleyControl.followWayPoints(5+autoDepositCount) == 0) {
                        chassis.headingTolerance = 3;
                        intake.openDeposit();
                        intake.motorArm.setTargetPosition(intake.topDepositPosition + 150);
                        intake.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        intake.motorArm.setPower(intake.armPower);
                        intake.intake(0);
                        chassis.moveLeftRight(0, 0);
                        //printTelemetry();
                        autoDepositCount++;
                        internalState++;
                        enterTS = System.currentTimeMillis();
                    }
                    break;
                case 8:
                    //objectFinder.autoAim(chassis, 0.5);
                    if (System.currentTimeMillis() - enterTS > 800) {
                        if (autoDepositCount >= 2
                                || System.currentTimeMillis() - startTime < minCycleTimeInMs) {
                            List<VectorF> shippingHubWayPoints = new ArrayList<>();
                            shippingHubWayPoints.add(position2VectorF(intakePos));
                            purePursuit.setWaypoints(shippingHubWayPoints);
                            internalState += 2;
                        } else {
                            List<VectorF> shippingHubWayPoints = new ArrayList<>();
                            Position midway
                                    = new Position(DistanceUnit.INCH,
                                    hyperSpace2.x - 8, hyperSpace2.y, 0, 0);
                            shippingHubWayPoints.add(position2VectorF(midway));
                            shippingHubWayPoints.add(position2VectorF(hyperSpace2));
                            purePursuit.setWaypoints(shippingHubWayPoints);
                            internalState++;
                        }
                        intake.motorArm.setTargetPosition(intake.topDepositPosition + 150);
                        intake.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        intake.motorArm.setPower(intake.armPower);
                        //intake.openDepositHalf();
                        chassis.setCargoOffset(0, 0, 0);
                        enterTS = System.currentTimeMillis();
                    }
                    break;
                case 9:
                    if (System.currentTimeMillis() - enterTS > 600) {
                        intake.shovel();
                        intake.motorArm.setTargetPosition(minDrivingLevelHeight);
                        intake.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        intake.motorArm.setPower(intake.armPower*0.6);
                    }

                    // back to hyper space
                    if (purePursuit.followWayPoints(6) == 0
                            || System.currentTimeMillis() - enterTS > 4000) {
                        intake.shovel();
                        intake.intake(0.8);
                        //chassis.moveLeftRight(0, 0);
                        intake.motorArm.setTargetPosition(minDrivingLevelHeight);
                        intake.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        intake.motorArm.setPower(intake.armPower);
                        enterTS = System.currentTimeMillis();
                        internalState = 0;
                        //printTelemetry();
                    }
                    break;
                case 10:
                    // park
                    if (purePursuit.followWayPoints(6) == 0) {
                        intake.closeDepositHalf();
                        chassis.moveLeftRight(0, 0);
                        intake.motorArm.setTargetPosition(0);
                        intake.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        intake.motorArm.setPower(intake.armPower);
                        enterTS = System.currentTimeMillis();
                        internalState++;
                    }
                    break;
                case 11:
                    if (System.currentTimeMillis() - enterTS > 500) {
                        intake.intake(0.8);
                        internalState ++;
                    }
                    break;
                default:
                    emit(nextState);
            }
        }

        @Override
        public void onLeave() {
            chassis.setCargoOffset(0, 0, 0);
            intake.intake(0.8);
            exitTS = System.currentTimeMillis();
        }
    }
}

