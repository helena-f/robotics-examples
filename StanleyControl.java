package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

public class StanleyControl extends PurePursuit{

    List<OpenGLMatrix> motionVector;
    OpenGLMatrix targetMat;
    OpenGLMatrix nextMat;
    OpenGLMatrix lastMat;
    Iterator<OpenGLMatrix> iteratorMat;
    double pathLength =0;
    OpenGLMatrix destination;
    double stanleyGain = 0.05; // apply gain based on needed turning degrees per second
    double maxTurnAngle = 90; // limiting the turn angle
    double chargingAngleThreshold = 10;
    int chaosCount = 0;
    double headingSkew =0;
    double endHeadingSkew =0;
    double stanleySkew;
    double stanleyDistance;
    double distanceError;
    double totalSkew;
    Telemetry telemetry = null;
    boolean backwardDriving = false;

    public StanleyControl(HardwareChassis c){
        super(c);
        useFastPID();
    }

    public void setTelemetry(Telemetry t) {
        telemetry = t;
    }

    public void useFastPID() {
        //power PID
        pidForwardPower = new PIDControl();
        pidForwardPower.setKp(0.05); //0/03
        pidForwardPower.setKi(0.004);
        pidForwardPower.setKd(0);
        pidForwardPower.setMaxIntegralError(20);

        // turn PID
        pidDeltaPower.setKp(0.02); // 0.01
        pidDeltaPower.setKi(0.005); //0.005
        pidDeltaPower.setKd(0); // -0.0005
        pidDeltaPower.setMaxIntegralError(250);
    }

    public void useSlowPID(){
        //power PID
        pidForwardPower = new PIDControl();
        pidForwardPower.setKp(0.01);
        pidForwardPower.setKi(0.004);
        pidForwardPower.setKd(0);
        pidForwardPower.setMaxIntegralError(20);

        // turn PID
        pidDeltaPower.setKp(0.01); // 0.01
        pidDeltaPower.setKi(0.001); //0.005
        pidDeltaPower.setKd(0); // -0.0005
        pidDeltaPower.setMaxIntegralError(250);
    }

    @Override
    public double goToWayPoint(OpenGLMatrix wayPoint, OpenGLMatrix curP) {
        lastTime = System.currentTimeMillis();

        if (backwardDriving){
            curP = curP.rotated(AngleUnit.DEGREES, 180, 0, 0, 1);
        }

        // compute the path skew angle in degree
        OpenGLMatrix invCuPos = curP.inverted();
        endHeadingSkew = Orientation.getOrientation(invCuPos.multiplied(wayPoint), EXTRINSIC, XYZ, DEGREES).thirdAngle;
        VectorF targetLoc = invCuPos.multiplied(wayPoint.getColumn(3)).normalized3D();
        headingSkew = ComputeUtils.getXYAngleRadian(targetLoc.get(0), targetLoc.get(1))*radian2Degree;

        // compute the shortest distance to the trajectory
        VectorF robotLoc = wayPoint.inverted().multiplied(curP.getColumn(3)).normalized3D();
        stanleyDistance = Math.abs(endHeadingSkew)<=90?-robotLoc.get(1):robotLoc.get(1);

        // compute distance
        //distanceError = (targetLoc.get(0))>0?targetLoc.magnitude():-targetLoc.magnitude();
        distanceError = targetLoc.magnitude();

        // compute stanley skew
        stanleySkew = Math.atan(stanleyGain * stanleyDistance / (1 + chassis.speed)) * 57.2958;
        totalSkew = Range.clip(headingSkew + stanleySkew, -360, 360);

        // compute steering delta power by PID
        deltaPower = Range.clip(pidDeltaPower.update(totalSkew,
                System.currentTimeMillis() * 0.001), -0.7, 0.7);
        double skePowerAdjust = Math.abs(1 - Math.min(Math.abs(totalSkew), maxTurnAngle) / maxTurnAngle);

        //compute moving power, f(path length )*g(skew angle), turning speed will reduce forward speed
        forwardPower = Range.clip(pidForwardPower.update(distanceError,
                System.currentTimeMillis() * 0.001), -1, 1)*skePowerAdjust;
        // move
        if (backwardDriving) {
            chassis.forwardMoveAtPower(-forwardPower, deltaPower);
        } else {
            chassis.forwardMoveAtPower(forwardPower, deltaPower);
        }
        lastMat = new OpenGLMatrix(wayPoint);

        return distanceError;
    }

    @Override
    public void reset() {
        super.reset();
        if (motionVector!=null){
            iteratorMat = motionVector.iterator();
            if (iteratorMat.hasNext()){
                targetMat = iteratorMat.next();
                nextMat = targetMat;
            }
            if (iteratorMat.hasNext()) {
                nextMat = iteratorMat.next();
            }
            if(targetMat!=null) {
                targetWaypoint = targetMat.getTranslation();
            }
        }
        convergenceCount = 0;
    }

    @Override
    protected void nextWaypoint() {
        targetMat = nextMat;
        if (iteratorMat!= null && iteratorMat.hasNext()) {
            nextMat = iteratorMat.next();
        } else {
            nextMat = null;
        }
        if (targetMat!=null) {
            targetWaypoint = targetMat.getTranslation();
        }
        chaosCount = 0;
    }

    @Override
    public void setWaypoints(List<VectorF> listWaypoints) {
        super.setWaypoints(listWaypoints);
        // compute motion vectors in OpenGLMatrix list
        motionVector = new ArrayList<>();
        pathLength = 0;
        VectorF currentPos = chassis.getPredictedCargoLocation().getTranslation();
        for(VectorF w : listWaypoints){
            OpenGLMatrix o = new OpenGLMatrix();
            // set location
            o.translate(w.get(0), w.get(1), w.get(2));
            // rotation axle vector
            //VectorF rotationV = crossProduct(w,currentPos);
            // rotation angle
            VectorF minusV = w.subtracted(currentPos);
            double angle = getVectorAngle(minusV.get(0), minusV.get(1));

            // set trajectory
            o.rotate(AngleUnit.RADIANS, (float)angle, 0,0,1);
            motionVector.add(o);
            // accumulate path length
            pathLength += minusV.magnitude();
            currentPos = w;
            destination = o;
        }
        reset();
    }

    @Override
    public void skipWaypoints(float lookAheadDistance) {
        currentPos = chassis.getPredictedCargoLocation();

        VectorF currentPos = this.currentPos.getTranslation();

        if (telemetry != null) {
            telemetry.addData("skipWaypoints currentPos:", currentPos);
        }

        // kip arrived Waypoint
        if (targetMat != null && targetWaypoint != null) {
            if (currentPos.subtracted(targetWaypoint).magnitude() < lookAheadDistance) {
                if (nextMat != null) {
                    nextWaypoint();
                } else {
                    // last position need convergence count
                    convergenceCount++;
                    if (convergenceCount > 2){
                        nextWaypoint();
                    }
                }
                state = 0;
                chaosCount = 0;
            } else {
                convergenceCount = 0;
            }
        }
    }

    @Override
    public int followWayPoints(float lookAheadDistance) {
        skipWaypoints(lookAheadDistance);
        // run
        if (targetMat != null) {
            goToWayPoint(targetMat, currentPos);
            return 1;
        } else {
            chassis.forwardMoveAtPower(0, 0);
            pidForwardPower.reset();
            pidDeltaPower.reset();
            state = 0;
            return 0;
        }
    }
}
