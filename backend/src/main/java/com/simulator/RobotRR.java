package com.simulator;

public class RobotRR {

    private double l1 = 1.0;
    private double l2 = 1.0;

    private double theta1 = 0.0;
    private double theta2 = 0.0;

    private double prevTheta1 = 0.0;
    private double prevTheta2 = 0.0;

    private double targetTheta1 = 0.0;
    private double targetTheta2 = 0.0;

    private double minTheta1 = -Math.PI;
    private double maxTheta1 = Math.PI;

    private double minTheta2 = -Math.PI;
    private double maxTheta2 = Math.PI;

    private static final double MAX_ANGULAR_VELOCITY = 2.0;
    private static final double SMOOTHING = 6.0;

    private double vel1 = 0.0;
    private double vel2 = 0.0;
    private double velX = 0.0;
    private double velY = 0.0;
    private double velEndEffector = 0.0;

    private double targetX = 0.0;
    private double targetY = 0.0;

    public enum GripperState {
        OPEN,
        CLOSED,
        HOLDING
    }

    private GripperState gripper = GripperState.OPEN;

    private double objectX = 0.0;
    private double objectY = 0.0;

    private boolean objectDetected = false;
    private boolean objectAttached = false;

    public enum ControlMode {
        DIRECT,
        INVERSE
    }

    private ControlMode mode = ControlMode.DIRECT;

    public enum MotionState {
        IDLE,
        MOVING,
        TARGET_REACHED
    }

    private MotionState motionState = MotionState.IDLE;

    public enum ElbowConfiguration {
        UP,
        DOWN
    }

    private ElbowConfiguration elbowConfiguration = ElbowConfiguration.UP;

    public RobotRR() {
        System.out.println("[ROBOT RR] -> modello inizializzato con successo");
    }

    public synchronized void update(double dt) {
        if (dt <= 0) return;

        prevTheta1 = theta1;
        prevTheta2 = theta2;

        if (mode == ControlMode.INVERSE) {
            targetTheta1 = computeTargetTheta1(targetX, targetY);
            targetTheta2 = computeTargetTheta2(targetX, targetY);
        }

        double angleError1 = Math.abs(targetTheta1 - theta1);
        double angleError2 = Math.abs(targetTheta2 - theta2);

        if (angleError1 < 0.001 && angleError2 < 0.001) {
            motionState = MotionState.TARGET_REACHED;
        } else {
            motionState = MotionState.MOVING;
        }

        double alpha = 1.0 - Math.exp(-SMOOTHING * dt);
        double delta1 = (targetTheta1 - theta1) * alpha;
        double delta2 = (targetTheta2 - theta2) * alpha;

        double maxStep = MAX_ANGULAR_VELOCITY * dt;
        delta1 = Math.max(-maxStep, Math.min(maxStep, delta1));
        delta2 = Math.max(-maxStep, Math.min(maxStep, delta2));

        theta1 += delta1;
        theta2 += delta2;

        vel1 = (theta1 - prevTheta1) / dt;
        vel2 = (theta2 - prevTheta2) / dt;

        velX = (getEndEffectorX() - getPreviousX()) / dt;
        velY = (getEndEffectorY() - getPreviousY()) / dt;
        velEndEffector = Math.hypot(velX, velY);

        checkObjectProximity();

        if (objectAttached) {
            objectX = getEndEffectorX();
            objectY = getEndEffectorY();
        }
    }

    public synchronized boolean setDirectKinematics(double l1, double l2, double targetT1, double targetT2) {
        if (l1 <= 0 || l2 <= 0) return false;

        if (targetT1 < minTheta1 || targetT1 > maxTheta1 || targetT2 < minTheta2 || targetT2 > maxTheta2) {
            return false;
        }

        this.l1 = l1;
        this.l2 = l2;
        this.mode = ControlMode.DIRECT;
        this.targetTheta1 = targetT1;
        this.targetTheta2 = targetT2;
        this.motionState = MotionState.MOVING;
        return true;
    }

    public synchronized boolean setInverseKinematics(double l1, double l2, double targetX, double targetY, ElbowConfiguration configuration) {
        if (l1 <= 0 || l2 <= 0 || configuration == null) return false;

        double r = Math.hypot(targetX, targetY);
        if (r > l1 + l2 || r < Math.abs(l1 - l2)) return false;

        double oldL1 = this.l1;
        double oldL2 = this.l2;
        this.l1 = l1;
        this.l2 = l2;
        this.elbowConfiguration = configuration;

        double candidateTheta1 = computeTargetTheta1(targetX, targetY);
        double candidateTheta2 = computeTargetTheta2(targetX, targetY);

        if (candidateTheta1 < minTheta1 || candidateTheta1 > maxTheta1 || candidateTheta2 < minTheta2 || candidateTheta2 > maxTheta2) {
            this.l1 = oldL1;
            this.l2 = oldL2;
            return false;
        }

        this.targetX = targetX;
        this.targetY = targetY;
        this.targetTheta1 = candidateTheta1;
        this.targetTheta2 = candidateTheta2;
        this.mode = ControlMode.INVERSE;
        this.motionState = MotionState.MOVING;
        return true;
    }

    public synchronized boolean setInverseKinematics(double l1, double l2, double targetX, double targetY) {
        return setInverseKinematics(l1, l2, targetX, targetY, ElbowConfiguration.UP);
    }

    private double computeTargetTheta2(double x, double y) {
        double cosTheta2 = (x * x + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2);
        cosTheta2 = Math.max(-1.0, Math.min(1.0, cosTheta2));
        double sinTheta2 = Math.sqrt(1.0 - cosTheta2 * cosTheta2);
        if (elbowConfiguration == ElbowConfiguration.DOWN) {
            sinTheta2 = -sinTheta2;
        }
        return Math.atan2(sinTheta2, cosTheta2);
    }

    private double computeTargetTheta1(double x, double y) {
        double cosTheta2 = (x * x + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2);
        cosTheta2 = Math.max(-1.0, Math.min(1.0, cosTheta2));
        double sinTheta2 = Math.sqrt(1.0 - cosTheta2 * cosTheta2);
        if (elbowConfiguration == ElbowConfiguration.DOWN) {
            sinTheta2 = -sinTheta2;
        }
        return Math.atan2(y, x) - Math.atan2(l2 * sinTheta2, l1 + l2 * cosTheta2);
    }

    public synchronized boolean setJointLimits(double minT1, double maxT1, double minT2, double maxT2) {
        if (minT1 >= maxT1 || minT2 >= maxT2) return false;
        if (theta1 < minT1 || theta1 > maxT1 || theta2 < minT2 || theta2 > maxT2) return false;

        this.minTheta1 = minT1;
        this.maxTheta1 = maxT1;
        this.minTheta2 = minT2;
        this.maxTheta2 = maxT2;
        return true;
    }

    public synchronized void placeObject(double x, double y) {
        this.objectX = x;
        this.objectY = y;
        this.objectDetected = false;
        this.objectAttached = false;
        this.gripper = GripperState.OPEN;
    }

    public synchronized void checkObjectProximity() {
        double currentX = getEndEffectorX();
        double currentY = getEndEffectorY();
        double distance = Math.hypot(currentX - objectX, currentY - objectY);

        if (distance < 0.05) {
            objectDetected = true;
            if (gripper == GripperState.CLOSED && !objectAttached) {
                objectAttached = true;
                gripper = GripperState.HOLDING;
            }
        } else {
            if (!objectAttached) {
                objectDetected = false;
            }
        }
    }

    public synchronized void releaseObject() {
        if (objectAttached) {
            objectAttached = false;
            objectDetected = false;
            gripper = GripperState.OPEN;
        } else {
            gripper = GripperState.OPEN;
        }
    }

    public double getEndEffectorX() {
        return l1 * Math.cos(theta1) + l2 * Math.cos(theta1 + theta2);
    }

    public double getEndEffectorY() {
        return l1 * Math.sin(theta1) + l2 * Math.sin(theta1 + theta2);
    }

    private double getPreviousX() {
        return l1 * Math.cos(prevTheta1) + l2 * Math.cos(prevTheta1 + prevTheta2);
    }

    private double getPreviousY() {
        return l1 * Math.sin(prevTheta1) + l2 * Math.sin(prevTheta1 + prevTheta2);
    }

    public double getTheta1() { return theta1; }
    public double getTheta2() { return theta2; }
    public double getVel1() { return vel1; }
    public double getVel2() { return vel2; }
    public double getVelX() { return velX; }
    public double getVelY() { return velY; }
    public double getEndEffectorVel() { return velEndEffector; }
    public GripperState getGripper() { return gripper; }
    public MotionState getMotionState() { return motionState; }
    public boolean isObjectDetected() { return objectDetected; }
    public boolean isObjectAttached() { return objectAttached; }
    public double getObjectX() { return objectX; }
    public double getObjectY() { return objectY; }
    public ControlMode getMode() { return mode; }
    public ElbowConfiguration getElbowConfiguration() { return elbowConfiguration; }

    public synchronized void setGripper(GripperState gripper) {
        if (gripper == null) return;
        if (gripper == GripperState.OPEN) {
            releaseObject();
        } else if (gripper == GripperState.CLOSED) {
            if (!objectAttached) {
                this.gripper = GripperState.CLOSED;
                checkObjectProximity();
            }
        }
    }
}