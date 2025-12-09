#include "Kinematics.h"

/* === Rotation Matrix to Euler Angles === */
Eigen::Vector3d Kinematics::rotationMatrixToEulerAngles(
    const Eigen::Matrix3d& rotationMatrix)
{
    double sy =
        std::sqrt(
            rotationMatrix(0,0)*rotationMatrix(0,0) +
            rotationMatrix(1,0)*rotationMatrix(1,0));

    double yaw, pitch, roll;
    if (sy < EPSILON_SY) {
        roll = std::atan2(
            rotationMatrix(1,2),
            rotationMatrix(1,1));
        pitch = std::atan2(
            -rotationMatrix(2,0),
            sy);
        yaw = 0.0;
    } else {
        roll = std::atan2(
            rotationMatrix(2,1),
            rotationMatrix(2,2));
        pitch = std::atan2(
            -rotationMatrix(2,0),
            sy);
        yaw = std::atan2(
            rotationMatrix(1,0),
            rotationMatrix(0,0));
    }
    return Eigen::Vector3d(
        yaw, pitch, roll) * RAD_TO_DEG;
}

/* === Transform Matrix to Pose Vector === */
PoseVector Kinematics::transformMatrixToPoseVector(
    const TransformMatrix& transformMatrix)
{
    Eigen::Vector3d eulerAngles =
        rotationMatrixToEulerAngles(
            transformMatrix.block<3,3>(0,0));
    PoseVector poseVector;
    poseVector(0) = transformMatrix(0,3);
    poseVector(1) = transformMatrix(1,3);
    poseVector(2) = transformMatrix(2,3);
    poseVector(3) = eulerAngles(0);
    poseVector(4) = eulerAngles(1);
    poseVector(5) = eulerAngles(2);
    return poseVector;
}

/* === Forward Kinematics === */
TransformMatrix Kinematics::computeForwardKinematics(
    const JointVector& jointAnglesDeg)
{
    JointVector jointAnglesRad =
        jointAnglesDeg * DEG_TO_RAD;

    double d[MAX_JOINT] = {
        DH_D_0, DH_D_1, DH_D_2,
        DH_D_3, DH_D_4, DH_D_5
    };
    double a[MAX_JOINT] = {
        DH_A_0, DH_A_1, DH_A_2,
        DH_A_3, DH_A_4, DH_A_5
    };
    double alpha[MAX_JOINT] = {
        DH_ALPHA_0, DH_ALPHA_1, DH_ALPHA_2,
        DH_ALPHA_3, DH_ALPHA_4, DH_ALPHA_5
    };

    TransformMatrix transform =
        TransformMatrix::Identity();

    for (int i = 0; i < MAX_JOINT; ++i) {
        double theta = jointAnglesRad(i);
        double ct = std::cos(theta);
        double st = std::sin(theta);
        double ca = std::cos(alpha[i]);
        double sa = std::sin(alpha[i]);

        TransformMatrix currentTransform;
        currentTransform <<
            ct, -st*ca, st*sa, a[i]*ct,
            st, ct*ca, -ct*sa, a[i]*st,
            0, sa, ca, d[i],
            0, 0, 0, 1;

        transform = transform * currentTransform;
    }
    return transform;
}

/* === Jacobian Matrix === */
JacobianMatrix Kinematics::computeJacobianMatrix(
    const JointVector& jointAnglesDeg)
{
    JacobianMatrix jacobian =
        JacobianMatrix::Zero();

    PoseVector basePose =
        transformMatrixToPoseVector(
            computeForwardKinematics(
                jointAnglesDeg));

    for (int jointIndex = 0;
         jointIndex < MAX_JOINT;
         ++jointIndex)
    {
        JointVector perturbedAngles =
            jointAnglesDeg;
        perturbedAngles(jointIndex) +=
            JACOBIAN_DELTA;

        PoseVector perturbedPose =
            transformMatrixToPoseVector(
                computeForwardKinematics(
                    perturbedAngles));

        jacobian.col(jointIndex) =
            (perturbedPose - basePose) /
            JACOBIAN_DELTA;
    }
    return jacobian;
}

/* === Inverse Kinematics Solver === */
JointVector Kinematics::inverseKinematicsSolver(
    const PoseVector& targetPoseVector,
    const JointVector& initialJointAnglesDeg,
    int maxIterations,
    double tolerance,
    double learningRate)
{
    JointVector jointAngles =
        initialJointAnglesDeg;

    for (int iter = 0;
         iter < maxIterations;
         ++iter)
    {
        PoseVector currentPoseVector =
            transformMatrixToPoseVector(
                computeForwardKinematics(
                    jointAngles));
        PoseVector poseError =
            targetPoseVector -
            currentPoseVector;

        if (poseError.norm() < tolerance)
            break;

        JacobianMatrix jacobian =
            computeJacobianMatrix(
                jointAngles);

        Eigen::Matrix<double, MAX_JOINT, 1>
            deltaTheta =
                learningRate *
                jacobian.completeOrthogonalDecomposition()
                .pseudoInverse() *
                poseError;

        jointAngles += deltaTheta;
    }
    return jointAngles;
}
