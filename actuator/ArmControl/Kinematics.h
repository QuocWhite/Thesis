//#ifndef KINEMATICS_H
//#define KINEMATICS_H
//
//#include "CommonDefines.h"
//#include <cmath>
//
///* === Kinematics Solver Class === */
//class Kinematics {
//public:
//    static Eigen::Vector3d
//        rotationMatrixToEulerAngles(
//            const Eigen::Matrix3d&
//                rotationMatrix);
//
//    static PoseVector
//        transformMatrixToPoseVector(
//            const TransformMatrix&
//                transformMatrix);
//
//    static TransformMatrix
//        computeForwardKinematics(
//            const JointVector&
//                jointAnglesDeg);
//
//    static JacobianMatrix
//        computeJacobianMatrix(
//            const JointVector&
//                jointAnglesDeg);
//
//    static JointVector
//        inverseKinematicsSolver(
//            const PoseVector&
//                targetPoseVector,
//            const JointVector&
//                initialJointAnglesDeg,
//            int maxIterations =
//                IK_MAX_ITERATIONS,
//            double tolerance =
//                IK_TOLERANCE,
//            double learningRate =
//                IK_LEARNING_RATE);
//};
//
//#endif // KINEMATICS_H
