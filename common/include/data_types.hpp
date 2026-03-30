#ifndef _data_types_hpp_
#define _data_types_hpp_

#include <Eigen/Dense>

namespace mors::wbic
{

template<int Rows, int Cols, typename Scalar = double>
using MatrixRxCd = Eigen::Matrix<Scalar, Rows, Cols>;

template<int Dim, typename Scalar = double>
using VectorNd = MatrixRxCd<Dim, 1, Scalar>;

template<int Dim, typename Scalar = double>
using RowVectorNd = MatrixRxCd<1, Dim, Scalar>;

template<int Dim, typename Scalar = double>
using MatrixNd = MatrixRxCd<Dim, Dim, Scalar>;

template<int Rows, int Cols, typename Scalar = double>
using ArrayRxCd = Eigen::Array<Scalar, Rows, Cols>;

template<int Dim, typename Scalar = double>
using LDLTMatrixNd = Eigen::LDLT<MatrixNd<Dim, Scalar>>;

template<int Dim, typename Scalar = double>
using LLTMatrixNd = Eigen::LLT<MatrixNd<Dim, Scalar>>;

inline constexpr int kNumLegs = 4;
inline constexpr int kLegDofs = 3;
inline constexpr int kBaseDofs = 6;
inline constexpr int kQuaternionDofs = 4;
inline constexpr int kTaskDim = 3;
inline constexpr int kSupportIneqPerLeg = 6;
inline constexpr int kForceDimPerLeg = 3;

inline constexpr int kJointDofs = kNumLegs * kLegDofs;
inline constexpr int kNv = kBaseDofs + kJointDofs;
inline constexpr int kNq = 7 + kJointDofs;
inline constexpr int kMaxContactLegs = kNumLegs;
inline constexpr int kMaxContactDim = kMaxContactLegs * kForceDimPerLeg;
inline constexpr int kMaxSupportIneq = kMaxContactLegs * kSupportIneqPerLeg;
inline constexpr int kMaxQpVars = kBaseDofs + kMaxContactDim;

template<int NumSupportLegs>
inline constexpr int kContactDim = NumSupportLegs * kForceDimPerLeg;

template<int NumSupportLegs>
inline constexpr int kSupportDim = NumSupportLegs * kSupportIneqPerLeg;

template<int NumSupportLegs>
inline constexpr int kQpDim = kBaseDofs + kContactDim<NumSupportLegs>;

template<typename Scalar = double>
using Vector3dT = VectorNd<3, Scalar>;

template<typename Scalar = double>
using Vector4dT = VectorNd<4, Scalar>;

template<typename Scalar = double>
using Vector6dT = VectorNd<6, Scalar>;

template<typename Scalar = double>
using Vector12dT = VectorNd<12, Scalar>;

template<typename Scalar = double>
using Vector18dT = VectorNd<18, Scalar>;

template<typename Scalar = double>
using Vector19dT = VectorNd<19, Scalar>;

template<typename Scalar = double>
using Vector24dT = VectorNd<24, Scalar>;

template<typename Scalar = double>
using Matrix3dT = MatrixNd<3, Scalar>;

template<typename Scalar = double>
using Matrix6dT = MatrixNd<6, Scalar>;

template<typename Scalar = double>
using Matrix12dT = MatrixNd<12, Scalar>;

template<typename Scalar = double>
using Matrix18dT = MatrixNd<18, Scalar>;

template<typename Scalar = double>
using Matrix24dT = MatrixNd<24, Scalar>;

template<typename Scalar = double>
using Matrix3_18dT = MatrixRxCd<3, 18, Scalar>;

template<typename Scalar = double>
using Matrix6_18dT = MatrixRxCd<6, 18, Scalar>;

template<typename Scalar = double>
using Matrix12_18dT = MatrixRxCd<12, 18, Scalar>;

template<typename Scalar = double>
using Matrix18_3dT = MatrixRxCd<18, 3, Scalar>;

template<typename Scalar = double>
using Matrix18_12dT = MatrixRxCd<18, 12, Scalar>;

template<typename Scalar = double>
using Matrix24_18dT = MatrixRxCd<24, 18, Scalar>;

template<typename Scalar = double>
using TaskVectord = Vector3dT<Scalar>;

template<typename Scalar = double>
using ConfigVectord = Vector19dT<Scalar>;

template<typename Scalar = double>
using GenCoordVectord = Vector18dT<Scalar>;

template<typename Scalar = double>
using JointVectord = Vector12dT<Scalar>;

template<typename Scalar = double>
using ForceVectord = Vector12dT<Scalar>;

template<typename Scalar = double>
using BaseVectord = Vector6dT<Scalar>;

template<typename Scalar = int>
using Vector4iT = VectorNd<4, Scalar>;

template<typename Scalar = int>
using LegStateVectori = Vector4iT<Scalar>;

template<typename Scalar = double>
using RotationMatrixd = Matrix3dT<Scalar>;

template<typename Scalar = double>
using MassMatrixd = Matrix18dT<Scalar>;

template<typename Scalar = double>
using NullProjectord = Matrix18dT<Scalar>;

template<typename Scalar = double>
using SelectorMatrixd = Matrix6_18dT<Scalar>;

template<typename Scalar = double>
using TaskJacobianMatrixd = Matrix3_18dT<Scalar>;

template<typename Scalar = double>
using TaskMetricMatrixd = Matrix3dT<Scalar>;

template<typename Scalar = double>
using TaskSolverMatrixd = Matrix18_3dT<Scalar>;

template<typename Scalar = double>
using ContactJacobianMaxMatrixd = Matrix12_18dT<Scalar>;

template<typename Scalar = double>
using ContactMetricMaxMatrixd = Matrix12dT<Scalar>;

template<typename Scalar = double>
using ContactSolverMaxMatrixd = Matrix18_12dT<Scalar>;

template<typename Scalar = double>
using SupportConstraintMaxMatrixd = Matrix24_18dT<Scalar>;

template<typename Scalar = double>
using SupportConstraintMaxVectord = Vector24dT<Scalar>;

template<typename Scalar = double>
using QpMaxMatrixd = Matrix18dT<Scalar>;

template<typename Scalar = double>
using QpMaxVectord = Vector18dT<Scalar>;

template<typename Scalar = double>
using QpEqMaxMatrixd = Matrix6_18dT<Scalar>;

using Vector3d = Vector3dT<>;
using Vector4d = Vector4dT<>;
using Vector6d = Vector6dT<>;
using Vector12d = Vector12dT<>;
using Vector18d = Vector18dT<>;
using Vector19d = Vector19dT<>;
using Vector24d = Vector24dT<>;
using Vector4i = Vector4iT<>;

using Matrix3d = Matrix3dT<>;
using Matrix6d = Matrix6dT<>;
using Matrix12d = Matrix12dT<>;
using Matrix18d = Matrix18dT<>;
using Matrix24d = Matrix24dT<>;

using Matrix3_18d = Matrix3_18dT<>;
using Matrix6_18d = Matrix6_18dT<>;
using Matrix12_18d = Matrix12_18dT<>;
using Matrix18_3d = Matrix18_3dT<>;
using Matrix18_12d = Matrix18_12dT<>;
using Matrix24_18d = Matrix24_18dT<>;

template<int NumSupportLegs, typename Scalar = double>
using ContactVectord = VectorNd<kContactDim<NumSupportLegs>, Scalar>;

template<int NumSupportLegs, typename Scalar = double>
using ContactJacobianMatrixd =
    MatrixRxCd<kContactDim<NumSupportLegs>, kNv, Scalar>;

template<int NumSupportLegs, typename Scalar = double>
using ContactMetricMatrixd =
    MatrixNd<kContactDim<NumSupportLegs>, Scalar>;

template<int NumSupportLegs, typename Scalar = double>
using ContactSolverMatrixd =
    MatrixRxCd<kNv, kContactDim<NumSupportLegs>, Scalar>;

template<int NumSupportLegs, typename Scalar = double>
using SupportConstraintMatrixd =
    MatrixRxCd<kSupportDim<NumSupportLegs>, kQpDim<NumSupportLegs>, Scalar>;

template<int NumSupportLegs, typename Scalar = double>
using SupportConstraintVectord =
    VectorNd<kSupportDim<NumSupportLegs>, Scalar>;

template<int NumSupportLegs, typename Scalar = double>
using QpMatrixd = MatrixNd<kQpDim<NumSupportLegs>, Scalar>;

template<int NumSupportLegs, typename Scalar = double>
using QpVectord = VectorNd<kQpDim<NumSupportLegs>, Scalar>;

template<int NumSupportLegs, typename Scalar = double>
using QpEqMatrixd =
    MatrixRxCd<kBaseDofs, kQpDim<NumSupportLegs>, Scalar>;

} // namespace mors::wbic

#endif //_data_types_hpp_
