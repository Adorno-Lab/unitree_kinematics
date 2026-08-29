#include <dqrobotics/robots/CFFSerialRobot.h>

/**
 * @brief _hstack stacks matrices in sequence horizontally
 * @param A First matrix
 * @param B Second matrix
 * @return The matrix [A B]
 */
MatrixXd _hstack(const MatrixXd &A, const MatrixXd &B);


/**
 * @brief _resize resizes a matrix A to a larger matrix of size (rowsxcols) that contains
 *               the matrix A. The additional elements are zeros.
 * @param A Input matrix to be resized
 * @param rows Desired number of rows (must be >= A.rows())
 * @param cols Desired number of columns (must be >= A.cols())
 * @return The resized matrix [A 0; 0 0]
 */
MatrixXd _resize(const MatrixXd &A, const int &rows, const int &cols);




MatrixXd _resize(const MatrixXd &A, const int &rows, const int &cols)
{
    MatrixXd aux = MatrixXd::Zero(rows, cols);
    int m = A.rows();
    int n = A.cols();

    if (m > rows)
    {
        throw std::runtime_error(std::string("The rows you used is smaller than the rows of the Matrix. ")
                                 +std::string("Incompatible rows for resize. Matrix A has ")
                                 +std::to_string(m)+ std::string(" rows. But you used ")
                                 +std::to_string(rows));
    }
    if (n > cols)
    {
        throw std::runtime_error(std::string("The cols you used is smaller than the cols of the Matrix. ")
                                 +std::string("Incompatible cols for resize. Matrix A has ")
                                 +std::to_string(n)+ std::string(" cols. But you used ")
                                 +std::to_string(cols));
    }

    aux.block(0,0, m, n) = A;
    return aux;
}

MatrixXd _hstack(const MatrixXd &A, const MatrixXd &B)
{

    int m_A = A.rows();
    int m_B = B.rows();

    if (m_A != m_B)
        throw std::runtime_error(std::string("Wrong usage of _hstack(A, B). ")
                                 + std::string("Incompatible sizes. The rows of Matrix A and B must have the same dimensions. ")
                                 + std::string("But A is ")+ std::to_string(m_A)+ std::string("x")+ std::to_string(A.cols())
                                 + std::string(" and B is ")+ std::to_string(m_B) + std::string("x")+ std::to_string(B.cols()));
    int n_A = A.cols();
    int n_B = B.cols();
    MatrixXd C = MatrixXd::Zero(m_A, n_A + n_B);
    C.block(0,0, m_A, n_A) = A;
    C.block(0, n_A, m_B, n_B) = B;
    return C;
}


namespace DQ_robotics
{


/**
 * @brief CFFSerialRobot::CFFSerialRobot Constructor that initializes a serial robot
 *        with a floating base (8 coefficients representing a unit dual quaternion
 *        base pose + arm joints).
 * @param robot_arm Shared pointer to the arm manipulator model
 */
DQ_robotics::CFFSerialRobot::CFFSerialRobot(const std::shared_ptr<DQ_SerialManipulator> &robot_arm)
    :kin_arm_{robot_arm}, arm_dim_configuration_space_{robot_arm->get_dim_configuration_space()}
{
    dim_configuration_space_ = base_dim_configuration_space_ + arm_dim_configuration_space_;

    min_index_ = base_dim_configuration_space_ - 1;   //5
    max_index_ = dim_configuration_space_      - 1;         //11


    for (auto i=0;i<3;i++){
        I_(i+1,i) = 1.0;
        I_(i+5,i+3) = 1.0;
    }
}

/**
 * @brief CFFSerialRobot::set_offset Sets the offset transformation between the
 *        base frame and the arm's mounting frame.
 * @param x_b_a The offset as a unit dual quaternion from base to arm
 */
void CFFSerialRobot::set_offset(const DQ &x_b_a)
{
    x_b_a_ = x_b_a;
}


/**
 * @brief CFFSerialRobot::get_six_dof_constraints Returns the equality constraints
 *        for enforcing different base motion modes.
 * @param mode The constraint mode (PLANAR_JOINT, STAND, or NONE)
 * @return A tuple {Aeq, beq} representing the linear constraints Aeq * q_dot = beq
 * @throws std::runtime_error if an invalid mode is provided
 */
std::tuple<MatrixXd, VectorXd> CFFSerialRobot::get_six_dof_constraints(const SIX_DOF_CONSTRAINT_MODE &mode)
{
    MatrixXd Aeq = MatrixXd::Zero(3,dim_configuration_space_);
    VectorXd beq = VectorXd::Zero(3);
    switch (mode) {
    case SIX_DOF_CONSTRAINT_MODE::PLANAR_JOINT:
    {
        // Walking Mode (planar joint)
        Aeq << 1,0,0,0,0,0,      MatrixXd::Zero(1,arm_dim_configuration_space_),
               0,1,0,0,0,0,      MatrixXd::Zero(1,arm_dim_configuration_space_),
               0,0,0,0,0,1,      MatrixXd::Zero(1,arm_dim_configuration_space_);
        break;
    }
    case SIX_DOF_CONSTRAINT_MODE::STAND:
    {
        //Forced Stand Mode
        Aeq << MatrixXd::Zero(3,3),      MatrixXd::Identity(3,3) , MatrixXd::Zero(3,arm_dim_configuration_space_);
        break;
    }
    case SIX_DOF_CONSTRAINT_MODE::NONE:
    {
        Aeq = MatrixXd::Zero(6,dim_configuration_space_);
        beq = VectorXd::Zero(6);
        Aeq << MatrixXd::Identity(3,3),  MatrixXd::Zero(3,3),      MatrixXd::Zero(3,arm_dim_configuration_space_),
               MatrixXd::Zero(3,3),      MatrixXd::Identity(3,3) , MatrixXd::Zero(3,arm_dim_configuration_space_);
        break;
    }
    default:
        throw std::runtime_error("Wrong constraint mode!");
    }
    return {Aeq,beq};
}

/**
 * @brief CFFSerialRobot::fkm Computes the forward kinematics for a specific link.
 * @param q configuration vector [vec8(base_pose), n-arm_joints] where:
 *        - vec8(base_pose): 8 coefficients of the unit dual quaternion representing the floating base pose
 *        - n-arm_joints: joint positions of the arm manipulator
 * @param ith Link index
 * @return The pose of the ith link as a dual quaternion
 * @throws std::runtime_error if q size is incorrect or ith is out of range
 */
DQ CFFSerialRobot::fkm(const VectorXd &q, const int &ith) const
{
    if (q.size() !=  8+arm_dim_configuration_space_)
        throw std::runtime_error("CFFSerialRobot::fkm: wrong configuration dimensions!");

    const VectorXd qbase = q.head(8);
    const VectorXd qarm  = q.tail(arm_dim_configuration_space_);
    DQ x;


    const DQ x_0_b = DQ(qbase);
    // DQ x_a_e = kin_arm_->fkm(qarm);
    // x_0_b*x_b_a_*x_a_e;

    if (ith < min_index_ or ith > max_index_)
        throw std::runtime_error("CFFSerialRobot::fkm: The minimum index is "+std::to_string(min_index_)+ ". "
                                  "The maximum index is "+std::to_string(max_index_)+ ". "
                                  "However, you used "+ std::to_string(ith));
    else if (ith == min_index_)
    {
        x = x_0_b;
    }else
    {
        x = x_0_b*x_b_a_*kin_arm_->fkm(qarm, ith-(min_index_+1));
    }
    return x;
}

/**
 * @brief CFFSerialRobot::fkm Computes the forward kinematics.
 * @param q configuration vector [vec8(base_pose), n-arm_joints] where:
 *        - vec8(base_pose): 8 coefficients of the unit dual quaternion representing the floating base pose
 *        - n-arm_joints: joint positions of the arm manipulator
 * @return The pose of the ith link as a dual quaternion
 * @throws std::runtime_error if q size is incorrect or ith is out of range
 */
DQ CFFSerialRobot::fkm(const VectorXd &q) const
{
    return fkm(q, get_dim_configuration_space()-1);
}


/**
 * @brief CFFSerialRobot::pose_jacobian Computes the pose Jacobian for a specific link.
 * @param q configuration vector [vec8(base_pose), n-arm_joints] where:
 *        - vec8(base_pose): 8 coefficients of the unit dual quaternion representing the floating base pose
 *        - n-arm_joints: joint positions of the arm manipulator
 * @param ith Link index
 * @return The 8xn pose Jacobian matrix
 * @throws std::runtime_error if q size is incorrect or ith is out of range
 */
MatrixXd CFFSerialRobot::pose_jacobian(const VectorXd &q, const int &ith) const
{
    if (q.size() !=  8+arm_dim_configuration_space_)
        throw std::runtime_error("CFFSerialRobot::fkm: wrong configuration dimensions!");
    VectorXd qbase = q.head(8);
    VectorXd qarm  = q.tail(arm_dim_configuration_space_);

    MatrixXd J;

    const DQ x_0_b = DQ(qbase);
    DQ x_a_e = kin_arm_->fkm(qarm);
    DQ x_b_e = x_b_a_*x_a_e;
    DQ x_0_a = x_0_b*x_b_a_;
    MatrixXd Jtwist = 0.5*hamiplus8(x_0_b)*I_;
    //MatrixXd Jarm = kin_arm_->pose_jacobian(qarm);

    if (ith < min_index_ or ith > max_index_)
        throw std::runtime_error("CFFSerialRobot::pose_jacobian: The minimum index is "+std::to_string(min_index_)+ ". "
                                 "The maximum index is "+std::to_string(max_index_)+ ". "
                                 "However, you used "+ std::to_string(ith));
    else if (ith == min_index_)
    {
        J = _resize(Jtwist, 8, get_dim_configuration_space());
    }else
    {
        MatrixXd Jarm = kin_arm_->pose_jacobian(qarm,  ith-(min_index_+1));
        J = _hstack(haminus8(x_b_e)*Jtwist, hamiplus8(x_0_a)*Jarm);
        if (J.cols() != get_dim_configuration_space())
            J = _resize(J, 8, get_dim_configuration_space());
    }
    return J;

}


/**
 * @brief CFFSerialRobot::pose_jacobian Computes the pose Jacobian for the end-effector.
 * @param q configuration vector [vec8(base_pose), n-arm_joints] where:
 *        - vec8(base_pose): 8 coefficients of the unit dual quaternion representing the floating base pose
 *        - n-arm_joints: joint positions of the arm manipulator
 * @return The 8xn pose Jacobian matrix for the end-effector
 */
MatrixXd CFFSerialRobot::pose_jacobian(const VectorXd &q) const
{
    return pose_jacobian(q, get_dim_configuration_space()-1);
}


/**
 * @brief CFFSerialRobot::get_dim_configuration_space Returns the dimension
 *        of the configuration space (vec8(base_pose) + arm DOF).
 * @return Total dimension of the configuration space
 */
int CFFSerialRobot::get_dim_configuration_space() const
{
    return dim_configuration_space_;
}


/**
 * @brief CFFSerialRobot::pose_jacobian_derivative Computes the time derivative
 *        of the pose Jacobian for a specific link.
 * @note This method is currently not implemented and will throw an exception.
 * @param q configuration vector [vec8(base_pose), n-arm_joints]
 * @param q_dot configuration velocity vector
 * @param to_ith_link Target link index
 */
MatrixXd CFFSerialRobot::pose_jacobian_derivative([[maybe_unused]] const VectorXd &q,
                                                  [[maybe_unused]] const VectorXd &q_dot,
                                                  [[maybe_unused]] const int &to_ith_link) const
{
    throw std::runtime_error("pose_jacobian_derivative is not implemented yet.");
}


/**
 * @brief CFFSerialRobot::pose_jacobian_derivative Computes the time derivative
 *        of the pose Jacobian for the end-effector.
 * @note This method is currently not implemented and will throw an exception.
 * @param q configuration vector [vec8(base_pose), n-arm_joints]
 * @param q_dot configuration velocity vector
 */
MatrixXd CFFSerialRobot::pose_jacobian_derivative(const VectorXd &q, const VectorXd &q_dot) const
{
    return pose_jacobian_derivative(q, q_dot, get_dim_configuration_space()-1);
}


}
