use nalgebra::Isometry3;

pub type Pose = Isometry3<f64>;
pub type FrameName = String;
/// Query to kinematics solver
#[derive(Debug)]
pub struct JointQuery {
    /// New position for every joint
    /// None means that no change in this joint
    pub joints: Vec<Option<f64>>,
    /// Name of every frame to compute position for
    pub links: Vec<FrameName>,
}
/// Error for forward kinematics solver
#[derive(Debug)]
pub enum FKError {
    /// Requested position of link that is not defined
    WrongJointName(FrameName),
    /// Query asked to move joint outside of its range
    JointOutOfRange(FrameName, f64),
}

impl std::fmt::Display for FKError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            FKError::WrongJointName(s) => write!(f, "Wrong joint name: {s}"),
            FKError::JointOutOfRange(s, a) => write!(f, "Joint {s} position {a} exceeds range"),
        }
    }
}
/// Solution from forward kinematics solver
#[derive(Debug)]
pub struct FKSolution {
    /// Returns pose for every asked frame
    pub frames: Vec<(FrameName, Pose)>,
}

/// Solver for forward kinematics
pub trait FKSolver {
    /// Computes positions of requested links when moving requested
    /// joints
    fn compute_move(joints: JointQuery) -> Result<FKSolution, FKError>;
}

/// Result of inverse kinematics solver
#[derive(Debug)]
pub enum IKSolution {
    /// Solution exactly matches end effector position
    Exact(Vec<f64>),
    /// Solution is approximate
    Approx(Vec<f64>, Vec<(FrameName, Pose)>),
}
impl IKSolution {
    /// Extracts joints solution while ignoring the
    /// approx or exact tag
    pub fn get_any_solution(self) -> Vec<f64> {
        match self {
            IKSolution::Exact(v) | IKSolution::Approx(v, _) => v,
        }
    }
    /// Returns `true` if solution is Exact
    pub fn is_exact(&self) -> bool {
        matches!(self, IKSolution::Exact(_))
    }
    /// Returns `true` if solution is Approximate
    pub fn is_approx(&self) -> bool {
        matches!(self, IKSolution::Approx(..))
    }
}
/// Error for inverse kinematics solver
pub enum IKError {
    /// Couldn't find any solution
    NotSolvable,
    /// Requested to solve for tip that
    /// doesn't have defined frame
    WrongTipName(FrameName),
}

impl std::fmt::Display for IKError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            IKError::NotSolvable => write!(f, "Pose is not solvable"),
            IKError::WrongTipName(s) => write!(f, "Invalid tip name: {s}"),
        }
    }
}

/// Solver for inverse kinematics
pub trait IKSolver {
    /// Computes how to move joints provided tip position
    fn solve_inverse_pose(
        tips: Vec<(FrameName, Pose)>,
        initial_guess: Option<Vec<f64>>,
    ) -> Result<IKSolution, IKError>;
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector3;

    #[test]
    fn fk_error_fmt() {
        let wrong_joint_err = FKError::WrongJointName("joint1".to_owned());
        let joint_out_of_range_err = FKError::JointOutOfRange("joint2".to_owned(), 1.0);
        assert_eq!("Wrong joint name: joint1", wrong_joint_err.to_string());
        assert_eq!(
            "Joint joint2 position 1 exceeds range",
            joint_out_of_range_err.to_string()
        );
    }

    #[test]
    fn ik_solution() {
        let solution = vec![0.0, 1.0, 2.0];
        let exact_solution = IKSolution::Exact(solution.clone());

        let axisangle = Vector3::y() * std::f64::consts::FRAC_PI_2;
        let translation = Vector3::new(1.0, 2.0, 3.0);
        let pose = Pose::new(translation, axisangle);
        let tip_pose_vec = vec![("tip1".to_owned(), pose)];
        let approx_solution = IKSolution::Approx(solution.clone(), tip_pose_vec);

        assert!(exact_solution.is_exact());
        assert!(!exact_solution.is_approx());
        assert_eq!(solution, exact_solution.get_any_solution());

        assert!(!approx_solution.is_exact());
        assert!(approx_solution.is_approx());
        assert_eq!(solution, approx_solution.get_any_solution());
    }

    #[test]
    fn ik_error_fmt() {
        let not_solvable = IKError::NotSolvable;
        let wrong_tip = IKError::WrongTipName("tip1".to_owned());
        assert_eq!("Pose is not solvable", not_solvable.to_string());
        assert_eq!("Invalid tip name: tip1", wrong_tip.to_string());
    }
}
