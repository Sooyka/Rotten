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

/// Solver for inverse kinematics
pub trait IKSolver {
    /// Computes how to move joints provided tip position
    fn solve_inverse_pose(
        tips: Vec<(FrameName, Pose)>,
        initial_guess: Option<Vec<f64>>,
    ) -> Result<IKSolution, IKError>;
}
