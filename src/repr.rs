use std::collections::BTreeMap;

use nalgebra::{Isometry3, Vector3};
pub type FrameName = String;
pub type JointName = FrameName;
pub type LinkName = FrameName;
pub type Pose = Isometry3<f64>;
pub struct Link {
    pub name: LinkName,
    pub origin: Pose,
    pub parent_joints: Vec<JointName>,
    pub child_joints: Vec<JointName>,
    //    pub physical:
}
pub struct Frame {
    pub name: FrameName,
    pub from_parent: Pose,
}
pub type Axis = Vector3<f64>;
pub enum JointType {
    Continuous { axis: Axis },
    Prismatic { axis: Axis },
    Floating,
    Fixed,
}
pub enum JointState {
    Continuous { angle: f64 },
    Prismatic { distance: f64 },
    Floating(Pose),
    Fixed,
}

pub struct JointDescription {
    pub name: JointName,
    pub parent_link: LinkName,
    pub child_link: LinkName,
    pub frame: FrameName,
    pub origin: Pose,
    pub joint_type: JointType,
    //    pub physical:
}
pub struct Joint {
    pub desc: JointDescription,
    pub state: JointState,
}
pub struct Robot {
    links: BTreeMap<LinkName, Link>,
    joints: BTreeMap<JointName, (JointDescription, JointState)>,
    frames: BTreeMap<FrameName, Frame>,
}

impl Robot {
    pub fn check_names(&self) -> Result<(), String> {
        for link in self.links.values() {
            for joint in &link.parent_joints {
                if !self.joints.contains_key(joint) {
                    return Err(format!(
                        "link {} references unknown joint {}",
                        link.name, joint
                    ));
                }
            }
            for joint in &link.child_joints {
                if !self.joints.contains_key(joint) {
                    return Err(format!(
                        "link {} references unknown joint {}",
                        link.name, joint
                    ));
                }
            }
        }
        for (joint, _) in self.joints.values() {
            if !self.links.contains_key(&joint.parent_link) {
                return Err(format!(
                    "joint {} references unknown link {}",
                    joint.name, joint.frame
                ));
            }
            if !self.links.contains_key(&joint.child_link) {
                return Err(format!(
                    "joint {} references unknown link {}",
                    joint.name, joint.frame
                ));
            }
            if !self.frames.contains_key(&joint.frame) {
                return Err(format!(
                    "joint {} references unknown frame {}",
                    joint.name, joint.frame
                ));
            }
        }
        Ok(())
    }
}
