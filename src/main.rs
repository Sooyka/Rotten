use rotten::kinematics::*;
fn main() {
    println!("Henlo");
    let x = FKError::WrongJointName("a".into());
    println!("{x}");
}
