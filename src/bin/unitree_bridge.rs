use humanoid_rs::unitree;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("Unitree Robot Bridge");
    println!("====================");
    println!("Connecting to Unitree G1/H1 via UDP...");
    
    // Connect to robot
    let mut robot = unitree::UnitreeRobot::connect("192.168.123.104:8080").await?;
    
    println!("Connected! Robot state:");
    println!("{:#?}", robot.state());
    
    // Test commands
    println!("\nTesting joint control...");
    let positions = vec![0.0; 29];
    robot.set_joint_positions(&positions)?;
    
    println!("Standing pose set");
    
    Ok(())
}
