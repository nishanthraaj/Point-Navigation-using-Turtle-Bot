
# yellow_light_sdf=$HOME/.gazebo/models/yellow_light/model.sdf
# ros2 run self_driving_car_pkg spawner_node $red_light_sdf red_light 0.0 0.0

'''
This Python script is used to spawn Gazebo models in a ROS2 environment.
 It takes the path of an SDF model file and the name of the model as arguments,
 and can optionally take the X, Y, and Z coordinates of the initial position of the model.
   The script creates a ROS2 node,
 connects to the Gazebo spawn_entity service, and sends a request to spawn the
 specified model at the specified position.
'''
import sys
import rclpy
from gazebo_msgs.srv import SpawnEntity
def main():
    argv = sys.argv[1:]
    
    # Validate arguments
    if len(argv) < 2:
        print("Usage: sdf_spawner <sdf_file> <entity_name> [x] [y]")
        print("Example: ros2 run autonomous_tb3 sdf_spawner model.sdf my_entity 0.0 0.0")
        return
    
    rclpy.init()
    node = rclpy.create_node("Spawning_Node")
    client = node.create_client(SpawnEntity, "/spawn_entity")
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info("connected to spawner")
    sdf_path = argv[0]
    request = SpawnEntity.Request()
    request.name = argv[1]
    # Use user defined positions (If provided)
    if len(argv)>3:
        request.initial_pose.position.x = float(argv[2])
        request.initial_pose.position.y = float(argv[3])
        if (argv[1] == 'beer'):
            request.initial_pose.position.z=1.5
    # Read SDF file with error handling
    try:
        with open(sdf_path, 'r') as f:
            request.xml = f.read()
    except FileNotFoundError:
        node.get_logger().error(f"SDF file not found: {sdf_path}")
        node.destroy_node()
        rclpy.shutdown()
        return
    except Exception as e:
        node.get_logger().error(f"Error reading SDF file: {e}")
        node.destroy_node()
        rclpy.shutdown()
        return

    node.get_logger().info("Sending service request to `/spawn_entity`")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError(
            'exception while calling service: %r' % future.exception())

    node.get_logger().info("Done! Shutting down node.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()