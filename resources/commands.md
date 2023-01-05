## Creating a Package
- CPP
    ```
    ros2 pkg create --build-type ament_cmake <package_name>
    ```
- Python
    ```
    ros2 pkg create --build-type ament_python <package_name>
    ```

## Getting Information
- Message
    ```
    ros2 interface show <msg/msg/type>
    ```
- Topic
    ```
    ros2 topic info <topic name>
    ```

## Locations
- For generic header files
    ```
    cd /opt/ros/humble/include/
    ```