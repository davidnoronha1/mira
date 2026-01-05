# control_utils

This package provides a header-only C++ library with utility classes and functions for control systems, such as a PID controller.

## How it Works

This is a utility package that other C++ nodes can depend on. It does not contain any nodes or executables itself. The primary feature is the `PID_Controller` class, which is defined in the `control_utils/control_utils.hpp` header.

-   **PID_Controller**: A class that implements a standard Proportional-Integral-Derivative controller. It tracks error over time and calculates a control output.

## Example Usage

This package is meant to be included in other C++ ROS 2 nodes.

```mermaid
graph TD
    subgraph control_utils
        A[control_utils.hpp<br><i>(PID_Controller class)</i>]
    end

    subgraph Other ROS Package
        B(my_controller_node.cpp)
    end

    A -- "#include" --> B;
```

## How to Use

To use the utilities in this package, you need to add it as a dependency to your own package and include the header file in your C++ source.

1.  **Add dependency in `package.xml`**:

    ```xml
    <depend>control_utils</depend>
    ```

2.  **Find package in `CMakeLists.txt`**:

    ```cmake
    find_package(control_utils REQUIRED)
    ```

3.  **Link against the library in `CMakeLists.txt`**:

    ```cmake
    ament_target_dependencies(your_node_name
      "rclcpp"
      "control_utils"
    )
    ```

4.  **Include the header in your C++ file**:

    ```cpp
    #include <control_utils/control_utils.hpp>

    // ...

    PID_Controller my_pid;
    my_pid.kp = 5.0;
    my_pid.ki = 0.1;
    my_pid.kd = 1.0;

    double control_output = my_pid.pid_control(error, delta_time);
    ```
