# Complete Step-by-Step Guide for Using the ROS2 Practice Platform

## Introduction

This package is a practice platform for ros2 developer or user, it is based on docker so a prerequisite to use this package is docker installed properly on your host system.
You do not necessarily need to have ros2 installed on your system to run this package, as you can solve the question in docker container itself.
You will need to have an free api of gemini, on which this code is tested.
    To get your API visit Google API studio [here](https://aistudio.google.com/app/apikey?_gl=1*rr28tc*_ga*MzIyNzk3NzIwLjE3MjUyMDM4NTM.*_ga_P1DBVKWT6V*MTc0MTAzMjkwNC41LjAuMTc0MTAzMjkwNC42MC4wLjk3NDYxNjkzNQ..)

Click on create API Key, get your api and paste at correct place holder present at line 12 of ros2_platform.py script.


## 1. Start the Platform

1. Run your Python script to start the Gradio interface:
   ```bash
   python3 ros2_platform.py
   ```

2. Open the Gradio interface in your browser (typically at http://localhost:7860)

## 2. Generate a Question

1. In the Gradio interface:
   - Select a question category (e.g., "basic_concepts", "nodes_topics", etc.)
   - Select the "Generate Question" action
   - Click the "Submit" button

2. The platform will display:
   - A ROS2 practice question
   - Requirements
   - Expected output
   - Hints
   - A Docker command

3. Note the Docker command shown in the interface - you'll use this to start the practice environment

## 3. Create and Enter the Docker Environment

1. Open a terminal and navigate to your project directory

2. Create a solution directory if it doesn't exist (this is important! and in most cases this folder will automatically create in your local system):
   ```bash
   mkdir -p solution
   ```

3. Run the Docker command provided by the interface:
   ```bash
   docker run -it --name ros2practice_container -v $(pwd)/solution:/ros2_ws/src ros2practice
   ```

4. You should now be inside the Docker container

## 4. Implement Your Solution

1. Inside the container, navigate to the source directory:
   ```bash
   cd /ros2_ws/src
   ```

2. Create a ROS2 package:
   ```bash
   ros2 pkg create --build-type ament_python my_solution_pkg --dependencies rclpy 
   
   ```
   ##### Note: you can create a cpp package too you are free to do the solution however you like 
3. Write your solution as asked in the question, look for specific patterns and all being generated in the question.json file or on the web interface

4. Build your package:
   ```bash
   cd /ros2_ws
   colcon build --packages-select my_solution_pkg
   source install/setup.bash
   ```

5. Test your node:
   ```bash
   ros2 run my_solution_pkg solution_node
   ```

6. In a separate terminal (outside Docker), connect to the container to verify:
   ```bash
   docker exec -it ros2practice_container bash
   source /opt/ros/humble/setup.bash
   source /ros2_ws/install/setup.bash
   ros2 topic echo /topic  # Or whatever topic your node is using
   ```

7. Exit the container when you're done testing:
   ```bash
   exit
   ```

## 5. Submit Your 

```
    - OPEN THE question.json file and there you will see the test test_commands
     "test_commands": [
        "ros2 run factorial_service factorial_server.py",
        "ros2 run factorial_service factorial_client.py 5",
        "ros2 run factorial_service factorial_client.py -2",
        "ros2 run factorial_service factorial_client.py 0"
    ],

    -Make sure to update the package name and executable names correctly in here and then proceed

```


1. Stop the running Docker container (if still running):
   ```bash
   docker stop ros2practice_container
   docker rm ros2practice_container
   ```

2. Go back to the Gradio interface in your browser
   - Change the "Action" to "Submit Solution"
   - Click the "Submit" button
   - The platform will evaluate your solution and provide feedback

## Troubleshooting Common Issues

If you're experiencing connection timeout or errors during submission, this is a common issue, once wait for few seconds and confirm the output status in the running terminal, if you see the output status(correct solution or incorrect solution) on terminal but error on webpage then that is fine too, but if error is on terminal too then make sure :

1. **Folder Structure Issue**: 
   - Make sure your solution is correctly placed in the mounted directory
   - The directory structure should be `solution/my_solution_pkg/...` on your host machine

2. **Docker Container Not Stopping**:
   - Before submitting, ensure you've exited and stopped the Docker container:
   ```bash
   docker stop ros2practice_container
   docker rm ros2practice_container
   ```

3. **File Permissions**:
   - Check that your solution files have the correct permissions:
   ```bash
   chmod -R 755 solution/
   ```

4. **Question.json File**:
   - Make sure the `question.json` file in your working directory has not been modified other than the correct package name and executable name

5. **Check Docker Image**:
   - Ensure your Docker image was built correctly:
   ```bash
   docker images | grep ros2practice
   ```

6. **Check Mounted Volume**:
   - Verify that your solution directory is properly mounted:
   ```bash
   docker inspect ros2practice_container | grep -A 10 Mounts
   ```
