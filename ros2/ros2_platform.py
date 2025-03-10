
import os
import random
import json
import subprocess
import docker
import gradio as gr
from google.generativeai import GenerativeModel
import google.generativeai as genai

# Configure the Gemini API
API_KEY = "YOUR-API-KEY"  # Replace with your actual API key
genai.configure(api_key=API_KEY)

# Questions database - will be expanded by Gemini API
QUESTION_CATEGORIES = [
    "basic_concepts",
    "nodes_topics",
    "services_actions",
    "parameters",
    "launch_files",
    "navigation",
    "visualization",
    "custom_interfaces",
    "tf2"
]

# Initialize Docker client
docker_client = docker.from_env()

def generate_question(category):
    """Generate a ROS2 question using Gemini API based on category."""
    model = GenerativeModel('gemini-1.5-pro')
    
    # Define a fallback question in case API fails
    fallback_question = {
        "question": f"Create a simple ROS2 node in the {category} category that publishes a string message.",
        "requirements": ["Create a ROS2 package", "Implement a publisher node", "Use std_msgs/String message type"],
        "expected_output": "The node should publish a string message to a topic at 1Hz rate.",
        "hints": ["Use the rclpy library", "Remember to import necessary message types"],
        "solution_reference": "import rclpy\nfrom rclpy.node import Node\nfrom std_msgs.msg import String\n\nclass SimplePublisher(Node):\n    def __init__(self):\n        super().__init__('simple_publisher')\n        self.publisher = self.create_publisher(String, 'topic', 10)\n        self.timer = self.create_timer(1.0, self.timer_callback)\n\n    def timer_callback(self):\n        msg = String()\n        msg.data = 'Hello ROS2'\n        self.publisher.publish(msg)\n\ndef main(args=None):\n    rclpy.init(args=args)\n    node = SimplePublisher()\n    rclpy.spin(node)\n    node.destroy_node()\n    rclpy.shutdown()\n\nif __name__ == '__main__':\n    main()",
        "test_commands": ["ros2 run <package_name> <node_name>", "ros2 topic echo /topic"],
        "evaluation_criteria": ["Package builds successfully", "Node publishes messages to the topic"]
    }
    
    try:
        prompt = f"""
        Generate a practice question for ROS2 in the category: {category}.
        
        The question should include:
        1. A clear problem statement
        2. Requirements for the solution
        3. Expected output or behavior
        4. Hints (optional)
        5. A solution reference that can be used to validate user submissions
        
        Format the response as a JSON object with these fields:
        - question: The problem statement
        - requirements: List of requirements
        - expected_output: What the correct solution should produce
        - hints: List of hints
        - solution_reference: Sample solution code
        - test_commands: Commands to verify the solution works
        - evaluation_criteria: What to check to determine if solution is correct
        """
        
        # Add retry logic for API calls
        max_retries = 3
        for attempt in range(max_retries):
            try:
                response = model.generate_content(prompt)
                response_text = response.text
                
                # Try multiple approaches to extract JSON
                try:
                    # First try: direct JSON parsing
                    question_data = json.loads(response_text)
                except json.JSONDecodeError:
                    # Second try: Look for JSON within code blocks
                    if "```json" in response_text:
                        parts = response_text.split("```json")
                        if len(parts) > 1:
                            json_text = parts[1].split("```")[0].strip()
                            question_data = json.loads(json_text)
                    elif "```" in response_text:
                        parts = response_text.split("```")
                        if len(parts) > 1:
                            json_text = parts[1].strip()
                            question_data = json.loads(json_text)
                    else:
                        # Third try: Look for JSON-like structure and clean it
                        # Remove any markdown or text surrounding potential JSON
                        cleaned_text = response_text.strip()
                        # Try to find the starting { and ending } of JSON
                        start_idx = cleaned_text.find('{')
                        end_idx = cleaned_text.rfind('}')
                        
                        if start_idx != -1 and end_idx != -1:
                            json_text = cleaned_text[start_idx:end_idx+1]
                            question_data = json.loads(json_text)
                        else:
                            # If all parsing attempts fail, use fallback
                            raise ValueError("Could not extract valid JSON from response")
                
                # Validate that all required fields are present
                required_fields = ["question", "requirements", "expected_output", 
                                  "solution_reference", "test_commands", "evaluation_criteria"]
                
                missing_fields = [field for field in required_fields if field not in question_data]
                if missing_fields:
                    for field in missing_fields:
                        question_data[field] = fallback_question[field]
                
                # Ensure hints are present (optional field)
                if "hints" not in question_data:
                    question_data["hints"] = fallback_question["hints"]
                    
                return question_data
                
            except Exception as e:
                print(f"Attempt {attempt+1} failed: {e}")
                if attempt == max_retries - 1:  # Last attempt
                    print(f"All attempts failed, using fallback question")
                    return fallback_question
                # Wait before retry (exponential backoff)
                import time
                time.sleep(2 ** attempt)  # 1, 2, 4 seconds
                
    except Exception as e:
        print(f"Error generating question: {e}")
        return fallback_question
        
def prepare_docker_environment(question_data):
    """Create a Docker environment for the user to solve the problem."""
    # Create a Dockerfile for ROS2 environment
    dockerfile_content = """
    FROM ros:humble
    
    # Install dependencies
    RUN apt-get update && apt-get install -y \
        python3-pip \
        vim \
        nano \
        git \
        && rm -rf /var/lib/apt/lists/*
        
    # Create workspace
    RUN mkdir -p /ros2_ws/src
    WORKDIR /ros2_ws
    
    # Add question details
    RUN mkdir -p /question
    WORKDIR /question
    
    # Keep container running
    CMD ["bash"]
    """
    
    # Save Dockerfile
    with open("Dockerfile.ros2practice", "w") as f:
        f.write(dockerfile_content)
    
    # Save question data
    with open("question.json", "w") as f:
        json.dump(question_data, f, indent=4)
    
    # Build Docker image
    subprocess.run(["docker", "build", "-t", "ros2practice", "-f", "Dockerfile.ros2practice", "."])
    
    # Create commands to start the container
    start_cmd = "docker run -it --name ros2practice_container -v $(pwd)/solution:/ros2_ws/src ros2practice"
    
    return start_cmd
def evaluate_solution(question_data):
    """Evaluate the user's solution against the reference solution."""
    try:
        # Debug log
        print("Starting evaluation process...")
        
        # Check if solution directory exists
        if not os.path.exists("solution"):
            print("Error: solution directory not found")
            return {
                "is_correct": False,
                "feedback": "Solution directory not found. Please create a ROS2 package inside the 'solution' directory.",
                "suggestions": ["Create a ROS2 package using 'ros2 pkg create' inside the solution directory"]
            }
            
        # List files in solution directory to debug
        print("Contents of solution directory:")
        for root, dirs, files in os.walk("solution"):
            for dir in dirs:
                print(f"  Dir: {os.path.join(root, dir)}")
            for file in files:
                print(f"  File: {os.path.join(root, file)}")
                
        # Create evaluation Dockerfile
        eval_dockerfile = """
        FROM ros:humble
        
        # Copy the solution
        COPY solution/ /ros2_ws/src/
        
        # Set up workspace
        WORKDIR /ros2_ws
        
        # Build the packages
        RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"
        RUN /bin/bash -c "source install/setup.bash"
        
        # Run tests based on test commands
        WORKDIR /ros2_ws
        """
        
        with open("Dockerfile.evaluation", "w") as f:
            f.write(eval_dockerfile)
        
        print("Evaluation Dockerfile created")
        
        # Build evaluation image
        build_process = subprocess.run(["docker", "build", "-t", "ros2practice_eval", "-f", "Dockerfile.evaluation", "."], 
                            capture_output=True, text=True)
        
        print(f"Docker build output: {build_process.stdout}")
        if build_process.returncode != 0:
            print(f"Docker build error: {build_process.stderr}")
            return {
                "is_correct": False,
                "feedback": f"Failed to build solution: {build_process.stderr}",
                "suggestions": ["Check that your ROS2 package builds correctly", 
                                "Verify package.xml and setup.py are configured correctly"]
            }
            
        print("Evaluation Docker image built successfully")
        
        # Create a network for containers to communicate
        network_name = "ros2_test_network"
        subprocess.run(["docker", "network", "create", "--driver", "bridge", network_name], 
                      capture_output=True, text=True)
        
        # Run tests from question data - each in a separate container
        results = []
        containers = []
        
        for i, test_cmd in enumerate(question_data.get("test_commands", [])):
            container_name = f"ros2practice_test_{i}"
            containers.append(container_name)
            
            print(f"Running test command: {test_cmd}")
            cmd = f"docker run -d --network {network_name} --name {container_name} ros2practice_eval /bin/bash -c 'source /opt/ros/humble/setup.bash && source install/setup.bash && {test_cmd}'"
            process = subprocess.run(cmd, shell=True, capture_output=True, text=True)
            
            # Store initial launch results
            launch_result = {
                "command": test_cmd,
                "launch_output": process.stdout,
                "launch_error": process.stderr,
                "launch_status": process.returncode
            }
            
            print(f"Container launch status: {process.returncode}")
            if process.returncode != 0:
                print(f"Container launch error: {process.stderr}")
        
        # Wait for containers to run a bit
        import time
        time.sleep(5)
        
        # Collect results from all containers
        for i, container_name in enumerate(containers):
            # Get logs from container
            cmd = f"docker logs {container_name}"
            process = subprocess.run(cmd, shell=True, capture_output=True, text=True)
            
            # Store results
            results.append({
                "command": question_data.get("test_commands", [])[i],
                "output": process.stdout,
                "error": process.stderr,
                "exit_code": 0 if "error" not in process.stderr.lower() else 1
            })
            
            print(f"Test output: {process.stdout}")
            if "error" in process.stderr.lower():
                print(f"Test error: {process.stderr}")
        
        # Clean up containers
        for container_name in containers:
            subprocess.run(["docker", "stop", container_name], capture_output=True)
            subprocess.run(["docker", "rm", container_name], capture_output=True)
        
        # Clean up network
        subprocess.run(["docker", "network", "rm", network_name], capture_output=True)
        
        # Create a manual evaluation result if API rate limited
        manual_evaluation = {
            "is_correct": all("error" not in result["error"].lower() for result in results),
            "feedback": "Your solution has been evaluated based on the test commands execution.",
            "suggestions": []
        }
        
        if manual_evaluation["is_correct"]:
            manual_evaluation["feedback"] += " All tests passed successfully!"
            manual_evaluation["suggestions"] = ["Great job! Your solution meets all the requirements."]
        else:
            manual_evaluation["feedback"] += " Some tests failed."
            manual_evaluation["suggestions"] = [
                "Check the test command outputs for error messages",
                "Verify that your implementation meets all the requirements",
                "Make sure your package builds correctly"
            ]
            
            # Add specific errors from test results
            for result in results:
                if "error" in result["error"].lower():
                    manual_evaluation["suggestions"].append(
                        f"Command '{result['command']}' failed with error: {result['error'][:100]}..."
                    )
        
        # Try using Gemini API but fall back to manual evaluation if it fails
        try:
            print("Attempting to use Gemini API for evaluation...")
            model = GenerativeModel('gemini-1.5-pro')
            
            eval_prompt = f"""
            Evaluate a ROS2 solution based on the following criteria and test results:
            
            Question: {question_data['question']}
            
            Expected output: {question_data['expected_output']}
            
            Evaluation criteria: {json.dumps(question_data['evaluation_criteria'])}
            
            Test results: {json.dumps(results)}
            
            Provide a detailed assessment of whether the solution is correct or not.
            If it's incorrect, explain what might be wrong and suggestions for improvement.
            Format your response as a JSON object with these fields:
            - is_correct: boolean
            - feedback: string
            - suggestions: list of strings
            """
            
            response = model.generate_content(eval_prompt)
            response_text = response.text.strip()
            
            print(f"Raw API response: {response_text}")
            
            # Try to extract JSON from the response with multiple methods
            try:
                # Direct JSON parsing
                eval_result = json.loads(response_text)
                print("Successfully parsed JSON directly")
            except json.JSONDecodeError as e:
                print(f"JSON decode error: {e}")
                # Look for JSON in code blocks
                if "```json" in response_text:
                    parts = response_text.split("```json")
                    json_text = parts[1].split("```")[0].strip()
                    eval_result = json.loads(json_text)
                    print("Successfully parsed JSON from code block")
                elif "```" in response_text:
                    parts = response_text.split("```")
                    json_text = parts[1].strip()
                    eval_result = json.loads(json_text)
                    print("Successfully parsed JSON from generic code block")
                else:
                    # Extract JSON by finding { and }
                    start_idx = response_text.find('{')
                    end_idx = response_text.rfind('}')
                    
                    if start_idx != -1 and end_idx != -1:
                        json_text = response_text[start_idx:end_idx+1]
                        print(f"Extracted potential JSON: {json_text}")
                        eval_result = json.loads(json_text)
                        print("Successfully parsed JSON by extracting {} content")
                    else:
                        print("All JSON parsing methods failed, using manual evaluation")
                        return manual_evaluation
            
            # Ensure required fields exist
            required_fields = ["is_correct", "feedback", "suggestions"]
            for field in required_fields:
                if field not in eval_result:
                    eval_result[field] = manual_evaluation[field]
                    
            return eval_result
            
        except Exception as e:
            print(f"API evaluation failed: {e}")
            print("Falling back to manual evaluation")
            return manual_evaluation
    
    except Exception as e:
        print(f"Error in evaluation process: {e}")
        import traceback
        traceback.print_exc()
        return {
            "is_correct": False,
            "feedback": f"Evaluation system error: {str(e)}",
            "suggestions": [
                "Please check that your solution directory contains a valid ROS2 package",
                "Try building your package locally with 'colcon build' before submitting"
            ]
        }

# Gradio interface
def ros2_platform(category, user_action):
    """Main function for the Gradio interface."""
    if user_action == "Generate Question":
        question_data = generate_question(category)
        docker_cmd = prepare_docker_environment(question_data)
        
        return (
            question_data.get("question", ""),
            "\n".join(question_data.get("requirements", [])),
            question_data.get("expected_output", ""),
            "\n".join(question_data.get("hints", [])),
            docker_cmd,
            "Question generated! Use the Docker command to start your environment."
        )
    
    elif user_action == "Submit Solution":
        # Load the last generated question
        try:
            with open("question.json", "r") as f:
                question_data = json.load(f)
                
            evaluation = evaluate_solution(question_data)
            
            status = "CORRECT" if evaluation.get("is_correct", False) else "INCORRECT"
            feedback = evaluation.get("feedback", "No feedback available.")
            suggestions = "\n".join(evaluation.get("suggestions", ["No suggestions available."]))
            
            return (
                question_data.get("question", ""),
                "\n".join(question_data.get("requirements", [])),
                question_data.get("expected_output", ""),
                "\n".join(question_data.get("hints", [])),
                "",
                f"Solution Status: {status}\n\nFeedback: {feedback}\n\nSuggestions:\n{suggestions}"
            )
        
        except Exception as e:
            return (
                "", "", "", "", "",
                f"Error evaluating solution: {str(e)}"
            )
    
    return "", "", "", "", "", "Please select an action."

# Create Gradio interface
with gr.Blocks(title="ROS2 Practice Platform") as app:
    gr.Markdown("# ROS2 Practice Platform")
    gr.Markdown("Generate practice questions for ROS2 and practice in a Docker environment")
    
    with gr.Row():
        category = gr.Dropdown(
            choices=QUESTION_CATEGORIES,
            label="Question Category",
            value="basic_concepts"
        )
        action = gr.Radio(
            choices=["Generate Question", "Submit Solution"],
            label="Action",
            value="Generate Question"
        )
    
    with gr.Row():
        submit_btn = gr.Button("Submit")
    
    with gr.Row():
        question = gr.Textbox(label="Question", lines=5, interactive=False)
    
    with gr.Row():
        col1, col2 = gr.Column(), gr.Column()
        
        with col1:
            requirements = gr.Textbox(label="Requirements", lines=5, interactive=False)
            expected = gr.Textbox(label="Expected Output", lines=3, interactive=False)
        
        with col2:
            hints = gr.Textbox(label="Hints", lines=5, interactive=False)
            docker_cmd = gr.Textbox(label="Docker Command", lines=1, interactive=False)
    
    with gr.Row():
        result = gr.Textbox(label="Result", lines=10, interactive=False)
    
    submit_btn.click(
        ros2_platform,
        inputs=[category, action],
        outputs=[question, requirements, expected, hints, docker_cmd, result]
    )

if __name__ == "__main__":
    app.launch()
