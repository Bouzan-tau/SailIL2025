import subprocess
import time
import socket

command_to_script = {
    "S": "/home/sail/Desktop/danel_gadya_sail_2025_git/bashes4/run_environment_ros2.sh",
    "SK": "/home/sail/Desktop/danel_gadya_sail_2025_git/bashes4/kill_environment_ros2.sh",
    "1": "/home/sail/Desktop/danel_gadya_sail_2025_git/bashes4/task1_run_ros2.sh",
    "1K": "/home/sail/Desktop/danel_gadya_sail_2025_git/bashes4/task1_kill_ros2.sh",
    "2": "/home/sail/Desktop/danel_gadya_sail_2025_git/bashes4/task2_run_ros2.sh",
    "2B": "/home/sail/Desktop/danel_gadya_sail_2025_git/bashes4/task2_run_ros2B.sh",
    "2BK": "/home/sail/Desktop/danel_gadya_sail_2025_git/bashes4/task2_kill_ros2B.sh",
    "2K": "/home/sail/Desktop/danel_gadya_sail_2025_git/bashes4/task2_kill_ros2.sh",
    "3": "/home/sail/Desktop/danel_gadya_sail_2025_git/bashes4/task3_run_ros2.sh",
    "3K": "/home/sail/Desktop/danel_gadya_sail_2025_git/bashes4/task3_kill_ros2.sh",
}

def run_bash_script(script_path):
    try:
        subprocess.run(["bash", script_path])
    except Exception as e:
        print(f"❌ Failed to run {script_path}: {e}")

def is_server_ready(host="localhost", port=5000):
    try:
        with socket.create_connection((host, port), timeout=1):
            return True
    except Exception:
        return False

def main():
    print("Welcome! Available commands: S, SK, 1, 1K, 2, 2B 2K, 3, 3K")

    stage = "awaiting_S"
    current_task = None

    while True:
        cmd = input("\n👉 Which Bash do you want to start? (type 'exit' to quit): ").strip().upper()

        if cmd == "EXIT":
            print("👋 Exiting. Bye!")
            break

        # first stage: must starting with S
        if stage == "awaiting_S":
            if cmd == "S":
                print("🚀 Running script for 'S'... (environment setup)")
                subprocess.Popen(["bash", command_to_script[cmd]])

                print("⏳ Waiting for server to be ready...")
                while not is_server_ready():
                    time.sleep(0.5)

                print("✅ Server is ready! You can continue.")
                stage = "task_selection"  # next stage
            else:
                print("⚠️ You must start with 'S' to set up the environment!")

        # second stage: choosing task or closing the environment
        elif stage == "task_selection":
            if cmd in ["1", "2", "3", "2B"]:
                print(f"🚀 Running task {cmd}...")
                run_bash_script(command_to_script[cmd])
                current_task = cmd
                stage = "awaiting_task_kill"  # if choosing a task - we must kill it
            elif cmd == "SK":
                print("🛑 Killing environment (SK)...")
                run_bash_script(command_to_script[cmd])
                print("👋 Environment closed. Exiting.")
                break
            else:
                print("⚠️ You can only run task '1', '2', '3' or 'SK' at this stage!")

        # third stage: must killing the task
        elif stage == "awaiting_task_kill":
            expected_kill_command = current_task + "K"
            if cmd == expected_kill_command:
                print(f"🛑 Killing task {current_task}...")
                run_bash_script(command_to_script[cmd])
                current_task = None
                stage = "task_selection" # choosing task or closing the environment
            else:
                print(f"⚠️ You must kill the current task with '{expected_kill_command}' before doing anything else!")

if __name__ == "__main__":
    main()
