import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import subprocess
import os
import signal


class ProgramManager(Node):
    def __init__(self):
        super().__init__('program_manager')

        # プロセス管理用の辞書
        self.processes = {
            "regular_rosbag": None,
            "experiment_rosbag": None,
            "plotjuggler": None,
        }

        # サービスの作成
        self.srv_set_regular_rosbag = self.create_service(SetBool, 'SetRegularRosbag', self.set_regular_rosbag_callback)
        self.srv_set_experiment_rosbag = self.create_service(SetBool, 'SetExperimentRosbag', self.set_experiment_rosbag_callback)
        self.srv_set_plotjuggler = self.create_service(SetBool, 'SetPlotJuggler', self.set_plotjuggler_callback)

        self.get_logger().info("ProgramManager is ready to handle services.")

    def start_program(self, key, command):
        """
        指定されたプログラムを開始する。
        """
        if self.processes[key] is not None:
            self.get_logger().warn(f"{key} is already running.")
            return False, f"{key} is already running."

        try:
            # プロセスを非同期で開始
            process = subprocess.Popen(command, shell=True, preexec_fn=os.setsid)
            self.processes[key] = process
            self.get_logger().info(f"Started {key}. Command: {command}")
            return True, f"Started {key}."
        except Exception as e:
            self.get_logger().error(f"Failed to start {key}: {e}")
            return False, f"Failed to start {key}: {e}"

    def stop_program(self, key):
        """
        指定されたプログラムを停止する。
        """
        process = self.processes.get(key)
        if process is None:
            self.get_logger().warn(f"{key} is not running.")
            return False, f"{key} is not running."

        try:
            # プロセスを終了
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            process.wait()
            self.processes[key] = None
            self.get_logger().info(f"Stopped {key}.")
            return True, f"Stopped {key}."
        except Exception as e:
            self.get_logger().error(f"Failed to stop {key}: {e}")
            return False, f"Failed to stop {key}: {e}"

    def set_regular_rosbag_callback(self, request, response):
        """
        'SetRegularRosbag'サービスのコールバック。
        """
        if request.data:  # trueの場合、開始
            success, message = self.start_program("regular_rosbag", f"{os.path.expanduser('~')}/a.sh")
        else:  # falseの場合、停止
            success, message = self.stop_program("regular_rosbag")

        response.success = success
        response.message = message
        return response

    def set_experiment_rosbag_callback(self, request, response):
        """
        'SetExperimentRosbag'サービスのコールバック。
        """
        if request.data:  # trueの場合、開始
            success, message = self.start_program("experiment_rosbag", "ros2 bag record -s mcap -a")
        else:  # falseの場合、停止
            success, message = self.stop_program("experiment_rosbag")

        response.success = success
        response.message = message
        return response

    def set_plotjuggler_callback(self, request, response):
        """
        'SetPlotJuggler'サービスのコールバック。
        """
        if request.data:  # trueの場合、開始
            success, message = self.start_program("plotjuggler", "ros2 run plotjuggler plotjuggler")
        else:  # falseの場合、停止
            success, message = self.stop_program("plotjuggler")

        response.success = success
        response.message = message
        return response


def main(args=None):
    rclpy.init(args=args)
    program_manager = ProgramManager()

    try:
        rclpy.spin(program_manager)
    except KeyboardInterrupt:
        pass
    finally:
        # 終了時にすべてのプロセスを停止
        for key in program_manager.processes.keys():
            program_manager.stop_program(key)
        program_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

