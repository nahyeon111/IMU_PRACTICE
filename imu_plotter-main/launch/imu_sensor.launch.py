import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        # ✅ IMU 데이터 퍼블리싱 노드 실행
        launch_ros.actions.Node(
            package='imu_plotter',
            executable='imu_publisher',
            name='imu_publisher',
            output='screen'
        ),

        # ✅ 중력 제거 및 비중력 가속도 계산 노드 실행
        launch_ros.actions.Node(
            package='imu_plotter',
            executable='imu_gravity_processor',  # ✅ 실행 파일 이름 확인!
            name='imu_gravity_processor',
            output='screen'
        ),

        # ✅ 중력 제거 및 비중력 가속도 계산 노드 실행
        launch_ros.actions.Node(
            package='imu_plotter',
            executable='imu_orientation_calculator',  # ✅ 실행 파일 이름 확인!
            name='imu_orientation_calculator',
            output='screen'
        ),

        launch_ros.actions.Node(
            package='imu_plotter',
            executable='imu_orientation_calculator2',  # ✅ 실행 파일 이름 확인!
            name='imu_orientation_calculator2',
            output='screen'
        ),

          launch_ros.actions.Node(
            package='imu_plotter',
            executable='imu_orientation_calculator3',  # ✅ 실행 파일 이름 확인!
            name='imu_orientation_calculator3',
            output='screen'
        ),
    ])
