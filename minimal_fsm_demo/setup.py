from setuptools import find_packages, setup

package_name = "minimal_fsm_demo"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/yellow_line_fsm_demo.launch.py"]),
        (f"share/{package_name}/launch", ["launch/yellow_line_with_vision.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="minimal_fsm_demo",
    maintainer_email="opensource@xiaomi.com",
    description="Minimal top-level FSM demo for cyberdog sim + locomotion.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "yellow_line_fsm_demo = minimal_fsm_demo.yellow_line_fsm_demo:main",
            "ball_color_detector = minimal_fsm_demo.ball_color_detector:main",
            "motion_ctrl = minimal_fsm_demo.motion_ctrl:main",
            "tf_monitor = minimal_fsm_demo.tf_monitor:main",
            "ball_push = minimal_fsm_demo.ball_push:main",
            "turn_demo = minimal_fsm_demo.turn_demo:main",
            "goto_target = minimal_fsm_demo.goto_target:main",
            "turn_pid_demo = minimal_fsm_demo.turn_pid_demo:main",
            "stage3_sprint = minimal_fsm_demo.stage3_sprint:main",
            "forward_turn_seek_orange = minimal_fsm_demo.forward_turn_seek_orange:main",
            "stage2_orange_scan = minimal_fsm_demo.stage2_orange_scan:main",
            "stage2_grid_hunt = minimal_fsm_demo.stage2_grid_hunt:main",
        ],
    },
)
