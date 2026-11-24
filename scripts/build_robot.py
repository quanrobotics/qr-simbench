"""Copy the upstream TurtleBot3 Waffle Gazebo SDF (nav2_minimal_tb3_sim, Apache-2.0) and add a
PosePublisher so the simulator's true model pose is available for ground-truth metrics.

Usage (ROS env active): python scripts/build_robot.py
"""

import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = Path(os.environ["CONDA_PREFIX"]) / "share/nav2_minimal_tb3_sim/urdf/gz_waffle.sdf.xacro"
OUT = ROOT / "simulation/robots/gz_waffle_gt.sdf.xacro"

PLUGIN = """
      <!-- Added by QR-SimBench (scripts/build_robot.py): true model pose for ground-truth metrics. -->
      <plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">
        <publish_model_pose>true</publish_model_pose>
        <publish_link_pose>false</publish_link_pose>
        <publish_collision_pose>false</publish_collision_pose>
        <publish_visual_pose>false</publish_visual_pose>
        <publish_nested_model_pose>false</publish_nested_model_pose>
        <use_pose_vector_msg>false</use_pose_vector_msg>
        <update_frequency>30</update_frequency>
      </plugin>
"""

text = SRC.read_text()
# Drop the unused RealSense depth camera: it renders every frame and costs GPU time in fleet runs.
_a = text.index('<sensor name="intel_realsense_r200_depth"')
text = text[:_a] + text[text.index("</sensor>", _a) + len("</sensor>"):]
idx = text.rindex("</model>")
header = f"<!-- Modified copy of {SRC.name} from nav2_minimal_tb3_sim (Apache-2.0). Generated; do not edit. -->\n"
first_nl = text.index("\n") + 1  # keep the XML declaration first
OUT.write_text(text[:first_nl] + header + text[first_nl:idx] + PLUGIN + "    " + text[idx:])
print(f"wrote {OUT.relative_to(ROOT)}")

# Demo variant: two robot-mounted cameras (chase and top-down follow) that save frames for video.
CAMS = """
      <link name="qr_demo_cameras">
        <pose>0 0 0 0 0 0</pose>
        <inertial><mass>0.001</mass><inertia><ixx>1e-6</ixx><iyy>1e-6</iyy><izz>1e-6</izz></inertia></inertial>
        <sensor name="chase" type="camera"><pose>-1.5 0 1.3 0 0.45 0</pose><always_on>1</always_on>
          <update_rate>15</update_rate><camera><horizontal_fov>1.4</horizontal_fov>
          <image><width>960</width><height>540</height></image><clip><near>0.05</near><far>40</far></clip>
          </camera><topic>$(arg namespace)/qr_demo/chase</topic></sensor>
        <sensor name="follow_top" type="camera"><pose>0 0 4.0 0 1.5707 0</pose><always_on>1</always_on>
          <update_rate>15</update_rate><camera><horizontal_fov>1.1</horizontal_fov>
          <image><width>960</width><height>540</height></image><clip><near>0.1</near><far>40</far></clip>
          </camera><topic>$(arg namespace)/qr_demo/follow_top</topic></sensor>
      </link>
      <joint name="qr_demo_cameras_joint" type="fixed"><parent>base_footprint</parent><child>qr_demo_cameras</child></joint>
"""
DEMO = OUT.with_name("gz_waffle_gt_demo.sdf.xacro")
DEMO.write_text(OUT.read_text().replace(PLUGIN, PLUGIN + CAMS))
print(f"wrote {DEMO.relative_to(ROOT)}")
