from omni.isaac.examples.base_sample import BaseSample
import numpy as np
# Can be used to create a new cube or to point to an already existing cube in stage.
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.utils import (
    stage, 
    nucleus, 
    prims,
    rotations,
    )
import carb
from pxr import Gf, UsdGeom
from omni.isaac.franka import Franka
from omni.isaac.franka.controllers import PickPlaceController


BACKGROUND_STAGE_PATH = "/background"
BACKGROUND_USD_PATH = "/Isaac/Environments/Simple_Room/simple_room.usd"
FRANKA_STAGE_PATH = "/Franka"
FRANKA_USD_PATH = "/Isaac/Robots/Franka/franka_alt_fingers.usd"


class HelloWorld(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        return

    def setup_scene(self):
        world = self.get_world()
        assets_root_path = nucleus.get_assets_root_path()
        if assets_root_path is None:
            # Use carb to log warnings, errors and infos in your application (shown on terminal)
            carb.log_error("Could not find nucleus server with /Isaac folder")
        
        
        stage.add_reference_to_stage(
        assets_root_path + BACKGROUND_USD_PATH, BACKGROUND_STAGE_PATH
        )

        franka = world.scene.add(
            Franka(
                prim_path="/World/Fancy_Franka", 
                name="fancy_franka",
                position=np.array([0, 0, 0]),
                orientation=rotations.gf_rotation_to_np_array(Gf.Rotation(Gf.Vec3d(0, 0, 1), 90)),
                ))

        # world.scene.add_default_ground_plane()
        fancy_cube = world.scene.add(
            DynamicCuboid(
                prim_path="/World/random_cube", # The prim path of the cube in the USD stage
                name="fancy_cube", # The unique name used to retrieve the object from the scene later on
                position=np.array([0.3, 0.3, 0.04101]), # Using the current stage units which is in meters by default.
                scale=np.array([0.0515, 0.0515, 0.0515]), # most arguments accept mainly numpy arrays.
                color=np.array([0, 0, 1.0]), # RGB channels, going from 0-1
            ))
        return
    
    async def setup_post_load(self):
        self._world = self.get_world()
        self._franka = self._world.scene.get_object("fancy_franka")
        self._fancy_cube = self._world.scene.get_object("fancy_cube")

        self._controller = PickPlaceController(
            name="pick_place_controller",
            gripper=self._franka.gripper,
            robot_articulation=self._franka,
        )
        self._world.add_physics_callback("sim_step", callback_fn=self.physics_step)
        self._franka.gripper.set_joint_positions(self._franka.gripper.joint_opened_positions)
        await self._world.play_async()
        return
    
    async def setup_post_reset(self):
        self._controller.reset()
        self._franka.gripper.set_joint_positions(self._franka.gripper.joint_opened_positions)
        await self._world.play_async()
        return 
    
    def physics_step(self, step_size):
        cube_position, cube_orientation = self._fancy_cube.get_world_pose()
        fraka_position,franka_orientation = self._franka.get_world_pose()
        print("cube_position is : " + str(cube_position))
        print("cube_orientation is : " + str(cube_orientation))
        goal_position = np.array([-0.3, 0.3, 0.07101])
        current_joint_positions = self._franka.get_joint_positions()
        actions = self._controller.forward(
            picking_position=cube_position,
            placing_position=goal_position,
            current_joint_positions=current_joint_positions,
        )
        self._franka.apply_action(actions)
        if self._controller.is_done():
            self._world.pause()
        return