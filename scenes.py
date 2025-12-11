"""Scene factory helpers.

Provide functions to create common demo scenes. Each factory returns a
tuple (scene, franka, blocks_state, end_effector) to be used by demos.
"""
from typing import Any, Dict, Tuple
import random
import time
random.seed(time.time())

import numpy as np
import genesis as gs
from robot_adapter import RobotAdapter


def _build_base_scene(camera_pos=(3, -1, 1.5), camera_lookat=(0.0, 0.0, 0.5)) -> gs.Scene:
    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=0.01, substeps=8),
        viewer_options=gs.options.ViewerOptions(
            camera_pos=camera_pos,
            camera_lookat=camera_lookat,
            camera_fov=30,
            max_FPS=60,
        ),
        show_viewer=True,
    )
    return scene

def _elevate_robot_base(franka: Any) -> None:
    """Slightly raise robot base to avoid initial collisions."""
    base_pos = np.asarray(franka.get_pos(), dtype=float)
    new_pos = base_pos.copy()
    new_pos[2] += 0.01
    franka.set_pos(new_pos) 

def _rand_xy(base, noise=0.05):
        dx = random.uniform(-noise, noise)
        dy = random.uniform(-noise, noise)
        return (base[0] + dx, base[1] + dy, base[2])

def add(pos, delta):
    return tuple(a + b for a, b in zip(pos, delta))

def create_scene_6blocks() -> Tuple[Any, Any, Dict[str, Any], Dict[str, Any]]:
    """Create the default demo scene (layout 1).

    Returns:
        scene, franka_adapter, blocks_state, end_effector
    """
    scene = _build_base_scene()

    # basic geometry
    plane = scene.add_entity(gs.morphs.Plane())
    # add some random noise up to 5 cm in x/y

    # TODO: Add back random position for red
    posR = (0.65, 0.0, 0.02)
    posG = _rand_xy((0.65, 0.2, 0.02))
    posB = _rand_xy((0.65, 0.4, 0.02))
    posY = _rand_xy((0.45, 0.0, 0.02))
    posM = _rand_xy((0.45, 0.2, 0.02))
    posC = _rand_xy((0.45, 0.4, 0.02))

    cubeR = scene.add_entity(
        gs.morphs.Box(size=(0.04, 0.04, 0.04), pos= posR),
        surface=gs.options.surfaces.Plastic(color=(1.0, 0.0, 0.0)),
    )
    cubeG = scene.add_entity(
        gs.morphs.Box(size=(0.04, 0.04, 0.04), pos= posG),
        surface=gs.options.surfaces.Plastic(color=(0.0, 1.0, 0.0)),
    )
    cubeB = scene.add_entity(
        gs.morphs.Box(size=(0.04, 0.04, 0.04), pos= posB),
        surface=gs.options.surfaces.Plastic(color=(0.0, 0.0, 1.0)),
    )
    cubeY = scene.add_entity(
        gs.morphs.Box(size=(0.04, 0.04, 0.04), pos=posY),
        surface=gs.options.surfaces.Plastic(color=(1.0, 1.0, 0.0)),
    )
    cubeM = scene.add_entity(
        gs.morphs.Box(size=(0.04, 0.04, 0.04), pos=posM),
        surface=gs.options.surfaces.Plastic(color=(1.0, 0, 1.0)),
    )

    cubeC = scene.add_entity(
        gs.morphs.Box(size=(0.04, 0.04, 0.04), pos=posC),
        surface=gs.options.surfaces.Plastic(color=(0, 1.0, 1.0)),
    )

    franka_raw = scene.add_entity(gs.morphs.MJCF(file="xml/franka_emika_panda/panda.xml"))
    franka = RobotAdapter(franka_raw, scene)

    # build scene (construct physics/visuals)
    scene.build()

    # initial robot pose (7 arm joints + 2 gripper fingers)
    franka.set_qpos(np.array([0.0, -0.5, -0.2, -1.0, 0.0, 1.00, 0.5, 0.02, 0.02]))

    # slightly raise robot base to avoid initial collisions
    _elevate_robot_base(franka)

    blocks_state: Dict[str, Any] = {"r": cubeR, "g": cubeG, "b": cubeB, "y": cubeY, "m": cubeM, "c": cubeC}

    return scene, franka, blocks_state


def create_scene_stacked() -> Tuple[Any, Any, Dict[str, Any], Dict[str, Any]]:
    """Create an alternative demo scene (layout 2) with cube positions. one on top of the other."""
    scene = _build_base_scene(camera_pos=(2.5, -1.2, 1.2), camera_lookat=(0.6, 0.0, 0.2))

    plane = scene.add_entity(gs.morphs.Plane())

    # slightly different positions
    startx, starty, _ = _rand_xy((0.45, 0.0, 0.02), noise=0.2) 
    cubeR = scene.add_entity(
        gs.morphs.Box(size=(0.04, 0.04, 0.04), pos=(startx, starty, 0.02)),
        surface=gs.options.surfaces.Plastic(color=(1.0, 0.0, 0.0)),
    )
    cubeG = scene.add_entity(
        gs.morphs.Box(size=(0.04, 0.04, 0.04), pos=(startx, starty, 0.06)),
        surface=gs.options.surfaces.Plastic(color=(0.0, 1.0, 0.0)),
    )
    cubeB = scene.add_entity(
        gs.morphs.Box(size=(0.04, 0.04, 0.04), pos=(startx, starty, 0.10)),
        surface=gs.options.surfaces.Plastic(color=(0.0, 0.0, 1.0)),
    )
    cubeY = scene.add_entity(
        gs.morphs.Box(size=(0.04, 0.04, 0.04), pos=(startx, starty, 0.14)),
        surface=gs.options.surfaces.Plastic(color=(1.0, 1.0, 0.0)),
    )

    cubeM = scene.add_entity(
        gs.morphs.Box(size=(0.04, 0.04, 0.04), pos=(startx, starty, 0.18)),
        surface=gs.options.surfaces.Plastic(color=(1.0, 0, 1.0)),
    )

    cubeC = scene.add_entity(
        gs.morphs.Box(size=(0.04, 0.04, 0.04), pos=(startx, starty, 0.22)),
        surface=gs.options.surfaces.Plastic(color=(0, 1.0, 1.0)),
    )

    franka_raw = scene.add_entity(gs.morphs.MJCF(file="xml/franka_emika_panda/panda.xml"))
    franka = RobotAdapter(franka_raw, scene)
    scene.build()

    franka.set_qpos(np.array([0.0, -0.5, -0.2, -1.0, 0.0, 1.00, 0.5, 0.02, 0.02]))

    _elevate_robot_base(franka)

    blocks_state: Dict[str, Any] = {"r": cubeR, "g": cubeG, "b": cubeB, "y": cubeY, "m": cubeM, "c": cubeC}

    return scene, franka, blocks_state



def create_scene_special_1() -> Tuple[Any, Any, Dict[str, Any],  Dict[str, Any]]:
    """Create the default demo scene (layout 1) but for the 1st special design

    Returns:
        scene, franka_adapter, blocks_state, slots_state
    """
    scene = _build_base_scene()

    # basic geometry
    plane = scene.add_entity(gs.morphs.Plane())
    # add some random noise up to 5 cm in x/y

    # Define random position for blocks
    # posR = ((0.65, 0.0, 0.02))
    posR = _rand_xy((0.65, 0.0, 0.02))
    posG = _rand_xy((0.65, 0.2, 0.02))
    posB = _rand_xy((0.65, 0.4, 0.02))
    posY = _rand_xy((0.45, 0.0, 0.02))
    posM = _rand_xy((0.45, 0.2, 0.02))
    posC = _rand_xy((0.45, 0.4, 0.02))

    cubeR = scene.add_entity(
        gs.morphs.Box(size=(0.04, 0.04, 0.04), pos= posR),
        surface=gs.options.surfaces.Plastic(color=(1.0, 0.0, 0.0)),
    )
    cubeG = scene.add_entity(
        gs.morphs.Box(size=(0.04, 0.04, 0.04), pos= posG),
        surface=gs.options.surfaces.Plastic(color=(0.0, 1.0, 0.0)),
    )
    cubeB = scene.add_entity(
        gs.morphs.Box(size=(0.04, 0.04, 0.04), pos= posB),
        surface=gs.options.surfaces.Plastic(color=(0.0, 0.0, 1.0)),
    )
    cubeY = scene.add_entity(
        gs.morphs.Box(size=(0.04, 0.04, 0.04), pos=posY),
        surface=gs.options.surfaces.Plastic(color=(1.0, 1.0, 0.0)),
    )
    cubeM = scene.add_entity(
        gs.morphs.Box(size=(0.04, 0.04, 0.04), pos=posM),
        surface=gs.options.surfaces.Plastic(color=(1.0, 0, 1.0)),
    )

    cubeC = scene.add_entity(
        gs.morphs.Box(size=(0.04, 0.04, 0.04), pos=posC),
        surface=gs.options.surfaces.Plastic(color=(0, 1.0, 1.0)),
    )

    # Define position of slots
    # Define all slots based on random first position
    # slot1_pos = ((0.65, 0.0, 0.02))
    slot1_pos = _rand_xy((0.45, -0.25, 0.02))
    slot2_pos = add(slot1_pos, (0.04, 0.0, 0.0))
    slot3_pos = add(slot1_pos, (0.0, 0.04, 0.0))
    slot4_pos = add(slot2_pos, (0.04, 0.04, 0.0))
    slot5_pos = add(slot3_pos, (0.04, 0.04, 0.0))
    slot6_pos = add(slot5_pos, (0.04, 0.0, 0.0))
    
    # slot1 = scene.add_entity(
    #     gs.morphs.Box(size=(0.04, 0.04, 0.00), pos= slot1_pos),
    #     surface=gs.options.surfaces.Plastic(color=(0.0, 0.0, 0.0)),
    # )
    # slot2 = scene.add_entity(
    #     gs.morphs.Box(size=(0.04, 0.04, 0.00), pos= slot2_pos),
    #     surface=gs.options.surfaces.Plastic(color=(0.0, 0.0, 0.0)),
    # )
    # slot3 = scene.add_entity(
    #     gs.morphs.Box(size=(0.04, 0.04, 0.00), pos= slot3_pos),
    #     surface=gs.options.surfaces.Plastic(color=(0.0, 0.0, 0.0)),
    # )
    # slot4 = scene.add_entity(
    #     gs.morphs.Box(size=(0.04, 0.04, 0.00), pos=slot4_pos),
    #     surface=gs.options.surfaces.Plastic(color=(0.0, 0.0, 0.0)),
    # )
    # slot5 = scene.add_entity(
    #     gs.morphs.Box(size=(0.04, 0.04, 0.00), pos=slot5_pos),
    #     surface=gs.options.surfaces.Plastic(color=(0.0, 0, 0.0)),
    # )
    # slot6 = scene.add_entity(
    #     gs.morphs.Box(size=(0.04, 0.04, 0.00), pos=slot6_pos),
    #     surface=gs.options.surfaces.Plastic(color=(0, 0.0, 0.0)),
    # )

    # Define position of Franka
    franka_raw = scene.add_entity(gs.morphs.MJCF(file="xml/franka_emika_panda/panda.xml"))
    franka = RobotAdapter(franka_raw, scene)

    # build scene (construct physics/visuals)
    scene.build()

    # initial robot pose (7 arm joints + 2 gripper fingers)
    franka.set_qpos(np.array([0.0, -0.5, -0.2, -1.0, 0.0, 1.00, 0.5, 0.02, 0.02]))

    # slightly raise robot base to avoid initial collisions
    _elevate_robot_base(franka)

    blocks_state: Dict[str, Any] = {"r": cubeR, "g": cubeG, "b": cubeB, "y": cubeY, "m": cubeM, "c": cubeC}

    slots_state: Dict[str, Any] = {"1": slot1_pos, "2": slot2_pos, "3": slot3_pos, "4": slot4_pos, "5": slot5_pos, "6": slot6_pos}

    return scene, franka, blocks_state, slots_state

def create_scene_special_2() -> Tuple[Any, Any, Dict[str, Any],  Dict[str, Any]]:
    """Create the default demo scene (layout 1) but for the 2nd special design

    Returns:
        scene, franka_adapter, blocks_state, slots_state
    """
    scene = _build_base_scene()

    # basic geometry
    plane = scene.add_entity(gs.morphs.Plane())
    # add some random noise up to 5 cm in x/y

    # Define random position for blocks
    posR = _rand_xy((0.65, 0.0, 0.02))
    posG = _rand_xy((0.65, 0.2, 0.02))
    posB = _rand_xy((0.65, 0.4, 0.02))
    posY = _rand_xy((0.45, 0.0, 0.02))
    posM = _rand_xy((0.45, 0.2, 0.02))
    posC = _rand_xy((0.45, 0.4, 0.02))
    # Add 4 additional blocks for second and third layer
    posO = _rand_xy((0.45, -0.2, 0.02))
    posP = _rand_xy((0.45, -0.4, 0.02))
    posBr = _rand_xy((0.65, -0.2, 0.02))
    posW = _rand_xy((0.65, -0.4, 0.02))

    cubeR = scene.add_entity(
        gs.morphs.Box(size=(0.04, 0.04, 0.04), pos= posR),
        surface=gs.options.surfaces.Plastic(color=(1.0, 0.0, 0.0)),
    )
    cubeG = scene.add_entity(
        gs.morphs.Box(size=(0.04, 0.04, 0.04), pos= posG),
        surface=gs.options.surfaces.Plastic(color=(0.0, 1.0, 0.0)),
    )
    cubeB = scene.add_entity(
        gs.morphs.Box(size=(0.04, 0.04, 0.04), pos= posB),
        surface=gs.options.surfaces.Plastic(color=(0.0, 0.0, 1.0)),
    )
    cubeY = scene.add_entity(
        gs.morphs.Box(size=(0.04, 0.04, 0.04), pos=posY),
        surface=gs.options.surfaces.Plastic(color=(1.0, 1.0, 0.0)),
    )
    cubeM = scene.add_entity(
        gs.morphs.Box(size=(0.04, 0.04, 0.04), pos=posM),
        surface=gs.options.surfaces.Plastic(color=(1.0, 0, 1.0)),
    )
    cubeC = scene.add_entity(
        gs.morphs.Box(size=(0.04, 0.04, 0.04), pos=posC),
        surface=gs.options.surfaces.Plastic(color=(0.0, 1.0, 1.0)),
    )

    cubeO = scene.add_entity(
        gs.morphs.Box(size=(0.04, 0.04, 0.04), pos= posO),
        surface=gs.options.surfaces.Plastic(color=(1.0, 0.647, 0.0)),
    )
    cubeP = scene.add_entity(
        gs.morphs.Box(size=(0.04, 0.04, 0.04), pos= posP),
        surface=gs.options.surfaces.Plastic(color=(1.0, 0.753, 0.796)),
    )
    cubeBr = scene.add_entity(
        gs.morphs.Box(size=(0.04, 0.04, 0.04), pos= posBr),
        surface=gs.options.surfaces.Plastic(color=(0.647, 0.165, 0.165)),
    )
    cubeW = scene.add_entity(
        gs.morphs.Box(size=(0.04, 0.04, 0.04), pos= posW),
        surface=gs.options.surfaces.Plastic(color=(1.0, 1.0, 1.0)),
    )

    # Define position of slots
    # Define all slots based on random first position
    # First layer
    slot11_pos = _rand_xy((0.45, -0.25, 0.02))
    slot12_pos = add(slot11_pos, (0.04, 0.0, 0.0))
    slot13_pos = add(slot11_pos, (0.0, 0.04, 0.0))
    slot14_pos = add(slot11_pos, (0.04, 0.04, 0.0))
    slot15_pos = add(slot12_pos, (0.04, 0.00, 0.0))
    slot16_pos = add(slot13_pos, (0.00, 0.04, 0.0))
    # Second layer
    slot21_pos = add(slot11_pos, (0.0, 0.0, 0.04))
    slot22_pos = add(slot12_pos, (0.0, 0.0, 0.04))
    slot23_pos = add(slot13_pos, (0.0, 0.0, 0.04))
    # Third layer
    slot31_pos = add(slot21_pos, (0.0, 0.0, 0.04))

    # # First layer
    # slot11 = scene.add_entity(
    #     gs.morphs.Box(size=(0.04, 0.04, 0.00), pos= slot11_pos),
    #     surface=gs.options.surfaces.Plastic(color=(0.0, 0.0, 0.0)),
    # )
    # slot12 = scene.add_entity(
    #     gs.morphs.Box(size=(0.04, 0.04, 0.00), pos= slot12_pos),
    #     surface=gs.options.surfaces.Plastic(color=(0.0, 0.0, 0.0)),
    # )
    # slot13 = scene.add_entity(
    #     gs.morphs.Box(size=(0.04, 0.04, 0.00), pos= slot13_pos),
    #     surface=gs.options.surfaces.Plastic(color=(0.0, 0.0, 0.0)),
    # )
    # slot14 = scene.add_entity(
    #     gs.morphs.Box(size=(0.04, 0.04, 0.00), pos=slot14_pos),
    #     surface=gs.options.surfaces.Plastic(color=(0.0, 0.0, 0.0)),
    # )
    # slot15 = scene.add_entity(
    #     gs.morphs.Box(size=(0.04, 0.04, 0.00), pos=slot15_pos),
    #     surface=gs.options.surfaces.Plastic(color=(0.0, 0, 0.0)),
    # )
    # slot16 = scene.add_entity(
    #     gs.morphs.Box(size=(0.04, 0.04, 0.00), pos=slot16_pos),
    #     surface=gs.options.surfaces.Plastic(color=(0, 0.0, 0.0)),
    # )
    # # Second layer
    # slot21 = scene.add_entity(
    #     gs.morphs.Box(size=(0.04, 0.04, 0.00), pos= slot21_pos),
    #     surface=gs.options.surfaces.Plastic(color=(1.0, 1.0, 1.0)),
    # )
    # slot22 = scene.add_entity(
    #     gs.morphs.Box(size=(0.04, 0.04, 0.00), pos= slot22_pos),
    #     surface=gs.options.surfaces.Plastic(color=(1.0, 1.0, 1.0)),
    # )
    # slot23 = scene.add_entity(
    #     gs.morphs.Box(size=(0.04, 0.04, 0.00), pos= slot23_pos),
    #     surface=gs.options.surfaces.Plastic(color=(1.0, 1.0, 1.0)),
    # )
    # # Third layer
    # slot31 = scene.add_entity(
    #     gs.morphs.Box(size=(0.04, 0.04, 0.00), pos= slot31_pos),
    #     surface=gs.options.surfaces.Plastic(color=(0.0, 0.0, 0.0)),
    # )

    # Define position of Franka
    franka_raw = scene.add_entity(gs.morphs.MJCF(file="xml/franka_emika_panda/panda.xml"))
    franka = RobotAdapter(franka_raw, scene)

    # build scene (construct physics/visuals)
    scene.build()

    # initial robot pose (7 arm joints + 2 gripper fingers)
    franka.set_qpos(np.array([0.0, -0.5, -0.2, -1.0, 0.0, 1.00, 0.5, 0.02, 0.02]))

    # slightly raise robot base to avoid initial collisions
    _elevate_robot_base(franka)

    blocks_state: Dict[str, Any] = {"r": cubeR, "g": cubeG, "b": cubeB, "y": cubeY, "m": cubeM, "c": cubeC,
                                    "o": cubeO, "w": cubeW, "br": cubeBr, "p": cubeP}

    slots_state: Dict[str, Any] = {"11": slot11_pos, "12": slot12_pos, "13": slot13_pos, "14": slot14_pos, "15": slot15_pos, "16": slot16_pos,
                                   "21": slot21_pos, "22": slot22_pos, "23": slot23_pos, "31": slot31_pos}

    return scene, franka, blocks_state, slots_state