import os
DATA_DIR = os.path.join("/root/share/catkin_ws/src/demo_traj/data")
ASSET_DIR = os.path.join("/root/share/assets")

REFINED_FINAL_PEG_POSE_DIR = os.path.join(DATA_DIR, "refined_final_peg_pose")
REFINED_KEYSTATE_PEG_POSE_DIR = os.path.join(DATA_DIR, "refined_keystate_peg_pose")
REFINED_DEMO_DIR = os.path.join(DATA_DIR, "refined_demo_traj")
try:
    os.makedirs(REFINED_FINAL_PEG_POSE_DIR, exist_ok=True)
    os.makedirs(REFINED_KEYSTATE_PEG_POSE_DIR, exist_ok=True)
    os.makedirs(REFINED_DEMO_DIR, exist_ok=True)
except Exception as e:
    print(f"Error creating directories: {e}")

def get_file_paths(bag_name, hole_part_name, peg_part_name):
    BAG_NAME = bag_name
    HOLE_DEMO_FOLDER = os.path.join(DATA_DIR, f"demo_traj/{bag_name}/{hole_part_name}")
    PEG_DEMO_FOLDER  = os.path.join(DATA_DIR, f"demo_traj/{bag_name}/{peg_part_name}")
    assert os.path.exists(HOLE_DEMO_FOLDER), f"Hole demo folder does not exist: {HOLE_DEMO_FOLDER}"
    assert os.path.exists(PEG_DEMO_FOLDER), f"Peg demo folder does not exist: {PEG_DEMO_FOLDER}"

    REFINED_DEMO_FOLDER = os.path.join(REFINED_DEMO_DIR, f"{bag_name}")
    os.makedirs(REFINED_DEMO_FOLDER, exist_ok=True)

    # if hole_part_name == "part1":   HOLE_MESH = os.path.join(ASSET_DIR, f'part1_from_LowerCase_scaled.obj')
    # if peg_part_name == "part11":   PEG_MESH  = os.path.join(ASSET_DIR, f'part11_from_MajorParts011_scaled_ratio02.obj')
    # elif peg_part_name == "part12": PEG_MESH  = os.path.join(ASSET_DIR, f'part12_from_MajorParts012_scaled.obj')
    # assert os.path.exists(HOLE_MESH), f"Hole mesh file does not exist: {HOLE_MESH}"
    # assert os.path.exists(PEG_MESH), f"Peg mesh file does not exist: {PEG_MESH}"

    # CLIPPED_HOLE_MESH             = os.path.join(ASSET_DIR, f'clipped_{hole_part_name}_near_{peg_part_name}_from_{bag_name}.obj')
    # CLIPPED_HOLE_MESH_RP_REFINED  = os.path.join(ASSET_DIR, f'clipped_{hole_part_name}_near_{peg_part_name}_from_{bag_name}_rp_refined.obj')
    # CLIPPED_HOLE_MESH_ORI_REFINED = os.path.join(ASSET_DIR, f'clipped_{hole_part_name}_near_{peg_part_name}_from_{bag_name}_ori_refined.obj')
    # CLIPPED_HOLE_MESH_REFINED     = os.path.join(ASSET_DIR, f'clipped_{hole_part_name}_near_{peg_part_name}_from_{bag_name}_refined.obj')
    FINAL_PEG_POSE_RP_REFINED     = os.path.join(REFINED_FINAL_PEG_POSE_DIR, f'{bag_name}_{peg_part_name}_rp_refined_final_pose.txt')
    FINAL_PEG_POSE_ORI_REFINED    = os.path.join(REFINED_FINAL_PEG_POSE_DIR, f'{bag_name}_{peg_part_name}_ori_refined_final_pose.txt')
    FINAL_PEG_POSE_REFINED        = os.path.join(REFINED_FINAL_PEG_POSE_DIR, f'{bag_name}_{peg_part_name}_refined_final_pose.txt')
    KEY_STATE_POSE_REFINED        = os.path.join(REFINED_KEYSTATE_PEG_POSE_DIR, f'{bag_name}_{peg_part_name}_refined_keystate_pose.txt')
    KEY_STATE_IDX_PKL             = os.path.join(DATA_DIR, f'keystate_idx.pkl')

    # if not os.path.exists(CLIPPED_HOLE_MESH): f"Clipped hole file does not exist: {CLIPPED_HOLE_MESH}"
    # if not os.path.exists(CLIPPED_HOLE_MESH_RP_REFINED): f"Clipped hole file does not exist: {CLIPPED_HOLE_MESH_RP_REFINED}"
    # if not os.path.exists(CLIPPED_HOLE_MESH_ORI_REFINED): f"Clipped hole file does not exist: {CLIPPED_HOLE_MESH_RP_REFINED}"
    # if not os.path.exists(CLIPPED_HOLE_MESH_REFINED): f"Clipped hole file does not exist: {CLIPPED_HOLE_MESH_REFINED}"
    if not os.path.exists(FINAL_PEG_POSE_RP_REFINED): f"Refined final peg pose file does not exist: {FINAL_PEG_POSE_RP_REFINED}"
    if not os.path.exists(FINAL_PEG_POSE_ORI_REFINED): f"Refined final peg pose file does not exist: {FINAL_PEG_POSE_ORI_REFINED}"
    if not os.path.exists(FINAL_PEG_POSE_REFINED): f"Refined final peg pose file does not exist: {FINAL_PEG_POSE_REFINED}"
    if not os.path.exists(KEY_STATE_POSE_REFINED): f"Refined keystate peg pose file does not exist: {KEY_STATE_POSE_REFINED}"

    return {
        "BAG_NAME": BAG_NAME,
        "HOLE_DEMO_FOLDER": HOLE_DEMO_FOLDER,
        "PEG_DEMO_FOLDER": PEG_DEMO_FOLDER,
        "REFINED_DEMO_FOLDER": REFINED_DEMO_FOLDER,
        # "HOLE_MESH": HOLE_MESH,
        # "PEG_MESH": PEG_MESH,
        # "CLIPPED_HOLE_MESH": CLIPPED_HOLE_MESH,
        # "CLIPPED_HOLE_MESH_RP_REFINED": CLIPPED_HOLE_MESH_RP_REFINED,
        # "CLIPPED_HOLE_MESH_ORI_REFINED": CLIPPED_HOLE_MESH_ORI_REFINED,
        # "CLIPPED_HOLE_MESH_REFINED": CLIPPED_HOLE_MESH_REFINED,
        "FINAL_PEG_POSE_RP_REFINED": FINAL_PEG_POSE_RP_REFINED,
        "FINAL_PEG_POSE_ORI_REFINED": FINAL_PEG_POSE_ORI_REFINED,
        "FINAL_PEG_POSE_REFINED": FINAL_PEG_POSE_REFINED,
        "KEY_STATE_POSE_REFINED": KEY_STATE_POSE_REFINED,
        "KEY_STATE_IDX_PKL": KEY_STATE_IDX_PKL,
    }