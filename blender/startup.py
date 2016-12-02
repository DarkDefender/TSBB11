# Load a Wavefront OBJ File
bpy.ops.import_scene.obj(filepath="./", filter_glob="*.obj;*.mtl", use_ngons=True,
use_edges=True, use_smooth_groups=True, use_split_objects=True, use_split_groups=True,
use_groups_as_vgroups=False, use_image_search=True, split_mode='ON',
global_clamp_size=0, axis_forward='-Z', axis_up='Y')
