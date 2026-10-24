"""Convert the apartment FBX into Gazebo meshes.

Writes into the meshes directory:
  visual_<group>.glb       visual meshes, textures embedded, one file per thermal group
                           (THERMAL_GROUPS) so model.sdf can give each its own temperature
  apartment_collision.glb  physics mesh: structural parts merged, coplanar faces
                           dissolved (surface unchanged), no materials

Run headless with Blender 4.2+:
  blender -b --factory-startup --python tools/export_apartment.py -- \
      Building/RS_Apartment_Visual.fbx src/sphere_scout_gazebo/models/apartment/meshes \
      [texture_dir ...]

The FBX stores absolute texture paths from the machine it was exported on.
Missing textures are looked up by file name in the given texture directories.

Both GLBs are written Z-up, not the glTF-standard Y-up: Gazebo Harmonic reads glTF
coordinates as-is, so a Y-up file shows the building lying on its side.
"""
import math
import os
import statistics
import sys

import bmesh
import bpy
from mathutils import Vector
from mathutils.bvhtree import BVHTree

# Parts the robot can pass through or will never touch; left out of the collision mesh.
COLLISION_SKIP = ("Glass", "Label", "Copper", "Water", "Fabric")
# Visual meshes grouped by how they behave in a thermal image. FBX objects are named
# Apartment_<material>; anything not listed goes in "other".
THERMAL_GROUPS = {
    "brick": ("Brick", "BrickCore"),
    "concrete": ("Concrete",),
    "finish": ("Plaster", "Paint", "Ivory"),
    "metal": ("Steel", "HA_Copper", "HA_Rust", "HA_Paint", "HA_Red"),
    "wood": ("Wood",),
    "glass": ("Glass",),
}
# Merge coplanar faces within this angle (keeps walls and floors exact). There is no
# further edge-collapse decimation: on this model it merges the top and bottom of thin
# floor slabs and leaves holes the robot falls through, at any triangle budget tried.
DISSOLVE_DEG = 2.0

args = sys.argv[sys.argv.index("--") + 1:]
fbx_path, meshes_dir, tex_dirs = args[0], args[1], args[2:]

bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.fbx(filepath=fbx_path)
os.makedirs(meshes_dir, exist_ok=True)


def tri_count(obj):
    return sum(len(p.vertices) - 2 for p in obj.data.polygons)


def apply_decimate(obj, **settings):
    mod = obj.modifiers.new("decimate", "DECIMATE")
    for key, value in settings.items():
        setattr(mod, key, value)
    with bpy.context.temp_override(object=obj, active_object=obj):
        bpy.ops.object.modifier_apply(modifier=mod.name)


def world_bvh(objs):
    # Triangulate with bmesh first: BVHTree fan-splits n-gons, which is wrong for concave ones.
    verts, tris = [], []
    for o in objs:
        bm = bmesh.new()
        bm.from_mesh(o.data)
        bm.transform(o.matrix_world)
        bmesh.ops.triangulate(bm, faces=bm.faces)
        base = len(verts)
        verts += [v.co.copy() for v in bm.verts]
        tris += [[base + v.index for v in f.verts] for f in bm.faces]
        bm.free()
    return BVHTree.FromPolygons(verts, tris), verts


def surface_error(reference, simplified, verts, step=0.5):
    """Cast rays straight down on a grid, from above the roof and from mid-height of each
    storey, and compare where they hit. Returns per-ray height errors and the hole count."""
    xs, ys, zs = zip(*verts)
    starts = [20.0, 1.6, 5.0, 8.4, 11.8]  # storeys are about 3.4 m tall
    errors, holes = [], 0
    for i in range(int((max(xs) - min(xs)) / step) + 1):
        for j in range(int((max(ys) - min(ys)) / step) + 1):
            for z0 in starts:
                origin = Vector((min(xs) + i * step, min(ys) + j * step, z0))
                ref = reference.ray_cast(origin, Vector((0, 0, -1)), 25.0)[3]
                if ref is None:
                    continue
                sim = simplified.ray_cast(origin, Vector((0, 0, -1)), 25.0)[3]
                if sim is None or sim - ref > 0.5:
                    holes += 1
                else:
                    errors.append(abs(sim - ref))
    return errors, holes


# --- visual mesh ---
for img in bpy.data.images:
    if os.path.exists(bpy.path.abspath(img.filepath)):
        continue
    name = os.path.basename(img.filepath)
    for root in tex_dirs:
        hits = [os.path.join(d, name) for d, _, files in os.walk(root) if name in files]
        if hits:
            img.filepath = hits[0]
            img.reload()
            print(f"relinked {name} -> {hits[0]}")
            break
    else:
        print(f"WARNING: texture {name} not found")

def thermal_group(obj):
    material = obj.name.removeprefix("Apartment_").split(".")[0]
    return next((g for g, names in THERMAL_GROUPS.items() if material in names), "other")


meshes = [o for o in bpy.data.objects if o.type == "MESH"]
for group in list(THERMAL_GROUPS) + ["other"]:
    members = [o for o in meshes if thermal_group(o) == group]
    for o in bpy.data.objects:
        o.select_set(o in members)
    visual_path = os.path.join(meshes_dir, f"visual_{group}.glb")
    bpy.ops.export_scene.gltf(
        filepath=visual_path,
        export_format="GLB",
        use_selection=True,
        export_apply=True,
        export_yup=False,
        export_image_format="AUTO",
    )
    print(f"wrote {visual_path} ({os.path.getsize(visual_path) / 1e6:.1f} MB): "
          + ", ".join(o.name.removeprefix("Apartment_") for o in members))

# --- collision mesh ---
parts = [o for o in bpy.data.objects
         if o.type == "MESH" and not any(s in o.name for s in COLLISION_SKIP)]
print("collision parts:", ", ".join(o.name for o in parts))
reference_bvh, reference_verts = world_bvh(parts)
with bpy.context.temp_override(active_object=parts[0], selected_objects=parts,
                               selected_editable_objects=parts):
    bpy.ops.object.join()
col = parts[0]
col.name = "apartment_collision"
col.data.materials.clear()
print(f"collision tris: {tri_count(col)} joined")

bm = bmesh.new()
bm.from_mesh(col.data)
bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=0.001)
bm.to_mesh(col.data)
bm.free()

apply_decimate(col, decimate_type="DISSOLVE", angle_limit=math.radians(DISSOLVE_DEG),
               use_dissolve_boundaries=False)
print(f"collision tris: {tri_count(col)} after planar dissolve")
col.data.validate()

errors, holes = surface_error(reference_bvh, world_bvh([col])[0], reference_verts)
errors.sort()
pct = lambda q: errors[min(len(errors) - 1, int(q * len(errors)))]
print(f"collision surface error over {len(errors) + holes} rays: median {statistics.median(errors) * 100:.1f} cm, "
      f"p90 {pct(0.9) * 100:.1f} cm, p99 {pct(0.99) * 100:.1f} cm, "
      f"> 5 cm: {sum(e > 0.05 for e in errors)}, holes (> 50 cm or missing): {holes}")

for o in bpy.data.objects:
    o.select_set(o is col)
collision_path = os.path.join(meshes_dir, "apartment_collision.glb")
bpy.ops.export_scene.gltf(
    filepath=collision_path,
    export_format="GLB",
    use_selection=True,
    export_materials="NONE",
    export_apply=True,
    export_yup=False,
)
print(f"wrote {collision_path} ({os.path.getsize(collision_path) / 1e6:.1f} MB)")
