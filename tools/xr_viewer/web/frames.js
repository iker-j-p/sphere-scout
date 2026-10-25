// Frame graph for the operator view. Pure three.js, no DOM, so it is unit-testable in Node.
//
//   viewRoot   turns odom +x to the headset's forward (-z); also the tabletop pivot
//    └ rosRoot   the odom frame in ROS axes (x forward, y left, z up); origin = robot spawn
//       ├ worldGroup   the Gazebo world expressed in odom (inverse of spawn_pose)
//       └ dyn          markers, robot, trail (all in odom)
//
// rosRoot is rotated -90 deg about X so ROS z maps to three.js y. The apartment GLB is Z-up
// (the exporter writes it Z-up and Gazebo reads it as-is), so it needs no extra rotation.
import * as THREE from 'three';

export function createFrames() {
  const viewRoot = new THREE.Group();
  viewRoot.rotation.y = Math.PI / 2;
  const rosRoot = new THREE.Group();
  rosRoot.rotation.x = -Math.PI / 2;
  viewRoot.add(rosRoot);
  const worldGroup = new THREE.Group();
  rosRoot.add(worldGroup);
  const dyn = new THREE.Group();
  rosRoot.add(dyn);

  // spawn_pose = [x, y, z, yaw] of base_link in the world frame (same as the localizer's
  // spawn_pose). Same transform as triangulation.world_to_odom:
  //   p_odom = Rz(-yaw) * (p_world - spawn)
  // Returns the odom z of the world floor (world z = 0), used for drop lines.
  function applySpawn([sx, sy, sz, yaw]) {
    worldGroup.rotation.set(0, 0, -yaw);
    const c = Math.cos(-yaw), s = Math.sin(-yaw);
    worldGroup.position.set(-(c * sx - s * sy), -(s * sx + c * sy), -sz);
    rosRoot.position.set(0, sz, 0);   // the user stands on the world floor, odom origin above it
    return -sz;
  }

  return { viewRoot, rosRoot, worldGroup, dyn, applySpawn };
}
