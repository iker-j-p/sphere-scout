#!/usr/bin/env python3
"""PreToolUse guard for file-writing tools (fs_write, str_replace, fs_append).

Usage (from .kiro/hooks/*.json):  python3 .kiro/scripts/guard.py <mode>
Reads the hook event as JSON on stdin and looks at the target path and the new text.

Modes
  reference   Ask the user before the tested reference modules are edited.
              Exit 0 with a JSON "ask" decision.
  harmonic    Block code that uses Gazebo Classic / Fortress names or ros2_control.
              Exit 2, reason on stderr (returned to the agent).
  interface   Print a reminder (added to the agent's context) when a change looks like it
              touches a shared interface. Exit 0, never blocks.

A PreToolUse hook that crashes blocks the tool call, so every path through this script
ends in exit 0 or exit 2. Any unexpected error or unrecognised event shape means "allow".
"""
import json
import os
import re
import sys

PATH_KEYS = {'path', 'targetFile', 'file_path', 'filePath'}
TEXT_KEYS = {'newStr', 'new_string', 'text', 'content'}

REFERENCE_FILES = {'triangulation.py', 'thermal_render.py'}

CODE_EXT = {'.py', '.sdf', '.xacro', '.urdf', '.xml', '.yaml', '.yml', '.cpp', '.h', '.ino',
            '.ini', '.cfg', '.txt', '.js', '.mjs', '.sh'}
FORBIDDEN = re.compile(
    r'gazebo_ros_|libgazebo|ignition[-:._]|ros2_control|controller_manager')

INTERFACE_CODE = re.compile(
    r'create_publisher|create_subscription|create_service|create_client|'
    r'rclc_publisher_init|rclc_subscription_init|rclc_service_init|'
    r'pixel_convention|child_frame_id|ros_topic_name|gz_topic_name')


def walk(node, out):
    """Collect (key, string value) pairs from any nested JSON."""
    if isinstance(node, dict):
        for k, v in node.items():
            if isinstance(v, str):
                out.append((k, v))
            else:
                walk(v, out)
    elif isinstance(node, list):
        for v in node:
            walk(v, out)


def load():
    raw = sys.stdin.read()
    event = json.loads(raw)
    pairs = []
    walk(event.get('tool_input', event), pairs)
    paths = [v for k, v in pairs if k in PATH_KEYS]
    texts = [v for k, v in pairs if k in TEXT_KEYS]
    return paths, '\n'.join(texts)


def rel(path):
    return os.path.normpath(path).replace(os.sep, '/')


def is_reference(path):
    return os.path.basename(path) in REFERENCE_FILES and 'sphere_scout_ai' in path


def mode_reference(paths, _text):
    hit = [p for p in paths if is_reference(rel(p))]
    if not hit:
        return 0
    reason = (f'{os.path.basename(hit[0])} is tested reference code (AGENTS.md / working '
              'agreements: do not rewrite). Approve only if this edit is intended and the '
              'unit tests will be re-run.')
    print(json.dumps({'hookSpecificOutput': {
        'permissionDecision': 'ask', 'permissionDecisionReason': reason}}))
    return 0


def mode_harmonic(paths, text):
    for p in paths:
        p = rel(p)
        ext = os.path.splitext(p)[1].lower()
        if ext not in CODE_EXT or p.startswith('.kiro/') or '/.kiro/' in p:
            continue
        if '/external/' in p or p.startswith('external/') or '/vendor/' in p:
            continue
        m = FORBIDDEN.search(text)
        if m:
            sys.stderr.write(
                f'Blocked: "{m.group(0)}" in {p}. This project uses Gazebo Harmonic names only '
                '(gz-sim-*-system, gz::sim::systems::*, gz.msgs.*) and no ros2_control. '
                'Use the Harmonic equivalent, or if this is only a comment, reword it.\n')
            return 2
    return 0


def mode_interface(paths, text):
    notes = []
    for p in paths:
        p = rel(p)
        base = os.path.basename(p)
        if base == 'bridge.yaml':
            notes.append(f'{p} is the ROS-Gazebo bridge table.')
        elif INTERFACE_CODE.search(text) and (
                '/src/' in '/' + p or p.startswith(('firmware/', 'tools/xr_viewer/'))) \
                and os.path.splitext(p)[1].lower() in CODE_EXT:
            notes.append(f'this edit to {p} touches a publisher, subscriber, service, topic '
                         'name, frame or pixel_convention.')
    if notes:
        print('Interface-change reminder: ' + ' '.join(notes) +
              ' If a topic, type, frame, parameter name, bridge entry or the thermal encoding '
              'changes, say so explicitly and update AGENTS.md and README.md (topics table), '
              'bridge.yaml, the node docstring and the firmware header in the same change '
              '(steering: #interface-change). If the edit leaves the interface unchanged, ignore '
              'this note.')
    return 0


MODES = {'reference': mode_reference, 'harmonic': mode_harmonic, 'interface': mode_interface}


def main():
    try:
        mode = MODES[sys.argv[1]]
        paths, text = load()
        return mode(paths, text)
    except Exception:  # fail open: a crashing PreToolUse hook would block the tool call
        return 0


if __name__ == '__main__':
    sys.exit(main())
