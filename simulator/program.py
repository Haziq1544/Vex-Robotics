"""Identify the saved robot source without executing it or trusting its name."""
import ast
from pathlib import Path


DEFAULT_PROGRAM = Path(__file__).resolve().parents[1] / "src" / "main.py"
VISION_KIND = "ai_vision_first_goal"


def describe_program(source, filename):
    assignments = {}
    for node in ast.parse(source, filename=str(filename)).body:
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name):
                    assignments[target.id] = node.value
    if "TARGET_X_MM" in assignments and "TARGET_Y_MM" in assignments:
        return {"kind": "coordinate_target", "name": "Fixed coordinate navigation"}
    declared = assignments.get("PROGRAM_KIND")
    if isinstance(declared, ast.Constant) and declared.value == VISION_KIND:
        mode = assignments.get("VISION_MODE")
        if isinstance(mode, ast.Constant) and mode.value == "tag_centering":
            return {"kind": VISION_KIND, "name": "AI Vision: detect and centre tag"}
        if isinstance(mode, ast.Constant) and mode.value == "tag_approach":
            return {"kind": VISION_KIND, "name": "AI Vision: calibrated Tag 2 approach"}
        return {"kind": VISION_KIND, "name": "AI Vision: first visible goal"}
    return {"kind": "custom", "name": "Custom program: " + Path(filename).name}


def read_program(path, require_default_vision=False):
    path = Path(path).resolve()
    source = path.read_text(encoding="utf-8-sig")
    info = describe_program(source, path)
    if require_default_vision and path == DEFAULT_PROGRAM and info["kind"] != VISION_KIND:
        raise ValueError("src/main.py is not the AI Vision first-goal program. "
                         "The saved source may have been replaced or undone. Reload the updated file before running.")
    return source, info
