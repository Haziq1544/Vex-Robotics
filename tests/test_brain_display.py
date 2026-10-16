"""Exercise the actual status display without the desktop enumerate builtin."""
import ast
import builtins
from pathlib import Path
from types import SimpleNamespace
import unittest


class BrainDisplayTests(unittest.TestCase):
    def test_status_rows_without_enumerate(self):
        source = Path(__file__).resolve().parents[1] / "src" / "main.py"
        tree = ast.parse(source.read_text(encoding="utf-8"))
        run_test = next(node for node in tree.body
                        if isinstance(node, ast.FunctionDef) and node.name == "run_test")
        display = next(node for node in run_test.body
                       if isinstance(node, ast.FunctionDef) and node.name == "display")
        cursors, lines = [], []
        screen = SimpleNamespace(clear_screen=lambda: lines.clear(),
                                 set_cursor=lambda row, col: cursors.append((row, col)),
                                 print=lines.append)
        namespace = {
            "__builtins__": {name: value for name, value in vars(builtins).items()
                             if name != "enumerate"},
            "brain": SimpleNamespace(screen=screen),
            "vision_target_id": None, "vision_clearance_mm": None,
            "AI_VISION_PORT": 16, "search_swept_deg": 0.0,
        }
        exec(compile(ast.Module(body=[display], type_ignores=[]), str(source), "exec"), namespace)
        namespace["display"]("Calibrating GPS for search")
        self.assertEqual(cursors, [(row, 1) for row in range(1, 7)])
        self.assertEqual(lines[1], "Calibrating GPS for search")
        self.assertEqual(lines[4], "Camera port: 17")
        namespace["vision_target_id"] = 4
        namespace["vision_clearance_mm"] = 65.2
        namespace["display"]("Arrived: stopped before goal")
        self.assertEqual(lines[2:4], ["Tag: 4", "Gap estimate: 65 mm"])
        self.assertEqual(namespace["run_status"], "Arrived: stopped before goal")
