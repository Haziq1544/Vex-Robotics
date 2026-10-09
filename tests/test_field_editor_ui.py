"""Exercise Tk event bindings and editor persistence, without driving the robot."""
import copy
import json
from pathlib import Path
import tempfile
import tkinter as tk
import unittest
from unittest.mock import patch

from simulator.config import load_config
from simulator.field_elements import ACTIVE_ELEMENT_SPECS, PRESETS, make_element, resolve_elements
from simulator.ui import ModelSettings, SimulatorApp


ROOT = Path(__file__).resolve().parents[1]


class FieldEditorTests(unittest.TestCase):
    def setUp(self):
        try:
            self.root = tk.Tk()
        except tk.TclError as error:
            self.skipTest("A Tk display is required: " + str(error))
        self.root.attributes("-alpha", 0.0)
        self.app = SimulatorApp(self.root, load_config(), ROOT / "src" / "main.py")
        self.root.geometry("1180x850+0+0")
        self.app.start_x.set("-900")
        self.app.start_y.set("-900")
        self.app.apply_start()
        self.app.sidebar.select(1)
        self.root.update()

    def tearDown(self):
        if hasattr(self, "app"):
            self.app.close()

    def choose(self, preset):
        self.app.layout_choice.set(PRESETS[preset])
        self.app.layout_combo.event_generate("<<ComboboxSelected>>")
        self.root.update()

    def field_event(self, sequence, x, y):
        px, py = self.app._point(x, y)
        self.app.canvas.event_generate(sequence, x=round(px), y=round(py))
        self.root.update()

    def palette_press(self, kind):
        index = list(ACTIVE_ELEMENT_SPECS).index(kind)
        self.app.palette.event_generate("<ButtonPress-1>", x=20, y=index * 36 + 17)
        self.root.update()

    def palette_drop(self, x, y):
        px, py = self.app._point(x, y)
        event_x = round(px + self.app.canvas.winfo_rootx() - self.app.palette.winfo_rootx())
        event_y = round(py + self.app.canvas.winfo_rooty() - self.app.palette.winfo_rooty())
        self.app.palette.event_generate("<B1-Motion>", x=event_x, y=event_y)
        self.root.update()
        self.app.palette.event_generate("<ButtonRelease-1>", x=event_x, y=event_y)
        self.root.update()

    def test_preset_dropdown_and_restore_after_custom_move(self):
        self.choose("override")
        initial = resolve_elements(self.app.config)
        self.assertEqual(len(initial), 13)
        self.assertEqual(self.app.config["layout"]["preset"], "override")
        # A selection alone must not replace a preset with Custom.
        self.field_event("<ButtonPress-1>", 0, 0)
        self.field_event("<ButtonRelease-1>", 0, 0)
        self.assertEqual(self.app.config["layout"]["preset"], "override")
        self.field_event("<ButtonPress-1>", 0, 0)
        self.field_event("<B1-Motion>", 250, 0)
        self.field_event("<ButtonRelease-1>", 250, 0)
        self.assertEqual(self.app.config["layout"]["preset"], "custom")
        self.app.restore_layout_button.invoke()
        self.assertEqual(resolve_elements(self.app.config), initial)

    def test_palette_drag_creates_element_at_scaled_coordinates(self):
        self.palette_press("goal_red")
        self.palette_drop(500, 700)
        elements = resolve_elements(self.app.config)
        self.assertEqual(len(elements), 1)
        self.assertEqual(elements[0]["kind"], "goal_red")
        self.assertAlmostEqual(elements[0]["x_mm"], 500, delta=4)
        self.assertAlmostEqual(elements[0]["y_mm"], 700, delta=4)
        self.assertEqual(self.app.config["layout"]["preset"], "custom")
        self.assertIsNone(self.app._place_kind)

    def test_palette_click_then_field_click_and_escape(self):
        self.palette_press("goal_neutral")
        index = list(ACTIVE_ELEMENT_SPECS).index("goal_neutral")
        self.app.palette.event_generate("<ButtonRelease-1>", x=20, y=index * 36 + 17)
        self.field_event("<ButtonPress-1>", 500, 600)
        self.assertEqual(len(resolve_elements(self.app.config)), 1)
        self.palette_press("loader")
        self.app.canvas.focus_force()
        self.app.canvas.event_generate("<Escape>")
        self.root.update()
        self.assertIsNone(self.app._place_kind)
        self.assertEqual(len(resolve_elements(self.app.config)), 1)

    def test_overlapping_robot_element_and_outside_placements_rejected(self):
        self.assertTrue(self.app._place_element("goal_blue", 0, 600))
        before = copy.deepcopy(self.app.config)
        for x, y in ((0, 600), (-900, -900), (1770, 500)):
            self.assertFalse(self.app._place_element("goal_neutral", x, y))
            self.assertEqual(self.app.config, before)
            self.assertIn("Cannot place layout:", self.app.status.get())

    def test_invalid_drag_keeps_original_and_delete_removes_selection(self):
        self.app._place_element("loader", 500, 700)
        before = resolve_elements(self.app.config)
        self.field_event("<ButtonPress-1>", 500, 700)
        self.field_event("<B1-Motion>", -900, -900)
        self.field_event("<ButtonRelease-1>", -900, -900)
        self.assertEqual(resolve_elements(self.app.config), before)
        self.app.rotate_button.invoke()
        self.assertEqual(resolve_elements(self.app.config)[0]["heading_deg"], 45)
        self.app.delete_button.invoke()
        self.assertEqual(resolve_elements(self.app.config), [])

    def test_reset_preserves_authored_layout_and_clears_old_snapshot(self):
        self.app._place_element("goal_neutral", 500, 700)
        authored = resolve_elements(self.app.config)
        pushed = copy.deepcopy(authored)
        pushed[0]["x_mm"] = 900
        self.app._accept_snapshot({"world": dict(self.app.world, elements=pushed), "time_s": 1})
        self.assertEqual(self.app.world["elements"][0]["x_mm"], 900)
        self.app.reset_button.invoke()
        self.assertIsNone(self.app.snapshot)
        self.assertEqual(resolve_elements(self.app.config), authored)
        self.assertEqual(self.app.config["layout"]["preset"], "custom")

    def test_save_load_uses_authored_positions_and_model_retains_layout(self):
        self.app._place_element("goal_neutral", 500, 700)
        authored = copy.deepcopy(self.app.config["layout"])
        pushed = resolve_elements(self.app.config)
        pushed[0]["x_mm"] = 1000
        self.app._accept_snapshot({"world": dict(self.app.world, elements=pushed), "time_s": 1})
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "layout.json"
            with patch("simulator.ui.filedialog.asksaveasfilename", return_value=str(path)):
                self.app.save_layout()
            self.assertEqual(json.loads(path.read_text())["layout"], authored)
            self.choose("empty")
            with patch("simulator.ui.filedialog.askopenfilename", return_value=str(path)):
                self.app.load_layout()
            self.assertEqual(self.app.config["layout"], authored)
        dialog = ModelSettings(self.app)
        try:
            self.assertEqual(dialog._read()["layout"], authored)
        finally:
            dialog.window.destroy()

    def test_editor_disabled_during_running_and_paused_sessions(self):
        class Session:
            alive = True

            def stop(self):
                self.alive = False

        self.app.session = Session()
        self.app.paused = True
        before = copy.deepcopy(self.app.config)
        self.app._update_controls()
        self.assertEqual(str(self.app.layout_combo.cget("state")), "disabled")
        self.assertEqual(str(self.app.restore_layout_button.cget("state")), "disabled")
        self.app._place_element("goal_neutral", 500, 700)
        self.app.restore_layout()
        self.app.delete_selected()
        self.assertEqual(self.app.config, before)
        self.app.stop()
        self.assertEqual(str(self.app.layout_combo.cget("state")), "readonly")

    def test_rejected_preset_does_not_change_existing_layout(self):
        self.app.start_x.set("0")
        self.app.start_y.set("0")
        self.app.apply_start()
        self.choose("override")
        self.assertEqual(self.app.config["layout"]["preset"], "empty")
        self.assertEqual(self.app.layout_choice.get(), PRESETS["empty"])
        self.assertIn("overlaps", self.app.status.get())

    def test_invalid_layout_file_does_not_mutate_editor(self):
        before = copy.deepcopy(self.app.config)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "invalid.json"
            path.write_text('{"layout": {"preset": "custom", "elements": [null]}}')
            with patch("simulator.ui.filedialog.askopenfilename", return_value=str(path)):
                self.app.load_layout()
        self.assertEqual(self.app.config, before)
        self.assertIn("Cannot place layout", self.app.status.get())

    def test_stackers_are_absent_from_palette_and_legacy_layout_rendering(self):
        palette_tags = [tag for item in self.app.palette.find_all() for tag in self.app.palette.gettags(item)]
        self.assertNotIn("kind:cup", palette_tags)
        self.assertFalse(self.app._place_element("cup", 500, 700))
        goal = make_element("goal_red", 500, 700)
        cup = make_element("cup", 0, 0)
        self.app.config["layout"] = {"preset": "custom", "elements": [goal, cup]}
        self.app._accept_snapshot({"world": dict(self.app.world, elements=[goal, cup]), "time_s": 1})
        with patch.object(self.app, "_polygon", wraps=self.app._polygon) as draw_polygon:
            self.app._draw_elements(self.app._transform[2])
        self.assertEqual(draw_polygon.call_count, 1)
        self.assertEqual(resolve_elements(self.app.config), [goal])

    def test_default_program_previews_vision_from_user_chosen_pose(self):
        self.choose("override")
        self.app.sidebar.select(3)
        self.app.start_x.set("1400")
        self.app.start_y.set("-1200")
        self.app.start_heading.set("270")
        self.app.apply_start()
        self.root.update()
        self.assertEqual(self.app.program_path, ROOT / "src" / "main.py")
        self.assertIn("first visible goal", self.app.program_name)
        self.assertEqual(self.app.world["x_mm"], 1400)
        self.assertEqual(self.app.world["y_mm"], -1200)
        self.assertEqual(self.app.world["heading_deg"], 270)
        self.assertIn("ID 4", self.app.vision_text.get())
        self.assertIn("Preview", self.app.vision_text.get())
        self.assertGreater(len(self.app.camera_canvas.find_all()), 1)
        dialog = ModelSettings(self.app)
        try:
            self.assertEqual(dialog._read()["vision"], self.app.config["vision"])
        finally:
            dialog.window.destroy()
        self.app.reset()
        self.root.update()
        self.assertEqual(self.app.world["x_mm"], 1400)
        self.assertEqual(self.app.world["y_mm"], -1200)
        self.assertIsNone(self.app.target)

    def test_gps_file_named_main_does_not_get_a_vision_label_or_preview(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "main.py"
            path.write_text((ROOT / "examples" / "gps_centre_test.py").read_text(encoding="utf-8"), encoding="utf-8")
            self.app.program_path = path
            self.app._refresh_program_info()
            self.app.redraw()
            self.assertEqual(self.app.program_name, "Fixed coordinate navigation")
            self.assertIsNone(self.app._vision_for_display())
            self.assertIn("has not started", self.app.vision_text.get())
            with patch("simulator.program.DEFAULT_PROGRAM", path.resolve()):
                self.app.start()
            self.assertIn("not the AI Vision", self.app.status.get())
            self.assertIsNone(self.app.session)

    def test_running_snapshot_uses_actual_camera_and_shows_locked_tag(self):
        self.choose("override")
        self.app.start_x.set("1400")
        self.app.start_y.set("-1200")
        self.app.start_heading.set("270")
        self.app.apply_start()
        state = self.app._vision_for_display()
        self.app._accept_snapshot({"world": dict(self.app.world), "time_s": .2,
                                   "vision": state, "vision_tracking": {"id": 4, "clearance_mm": 500}})
        self.assertIn("Locked goal: ID 4", self.app.vision_text.get())
        self.app._accept_snapshot({"world": dict(self.app.world), "time_s": .3, "vision": None})
        self.assertIsNone(self.app._vision_for_display(), "Never fabricate camera readings for a running program")
        self.assertIn("has not started", self.app.vision_text.get())


if __name__ == "__main__":
    unittest.main()
