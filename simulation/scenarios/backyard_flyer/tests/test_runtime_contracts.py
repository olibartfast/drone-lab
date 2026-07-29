import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


SCENARIO_DIR = Path(__file__).resolve().parents[1]


def load_module(name, filename):
    spec = importlib.util.spec_from_file_location(name, SCENARIO_DIR / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


compose_contract = load_module("check_compose_contract", "check_compose_contract.py")
resources = load_module("summarize_resources", "summarize_resources.py")


class ComposeContractTest(unittest.TestCase):
    def valid_config(self):
        return {
            "services": {
                "px4-gazebo": {
                    "environment": {
                        "HEADLESS": "1",
                        "LIBGL_ALWAYS_SOFTWARE": "1",
                        "XDG_RUNTIME_DIR": "/tmp/drone-lab-xdg",
                    },
                    "cpus": 4.0,
                    "mem_limit": 8 * 1024**3,
                    "shm_size": 1024**3,
                    "volumes": [{"target": "/tmp/.X11-unix"}],
                    "devices": [],
                }
            }
        }

    def test_accepts_bounded_gui_contract(self):
        self.assertEqual(compose_contract.validate(self.valid_config()), [])

    def test_accepts_hardware_gui_contract(self):
        config = self.valid_config()
        config["services"]["px4-gazebo"]["environment"]["LIBGL_ALWAYS_SOFTWARE"] = "0"
        config["services"]["px4-gazebo"]["devices"] = [{"path_in_container": "/dev/dri"}]
        self.assertEqual(compose_contract.validate(config, expect_hardware=True), [])

    def test_rejects_unbounded_non_gui_contract(self):
        config = self.valid_config()
        del config["services"]["px4-gazebo"]["cpus"]
        config["services"]["px4-gazebo"]["environment"]["HEADLESS"] = "0"
        errors = compose_contract.validate(config)
        self.assertEqual(len(errors), 2)


class LauncherContractTest(unittest.TestCase):
    def test_x11_access_survives_until_container_start(self):
        script = (SCENARIO_DIR / "launch.sh").read_text(encoding="utf-8")
        grant = script.index("xhost +si:localuser:root")
        start = script.index("up --detach px4-gazebo")
        self.assertLess(grant, start)
        self.assertNotIn("cleanup\n\"${COMPOSE[@]}\" up", script)

    def test_launcher_owns_and_monitors_gui_client(self):
        script = (SCENARIO_DIR / "launch.sh").read_text(encoding="utf-8")
        self.assertIn("exec --detach px4-gazebo", script)
        self.assertIn("gz sim -g --verbose=4", script)
        self.assertIn("gazebo_gui_startup", script)
        self.assertIn("gazebo_gui_runtime", script)


class ResourceSummaryTest(unittest.TestCase):
    def test_reports_peaks_and_limit(self):
        samples = [
            {"stats": {"CPUPerc": "125.5%", "MemUsage": "1.5GiB / 8GiB", "PIDs": "42"}},
            {"stats": {"CPUPerc": "310.0%", "MemUsage": "2048MiB / 8GiB", "PIDs": "51"}},
        ]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "resources.jsonl"
            path.write_text("\n".join(json.dumps(sample) for sample in samples) + "\n", encoding="utf-8")
            report = resources.summarize(path)
        self.assertEqual(report["samples"], 2)
        self.assertEqual(report["peak_cpu_percent"], 310.0)
        self.assertEqual(report["peak_memory_bytes"], 2 * 1024**3)
        self.assertEqual(report["memory_limit_bytes"], 8 * 1024**3)
        self.assertEqual(report["peak_pids"], 51)

    def test_rejects_empty_log(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "resources.jsonl"
            path.write_text("", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "no resource samples"):
                resources.summarize(path)


if __name__ == "__main__":
    unittest.main()
