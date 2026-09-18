import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, call, patch

from example.arm_teleop import gen3_operator_keyboard as keyboard
from example.operator_hold import STOP


spec = importlib.util.spec_from_file_location(
    "gen3_teleop_shutdown", Path(__file__).resolve().parents[1] / "scripts/gen3_teleop_shutdown.py")
shutdown = importlib.util.module_from_spec(spec)
spec.loader.exec_module(shutdown)


class ShutdownTests(unittest.TestCase):
    repo = Path("/tmp/test-teleop")
    operator_dir = repo / "operator"

    def test_q_overrides_held_motion_and_publishes_stop_before_exit(self):
        for held in ("a", "b"):
            with self.subTest(held=held):
                scheduled = []
                root, writer, keys, events = Mock(), Mock(), Mock(), Mock()
                events.attach_mock(root, "root")
                events.attach_mock(writer, "writer")
                root.after.side_effect = lambda _delay, callback: scheduled.append(callback)
                root.mainloop.side_effect = lambda: scheduled[0]()
                root.focus_displayof.return_value = root
                keys.read.return_value = {"a": held == "a", "b": held == "b", "q": True, "Escape": False}
                fake_tk = SimpleNamespace(Tk=lambda: root, Label=Mock())
                with patch.dict("sys.modules", {"tkinter": fake_tk}), \
                        patch.object(keyboard, "X11Keys", return_value=keys), \
                        patch.object(keyboard, "IntentWriter", return_value=writer), \
                        patch.object(keyboard.signal, "signal"), patch("builtins.print"):
                    self.assertEqual(keyboard.main([]), keyboard.SHUTDOWN_STACK_EXIT)
                self.assertLess(events.mock_calls.index(call.writer.publish(STOP)),
                                events.mock_calls.index(call.root.quit()))
                writer.close.assert_called_once()

    def container(self, n, service, project="arm", operator=None, repo=None, config=None):
        repo = repo or self.repo
        config = config or ("compose.yaml" if service in {"real-wh110-manus", "manus-sdk-udp-bridge-unicast"}
                            else "docker-compose.gen3-vive.yml")
        return {"Id": f"{n:064x}", "Config": {"Labels": {
            "com.docker.compose.project.working_dir": str(repo),
            "com.docker.compose.project.config_files": str(repo / config),
            "com.docker.compose.project": project,
            "com.docker.compose.service": service,
        }}, "Mounts": [] if operator is None else [{"Destination": "/operator", "Source": str(operator)}]}

    def test_scopes_by_repo_mount_project_and_service(self):
        containers = [
            self.container(1, "wrapper-real", operator=self.operator_dir),
            self.container(2, "gen3-servo-real"), self.container(3, "receiver"),
            self.container(4, "real-wh110-manus", "support", self.operator_dir),
            self.container(5, "manus-sdk-udp-bridge-unicast", "support"),
            self.container(6, "operator", "support", self.operator_dir),
            self.container(7, "wrapper-real", "other-arm", self.repo / "other-operator"),
            self.container(8, "gen3-servo-real", "other-arm"),
            self.container(9, "real-wh110-manus", "other-support", self.repo / "other-operator"),
            self.container(10, "manus-sdk-udp-bridge-unicast", "other-support"),
            self.container(11, "unrelated", "arm"),
            self.container(12, "wrapper-real", operator=self.operator_dir, repo=Path("/tmp/other-repo")),
        ]
        motion, support = shutdown.shutdown_groups(containers, self.repo, self.operator_dir, "support")
        self.assertEqual(motion, [f"{n:064x}" for n in (1, 4)])
        self.assertEqual(support, [f"{n:064x}" for n in (2, 3, 5, 6)])

    def test_foreign_hand_blocks_orphan_bridge_fallback(self):
        bridge = self.container(1, "manus-sdk-udp-bridge-unicast", "support")
        foreign_hand = self.container(2, "real-wh110-manus", "support", self.repo / "other-operator")
        self.assertEqual(shutdown.shutdown_groups([bridge, foreign_hand], self.repo,
                                                 self.operator_dir, "support"), ([], []))
        self.assertEqual(shutdown.shutdown_groups([bridge], self.repo, self.operator_dir,
                                                 "support"), ([], [bridge["Id"]]))

    def test_motion_stopped_before_support_and_removed_containers_are_ok(self):
        groups = [["1" * 64, "2" * 64], ["3" * 64]]
        with patch.object(shutdown, "run", return_value=SimpleNamespace(returncode=1, stderr="gone")) as run, \
                patch.object(shutdown, "inventory", return_value=[]):
            self.assertEqual(shutdown.stop_groups(groups, self.repo), [])
        self.assertEqual([call.args[0] for call in run.call_args_list],
                         [["docker", "stop", "--time", "3", *ids] for ids in groups])

    def test_tmux_requires_both_ownership_markers(self):
        with patch.object(shutdown, "run", side_effect=[
            SimpleNamespace(returncode=0, stdout=str(self.repo) + "\n"),
            SimpleNamespace(returncode=0, stdout="/tmp/other-operator\n"),
        ]):
            self.assertFalse(shutdown.marked_session("user-session", self.repo, self.operator_dir))
        with patch.object(shutdown, "run") as run:
            self.assertFalse(shutdown.marked_session("", self.repo, self.operator_dir))
            run.assert_not_called()


if __name__ == "__main__":
    unittest.main()
