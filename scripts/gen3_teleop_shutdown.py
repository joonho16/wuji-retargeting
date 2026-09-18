#!/usr/bin/env python3
"""Host-side Q shutdown, scoped to this checkout and operator lease directory."""
import argparse
import json
from pathlib import Path
import re
import subprocess
import time


def run(args, **kwargs):
    return subprocess.run(args, text=True, capture_output=True, **kwargs)


def inventory(repo):
    result = run(["docker", "ps", "--quiet", "--filter",
                  f"label=com.docker.compose.project.working_dir={repo}"], check=True)
    ids = result.stdout.split()
    # A compose supervisor may remove a container between ps and inspect.
    containers = []
    for cid in ids:
        result = run(["docker", "inspect", cid])
        if result.returncode == 0:
            containers.extend(json.loads(result.stdout))
    return containers


def shutdown_groups(containers, repo, operator_dir, operator_project):
    """Return motion owners first, followed by only their support containers."""
    repo, operator_dir = Path(repo).resolve(), Path(operator_dir).resolve()
    arm_file = repo / "docker-compose.gen3-vive.yml"
    hand_file = repo / "compose.yaml"
    eligible = []
    for container in containers:
        labels = container.get("Config", {}).get("Labels") or {}
        cid = container.get("Id", "")
        if not re.fullmatch(r"[a-f0-9]{12,64}", cid):
            continue
        if Path(labels.get("com.docker.compose.project.working_dir", "/")).resolve() != repo:
            continue
        files = {Path(f).resolve() for f in labels.get("com.docker.compose.project.config_files", "").split(",") if f}
        kind = "arm" if arm_file in files else "hand" if hand_file in files else None
        project = labels.get("com.docker.compose.project", "")
        if not kind or not project:
            continue
        service = labels.get("com.docker.compose.service", "")
        matching_mount = any(m.get("Destination") == "/operator"
                             and Path(m.get("Source", "/")).resolve() == operator_dir
                             for m in container.get("Mounts", []))
        eligible.append((cid, project, kind, service, matching_mount))

    arm_projects = {p for _, p, k, s, m in eligible if k == "arm" and s == "wrapper-real" and m}
    hand_projects = {p for _, p, k, s, m in eligible if k == "hand" and s == "real-wh110-manus" and m}
    # The just-exited operator was explicitly launched under this project. If
    # its hand has already exited, its bridge still needs shutting down. A hand
    # attached to another lease prevents this fallback from touching its bridge.
    foreign_hand_projects = {p for _, p, k, s, m in eligible
                             if k == "hand" and s == "real-wh110-manus" and not m}
    bridge_projects = hand_projects | ({operator_project} - foreign_hand_projects)
    motion, support = [], []
    for cid, project, kind, service, matching in eligible:
        if matching and ((kind == "arm" and service == "wrapper-real")
                         or (kind == "hand" and service == "real-wh110-manus")):
            motion.append(cid)
        elif ((kind == "arm" and project in arm_projects
               and service in {"receiver", "gen3-servo-real"})
              or (kind == "hand" and project in bridge_projects
                  and service == "manus-sdk-udp-bridge-unicast")
              or (kind == "arm" and service == "operator" and matching)):
            support.append(cid)
    return motion, support


def resolve_session(session):
    """Resolve an exact session name or ID before commands that target panes."""
    if not session:
        return ""
    try:
        result = run(["tmux", "list-sessions", "-F", "#{session_id}\t#{session_name}"])
    except FileNotFoundError:
        return ""
    if result.returncode == 0:
        for line in result.stdout.splitlines():
            session_id, _, name = line.partition("\t")
            if re.fullmatch(r"\$[0-9]+", session_id) and session in (session_id, name):
                return session_id
    return ""


def marked_session(session, repo, operator_dir):
    if not session:
        return False
    try:
        for option, expected in (("@gen3_repo_root", repo), ("@gen3_operator_dir", operator_dir)):
            result = run(["tmux", "show-options", "-qv", "-t", session, option])
            if result.returncode or result.stdout.strip() != str(Path(expected).resolve()):
                return False
    except FileNotFoundError:
        return False
    return True


def cancel_stack_windows(session):
    """Cancel build/startup supervisors while retaining our deadman window."""
    result = run(["tmux", "list-windows", "-t", session, "-F", "#{window_id}\t#{window_name}"], check=True)
    for line in result.stdout.splitlines():
        window_id, _, name = line.partition("\t")
        if name in {"gen3", "manus", "hand"} and re.fullmatch(r"@[0-9]+", window_id):
            run(["tmux", "kill-window", "-t", window_id])


def stop_groups(groups, repo):
    errors = []
    for ids in groups:
        if not ids:
            continue
        result = run(["docker", "stop", "--time", "3", *ids])
        if result.returncode:
            # --rm and the arm compose supervisor can win this race normally.
            remaining = {c["Id"] for c in inventory(repo)}.intersection(ids)
            if remaining:
                errors.append(result.stderr.strip() or f"Could not stop {sorted(remaining)}")
    return errors


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--operator-dir", type=Path, required=True)
    parser.add_argument("--operator-project", required=True)
    parser.add_argument("--tmux-session", default="")
    args = parser.parse_args(argv)
    repo, operator_dir = args.repo.resolve(), args.operator_dir.resolve()
    print("Q: stopping Gen3, WH110 and the teleop stack...", flush=True)
    # GUI already published STOP and closed its lease before returning 42.
    # Allow at least one 100 ms stale-lease interval before stopping processes.
    time.sleep(0.15)
    session = resolve_session(args.tmux_session)
    owns_session = marked_session(session, repo, operator_dir)
    initial = shutdown_groups(inventory(repo), repo, operator_dir, args.operator_project)
    if owns_session:
        cancel_stack_windows(session)
    current = shutdown_groups(inventory(repo), repo, operator_dir, args.operator_project)
    # Keep the initial arm anchor even if its mapper disappears during cleanup.
    groups = [list(dict.fromkeys(before + after)) for before, after in zip(initial, current)]
    errors = stop_groups(groups, repo)
    if errors:
        raise SystemExit("Shutdown incomplete: " + "; ".join(errors))
    print("Teleop stack stopped.", flush=True)
    if owns_session:
        # Last operation: this also closes the window that launched this helper.
        run(["tmux", "kill-session", "-t", session])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
