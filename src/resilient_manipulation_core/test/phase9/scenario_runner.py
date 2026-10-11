#!/usr/bin/env python3

import argparse
import re
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional

# ============================================================
# Phase 9 — Automated Scenario Test Runner
#
# Phase 9.2:
# - runner framework
# - scenario registration
# - dry-run support
# - log capture
# - exit-code + evidence-marker evaluation
#
# Actual P9-S01 execution will be verified in Phase 9.3.
# ============================================================


@dataclass(frozen=True)
class ScenarioDefinition:
    scenario_id: str
    name: str
    expected_exit_code: int
    expected_markers: List[str]
    environment: Dict[str, str]
    helper_command: Optional[List[str]] = None
    helper_expected_exit_code: Optional[int] = None
    forbidden_markers: Optional[List[str]] = None

SCENARIOS: Dict[str, ScenarioDefinition] = {

    "P9-S01": ScenarioDefinition(
        scenario_id="P9-S01",
        name="Fresh nominal",
        expected_exit_code=0,
        expected_markers=[
            "PHASE 8.2 RECOVERY RESULT: NOT_REQUIRED",
            "PHASE 8.3 PLANNING RECOVERY RESULT: NOT_REQUIRED",
            "PHASE 8.4 EXECUTION FAILURE HANDLING: NOT_REQUIRED",
            "PHASE 7 INTEGRATED ESTIMATE-DRIVEN MANIPULATION SUCCESS.",
        ],
        environment={},
        helper_command=[
            sys.executable,
            str(
                Path(__file__).resolve().parent
                / "controlled_estimate_publisher.py"
            ),
            "--mode",
            "fresh",
        ],
        helper_expected_exit_code=0,
        forbidden_markers=[
            "SAFE_ABORT",
        ],
    ),
    "P9-S02": ScenarioDefinition(
        scenario_id="P9-S02",
        name="Stale to fresh estimate recovery",
        expected_exit_code=0,
        expected_markers=[
            "PHASE 8.2 RECOVERY TRIGGERED",
            "PHASE 8.2 RECOVERY RESULT: RECOVERED",
            "PHASE 8.2 ESTIMATE ACQUISITION: PASS",
            "PHASE 7 INTEGRATED ESTIMATE-DRIVEN MANIPULATION SUCCESS.",
        ],
        environment={},
        helper_command=[
            sys.executable,
            str(
                Path(__file__).resolve().parent
                / "controlled_estimate_publisher.py"
            ),
        ],
        helper_expected_exit_code=0,
    ),
    "P9-S03": ScenarioDefinition(
        scenario_id="P9-S03",
        name="Persistent stale estimate safe abort",
        expected_exit_code=1,
        expected_markers=[
            "PHASE 8.2 RECOVERY TRIGGERED",
            "PHASE 8.2 RECOVERY RESULT: SAFE_ABORT",
        ],
        environment={},
        helper_command=[
            sys.executable,
            str(
                Path(__file__).resolve().parent
                / "controlled_estimate_publisher.py"
            ),
            "--mode",
            "persistent-stale",
        ],
        helper_expected_exit_code=0,
        forbidden_markers=[
            "[1/7] Moving arm to SRDF 'ready' state...",
        ],
    ),
    "P9-S04": ScenarioDefinition(
        scenario_id="P9-S04",
        name="Single planning failure recovery",
        expected_exit_code=0,
        expected_markers=[
            "Pose-goal planning attempt 1/2.",
            "PHASE 8.3 PLANNING RECOVERY TRIGGERED",
            "Pose-goal planning attempt 2/2.",
            "PHASE 8.3 PLANNING RECOVERY RESULT: RECOVERED | stage=Pose-goal",
            "PHASE 7 INTEGRATED ESTIMATE-DRIVEN MANIPULATION SUCCESS.",
        ],
        environment={
            "RESILIENT_TEST_PLANNING_FAILURE_STAGE": "Pose-goal",
        },
        helper_command=[
            sys.executable,
            str(
                Path(__file__).resolve().parent
                / "controlled_estimate_publisher.py"
            ),
            "--mode",
            "fresh",
        ],
        helper_expected_exit_code=0,
        forbidden_markers=[
            "SAFE_ABORT",
        ],
    ),
    "P9-S05": ScenarioDefinition(
        scenario_id="P9-S05",
        name="Persistent planning failure safe abort",
        expected_exit_code=1,
        expected_markers=[
            "Pose-goal planning attempt 1/2.",
            "PHASE 8.3 PLANNING RECOVERY TRIGGERED",
            "Pose-goal planning attempt 2/2.",
            "PHASE 8.3 PLANNING RECOVERY RESULT: SAFE_ABORT | stage=Pose-goal",
        ],
        environment={
            "RESILIENT_TEST_PERSISTENT_PLANNING_FAILURE_STAGE": "Pose-goal",
        },
        helper_command=[
            sys.executable,
            str(
                Path(__file__).resolve().parent
                / "controlled_estimate_publisher.py"
            ),
            "--mode",
            "fresh",
        ],
        helper_expected_exit_code=0,
        forbidden_markers=[
            "PHASE 8.3 PLANNING RECOVERY RESULT: RECOVERED | stage=Pose-goal",
            "PHASE 7 INTEGRATED ESTIMATE-DRIVEN MANIPULATION SUCCESS.",
        ],
    ),
    "P9-S06": ScenarioDefinition(
        scenario_id="P9-S06",
        name="Execution failure safe abort",
        expected_exit_code=1,
        expected_markers=[
            "Pose-goal planning SUCCESS on attempt 1.",
            "PHASE 8.3 PLANNING RECOVERY RESULT: NOT_REQUIRED | stage=Pose-goal",
            "PHASE 8.4 EXECUTION FAILURE HANDLING: SAFE_ABORT | stage=Pose-goal | automatic_retry=DISABLED",
            "Mission will not continue after execution failure.",
        ],
        environment={
            "RESILIENT_TEST_EXECUTION_FAILURE_STAGE": "Pose-goal",
        },
        helper_command=[
            sys.executable,
            str(
                Path(__file__).resolve().parent
                / "controlled_estimate_publisher.py"
            ),
            "--mode",
            "fresh",
        ],
        helper_expected_exit_code=0,
        forbidden_markers=[
            "Pose-goal planning attempt 2/2.",
            "PHASE 7 INTEGRATED ESTIMATE-DRIVEN MANIPULATION SUCCESS.",
        ],
    ),

}


MISSION_COMMAND = [
    "ros2",
    "launch",
    "resilient_manipulation_core",
    "manipulation_mission.launch.py",
]


MISSION_TIMEOUT_SECONDS = 120.0


TEST_ENVIRONMENT_VARIABLES = [
    "RESILIENT_TEST_PLANNING_FAILURE_STAGE",
    "RESILIENT_TEST_PERSISTENT_PLANNING_FAILURE_STAGE",
    "RESILIENT_TEST_EXECUTION_FAILURE_STAGE",
]


def build_environment(
    scenario: ScenarioDefinition,
) -> Dict[str, str]:
    """
    Build a clean scenario-specific environment.

    Existing Phase-8 fault-injection variables are removed first.
    Only variables explicitly required by the selected scenario
    are then applied.
    """

    environment = os.environ.copy()

    for variable_name in TEST_ENVIRONMENT_VARIABLES:
        environment.pop(variable_name, None)

    environment.update(scenario.environment)

    return environment

def print_scenario(
    scenario: ScenarioDefinition,
) -> None:
    print("=" * 70)
    print(
        f"Scenario ID   : {scenario.scenario_id}"
    )
    print(
        f"Scenario Name : {scenario.name}"
    )
    print(
        f"Expected Exit : {scenario.expected_exit_code}"
    )

    print("Expected Markers:")

    for marker in scenario.expected_markers:
        print(
            f"  - {marker}"
        )


    if scenario.forbidden_markers:
        print("Forbidden Markers:")

        for marker in scenario.forbidden_markers:
            print(
                f"  - {marker}"
            )


    if scenario.environment:
        print("Scenario Environment:")

        for key, value in scenario.environment.items():
            print(
                f"  - {key}={value}"
            )

    else:
        print(
            "Scenario Environment: no fault injection"
        )


    if scenario.helper_command is not None:
        print(
            "Scenario Helper : enabled"
        )

    else:
        print(
            "Scenario Helper : none"
        )


    print("=" * 70)

def terminate_process(
    process: subprocess.Popen,
) -> None:
    """
    Terminate the ros2 launch process group cleanly.
    """

    if process.poll() is not None:
        return

    try:
        os.killpg(
            os.getpgid(process.pid),
            signal.SIGINT,
        )

        process.wait(timeout=5.0)

    except subprocess.TimeoutExpired:
        os.killpg(
            os.getpgid(process.pid),
            signal.SIGKILL,
        )

        process.wait()

    except ProcessLookupError:
        pass
def extract_mission_exit_code(
    output_text: str,
):
    """
    Extract the actual manipulation_mission child-process
    exit code from ros2 launch output.
    """

    if "process has finished cleanly" in output_text:
        return 0

    matches = re.findall(
        r"process has died .*?exit code (-?\d+)",
        output_text,
    )

    if matches:
        return int(matches[-1])

    return None

def evaluate_result(
    scenario: ScenarioDefinition,
    launch_exit_code: int,
    mission_exit_code,
    helper_exit_code,
    output_text: str,
) -> bool:


    """
    Evaluate expected process exit behavior and evidence markers.
    """

    exit_code_pass = (
        mission_exit_code == scenario.expected_exit_code
    )
    missing_markers = [
        marker
        for marker in scenario.expected_markers
        if marker not in output_text
    ]

    markers_pass = (
        len(missing_markers) == 0
    )
    forbidden_markers_found = []

    if scenario.forbidden_markers is not None:
        forbidden_markers_found = [
            marker
            for marker in scenario.forbidden_markers
            if marker in output_text
        ]

    forbidden_markers_pass = (
        len(forbidden_markers_found) == 0
    )
    helper_exit_pass = True

    if scenario.helper_expected_exit_code is not None:
        helper_exit_pass = (
            helper_exit_code
            == scenario.helper_expected_exit_code
        )

    if scenario.forbidden_markers is not None:
        if forbidden_markers_pass:
            print(
                "Forbidden markers : PASS"
            )

        else:
            print(
                "Forbidden markers : FAIL"
            )

            print(
                "Unexpected markers found:"
            )

            for marker in forbidden_markers_found:
                print(
                    f"  - {marker}"
                )


    print()
    print("=" * 70)
    print("PHASE 9 SCENARIO RESULT")
    print("=" * 70)

    print(f"Scenario          : {scenario.scenario_id}")
    print(
        f"Expected mission exit : "
        f"{scenario.expected_exit_code}"
    )

    print(
        f"Actual mission exit   : "
        f"{mission_exit_code}"
    )

    print(
        f"ros2 launch exit      : "
        f"{launch_exit_code}"
    )

    print(
        "Mission-exit check    : "
        + ("PASS" if exit_code_pass else "FAIL")
    )
    if scenario.helper_expected_exit_code is not None:
        print(
            f"Expected helper exit  : "
            f"{scenario.helper_expected_exit_code}"
        )

        print(
            f"Actual helper exit    : "
            f"{helper_exit_code}"
        )

        print(
            "Helper-exit check     : "
            + ("PASS" if helper_exit_pass else "FAIL")
        )
    if markers_pass:
        print("Evidence markers  : PASS")
    else:
        print("Evidence markers  : FAIL")
        print("Missing markers:")

        for marker in missing_markers:
            print(f"  - {marker}")

    overall_pass = (
        exit_code_pass
        and helper_exit_pass
        and markers_pass
        and forbidden_markers_pass
    )
    print(
        "AUTOMATED RESULT  : "
        + ("PASS" if overall_pass else "FAIL")
    )

    print("=" * 70)

    return overall_pass


def run_scenario(
    scenario: ScenarioDefinition,
) -> bool:
    """
    Execute one registered scenario.

    MoveIt, controllers and RViz remain external infrastructure.
    """

    script_directory = (
        Path(__file__).resolve().parent
    )

    log_directory = (
        script_directory / "logs"
    )

    log_directory.mkdir(
        parents=True,
        exist_ok=True,
    )

    log_file = (
        log_directory
        / f"{scenario.scenario_id}.log"
    )

    environment = build_environment(
        scenario
    )

    print_scenario(
        scenario
    )
    helper_process = None

    helper_log_file = (
        log_directory
        / f"{scenario.scenario_id}_helper.log"
    )


    print("Launching manipulation mission...")
    print(f"Log file: {log_file}")
    print()

    process = subprocess.Popen(
        MISSION_COMMAND,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
        env=environment,
        start_new_session=True,
    )

    output_lines: List[str] = []

    start_time = time.monotonic()

    try:
        assert process.stdout is not None

        while True:
            elapsed_time = (
                time.monotonic()
                - start_time
            )

            if elapsed_time > MISSION_TIMEOUT_SECONDS:
                print()
                print(
                    "ERROR: mission timeout reached."
                )

                terminate_process(
                    process
                )

                output_lines.append(
                    "\nPHASE9_RUNNER_TIMEOUT\n"
                )

                exit_code = process.returncode

                if exit_code is None:
                    exit_code = -1

                break

            line = process.stdout.readline()

            if line:
                print(
                    line,
                    end="",
                )

                output_lines.append(
                    line
                )
                if (
                    scenario.helper_command is not None
                    and helper_process is None
                    and
                    "PHASE 7 — Waiting for valid /object_pose_estimate..."
                    in line
                ):
                    print(
                        "PHASE 9 RUNNER: mission estimate interface ready."
                    )

                    print(
                        "Launching scenario helper..."
                    )

                    helper_process = subprocess.Popen(
                        scenario.helper_command,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.STDOUT,
                        text=True,
                        env=environment,
                        start_new_session=True,
                    )
            if (
                process.poll() is not None
                and not line
            ):
                exit_code = process.returncode

                if exit_code is None:
                    exit_code = -1

                break

        output_text = "".join(
            output_lines
        )

        log_file.write_text(
            output_text,
            encoding="utf-8",
        )
        helper_exit_code = None


        if helper_process is not None:
            try:
                helper_output, _ = (
                    helper_process.communicate(
                        timeout=5.0
                    )
                )

            except subprocess.TimeoutExpired:
                terminate_process(
                    helper_process
                )

                helper_output, _ = (
                    helper_process.communicate()
                )


            helper_exit_code = (
                helper_process.returncode
            )

            if helper_exit_code is None:
                helper_exit_code = -1


            helper_log_file.write_text(
                helper_output or "",
                encoding="utf-8",
            )
        mission_exit_code = extract_mission_exit_code(
            output_text
        )

        return evaluate_result(
            scenario,
            exit_code,
            mission_exit_code,
            helper_exit_code,
            output_text,
        )
    except KeyboardInterrupt:
        print()
        print(
            "Runner interrupted by user."
        )

        terminate_process(
            process
        )
        if helper_process is not None:
            terminate_process(
                helper_process
            )

        return False


def list_scenarios() -> None:
    print(
        "Registered Phase-9 scenarios:"
    )

    for scenario in SCENARIOS.values():
        print(
            f"{scenario.scenario_id}: "
            f"{scenario.name}"
        )


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Phase-9 automated scenario test runner."
        )
    )

    parser.add_argument(
        "--scenario",
        choices=sorted(
            SCENARIOS.keys()
        ),
        help="Scenario to execute.",
    )

    parser.add_argument(
        "--list",
        action="store_true",
        help="List registered scenarios.",
    )

    parser.add_argument(
        "--dry-run",
        action="store_true",
        help=(
            "Show scenario configuration without "
            "launching the mission."
        ),
    )

    return parser.parse_args()


def main() -> int:
    arguments = parse_arguments()

    if arguments.list:
        list_scenarios()

        return 0

    if arguments.scenario is None:
        print(
            "ERROR: specify --scenario or --list."
        )

        return 2

    scenario = SCENARIOS[
        arguments.scenario
    ]

    if arguments.dry_run:
        print_scenario(
            scenario
        )

        print(
            "DRY RUN ONLY — mission was not launched."
        )

        return 0

    passed = run_scenario(
        scenario
    )

    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(
        main()
    )
