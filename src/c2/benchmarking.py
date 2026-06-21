"""Mock-first C2 benchmark readiness report for simulator profile schemas."""

from __future__ import annotations

import copy
import json
from typing import Dict, List, Optional, Sequence

from src.c2.dashboard_replay import C2MetricsExporter, C2ReplayReportBuilder, DashboardReplayBuilder
from src.c2.mission_types import (
    FleetAsset,
    HumanApprovalRecord,
    MissionEvent,
    MissionTask,
    PlannedRoute,
    RiskSignal,
    SafetyDecision,
    ThreatAssessment,
)


SCHEMA_VERSION = "v2-6-c2-benchmark-readiness"
PROFILE_NAMES = ("mock", "isaac_readiness", "cosys_airsim_family_readiness")
SCHEMA_GROUPS = ("mission", "risk", "route", "replay", "dashboard", "metrics")


class C2BenchmarkReadinessBuilder:
    """Compare C2 schema readiness across mock and simulator-readiness profiles.

    The builder is read-only and runtime-free. It does not import or probe
    Isaac Sim, Cosys-AirSim, legacy AirSim, ROS2, MAVSDK, PX4, Nav2, network,
    database, or hardware packages.
    """

    def __init__(
        self,
        dashboard_builder: Optional[DashboardReplayBuilder] = None,
        metrics_exporter: Optional[C2MetricsExporter] = None,
        report_builder: Optional[C2ReplayReportBuilder] = None,
    ) -> None:
        if dashboard_builder is not None and not isinstance(dashboard_builder, DashboardReplayBuilder):
            raise ValueError("dashboard_builder must be a DashboardReplayBuilder")
        if metrics_exporter is not None and not isinstance(metrics_exporter, C2MetricsExporter):
            raise ValueError("metrics_exporter must be a C2MetricsExporter")
        if report_builder is not None and not isinstance(report_builder, C2ReplayReportBuilder):
            raise ValueError("report_builder must be a C2ReplayReportBuilder")
        self.dashboard_builder = dashboard_builder or DashboardReplayBuilder()
        self.metrics_exporter = metrics_exporter or C2MetricsExporter()
        self.report_builder = report_builder or C2ReplayReportBuilder(
            dashboard_builder=self.dashboard_builder,
            metrics_exporter=self.metrics_exporter,
        )

    def build_readiness_report(
        self,
        events: Optional[Sequence[MissionEvent]] = None,
    ) -> Dict[str, object]:
        checked_events = self._validate_events(events or default_benchmark_events())
        replay_payload = self.dashboard_builder.build_replay_payload(checked_events)
        metrics_summary = self.metrics_exporter.summarize_events(checked_events)
        metrics_payload = self.metrics_exporter.build_metrics_payload(metrics_summary)
        audit_report = self.report_builder.build_json_report(replay_payload, metrics_payload)
        schema_reference = self.build_schema_reference(replay_payload, metrics_payload, audit_report)
        profiles = self.build_profiles(schema_reference)
        report = {
            "schema_version": SCHEMA_VERSION,
            "status": "passed",
            "profile_count": len(profiles),
            "profiles": profiles,
            "schema_groups": schema_reference,
            "compatibility_matrix": self.build_compatibility_matrix(profiles),
            "source_fixture_summary": {
                "event_count": len(checked_events),
                "event_types": sorted({event.event_type for event in checked_events}),
                "timeline_families": self._timeline_families(replay_payload),
            },
            "safety_summary": {
                "read_only": True,
                "runtime_free": True,
                "simulators_launched": False,
                "runtime_connections_attempted": False,
                "hardware_connections_attempted": False,
                "vehicle_commands_enabled": False,
                "route_upload_enabled": False,
                "simulator_parity_claimed": False,
                "production_readiness_claimed": False,
                "offensive_automation_enabled": False,
            },
            "audit_boundary": {
                "mock_first": True,
                "schema_only": True,
                "command_free": True,
                "no_runtime_probe": True,
                "no_file_writes": True,
            },
        }
        return self._json_safe_dict(report)

    def build_schema_reference(
        self,
        replay_payload: Dict[str, object],
        metrics_payload: Dict[str, object],
        audit_report: Dict[str, object],
    ) -> Dict[str, object]:
        if not isinstance(replay_payload, dict):
            raise ValueError("invalid_replay_payload: replay payload must be a dict")
        if not isinstance(metrics_payload, dict):
            raise ValueError("invalid_metrics_payload: metrics payload must be a dict")
        if not isinstance(audit_report, dict):
            raise ValueError("invalid_audit_report: audit report must be a dict")
        timeline = replay_payload.get("timeline", [])
        if not isinstance(timeline, list):
            timeline = []
        final_snapshot = replay_payload.get("final_snapshot", {})
        if not isinstance(final_snapshot, dict):
            final_snapshot = {}
        reference = {
            "mission": {
                "event_families": self._filter_families(timeline, {"mission", "fleet", "uav"}),
                "snapshot_sections": [
                    key
                    for key in ("mission_requests", "mission_tasks", "fleet_assets", "uav_states")
                    if key in final_snapshot
                ],
                "schema_classes": ["MissionEvent", "MissionTask", "FleetAsset", "UAVState"],
            },
            "risk": {
                "event_families": self._filter_families(timeline, {"risk", "threat"}),
                "metric_sections": ["risk_counts"],
                "schema_classes": ["RiskSignal", "ThreatAssessment"],
            },
            "route": {
                "event_families": self._filter_families(timeline, {"route", "safety", "human_approval"}),
                "metric_sections": ["route_verdict_counts", "safety_decision_counts", "human_approval_counts"],
                "schema_classes": ["PlannedRoute", "SafetyDecision", "HumanApprovalRecord"],
            },
            "replay": {
                "payload_schema_version": replay_payload.get("schema_version"),
                "payload_keys": sorted(str(key) for key in replay_payload),
                "frame_count": replay_payload.get("frame_count", 0),
            },
            "dashboard": {
                "timeline_entry_keys": self._timeline_entry_keys(timeline),
                "snapshot_keys": sorted(str(key) for key in final_snapshot),
                "audit_report_schema_version": audit_report.get("schema_version"),
            },
            "metrics": {
                "payload_schema_version": metrics_payload.get("schema_version"),
                "payload_keys": sorted(str(key) for key in metrics_payload),
                "required_sections": [
                    "event_type_counts",
                    "risk_counts",
                    "route_verdict_counts",
                    "task_status_counts",
                    "fleet_assignment_counts",
                    "safety_decision_counts",
                    "human_approval_counts",
                ],
            },
        }
        return self._json_safe_dict(reference)

    def build_profiles(self, schema_reference: Dict[str, object]) -> List[Dict[str, object]]:
        profiles = [
            self._profile(
                name="mock",
                label="Mock C2 schema fixture",
                source="in_memory_c2_fixture",
                readiness_status="ready",
                default_profile=True,
                coordinate_frame="project_default",
                runtime_family="mock",
                primary_runtime=None,
                fallback_runtime=None,
                phase6_mainline=False,
                schema_reference=schema_reference,
            ),
            self._profile(
                name="isaac_readiness",
                label="Isaac Sim / Isaac Lab readiness profile",
                source="readiness_profile_only",
                readiness_status="schema_ready_runtime_gated",
                default_profile=False,
                coordinate_frame="isaac_z_up_metadata_only",
                runtime_family="isaac",
                primary_runtime="isaacsim",
                fallback_runtime=None,
                phase6_mainline=True,
                schema_reference=schema_reference,
            ),
            self._profile(
                name="cosys_airsim_family_readiness",
                label="Cosys-AirSim-family readiness profile",
                source="readiness_profile_only",
                readiness_status="schema_ready_runtime_gated",
                default_profile=False,
                coordinate_frame="airsim_ned_metadata_only",
                runtime_family="airsim",
                primary_runtime="cosysairsim",
                fallback_runtime="airsim",
                phase6_mainline=False,
                schema_reference=schema_reference,
            ),
        ]
        return profiles

    def build_compatibility_matrix(self, profiles: Sequence[Dict[str, object]]) -> Dict[str, object]:
        matrix: Dict[str, object] = {}
        for profile in profiles:
            name = str(profile["name"])
            matrix[name] = {
                group: {
                    "compatible": True,
                    "comparison_scope": "schema_readiness_only",
                    "runtime_required": False,
                }
                for group in SCHEMA_GROUPS
            }
        return self._json_safe_dict(matrix)

    def _profile(
        self,
        name: str,
        label: str,
        source: str,
        readiness_status: str,
        default_profile: bool,
        coordinate_frame: str,
        runtime_family: str,
        primary_runtime: Optional[str],
        fallback_runtime: Optional[str],
        phase6_mainline: bool,
        schema_reference: Dict[str, object],
    ) -> Dict[str, object]:
        profile = {
            "name": name,
            "label": label,
            "source": source,
            "readiness_status": readiness_status,
            "default_profile": default_profile,
            "runtime_family": runtime_family,
            "primary_runtime": primary_runtime,
            "fallback_runtime": fallback_runtime,
            "phase6_mainline": phase6_mainline,
            "schema_groups": list(SCHEMA_GROUPS),
            "schema_group_count": len(SCHEMA_GROUPS),
            "coordinate_frame": coordinate_frame,
            "coordinate_conversion_applied": False,
            "runtime_required_for_schema_check": False,
            "runtime_availability_probed": False,
            "runtime_connection_attempted": False,
            "simulator_launched": False,
            "hardware_connection_attempted": False,
            "vehicle_command_enabled": False,
            "route_upload_enabled": False,
            "safety_gate_required_for_future_execution": True,
            "human_approval_required_for_future_execution": True,
            "performance_parity_claimed": False,
            "schema_reference_keys": sorted(str(key) for key in schema_reference),
        }
        return self._json_safe_dict(profile)

    def _validate_events(self, events: Sequence[MissionEvent]) -> List[MissionEvent]:
        if isinstance(events, (str, bytes)) or not isinstance(events, Sequence):
            raise ValueError("invalid_events: events must be a sequence of MissionEvent objects")
        checked_events: List[MissionEvent] = []
        for event in events:
            if not isinstance(event, MissionEvent):
                raise ValueError("invalid_events: events must be a sequence of MissionEvent objects")
            event.validate()
            checked_events.append(copy.deepcopy(event))
        return checked_events

    @staticmethod
    def _filter_families(timeline: Sequence[object], families: set[str]) -> List[str]:
        values = []
        for entry in timeline:
            if isinstance(entry, dict) and entry.get("family") in families:
                values.append(str(entry["family"]))
        return sorted(set(values))

    @staticmethod
    def _timeline_entry_keys(timeline: Sequence[object]) -> List[str]:
        if not timeline:
            return []
        first = timeline[0]
        if not isinstance(first, dict):
            return []
        return sorted(str(key) for key in first)

    @staticmethod
    def _timeline_families(replay_payload: Dict[str, object]) -> List[str]:
        timeline = replay_payload.get("timeline", [])
        if not isinstance(timeline, list):
            return []
        families = [
            str(entry["family"])
            for entry in timeline
            if isinstance(entry, dict) and isinstance(entry.get("family"), str)
        ]
        return sorted(set(families))

    @staticmethod
    def _json_safe_dict(value: Dict[str, object]) -> Dict[str, object]:
        try:
            json.dumps(value, allow_nan=False, sort_keys=True)
        except (TypeError, ValueError) as exc:
            raise ValueError("benchmark readiness payload must be JSON-safe") from exc
        return value


def default_benchmark_events() -> List[MissionEvent]:
    """Return deterministic, command-free C2 events for schema comparison."""
    task = MissionTask(
        task_id="task-001",
        request_id="req-001",
        objective="Benchmark schema readiness",
        status="assigned",
        priority=1,
        assigned_asset_id="uav-001",
    )
    asset = FleetAsset(
        asset_id="uav-001",
        backend="mock",
        capabilities=["survey"],
        available=True,
        current_task_id="task-001",
    )
    risk = RiskSignal(
        signal_id="risk-001",
        category="communication degradation",
        severity=0.2,
        confidence=0.8,
        timestamp=3.0,
    )
    assessment = ThreatAssessment(
        assessment_id="assessment-001",
        mission_id="req-001",
        risk_signals=[{"signal_id": "risk-001", "category": "communication degradation"}],
        total_risk=0.2,
        recommendation="continue",
        explanation="Mock benchmark risk remains low.",
        timestamp=4.0,
    )
    route = PlannedRoute(
        route_id="route-001",
        task_id="task-001",
        waypoints=[{"x": 0.0, "y": 0.0}, {"x": 1.0, "y": 1.0}],
        score=1.0,
        risk_score=0.2,
        constraint_verdict="valid",
        metadata={"selected": True, "executable": False},
    )
    decision = SafetyDecision(
        decision_id="decision-001",
        target_id="route-001",
        status="needs_review",
        reason="Benchmark reports are observational only.",
        requires_human_approval=True,
        timestamp=6.0,
    )
    approval = HumanApprovalRecord(
        approval_id="approval-001",
        operator_id="operator-a",
        target_id="route-001",
        decision="deferred",
        notes="No execution requested.",
        timestamp=7.0,
    )
    return [
        _event("evt-000001", "mission.task.created", task.to_dict(), 1.0),
        _event("evt-000002", "fleet.asset.updated", asset.to_dict(), 2.0),
        _event("evt-000003", "risk.signal.created", risk.to_dict(), 3.0),
        _event("evt-000004", "threat.assessment.created", assessment.to_dict(), 4.0),
        _event("evt-000005", "route.planned", route.to_dict(), 5.0),
        _event("evt-000006", "safety.decision.created", decision.to_dict(), 6.0),
        _event("evt-000007", "human_approval.recorded", approval.to_dict(), 7.0),
    ]


def build_c2_benchmark_readiness_report(
    events: Optional[Sequence[MissionEvent]] = None,
) -> Dict[str, object]:
    """Build the default mock-first C2 benchmark-readiness report."""
    return C2BenchmarkReadinessBuilder().build_readiness_report(events)


def _event(event_id: str, event_type: str, payload: Dict[str, object], timestamp: float) -> MissionEvent:
    return MissionEvent(
        event_id=event_id,
        event_type=event_type,
        timestamp=timestamp,
        source="c2_benchmark_readiness",
        payload=payload,
        metadata={"benchmark_readiness_fixture": True},
    )
