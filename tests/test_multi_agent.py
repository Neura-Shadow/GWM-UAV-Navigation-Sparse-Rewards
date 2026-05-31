"""Tests for the Multi-Agent (Axis 4) subsystem — WP7.

Covers:
- AgentRegistry: CRUD, filtered queries, unregister
- SharedSpatiotemporalMap: update, query, positions, cleanup
- MockDDSChannel: send/receive, subscription filtering
- SwarmCoordinator: task assignment, conflict detection
"""

from __future__ import annotations

import time
from typing import Any, Dict, List

import numpy as np
import pytest

from src.multi_agent.agent_state import AgentRegistry
from src.multi_agent.communication import (
    AgentMessage,
    MessageType,
    MockDDSChannel,
    QoSProfile,
)
from src.multi_agent.shared_map import SharedSpatiotemporalMap
from src.multi_agent.swarm_coordinator import SwarmCoordinator
from src.utils.data_types import AgentState, AgentStatus, SensorObservation, VehicleType


# =========================================================================
# Fixtures / helpers
# =========================================================================

def _make_agent(
    agent_id: str,
    vehicle_type: VehicleType = VehicleType.UAV,
    status: AgentStatus = AgentStatus.IDLE,
    pose: tuple = (0.0, 0.0, 0.0),
) -> AgentState:
    """Create a simple test agent."""
    obs = SensorObservation(
        timestamp=time.time(),
        pose=pose,
        velocity=(0.0, 0.0, 0.0),
    )
    return AgentState(
        agent_id=agent_id,
        vehicle_type=vehicle_type,
        observation=obs,
        status=status,
    )


# =========================================================================
# AgentRegistry
# =========================================================================

class TestAgentRegistry:
    """Tests for AgentRegistry."""

    def test_agent_registry_register_and_get(self) -> None:
        """Register an agent and retrieve it by ID."""
        reg = AgentRegistry()
        agent = _make_agent("uav_1")
        reg.register(agent)

        result = reg.get("uav_1")
        assert result is not None
        assert result.agent_id == "uav_1"
        assert len(reg) == 1

    def test_agent_registry_get_by_type(self) -> None:
        """Filter agents by vehicle type."""
        reg = AgentRegistry()
        reg.register(_make_agent("uav_1", VehicleType.UAV))
        reg.register(_make_agent("ugv_1", VehicleType.UGV))
        reg.register(_make_agent("uav_2", VehicleType.UAV))

        uavs = reg.get_by_type(VehicleType.UAV)
        assert len(uavs) == 2
        assert all(a.vehicle_type == VehicleType.UAV for a in uavs)

        ugvs = reg.get_by_type(VehicleType.UGV)
        assert len(ugvs) == 1

    def test_agent_registry_get_by_status(self) -> None:
        """Filter agents by status."""
        reg = AgentRegistry()
        reg.register(_make_agent("a1", status=AgentStatus.IDLE))
        reg.register(_make_agent("a2", status=AgentStatus.NAVIGATING))
        reg.register(_make_agent("a3", status=AgentStatus.IDLE))

        idle = reg.get_by_status(AgentStatus.IDLE)
        assert len(idle) == 2

        navigating = reg.get_by_status(AgentStatus.NAVIGATING)
        assert len(navigating) == 1

    def test_agent_registry_unregister(self) -> None:
        """Unregister removes the agent; repeat raises KeyError."""
        reg = AgentRegistry()
        reg.register(_make_agent("a1"))
        assert len(reg) == 1

        reg.unregister("a1")
        assert len(reg) == 0
        assert reg.get("a1") is None

        with pytest.raises(KeyError):
            reg.unregister("a1")

    def test_agent_registry_update_state(self) -> None:
        """Update fields on a registered agent."""
        reg = AgentRegistry()
        reg.register(_make_agent("a1"))
        reg.update_state("a1", status=AgentStatus.NAVIGATING, task_id="task_42")

        agent = reg.get("a1")
        assert agent is not None
        assert agent.status == AgentStatus.NAVIGATING
        assert agent.task_id == "task_42"

    def test_agent_registry_get_all(self) -> None:
        """get_all() returns all registered agents."""
        reg = AgentRegistry()
        for i in range(5):
            reg.register(_make_agent(f"a{i}"))
        assert len(reg.get_all()) == 5

    def test_agent_registry_contains(self) -> None:
        """__contains__ works for membership checks."""
        reg = AgentRegistry()
        reg.register(_make_agent("a1"))
        assert "a1" in reg
        assert "a99" not in reg


# =========================================================================
# SharedSpatiotemporalMap
# =========================================================================

class TestSharedSpatiotemporalMap:
    """Tests for SharedSpatiotemporalMap."""

    def test_shared_map_update_and_query(self) -> None:
        """Update with an agent, then query nearby → find it."""
        smap = SharedSpatiotemporalMap(
            bounds=(-100, -100, -50, 100, 100, 0),
            resolution=1.0,
            time_window=30.0,
        )
        smap.update("uav_1", (10.0, 10.0, -5.0), timestamp=1.0)
        smap.update("uav_2", (12.0, 10.0, -5.0), timestamp=1.0)

        result = smap.query((11.0, 10.0, -5.0), radius=5.0)
        assert "uav_1" in result["nearby_agents"]
        assert "uav_2" in result["nearby_agents"]
        assert result["count"] == 2

    def test_shared_map_query_excludes_far_agents(self) -> None:
        """Agents outside the radius are excluded from the query."""
        smap = SharedSpatiotemporalMap()
        smap.update("near", (5.0, 0.0, 0.0), timestamp=1.0)
        smap.update("far", (500.0, 0.0, 0.0), timestamp=1.0)

        result = smap.query((0.0, 0.0, 0.0), radius=10.0)
        assert "near" in result["nearby_agents"]
        assert "far" not in result["nearby_agents"]

    def test_shared_map_agent_positions(self) -> None:
        """get_agent_positions returns most recent positions."""
        smap = SharedSpatiotemporalMap()
        smap.update("a1", (1.0, 2.0, 3.0), timestamp=10.0)
        smap.update("a2", (4.0, 5.0, 6.0), timestamp=10.0)

        positions = smap.get_agent_positions(max_age=5.0)
        assert "a1" in positions
        assert "a2" in positions
        assert positions["a1"] == (1.0, 2.0, 3.0)

    def test_shared_map_agent_positions_filters_old(self) -> None:
        """Old entries are excluded from get_agent_positions."""
        smap = SharedSpatiotemporalMap()
        smap.update("old", (1.0, 0.0, 0.0), timestamp=1.0)
        smap.update("new", (2.0, 0.0, 0.0), timestamp=100.0)

        positions = smap.get_agent_positions(max_age=5.0)
        assert "old" not in positions
        assert "new" in positions

    def test_shared_map_clear_old_entries(self) -> None:
        """clear_old_entries removes old history entries."""
        smap = SharedSpatiotemporalMap()
        smap.update("a1", (0.0, 0.0, 0.0), timestamp=1.0)
        smap.update("a1", (1.0, 0.0, 0.0), timestamp=5.0)
        smap.update("a1", (2.0, 0.0, 0.0), timestamp=10.0)

        removed = smap.clear_old_entries(max_age=3.0)
        assert removed == 2  # entries at t=1 and t=5 should be removed

    def test_shared_map_empty_clear(self) -> None:
        """clear_old_entries on empty map returns 0."""
        smap = SharedSpatiotemporalMap()
        assert smap.clear_old_entries(max_age=5.0) == 0


# =========================================================================
# MockDDSChannel
# =========================================================================

class TestMockDDSChannel:
    """Tests for MockDDSChannel."""

    def test_mock_dds_channel_send_receive(self) -> None:
        """Send a message → subscriber receives it."""
        ch = MockDDSChannel()
        ch.subscribe("receiver_1", [MessageType.STATE_BROADCAST])

        msg = AgentMessage(
            sender_id="sender_1",
            message_type=MessageType.STATE_BROADCAST,
            payload={"position": [1, 2, 3]},
            timestamp=time.time(),
        )
        ok = ch.send(msg)
        assert ok is True

        inbox = ch.receive("receiver_1")
        assert len(inbox) == 1
        assert inbox[0].sender_id == "sender_1"

    def test_mock_dds_channel_no_self_delivery(self) -> None:
        """Sender should not receive their own messages."""
        ch = MockDDSChannel()
        ch.subscribe("agent_A", [MessageType.STATE_BROADCAST])

        msg = AgentMessage(
            sender_id="agent_A",
            message_type=MessageType.STATE_BROADCAST,
            payload={},
            timestamp=time.time(),
        )
        ch.send(msg)

        inbox = ch.receive("agent_A")
        assert len(inbox) == 0

    def test_mock_dds_channel_subscribe_filter(self) -> None:
        """Only subscribed message types are delivered."""
        ch = MockDDSChannel()
        ch.subscribe("r1", [MessageType.EMERGENCY_ALERT])

        # Send a STATE_BROADCAST — r1 is NOT subscribed to it
        msg_state = AgentMessage(
            sender_id="s1",
            message_type=MessageType.STATE_BROADCAST,
            payload={},
            timestamp=time.time(),
        )
        ch.send(msg_state)

        # Send an EMERGENCY_ALERT — r1 IS subscribed
        msg_alert = AgentMessage(
            sender_id="s1",
            message_type=MessageType.EMERGENCY_ALERT,
            payload={"alert": "collision"},
            timestamp=time.time(),
        )
        ch.send(msg_alert)

        inbox = ch.receive("r1")
        assert len(inbox) == 1
        assert inbox[0].message_type == MessageType.EMERGENCY_ALERT

    def test_mock_dds_channel_priority_ordering(self) -> None:
        """Higher-priority messages appear first in receive()."""
        ch = MockDDSChannel()
        ch.subscribe("r1", [MessageType.STATE_BROADCAST, MessageType.EMERGENCY_ALERT])

        now = time.time()
        ch.send(AgentMessage("s1", MessageType.STATE_BROADCAST, {}, now, priority=0))
        ch.send(AgentMessage("s2", MessageType.EMERGENCY_ALERT, {}, now, priority=10))

        inbox = ch.receive("r1")
        assert len(inbox) == 2
        assert inbox[0].priority >= inbox[1].priority

    def test_mock_dds_channel_history_depth(self) -> None:
        """Inbox is bounded by QoS history_depth."""
        qos = QoSProfile(history_depth=3)
        ch = MockDDSChannel(default_qos=qos)
        ch.subscribe("r1", [MessageType.STATE_BROADCAST])

        now = time.time()
        for i in range(10):
            ch.send(AgentMessage(f"s{i}", MessageType.STATE_BROADCAST, {}, now))

        inbox = ch.receive("r1")
        assert len(inbox) <= 3

    def test_mock_dds_sent_log(self) -> None:
        """All sent messages are recorded in sent_log regardless of delivery."""
        ch = MockDDSChannel()
        now = time.time()
        for i in range(5):
            ch.send(AgentMessage(f"s{i}", MessageType.STATE_BROADCAST, {}, now))
        assert len(ch.sent_log) == 5


# =========================================================================
# SwarmCoordinator
# =========================================================================

class TestSwarmCoordinator:
    """Tests for SwarmCoordinator."""

    def _build_coordinator(self) -> SwarmCoordinator:
        """Set up a coordinator with a 3-agent fleet."""
        reg = AgentRegistry()
        smap = SharedSpatiotemporalMap()
        ch = MockDDSChannel()

        # Subscribe all agents to all message types
        for aid in ("uav_1", "uav_2", "uav_3"):
            ch.subscribe(aid, list(MessageType))

        for aid, pos in [("uav_1", (0, 0, 0)), ("uav_2", (10, 0, 0)), ("uav_3", (20, 0, 0))]:
            agent = _make_agent(aid, pose=pos)
            reg.register(agent)
            smap.update(aid, pos, timestamp=time.time())

        return SwarmCoordinator(reg, smap, ch, safety_radius=5.0)

    def test_swarm_coordinator_assign_tasks(self) -> None:
        """Tasks are assigned to idle agents; returned mapping is correct."""
        coord = self._build_coordinator()
        tasks = [
            {"task_id": "t1", "goal": [50, 0, 0]},
            {"task_id": "t2", "goal": [60, 0, 0]},
        ]
        assignments = coord.assign_tasks(tasks)

        assert len(assignments) == 2
        assert "t1" in assignments.values()
        assert "t2" in assignments.values()

        # Assigned agents should now be NAVIGATING
        for aid in assignments:
            agent = coord.registry.get(aid)
            assert agent is not None
            assert agent.status == AgentStatus.NAVIGATING

    def test_swarm_coordinator_no_idle_agents(self) -> None:
        """If all agents are busy, only available ones are assigned."""
        coord = self._build_coordinator()
        # Mark all as NAVIGATING
        for agent in coord.registry.get_all():
            coord.registry.update_state(agent.agent_id, status=AgentStatus.NAVIGATING)

        assignments = coord.assign_tasks([{"task_id": "t1"}])
        assert len(assignments) == 0

    def test_swarm_coordinator_check_conflicts(self) -> None:
        """Two agents within safety_radius → conflict detected."""
        reg = AgentRegistry()
        smap = SharedSpatiotemporalMap()
        ch = MockDDSChannel()

        now = time.time()
        # Place agents very close together
        reg.register(_make_agent("a1", pose=(0, 0, 0)))
        reg.register(_make_agent("a2", pose=(2, 0, 0)))
        smap.update("a1", (0, 0, 0), now)
        smap.update("a2", (2, 0, 0), now)

        coord = SwarmCoordinator(reg, smap, ch, safety_radius=5.0)
        conflicts = coord.check_conflicts()

        assert len(conflicts) >= 1
        assert conflicts[0]["distance"] < 5.0

    def test_swarm_coordinator_no_conflicts(self) -> None:
        """Widely spaced agents → no conflicts."""
        coord = self._build_coordinator()
        # Default spacing is (0,0,0), (10,0,0), (20,0,0) — all > 5m apart
        conflicts = coord.check_conflicts()
        assert len(conflicts) == 0

    def test_swarm_coordinator_resolve_conflicts(self) -> None:
        """Conflict resolution produces avoidance commands."""
        reg = AgentRegistry()
        smap = SharedSpatiotemporalMap()
        ch = MockDDSChannel()

        now = time.time()
        smap.update("a1", (0, 0, 0), now)
        smap.update("a2", (1, 0, 0), now)
        reg.register(_make_agent("a1", pose=(0, 0, 0)))
        reg.register(_make_agent("a2", pose=(1, 0, 0)))

        coord = SwarmCoordinator(reg, smap, ch, safety_radius=5.0)
        conflicts = coord.check_conflicts()
        commands = coord.resolve_conflicts(conflicts)

        # Should produce 2 commands (one per agent in the pair)
        assert len(commands) == 2
        # Commands should push agents apart (opposite directions)
        assert commands[0].vx != 0.0 or commands[0].vy != 0.0 or commands[0].vz != 0.0
