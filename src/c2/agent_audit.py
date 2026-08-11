"""Deterministic in-memory audit storage for explicitly mock-only tool calls."""

from __future__ import annotations

from copy import deepcopy
from math import isfinite
from typing import Any, Dict, List, Optional, Sequence

from src.c2.agent_types import AgentAuditRecord, AgentContractError


AGENT_AUDIT_SCHEMA_VERSION = "agent-audit-log-v1"
AGENT_AUDIT_EVENT_TYPES = frozenset(
    {
        "tool_call_started",
        "tool_call_completed",
        "tool_call_denied",
        "tool_call_failed",
        "tool_call_timed_out",
        "tool_result_invalid",
    }
)

_SENSITIVE_KEYS = frozenset(
    {
        "password",
        "passwd",
        "secret",
        "token",
        "api_key",
        "apikey",
        "credential",
        "credentials",
        "private_key",
        "hostname",
        "host",
        "runtime_handle",
        "socket",
        "session",
        "shell",
        "command",
        "code_payload",
        "file_handle",
        "chain_of_thought",
        "private_chain_of_thought",
        "reasoning_trace",
        "hidden_reasoning",
        "provider_prompt",
        "provider_response",
    }
)
_REDACTED_MARKER = "[REDACTED]"


def _error(code: str, message: str, field_name: Optional[str] = None) -> AgentContractError:
    return AgentContractError(
        code,
        message,
        field_name=field_name,
        contract_name="AgentAuditLog",
    )


def _normalize_key(key: str) -> str:
    return key.strip().lower().replace("-", "_").replace(" ", "_")


def _is_sensitive_key(key: str) -> bool:
    normalized = _normalize_key(key)
    return normalized in _SENSITIVE_KEYS or any(
        normalized.endswith(f"_{item}") for item in _SENSITIVE_KEYS
    )


def redact_for_audit(value: Any) -> Any:
    """Return a new JSON-safe value with sensitive original keys omitted."""

    if value is None or isinstance(value, (str, bool)):
        return value
    if isinstance(value, int) and not isinstance(value, bool):
        return value
    if isinstance(value, float):
        return value if isfinite(value) else _REDACTED_MARKER
    if isinstance(value, list):
        return [redact_for_audit(item) for item in value]
    if isinstance(value, dict):
        sanitized: Dict[str, Any] = {}
        redacted_fields = set()
        for key, item in value.items():
            if not isinstance(key, str):
                redacted_fields.add("[non_string_key]")
                continue
            if _is_sensitive_key(key):
                redacted_fields.add(_normalize_key(key))
                continue
            if key == "redacted_fields":
                if isinstance(item, list):
                    redacted_fields.update(
                        entry for entry in item if isinstance(entry, str) and entry
                    )
                else:
                    redacted_fields.add("redacted_fields")
                continue
            sanitized[key] = redact_for_audit(item)
        if redacted_fields:
            sanitized["redacted_fields"] = sorted(redacted_fields)
        return sanitized
    return _REDACTED_MARKER


class AgentAuditLog:
    """Append-oriented audit records with defensive copies and atomic restore."""

    def __init__(self) -> None:
        self._records: List[AgentAuditRecord] = []

    def next_record_id(self) -> str:
        return f"audit-{len(self._records) + 1:06d}"

    def append(self, record: AgentAuditRecord) -> AgentAuditRecord:
        if not isinstance(record, AgentAuditRecord):
            raise _error("invalid_contract", "append requires an AgentAuditRecord")
        record.validate()
        if record.event_type not in AGENT_AUDIT_EVENT_TYPES:
            raise _error("invalid_contract", "audit event type is not allowed", "event_type")
        if record.record_id != self.next_record_id():
            raise _error(
                "invalid_contract",
                "audit record id is not the next deterministic id",
                "record_id",
            )
        stored = AgentAuditRecord.from_dict(record.to_dict())
        self._records.append(stored)
        return AgentAuditRecord.from_dict(stored.to_dict())

    def build_record(
        self,
        *,
        workflow_id: str,
        event_type: str,
        actor_id: str,
        timestamp: float,
        input_refs: Sequence[str] = (),
        tool_name: Optional[str] = None,
        tool_version: Optional[str] = None,
        validated_parameters_summary: Any = None,
        result_summary: Any = None,
        decision: Optional[str] = None,
        approval_state: Optional[str] = None,
        errors: Sequence[Any] = (),
        metadata: Optional[Dict[str, Any]] = None,
    ) -> AgentAuditRecord:
        return AgentAuditRecord(
            record_id=self.next_record_id(),
            workflow_id=workflow_id,
            event_type=event_type,
            actor_id=actor_id,
            timestamp=timestamp,
            input_refs=list(input_refs),
            tool_name=tool_name,
            tool_version=tool_version,
            validated_parameters_summary=redact_for_audit(
                {} if validated_parameters_summary is None else validated_parameters_summary
            ),
            result_summary=redact_for_audit(
                {} if result_summary is None else result_summary
            ),
            decision=decision,
            approval_state=approval_state,
            errors=redact_for_audit(list(errors)),
            metadata=redact_for_audit({} if metadata is None else metadata),
        )

    def list_records(self, workflow_id: Optional[str] = None) -> List[AgentAuditRecord]:
        if workflow_id is not None and (
            not isinstance(workflow_id, str) or not workflow_id
        ):
            raise _error(
                "invalid_contract",
                "workflow_id must be a non-empty string",
                "workflow_id",
            )
        return [
            AgentAuditRecord.from_dict(record.to_dict())
            for record in self._records
            if workflow_id is None or record.workflow_id == workflow_id
        ]

    def find_by_request(self, request_id: str) -> List[AgentAuditRecord]:
        if not isinstance(request_id, str) or not request_id:
            raise _error(
                "invalid_contract",
                "request_id must be a non-empty string",
                "request_id",
            )
        return [
            AgentAuditRecord.from_dict(record.to_dict())
            for record in self._records
            if request_id in record.input_refs
        ]

    def snapshot(self) -> Dict[str, Any]:
        records = [record.to_dict() for record in self._records]
        return {
            "schema_version": AGENT_AUDIT_SCHEMA_VERSION,
            "record_count": len(records),
            "records": deepcopy(records),
        }

    def restore(self, snapshot: Dict[str, Any]) -> None:
        if not isinstance(snapshot, dict):
            raise _error("invalid_contract", "snapshot must be a dictionary")
        if set(snapshot) != {"schema_version", "record_count", "records"}:
            raise _error("invalid_contract", "snapshot fields do not match the audit schema")
        if snapshot["schema_version"] != AGENT_AUDIT_SCHEMA_VERSION:
            raise _error(
                "invalid_contract",
                "snapshot schema version is not supported",
                "schema_version",
            )
        records_data = snapshot["records"]
        if not isinstance(records_data, list):
            raise _error("invalid_contract", "snapshot records must be a list", "records")
        record_count = snapshot["record_count"]
        if isinstance(record_count, bool) or not isinstance(record_count, int):
            raise _error("invalid_contract", "record_count must be an integer", "record_count")
        if record_count != len(records_data):
            raise _error(
                "invalid_contract",
                "record_count does not match records",
                "record_count",
            )

        candidate: List[AgentAuditRecord] = []
        for index, item in enumerate(deepcopy(records_data), start=1):
            record = AgentAuditRecord.from_dict(item)
            if record.event_type not in AGENT_AUDIT_EVENT_TYPES:
                raise _error("invalid_contract", "audit event type is not allowed", "event_type")
            if record.record_id != f"audit-{index:06d}":
                raise _error(
                    "invalid_contract",
                    "snapshot record ids are not sequential",
                    "record_id",
                )
            candidate.append(record)
        self._records = candidate

    def clear(self) -> None:
        """Clear records for deterministic test lifecycle only."""

        self._records.clear()
