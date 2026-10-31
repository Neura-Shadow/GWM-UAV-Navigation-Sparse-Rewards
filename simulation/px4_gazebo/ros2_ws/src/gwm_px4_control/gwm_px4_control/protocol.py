"""Local transactions; PX4 ACK target fields identify the command sender."""
import math

RESULTS = {0: "accepted", 1: "temporarily_rejected", 2: "denied", 3: "unsupported",
           4: "failed", 5: "in_progress", 6: "cancelled"}


class AckTracker:
    def __init__(self, config):
        self.config = config
        self.pending = None
        self.records = []

    def issue(self, command, phase, sim, monotonic):
        if self.pending is not None:
            raise ValueError("Outstanding command transaction")
        if any(r["command"] == command for r in self.records):
            raise ValueError("Bounded command policy prohibits blind retry/re-arm")
        record = {"local_transaction": len(self.records) + 1, "command": command,
                  "phase": phase, "sent_sim_s": sim, "sent_monotonic_s": monotonic,
                  "attempt": 1, "status": "pending", "acks": []}
        self.records.append(record)
        self.pending = record
        return record

    def receive(self, ack, phase, sim, monotonic):
        record = self.pending
        if (record is None or phase != record["phase"] or ack.get("command") != record["command"]
                or ack.get("target_system") != self.config["source_system"]
                or ack.get("target_component") != self.config["source_component"]
                or ack.get("from_external") is not False
                or not isinstance(ack.get("timestamp"), (int, float))
                or isinstance(ack.get("timestamp"), bool) or not math.isfinite(ack["timestamp"])
                or ack["timestamp"] / 1e6 < record["sent_sim_s"]
                or ack["timestamp"] / 1e6 > sim + self.config["clock_offset_tolerance_s"]
                or sim - record["sent_sim_s"] > self.config["ack_deadline_sim_s"]
                or monotonic - record["sent_monotonic_s"] > self.config["ack_deadline_wall_s"]):
            return "unmatched"
        result = RESULTS.get(ack.get("result"), "unknown_result")
        record["acks"].append({"ack": ack, "received_sim_s": sim,
                               "received_monotonic_s": monotonic})
        record["status"] = result
        if result != "in_progress":
            self.pending = None
        return result

    def check_deadline(self, sim, monotonic):
        if self.pending and (sim - self.pending["sent_sim_s"] > self.config["ack_deadline_sim_s"]
                             or monotonic - self.pending["sent_monotonic_s"] > self.config["ack_deadline_wall_s"]):
            self.pending["status"] = "timeout"
            self.pending = None
            raise TimeoutError("command_ack_timeout")
