// Validation-only POSIX SITL sidecar. No uORB ABI or control-law changes.
#pragma once
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <unistd.h>

namespace gwm_trace {
constexpr uint64_t capacity = 200000;
enum Word { Sequence, BeginWall, BeginCpu, CopyWall, CopyCpu, EndWall, EndCpu,
	OverheadWall, OverheadCpu, Generation, TrajectoryGeneration, Flags,
	StateOrder, OutputOrder, Events, Attempts, InputSets, Published, BeginHrt, EndHrt, WordCount };
struct Record {
	uint64_t word[WordCount];
	vehicle_local_position_s input;
	trajectory_setpoint_s supplied[3];
	uint64_t attempt_clock[3][4]; // monotonic begin/end, thread CPU begin/end
	uint64_t attempt_order[3][3]; // input assignment, update entry, update return
	float attempt_yaw[3][4]; // actual state yaw, actual yaw/yawspeed at update entry, dt
	uint32_t attempt_info[3][3]; // path, _inputValid(), update() return
	vehicle_local_position_setpoint_s output;
	float state_yaw;
};
struct Header {
	uint64_t word[16]; // magic, version, record size, capacity, copies, stored, outputs,
	// attempts, overflow, clean close, input size, trajectory size, output size,
	// clock read error, initialization wall ns, initialization CPU ns
	char run_id[128];
	char overlay_id[65];
};
class Trace {
public:
	void init();
	void close();
	void begin(const vehicle_local_position_s &input, uint32_t generation, uint64_t hrt);
	void context(uint32_t generation, uint32_t flags);
	void state(float yaw);
	void path(uint32_t value) { _path = value; }
	void input(const trajectory_setpoint_s &value);
	void before(float yaw, float yaw_sp, float speed_sp, float dt);
	void input_valid(bool valid);
	bool result(bool valid);
	void output(const vehicle_local_position_setpoint_s &value);
	void end(uint64_t hrt);
private:
	uint64_t clock(clockid_t id);
	struct Span {
		Trace &trace; uint64_t wall, cpu;
		explicit Span(Trace &t) : trace(t), wall(t.clock(CLOCK_MONOTONIC)), cpu(t.clock(CLOCK_THREAD_CPUTIME_ID)) {}
		~Span() { if (trace._current) { trace._current->word[OverheadCpu] += trace.clock(CLOCK_THREAD_CPUTIME_ID)-cpu;
			trace._current->word[OverheadWall] += trace.clock(CLOCK_MONOTONIC)-wall; } }
	};
	Header _header{};
	Record *_records{nullptr}, *_current{nullptr};
	trajectory_setpoint_s _supplied{};
	uint64_t _input_order{0};
	uint32_t _path{0};
	const char *_file{nullptr};
};
extern Trace recorder;
}
