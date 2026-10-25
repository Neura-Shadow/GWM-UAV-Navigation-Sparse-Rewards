#include "GwmMcTrace.hpp"
#include <cerrno>
#include <new>
#ifndef GWM_MC_OVERLAY_ID
#error GWM_MC_OVERLAY_ID must identify the reviewed overlay
#endif
namespace gwm_trace {
Trace recorder;
uint64_t Trace::clock(clockid_t id)
{
	struct timespec value{};
	// libc clock_gettime, never PX4 lockstep HRT.
	if (::clock_gettime(id, &value) != 0) { _header.word[13]++; }
	return uint64_t(value.tv_sec)*1000000000ULL + value.tv_nsec;
}
void Trace::init()
{
	_file = std::getenv("GWM_MC_TRACE_FILE");
	if (!_file) { return; }
	if (_records) { _header.word[8]++; return; }
	const uint64_t wall = clock(CLOCK_MONOTONIC), cpu = clock(CLOCK_THREAD_CPUTIME_ID);
	const char *run = std::getenv("GWM_MC_TRACE_RUN_ID");
	if (!run || std::strlen(run) >= sizeof(_header.run_id)) { _file = nullptr; return; }
	std::strcpy(_header.run_id, run);
	std::strcpy(_header.overlay_id, GWM_MC_OVERLAY_ID);
	_header.word[0] = 0x314543415254434dULL; // MCTRACE1, little endian
	_header.word[1] = 1; _header.word[2] = sizeof(Record); _header.word[3] = capacity;
	_header.word[10] = sizeof(vehicle_local_position_s);
	_header.word[11] = sizeof(trajectory_setpoint_s);
	_header.word[12] = sizeof(vehicle_local_position_setpoint_s);
	_records = new(std::nothrow) Record[capacity];
	if (!_records) { _header.word[8]++; return; }
	// Pre-touch every byte before callback registration; no allocation or IO in Run.
	std::memset(_records, 0, sizeof(Record)*capacity);
	_header.word[15] = clock(CLOCK_THREAD_CPUTIME_ID)-cpu;
	_header.word[14] = clock(CLOCK_MONOTONIC)-wall;
}
void Trace::begin(const vehicle_local_position_s &input, uint32_t generation, uint64_t hrt)
{
	if (!_file) { return; }
	const uint64_t wall = clock(CLOCK_MONOTONIC), cpu = clock(CLOCK_THREAD_CPUTIME_ID);
	_header.word[4]++;
	if (!_records || _header.word[5] == capacity) { _header.word[8]++; _current = nullptr; return; }
	_current = &_records[_header.word[5]++];
	_current->word[Sequence] = _header.word[4];
	_current->word[BeginWall] = wall; _current->word[BeginCpu] = cpu;
	_current->word[BeginHrt] = hrt;
	_current->input = input; _current->word[Generation] = generation;
	_current->word[Events] = 1; _path = 0; _input_order = 0;
	_current->word[CopyCpu] = clock(CLOCK_THREAD_CPUTIME_ID);
	_current->word[CopyWall] = clock(CLOCK_MONOTONIC);
	_current->word[OverheadCpu] = _current->word[CopyCpu]-cpu;
	_current->word[OverheadWall] = _current->word[CopyWall]-wall;
}
void Trace::context(uint32_t generation, uint32_t flags)
{
	if (!_current) { return; } Span span(*this);
	_current->word[TrajectoryGeneration] = generation; _current->word[Flags] = flags;
}
void Trace::state(float yaw)
{
	if (!_current) { return; } Span span(*this);
	_current->state_yaw = yaw; _current->word[StateOrder] = ++_current->word[Events];
}
void Trace::input(const trajectory_setpoint_s &value)
{
	if (!_current) { return; } Span span(*this);
	_supplied = value; _input_order = ++_current->word[Events]; _current->word[InputSets]++;
}
void Trace::before(float yaw, float yaw_sp, float speed_sp, float dt)
{
	if (!_current) { return; } Span span(*this);
	const uint64_t n = _current->word[Attempts]++;
	_header.word[7]++;
	if (n >= 3) { _header.word[8]++; return; }
	_current->supplied[n] = _supplied;
	_current->attempt_yaw[n][0] = yaw; _current->attempt_yaw[n][1] = yaw_sp;
	_current->attempt_yaw[n][2] = speed_sp; _current->attempt_yaw[n][3] = dt;
	_current->attempt_order[n][0] = _input_order;
	_current->attempt_order[n][1] = ++_current->word[Events];
	_current->attempt_info[n][0] = _path;
	_current->attempt_clock[n][0] = clock(CLOCK_MONOTONIC);
	_current->attempt_clock[n][2] = clock(CLOCK_THREAD_CPUTIME_ID);
}
void Trace::input_valid(bool valid)
{
	if (!_current) { return; } Span span(*this);
	const uint64_t n = _current->word[Attempts]-1;
	if (n < 3) { _current->attempt_info[n][1] = valid; }
}
bool Trace::result(bool valid)
{
	if (!_current) { return valid; } Span span(*this);
	const uint64_t n = _current->word[Attempts]-1;
	if (n < 3) {
		_current->attempt_clock[n][3] = clock(CLOCK_THREAD_CPUTIME_ID);
		_current->attempt_clock[n][1] = clock(CLOCK_MONOTONIC);
		_current->attempt_info[n][2] = valid;
		_current->attempt_order[n][2] = ++_current->word[Events];
	}
	return valid;
}
void Trace::output(const vehicle_local_position_setpoint_s &value)
{
	if (!_current) { return; } Span span(*this);
	_current->output = value; _current->word[OutputOrder] = ++_current->word[Events];
	_current->word[Published]++; _header.word[6]++;
}
void Trace::end(uint64_t hrt)
{
	if (!_current) { return; }
	_current->word[EndHrt] = hrt;
	_current->word[EndCpu] = clock(CLOCK_THREAD_CPUTIME_ID);
	_current->word[EndWall] = clock(CLOCK_MONOTONIC);
	_current = nullptr;
}
void Trace::close()
{
	if (!_file) { return; }
	// Called after unregisterCallback, from the MC destructor. No live producer.
	_header.word[9] = _current == nullptr && _records != nullptr;
	const int fd = ::open(_file, O_WRONLY|O_CREAT|O_EXCL, 0600);
	if (fd >= 0) {
		auto write_all = [fd](const void *data, size_t size) {
			const char *p = static_cast<const char *>(data);
			while (size) { const ssize_t n = ::write(fd, p, size);
				if (n < 0 && errno == EINTR) { continue; }
				if (n <= 0) { return false; } p += n; size -= n; }
			return true;
		};
		bool ok = write_all(&_header, sizeof(_header));
		if (ok && _records) { ok = write_all(_records, sizeof(Record)*_header.word[5]); }
		if (ok) { ok = ::fsync(fd) == 0; }
		const int closed = ::close(fd);
		// A distinct durable completion marker makes a short write/fsync error fail closed.
		if (ok && closed == 0) {
			char marker[4096]; const int count = std::snprintf(marker, sizeof(marker), "%s.closed", _file);
			if (count > 0 && size_t(count) < sizeof(marker)) {
				const int done = ::open(marker, O_WRONLY|O_CREAT|O_EXCL, 0600);
				if (done >= 0) { const char value[] = "closed-drained-fsynced\n";
					const ssize_t written = ::write(done, value, sizeof(value)-1);
					if (written != sizeof(value)-1 || ::fsync(done) != 0) { ::unlink(marker); }
					::close(done); }
			}
		}
	}
	delete[] _records; _records = nullptr; _file = nullptr;
}
}
