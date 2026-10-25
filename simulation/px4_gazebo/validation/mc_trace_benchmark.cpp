// Deterministic offline comparison against the actual built PositionControl library.
// It launches no simulator and publishes no command or uORB message.
#include "PositionControl.hpp"
#include "GwmMcTrace.hpp"
#include <vector>
#include <cstdio>
#include <ctime>
#include <cmath>
#include <cstddef>

static uint64_t ns(clockid_t id)
{
	struct timespec t{}; ::clock_gettime(id, &t);
	return uint64_t(t.tv_sec)*1000000000ULL+t.tv_nsec;
}
int main(int argc, char **argv)
{
	if (argc != 3) { return 2; }
	constexpr unsigned count = 10000;
	PositionControl control;
	control.setPositionGains(matrix::Vector3f(1.f, 1.f, 1.f));
	control.setVelocityGains(matrix::Vector3f(2.f, 2.f, 2.f), matrix::Vector3f(.2f,.2f,.2f), matrix::Vector3f(.1f,.1f,.1f));
	control.setVelocityLimits(3.f, 2.f, 2.f); control.setThrustLimits(.1f,.9f);
	control.setHorizontalThrustMargin(.3f); control.setTiltLimit(.7f); control.setHoverThrust(.5f);
	std::vector<vehicle_local_position_setpoint_s> outputs(count);
	std::vector<uint64_t> timings(count*2);
#if GWM_BENCH_TRACE
	gwm_trace::recorder.init();
#endif
	for (unsigned i = 0; i < count; i++) {
		vehicle_local_position_s input{};
		input.timestamp = uint64_t(i+1)*8000; input.timestamp_sample = input.timestamp-4000;
		input.heading = std::sin(float(i)*.001f); input.xy_valid = input.z_valid = input.v_xy_valid = input.v_z_valid = true;
		PositionControlStates state{}; state.yaw = input.heading;
		trajectory_setpoint_s good = PositionControl::empty_trajectory_setpoint;
		good.timestamp = input.timestamp; good.position[0] = .1f; good.position[1] = -.2f; good.position[2] = -2.f;
		good.yaw = i%2 ? .3f : NAN; good.yawspeed = i%2 ? .01f : NAN;
		const bool fallback = i%4 >= 2;
		trajectory_setpoint_s initial = fallback ? PositionControl::empty_trajectory_setpoint : good;
		const uint64_t start = ns(CLOCK_MONOTONIC), cpu = ns(CLOCK_THREAD_CPUTIME_ID);
#if GWM_BENCH_TRACE
		gwm_trace::recorder.begin(input,i+1,input.timestamp);
		gwm_trace::recorder.context(i+1,7); gwm_trace::recorder.path(1);
#endif
		control.setInputSetpoint(initial); control.setState(state);
		if (!control.update(.008f)) {
			if (i%4 == 2) {
#if GWM_BENCH_TRACE
				gwm_trace::recorder.path(2);
#endif
				control.setInputSetpoint(good);
			}
			if (!control.update(.008f)) {
#if GWM_BENCH_TRACE
				gwm_trace::recorder.path(3);
#endif
				control.setInputSetpoint(good); control.update(.008f);
			}
		}
		control.getLocalPositionSetpoint(outputs[i]); outputs[i].timestamp = input.timestamp;
#if GWM_BENCH_TRACE
		gwm_trace::recorder.output(outputs[i]); gwm_trace::recorder.end(input.timestamp);
#endif
		timings[i*2+1] = ns(CLOCK_THREAD_CPUTIME_ID)-cpu;
		timings[i*2] = ns(CLOCK_MONOTONIC)-start;
	}
#if GWM_BENCH_TRACE
	gwm_trace::recorder.close();
#endif
	FILE *f = std::fopen(argv[1],"wb"); if (!f) { return 3; }
	if (std::fwrite(outputs.data(),sizeof(outputs[0]),count,f) != count || std::fclose(f)) { return 4; }
	f = std::fopen(argv[2],"wb"); if (!f) { return 3; }
	if (std::fwrite(timings.data(),sizeof(uint64_t),count*2,f) != count*2 || std::fclose(f)) { return 4; }
	std::printf("{\"header_size\":%zu,\"record_size\":%zu,\"offsets\":{\"word\":%zu,\"input\":%zu,\"supplied\":%zu,\"attempt_clock\":%zu,\"attempt_order\":%zu,\"attempt_yaw\":%zu,\"attempt_info\":%zu,\"output\":%zu,\"state_yaw\":%zu}}\n",
		sizeof(gwm_trace::Header),sizeof(gwm_trace::Record),offsetof(gwm_trace::Record,word),offsetof(gwm_trace::Record,input),
		offsetof(gwm_trace::Record,supplied),offsetof(gwm_trace::Record,attempt_clock),offsetof(gwm_trace::Record,attempt_order),
		offsetof(gwm_trace::Record,attempt_yaw),offsetof(gwm_trace::Record,attempt_info),offsetof(gwm_trace::Record,output),offsetof(gwm_trace::Record,state_yaw));
	return 0;
}
