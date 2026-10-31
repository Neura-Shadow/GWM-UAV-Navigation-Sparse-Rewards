"""Adversarial synthetic acquisition tests; no synthetic flight credit."""
import ctypes as C
import math
from pathlib import Path
import sys

import pytest

SIM = Path(__file__).resolve().parents[1]/'simulation/px4_gazebo'
sys.path.insert(0,str(SIM/'validation'))
sys.path.insert(0,str(SIM/'scripts'))
from mc_trace_schema import layouts, WORDS
from mc_trace_evidence import validate_records, acquisition
from p3_provenance import atomic_json, digest

HEADERS = {
    'vehicle_local_position.h': 'uint64_t timestamp;\nuint64_t timestamp_sample;\nfloat heading;\nuint8_t heading_reset_counter;\n',
    'trajectory_setpoint.h': 'uint64_t timestamp;\nfloat yaw;\nfloat yawspeed;\n',
    'vehicle_local_position_setpoint.h': 'uint64_t timestamp;\nfloat yaw;\nfloat yawspeed;\n',
}
CONFIG = dict(max_sample_gap_sim_s=.2,position_fresh_sim_s=.25)


def sample(count=2):
    Header, Record = layouts(HEADERS)
    header = Header(); records = (Record*count)()
    header.word[6] = count; header.word[7] = count
    for index,row in enumerate(records):
        values = dict(sequence=index+1,begin_wall=10000000+index*8000000,
            begin_cpu=1000+index*1000,copy_wall=10000100+index*8000000,
            copy_cpu=1100+index*1000,end_wall=10000500+index*8000000,
            end_cpu=1500+index*1000,overhead_wall=100,overhead_cpu=100,
            generation=10+index,trajectory_generation=5,flags=7,state_order=3,
            output_order=6,events=6,attempts=1,input_sets=1,published=1,
            begin_hrt=100000+index*8000,end_hrt=100000+index*8000)
        for key,value in values.items(): row.word[WORDS.index(key)] = value
        row.input.timestamp = values['begin_hrt']; row.input.timestamp_sample = row.input.timestamp-4000
        row.input.heading = .2+index*.1; row.state_yaw = row.input.heading
        row.supplied[0].yaw = math.nan; row.supplied[0].yawspeed = 0
        row.attempt_clock[0][:] = [values['copy_wall'],values['end_wall'],values['copy_cpu'],values['end_cpu']]
        row.attempt_order[0][:] = [2,4,5]
        row.attempt_yaw[0][:] = [row.input.heading,math.nan,0,.008]
        row.attempt_info[0][:] = [1,1,1]
        row.output.timestamp = row.input.timestamp; row.output.yaw = row.input.heading
    return header,records


def check(header,records): return validate_records(header,records,CONFIG)


def test_equal_hrt_actual_copy_not_nearest_publication_is_authoritative():
    header,records = sample()
    records[1].input.timestamp = records[0].input.timestamp
    records[1].input.timestamp_sample = records[0].input.timestamp_sample+1
    records[1].output.timestamp = records[0].output.timestamp
    assert check(header,records)['status'] == 'verified'
    records[0].output.yaw = records[1].input.heading
    assert check(header,records)['status'] == 'contradicted'


@pytest.mark.parametrize('field,value', [('sequence',3),('generation',10),('output_order',4),
    ('state_order',5),('attempts',4),('published',0),('begin_wall',1)])
def test_missing_identity_order_attempt_or_timing_is_rejected(field,value):
    header,records = sample(); records[1].word[WORDS.index(field)] = value
    with pytest.raises(ValueError): check(header,records)


def test_failed_update_cannot_be_accepted_by_a_matching_yaw():
    header,records = sample(); records[1].attempt_info[0][2] = 0
    assert check(header,records)['status'] == 'contradicted'


@pytest.mark.parametrize('path', [2,3])
def test_fallback_compares_actual_finite_input_and_reconciles_every_attempt(path):
    header,records = sample(1); row = records[0]
    row.word[WORDS.index('attempts')] = 2; header.word[7] = 2
    row.attempt_info[0][2] = 0
    row.supplied[1].yaw = .7; row.supplied[1].yawspeed = .1
    row.attempt_order[1][:] = [6,7,8]
    row.attempt_info[1][:] = [path,1,1]
    row.attempt_yaw[1][:] = [row.input.heading,.7,.1,.008]
    row.attempt_clock[1][:] = row.attempt_clock[0][:]
    row.word[WORDS.index('output_order')] = 9; row.word[WORDS.index('events')] = 9
    row.output.yaw = .7; row.output.yawspeed = .1
    assert check(header,records)['status'] == 'verified'
    row.output.yaw = row.input.heading
    assert check(header,records)['status'] == 'contradicted'


def write_acquisition(tmp_path):
    header,records = sample(); record_type = type(records)._type_
    receipt = dict(contract='px4-mc-causal-trace-v1',status='built',binary_sha256='a'*64,overlay_id='b'*64,
                   generated_headers={})
    abi = tmp_path/'mc-trace-abi'; abi.mkdir()
    for name,text in HEADERS.items():
        (abi/name).write_text(text); receipt['generated_headers'][name] = digest(abi/name)
    summary = dict(run_id=tmp_path.name,identity=dict(mc_trace=receipt,binary_sha256='a'*64),config=CONFIG)
    header.word[0:6] = [0x314543415254434d,1,C.sizeof(record_type),200000,2,2]
    header.word[9:13] = [1,C.sizeof(record_type._fields_[1][1]),C.sizeof(record_type._fields_[2][1]._type_),
                           C.sizeof(record_type._fields_[7][1])]
    header.run_id = tmp_path.name.encode(); header.overlay_id = b'b'*64
    atomic_json(tmp_path/'mc-trace-build.json',receipt)
    (tmp_path/'mc-trace.bin').write_bytes(bytes(header)+bytes(records))
    (tmp_path/'mc-trace.bin.closed').write_bytes(b'closed-drained-fsynced\n')
    return header,records,summary


@pytest.mark.parametrize('damage', ['short','extra','overflow','wrong_run','wrong_binary','missing_close','wrong_abi','count','clock'])
def test_acquisition_fails_closed_on_integrity_or_provenance_damage(tmp_path,damage):
    header,records,summary = write_acquisition(tmp_path)
    assert acquisition(tmp_path,summary)['status'] == 'verified'
    if damage == 'overflow': header.word[8] = 1
    elif damage == 'wrong_run': header.run_id = b'other'
    elif damage == 'wrong_binary': summary['identity']['binary_sha256'] = 'c'*64
    elif damage == 'missing_close': (tmp_path/'mc-trace.bin.closed').unlink()
    elif damage == 'wrong_abi': (tmp_path/'mc-trace-abi/trajectory_setpoint.h').write_text('changed')
    elif damage == 'count': header.word[6] += 1
    elif damage == 'clock': header.word[13] = 1
    raw = bytes(header)+bytes(records)
    if damage == 'short': raw = raw[:-1]
    if damage == 'extra': raw += b'\0'
    (tmp_path/'mc-trace.bin').write_bytes(raw)
    with pytest.raises((ValueError,FileNotFoundError)): acquisition(tmp_path,summary)


def test_overlay_requires_unique_pinned_anchors():
    from build_mc_trace import replace_once
    with pytest.raises(ValueError): replace_once('absent','anchor','new')
    with pytest.raises(ValueError): replace_once('anchor anchor','anchor','new')


def test_opt_in_runtime_selection_never_implicitly_switches_binary(monkeypatch,tmp_path):
    from mc_trace_runtime import selected_build
    monkeypatch.delenv('GWM_PX4_MC_TRACE',raising=False)
    assert selected_build(tmp_path,SIM) is None
    monkeypatch.setenv('GWM_PX4_MC_TRACE','wrong')
    with pytest.raises(ValueError): selected_build(tmp_path,SIM)


def test_real_pinned_native_abi_is_fully_retained():
    from mc_trace_schema import schema
    directory=Path(__file__).with_name('fixtures')/'mc_trace_abi'
    headers={path.name:path.read_text() for path in directory.glob('*.h')}
    header,record=layouts(headers)
    assert C.sizeof(record._fields_[1][1]) == 192
    assert len(record._fields_[1][1]._fields_) == 53
    assert 'heading_reset_counter' in dict(record._fields_[1][1]._fields_)
    assert schema(headers)['record_size'] == C.sizeof(record)
