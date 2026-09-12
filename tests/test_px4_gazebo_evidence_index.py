"""Differential lookup semantics and immutable per-evaluation snapshots."""
import copy
from pathlib import Path
import sys

import pytest

SIM=Path(__file__).resolve().parents[1]/'simulation/px4_gazebo'
sys.path.insert(0,str(SIM/'validation'));sys.path.insert(0,str(SIM/'ros2_ws/src/gwm_px4_control'))
from sample_evidence import ValidatedStreams, causal_asof, validate_stream


def rows():
    return [dict(timestamp=t,timestamp_sample=s,q=[1.,0.,0.,0.],quat_reset_counter=0)
            for t,s in [(100,90),(200,180),(200,190),(300,290)]]


@pytest.mark.parametrize('stamp', [99,100,101,200,201,300,301,2000])
def test_index_matches_original_strict_prior_group_lookup_including_failures(stamp):
    raw=rows(); expected=validate_stream(raw,'vehicle_attitude','r')['records']
    index=ValidatedStreams({'vehicle_attitude':raw},'r')
    try: old=causal_asof(expected,stamp,.001,'vehicle_attitude')
    except ValueError as exc:
        with pytest.raises(ValueError,match=str(exc)):
            index.asof('vehicle_attitude',stamp,.001)
    else:
        selected=index.asof('vehicle_attitude',stamp,.001)
        assert selected['timestamp']==old['timestamp']
        assert selected['timestamp_sample']==old['timestamp_sample']
        assert selected['_evidence']==old['_evidence']


def test_input_mutation_cannot_change_validated_snapshot_and_return_is_immutable():
    raw=rows(); index=ValidatedStreams({'vehicle_attitude':raw},'r')
    raw[2]['q'][0]=.1; raw.reverse()
    selected=index.asof('vehicle_attitude',201,.001)
    assert selected['q']==(1.,0.,0.,0.)
    with pytest.raises(TypeError): selected['q']=(.1,0.,0.,0.)
    with pytest.raises(TypeError): selected['_evidence']['run_id']='other'
    assert index.statistics['vehicle_attitude']['equal_publication_distinct_outputs']==1


@pytest.mark.parametrize('kind', ['regression','conflict','cross_run'])
def test_whole_stream_rejection_still_precedes_lookup(kind):
    raw=rows()
    if kind=='regression':raw.reverse()
    if kind=='conflict':raw.insert(1,{**raw[0],'q':[0.,1.,0.,0.]})
    if kind=='cross_run':raw[-1]['_run_id']='other'
    with pytest.raises(ValueError):ValidatedStreams({'vehicle_attitude':raw},'r')
