"""Native image identity stays exact even when display seconds collide."""
from pathlib import Path
import sys
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'simulation/px4_gazebo/validation'))
from collect_p3_evidence import acquisition_ns


def test_native_acquisition_does_not_use_rounded_display_seconds():
    first={'acquisition_sim_ns':9007199254740992,'acquisition_sim_s':9007199.254740993}
    second={'acquisition_sim_ns':9007199254740993,'acquisition_sim_s':9007199.254740993}
    assert acquisition_ns(second,True)-acquisition_ns(first,True)==1


@pytest.mark.parametrize('value',[True,1.0,-1,2**63,None])
def test_invalid_native_image_time_never_falls_back_to_display(value):
    with pytest.raises(ValueError):
        acquisition_ns({'acquisition_sim_ns':value,'acquisition_sim_s':1.},True)


def test_new_record_requires_native_image_time():
    with pytest.raises(ValueError):
        acquisition_ns({'acquisition_sim_s':1.},True)
