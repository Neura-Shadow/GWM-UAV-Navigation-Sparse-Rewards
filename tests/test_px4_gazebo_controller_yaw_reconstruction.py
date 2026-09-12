"""Actual retained first-selection quaternion; no synthetic timestamp rewrite."""
from pathlib import Path
import sys
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'simulation/px4_gazebo/validation'))
from sample_evidence import controller_yaw_reconstruction


def test_recorded_controller_yaw_requires_original_normalization():
    # Historical bag message2417, attitude ordinal594, publication19444000us.
    q=[0.6703062653541565,0.00010014112194767222,-0.00021170481340959668,0.7420845627784729]
    assert controller_yaw_reconstruction(q)==1.6723498412394
    assert controller_yaw_reconstruction(q)!=1.672349883554586


@pytest.mark.parametrize('q',[[0.,0.,0.,0.],[1.1,0.,0.,0.],[float('nan'),0.,0.,0.]])
def test_invalid_original_controller_attitude_still_fails(q):
    with pytest.raises(ValueError): controller_yaw_reconstruction(q)
