"""Real metric depth must be fresh, co-located, projected and conservative."""
import importlib.util
from pathlib import Path

import numpy as np
import pytest

from neurovector_vision.models import BoundingBox, Detection, DetectResponse
from school_depth_ranges import with_depth_ranges


def result():
    return DetectResponse(drone_id='school-drone-001',forwarded=1,elapsed_ms=1,
                          detections=[Detection(label='tree_region',priority_level=2,confidence=.9,
                                                box=BoundingBox(x1=280,y1=140,x2=360,y2=220))])


def test_depth_projects_rgb_center_onto_taller_depth_image(tmp_path):
    depth=np.full((480,640),np.inf,dtype=np.float32)
    depth[220:260,300:340]=20
    path=tmp_path/'depth.npz';np.savez(path,depth=depth,timestamp=100,near_m=.2,far_m=100)
    observed=with_depth_ranges(result(),path,(360,640),100,100.3)
    assert observed.detections[0].distance_m==20
    assert observed.detections[0].distance_source=='gazebo_depth'
    assert observed.detections[0].distance_confidence is None


@pytest.mark.parametrize('stamp,now,camera',[(95,100,100),(101,100,100),(100,100.2,98)])
def test_stale_future_or_unpaired_depth_never_invents_distance(tmp_path,stamp,now,camera):
    path=tmp_path/'depth.npz';np.savez(path,depth=np.full((480,640),20,dtype=np.float32),timestamp=stamp,near_m=.2,far_m=100)
    assert with_depth_ranges(result(),path,(360,640),camera,now).detections[0].distance_m is None


def test_near_quantile_preserves_real_obstacle_hold_and_critical_people(tmp_path):
    path=tmp_path/'depth.npz';np.savez(path,depth=np.full((480,640),2,dtype=np.float32),timestamp=100,near_m=.2,far_m=100)
    observed=with_depth_ranges(result(),path,(360,640),100,100.2)
    spec=importlib.util.spec_from_file_location('depth_relay',Path(__file__).parents[1]/'scripts/school-vision-relay.py')
    relay=importlib.util.module_from_spec(spec);spec.loader.exec_module(relay)
    assert relay.blocked_by(observed)
    far=observed.model_copy(update={'detections':[observed.detections[0].model_copy(update={'distance_m':20})]})
    assert not relay.blocked_by(far)
    critical=far.model_copy(update={'detections':[far.detections[0].model_copy(update={'priority_level':1})]})
    assert relay.blocked_by(critical)


def test_invalid_depth_and_missing_file_fail_closed(tmp_path):
    path=tmp_path/'depth.npz'
    assert with_depth_ranges(result(),path,(360,640),100,100).detections[0].distance_m is None
    np.savez(path,depth=np.full((480,640),np.nan,dtype=np.float32),timestamp=100,near_m=.2,far_m=100)
    assert with_depth_ranges(result(),path,(360,640),100,100).detections[0].distance_m is None


def test_far_clipped_positive_infinity_is_an_explicit_lower_bound(tmp_path):
    path=tmp_path/'depth.npz';np.savez(path,depth=np.full((480,640),np.inf,dtype=np.float32),timestamp=100,near_m=.2,far_m=19.1)
    observed=with_depth_ranges(result(),path,(360,640),100,100.2).detections[0]
    assert observed.distance_m==19.1
    assert observed.distance_source=='gazebo_depth_lower_bound'
    np.savez(path,depth=np.full((480,640),-np.inf,dtype=np.float32),timestamp=100,near_m=.2,far_m=19.1)
    assert with_depth_ranges(result(),path,(360,640),100,100.2).detections[0].distance_m==pytest.approx(.2)
