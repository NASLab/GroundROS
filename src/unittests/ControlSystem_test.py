from sys import path
from math import isnan
import pytest
path.append('..')
import control_system

acceptable_error = .001


@pytest.fixture
def ctrl_sys_no_gain():
    return control_system.ControlSystem()


@pytest.fixture
def ctrl_sys_gain():
    obj = control_system.ControlSystem()
    obj.setGain(1, 0, 0)
    return obj


def test_bangbang(ctrl_sys_no_gain):
    assert ctrl_sys_no_gain.bangbang(.1) == True
    assert ctrl_sys_no_gain.bangbang(-10) == False


def test_proportaional_with_gain(ctrl_sys_gain):
    assert ctrl_sys_gain.proportional(1.0) == 1.0
    assert ctrl_sys_gain.proportional(0.0) == 0.0
    assert ctrl_sys_gain.proportional(-20) == -20


def test_proportional_no_gain(ctrl_sys_no_gain):
    assert isnan(ctrl_sys_no_gain.proportional(1.0))
