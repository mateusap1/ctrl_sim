from controladores.robot_controller import MinimalPublisher

import pytest
import math


def test_calculate_errors():
    # x is up, y is right

    assert MinimalPublisher.expected_angle((0.0, 0.0), (3.0, 0.0)) == pytest.approx(0.0)
    assert MinimalPublisher.expected_angle((0.0, 0.0), (-3.0, 0.0)) == pytest.approx(math.pi)

    assert MinimalPublisher.expected_angle((0.0, 0.0), (0.0, 3.0)) == pytest.approx(math.pi / 2)
    assert MinimalPublisher.expected_angle((0.0, 0.0), (0.0, -3.0)) == pytest.approx(-math.pi / 2)

    assert MinimalPublisher.expected_angle((0.0, 0.0), (3.0, 3.0)) == pytest.approx(0.785398)
    assert MinimalPublisher.expected_angle((0.0, 0.0), (-3.0, 3.0)) == pytest.approx(2.3561945)

    assert MinimalPublisher.expected_angle((0.0, 0.0), (3.0, -3.0)) == pytest.approx(-0.785398)
    assert MinimalPublisher.expected_angle((0.0, 0.0), (-3.0, -3.0)) == pytest.approx(-2.3561945)
