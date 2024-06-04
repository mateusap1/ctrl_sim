from controladores.dodge_objects import MinimalPublisher

import pytest
import math


def test_front_clear_range():
    # x is up, y is right

    assert MinimalPublisher.front_clear_range(
        -1.0, 0.1, [math.inf, math.inf, math.inf, math.inf], 5.0
    ) == [(-1.0, -0.7)]

    assert MinimalPublisher.front_clear_range(
        -1.0, 0.1, [6.0, 5.1, 7.2, 9.0], 5.0
    ) == [(-1.0, -0.7)]

    assert (
        MinimalPublisher.front_clear_range(-1.0, 0.1, [4.0, 5.0, 0.3, 1.0], 5.0) == []
    )

    assert MinimalPublisher.front_clear_range(
        -1.0, 0.1, [5.0, math.inf, math.inf, math.inf], 5.0
    ) == [(-0.9, -0.7)]

    assert MinimalPublisher.front_clear_range(
        -1.0, 0.1, [5.0, math.inf, math.inf, 1.0], 5.0
    ) == [(-0.9, -0.8)]

    assert MinimalPublisher.front_clear_range(
        -1.0, 0.1, [math.inf, 1.0, math.inf, math.inf], 5.0
    ) == [(-1.0, -1.0), (-0.8, -0.7)]

    assert MinimalPublisher.front_clear_range(
        -1.0, 0.1, [math.inf, 1.0, 4.0, math.inf], 5.0
    ) == [(-1.0, -1.0), (-0.7, -0.7)]

    assert MinimalPublisher.front_clear_range(
        -1.0, 0.1, [math.inf, 2.0, math.inf, 5.0, 4.0, math.inf, math.inf], 5.0
    ) == [(-1.0, -1.0), (-0.8, -0.8), (-0.5, pytest.approx(-0.4))]
