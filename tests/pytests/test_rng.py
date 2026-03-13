import pytest
import math
from ompl import util as ou



def test_rng_basic():
    rng = ou.RNG()

    # Test uniform01
    for _ in range(10):
        val = rng.uniform01()
        assert 0.0 <= val <= 1.0

    # Test uniformReal
    for _ in range(10):
        val = rng.uniformReal(-5.0, 5.0)
        assert -5.0 <= val <= 5.0

    # Test uniformInt
    for _ in range(10):
        val = rng.uniformInt(0, 10)
        assert 0 <= val <= 10
        assert isinstance(val, int)

    # Test uniformBool
    true_count = 0
    for _ in range(100):
        if rng.uniformBool():
            true_count += 1
    # Should have some True and some False
    assert 10 < true_count < 90


def test_rng_gaussian():
    rng = ou.RNG()

    # Test gaussian01
    vals = [rng.gaussian01() for _ in range(1000)]
    mean = sum(vals) / len(vals)
    # Mean should be close to 0
    assert -0.2 < mean < 0.2

    # Test gaussian with custom mean and stddev
    vals = [rng.gaussian(5.0, 1.0) for _ in range(1000)]
    mean = sum(vals) / len(vals)
    # Mean should be close to 5
    assert 4.5 < mean < 5.5


def test_rng_half_normal():
    rng = ou.RNG()

    # Test halfNormalReal
    for _ in range(10):
        val = rng.halfNormalReal(0.0, 1.0, 3.0)
        assert 0.0 <= val <= 1.0

    # Test halfNormalInt
    for _ in range(10):
        val = rng.halfNormalInt(0, 10, 3.0)
        assert 0 <= val <= 10
        assert isinstance(val, int)


def test_rng_quaternion():
    rng = ou.RNG()

    q = rng.quaternion()
    assert len(q) == 4
    # Check it's normalized (unit quaternion)
    norm = math.sqrt(sum(x * x for x in q))
    assert norm == pytest.approx(1.0, abs=1e-6)


def test_rng_euler_rpy():
    rng = ou.RNG()

    rpy = rng.eulerRPY()
    assert len(rpy) == 3
    # Each angle should be in (-pi, pi]
    for angle in rpy:
        assert -math.pi < angle <= math.pi


def test_rng_seed():
    # Test deterministic behavior with seed
    seed = 12345
    rng1 = ou.RNG(seed)
    rng2 = ou.RNG(seed)

    vals1 = [rng1.uniform01() for _ in range(10)]
    vals2 = [rng2.uniform01() for _ in range(10)]
    assert vals1 == vals2


def test_rng_local_seed():
    rng = ou.RNG()
    initial_seed = rng.getLocalSeed()
    assert initial_seed > 0

    new_seed = 99999
    rng.setLocalSeed(new_seed)
    assert rng.getLocalSeed() == new_seed

if __name__ == "__main__":
    test_rng_basic()
    test_rng_gaussian()
    test_rng_half_normal()
    test_rng_quaternion()
    test_rng_euler_rpy()
    test_rng_seed()
    test_rng_local_seed()
    print("All tests passed!")
