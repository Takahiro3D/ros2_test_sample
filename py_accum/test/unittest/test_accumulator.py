from py_accum.accumulator import Accumulator


def test_success():
    acc = Accumulator()
    for i in range(10):
        acc.add(i)

    assert acc.get() == 45
