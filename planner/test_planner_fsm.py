import pytest
import time
from messages import Direction, DetectedType
from planner_fsm import PlannerFSM


def test_step1_yaw_until_gate_detected(monkeypatch):
    moves = []
    detection_calls = {'count': 0}

    class FakeNode:
        def publish_move(self, direction, magnitude):
            moves.append((direction, magnitude))

        def get_last_detection(self):
            # Return GATE after 3 calls
            detection_calls['count'] += 1
            if detection_calls['count'] >= 3:
                return DetectedType.GATE
            return None

    node = FakeNode()
    fsm = PlannerFSM(node)

    # Run only step1
    fsm.step1()

    # Expect first an infinite‐magnitude yaw, then a zero‐magnitude stop
    assert len(moves) == 2
    first_dir, first_mag = moves[0]
    second_dir, second_mag = moves[1]
    assert first_dir == Direction.YAW
    assert first_mag == float('inf')
    assert second_dir == Direction.YAW
    assert second_mag == 0.0

    # After detection, step1 should be marked complete
    assert fsm.step_complete[1]


def test_mark_step_complete():
    class DummyNode:
        pass

    fsm = PlannerFSM(DummyNode())
    assert not fsm.step_complete[5]
    fsm.mark_step_complete(5)
    assert fsm.step_complete[5]


def test_step2_align_and_drive(monkeypatch):
    moves = []
    # sequence of gate_centered states: first two checks False, then True (align),
    # then True (keep moving), then False (gate leaves view)
    statuses = [False, False, True, True, False]

    class FakeNode:
        def publish_move(self, direction, magnitude):
            moves.append((direction, magnitude))
        def is_gate_centered(self):
            # pop next status or return False
            return statuses.pop(0) if statuses else False

    # speed up test by disabling sleep
    monkeypatch.setattr(time, 'sleep', lambda s: None)

    node = FakeNode()
    fsm = PlannerFSM(node)

    # Run only step2
    fsm.step2()

    # Expect publish_move calls: start infinite, extra 0.5, then stop
    assert moves == [
        (Direction.X, float('inf')),
        (Direction.X, 0.5),
        (Direction.X, 0.0)
    ]
    # After execution, step2 should be marked complete
    assert fsm.step_complete[2]


def test_step3_slalom_sequence(monkeypatch):
     moves = []
     # detections: first two calls None, then SLALOM
     detections = [None, None, DetectedType.SLALOM]
     # centering statuses: first uncentered, then centered twice, then uncentered to stop
     centered = [False, False, True, True, False]

     class FakeNode:
         def publish_move(self, direction, magnitude):
             moves.append((direction, magnitude))
         def get_last_detection(self):
             return detections.pop(0) if detections else DetectedType.SLALOM
         def is_slalom_centered(self):
             return centered.pop(0) if centered else False

     # disable sleep
     monkeypatch.setattr(time, 'sleep', lambda s: None)

     node = FakeNode()
     fsm = PlannerFSM(node)

     # Run only step3
     fsm.step3()

     # Expect: start yaw infinite, stop yaw, start forward infinite, stop forward
     assert moves == [
         (Direction.YAW, float('inf')),
         (Direction.YAW, 0.0),
         (Direction.X, float('inf')),
         (Direction.X, 0.0)
     ]
     assert fsm.step_complete[3]


def test_step4_drive_until_clear(monkeypatch):
     moves = []
     # slalom centering statuses: two True loops, then False
     statuses = [True, True, False]

     class FakeNode:
         def publish_move(self, direction, magnitude):
             moves.append((direction, magnitude))
         def is_slalom_centered(self):
             return statuses.pop(0) if statuses else False

     monkeypatch.setattr(time, 'sleep', lambda s: None)

     node = FakeNode()
     fsm = PlannerFSM(node)

     # Run only step4
     fsm.step4()

     # Expect two infinite moves then a stop
     assert moves == [
         (Direction.X, float('inf')),
         (Direction.X, float('inf')),
         (Direction.X, 0.0)
     ]
     assert fsm.step_complete[4]
