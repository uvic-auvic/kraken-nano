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


def test_step5_yaw_until_orange_path(monkeypatch):
     moves = []
     # bottom detections: first two checks None, then ORANGE_PATH
     bottoms = [None, None, DetectedType.ORANGE_PATH]

     class FakeNode:
         def publish_move(self, direction, magnitude):
             moves.append((direction, magnitude))
         def get_last_bottom_detection(self):
             return bottoms.pop(0) if bottoms else DetectedType.ORANGE_PATH

     # disable sleep
     monkeypatch.setattr(time, 'sleep', lambda s: None)

     node = FakeNode()
     fsm = PlannerFSM(node)

     # Run only step5
     fsm.step5()

     # Expect one infinite yaw and then a zero stop
     assert moves == [
         (Direction.YAW, float('inf')),
         (Direction.YAW, 0.0)
     ]
     assert fsm.step_complete[5]


def test_step6_follow_orange_path_until_bin(monkeypatch):
     moves = []
     # bottom detections: first two checks None, then BIN
     bottoms = [None, None, DetectedType.BIN]

     class FakeNode:
         def publish_move(self, direction, magnitude):
             moves.append((direction, magnitude))
         def get_last_bottom_detection(self):
             return bottoms.pop(0) if bottoms else DetectedType.BIN

     # disable sleep
     monkeypatch.setattr(time, 'sleep', lambda s: None)

     node = FakeNode()
     fsm = PlannerFSM(node)

     # Run only step6
     fsm.step6()

     # Expect one infinite forward and then a stop
     assert moves == [
         (Direction.X, float('inf')),
         (Direction.X, 0.0)
     ]
     assert fsm.step_complete[6]


def test_step7_drop_ball(monkeypatch):
     drops = []
     class FakeNode:
         def publish_dropper(self, state):
             drops.append(state)
     # disable sleep
     monkeypatch.setattr(time, 'sleep', lambda s: None)
     node = FakeNode()
     fsm = PlannerFSM(node)
     fsm.step7()
     assert drops == [Direction.ON, Direction.OFF]
     assert fsm.step_complete[7]


def test_step8_find_and_center_picture(monkeypatch):
     moves = []
     # detection sequence: first two None, then PICTURE
     detections = [None, None, DetectedType.PICTURE]
     # centering sequence: first two False, then True
     centered = [False, False, True]

     class FakeNode:
         def publish_move(self, direction, magnitude):
             moves.append((direction, magnitude))
         def get_last_detection(self):
             return detections.pop(0) if detections else DetectedType.PICTURE
         def is_picture_centered(self):
             return centered.pop(0) if centered else True

     # disable sleep to speed up
     monkeypatch.setattr(time, 'sleep', lambda s: None)

     node = FakeNode()
     fsm = PlannerFSM(node)

     # Run only step8
     fsm.step8()

     # Expect: start search yaw, stop search yaw, start center yaw, stop center yaw
     assert moves == [
         (Direction.YAW, float('inf')),
         (Direction.YAW, 0.0),
         (Direction.YAW, float('inf')),
         (Direction.YAW, 0.0)
     ]
     assert fsm.step_complete[8]


def test_step9_search_and_center_target(monkeypatch):
     moves = []
     # detection sequence: first two None, then RED_SQUARE
     detections = [None, None, DetectedType.RED_SQUARE]
     # centering sequence: first two False, then True
     centered = [False, False, True]

     class FakeNode:
         def publish_move(self, direction, magnitude):
             moves.append((direction, magnitude))
         def get_last_detection(self):
             return detections.pop(0) if detections else DetectedType.RED_SQUARE
         def is_target_centered(self):
             return centered.pop(0) if centered else True

     # disable sleep
     monkeypatch.setattr(time, 'sleep', lambda s: None)

     node = FakeNode()
     fsm = PlannerFSM(node)

     # Run only step9
     fsm.step9()

     # Expect: start search yaw, stop search yaw, start center yaw, stop center yaw
     assert moves == [
         (Direction.YAW, float('inf')),
         (Direction.YAW, 0.0),
         (Direction.YAW, float('inf')),
         (Direction.YAW, 0.0)
     ]
     assert fsm.step_complete[9]


def test_step10_approach_and_launch(monkeypatch):
     moves = []
     torpedos = []
     # distances: first 1.0, then 0.8, then 0.4 (<=0.5 triggers stop)
     distances = [1.0, 0.8, 0.4]
     # centering statuses for yaw: first False, then True
     centered = [False, True]

     class FakeNode:
         def publish_move(self, direction, magnitude):
             moves.append((direction, magnitude))
         def publish_torpedo(self):
             torpedos.append(True)
         def get_target_distance(self):
             return distances.pop(0) if distances else 0.4
         def is_target_centered(self):
             return centered.pop(0) if centered else True

     # speed up test
     monkeypatch.setattr(time, 'sleep', lambda s: None)

     node = FakeNode()
     fsm = PlannerFSM(node)
     # Run only step10
     fsm.step10()

     # Expect: start forward inf, stop (0.0), start center yaw inf, stop, then launch
     assert moves[0] == (Direction.X, float('inf'))
     assert (Direction.X, 0.0) in moves
     assert (Direction.YAW, float('inf')) in moves
     assert (Direction.YAW, 0.0) in moves
     assert torpedos == [True]
     assert fsm.step_complete[10]


def test_step11_ascend_to_surface(monkeypatch):
     moves = []
     class FakeNode:
         def publish_move(self, direction, magnitude):
             moves.append((direction, magnitude))
     node = FakeNode()
     fsm = PlannerFSM(node)
     fsm.step11()
     assert moves == [(Direction.Z, float('inf'))]
     assert fsm.step_complete[11]
