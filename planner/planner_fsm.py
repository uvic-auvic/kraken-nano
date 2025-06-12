import time
from publisher import PlannerPublisher
from planner_node import PlannerNode, dummy_send_func
from messages import Direction, DetectedType 


class PlannerFSM:
    def __init__(self, node: PlannerNode):
        self.node = node
        self.current_step = 1
        self.num_steps = 14
        # Completion flags for each step, to be set externally
        self.step_complete = {i: False for i in range(1, self.num_steps + 1)}
        self.finished = False

    def run(self):
        while self.current_step <= self.num_steps and not self.finished:
            step_func = getattr(self, f"step{self.current_step}")
            print(f"--- Starting Step {self.current_step} ---")
            step_func()  # Call the step's function (loops internally until done)
            print(f"--- Step {self.current_step} Done ---\n")
            self.current_step += 1
        print("All steps complete. Mission finished.")
        self.finished = True

    def wait_for_step_complete(self, step_number):
        while not self.step_complete[step_number]:
            print(f"Waiting for completion of step{step_number}...")
            time.sleep(0.5)

    def mark_step_complete(self, step_number):
        self.step_complete[step_number] = True

    #step one will be to find the gate
    def step1(self):
        print("Performing continuous Yaw until gate detected")
        # start yawing indefinitely
        self.node.publish_move(Direction.YAW, float('inf'))
        # watch detections until gate appears
        while self.node.get_last_detection() != DetectedType.GATE:
            time.sleep(0.5)
            print(f" Seen so far: {self.node.get_last_detection()}")
        print("Gate detected! Stopping yaw.")
        # stop yaw immediately
        self.node.publish_move(Direction.YAW, 0.0)
        # mark step complete so run() can advance
        self.mark_step_complete(1)

    #go through the gate orient with picture for extra points
    def step2(self):
        print("Step 2: align with gate then drive through")
        # wait until vision reports gate is centered
        while not self.node.is_gate_centered():
            time.sleep(0.1)
        print("Gate centered: starting forward motion")
        # start moving forward indefinitely
        self.node.publish_move(Direction.X, float('inf'))
        # continue until gate leaves FOV
        while self.node.is_gate_centered():
            time.sleep(0.1)
        print("Gate out of view: advance extra 0.5m and stop")
        # move extra 0.5 meters
        self.node.publish_move(Direction.X, 0.5)
        # stop movement
        self.node.publish_move(Direction.X, 0.0)
        # mark this step done
        self.mark_step_complete(2)

    #find the start of the slalom
    def step3(self):
        print("Step 3: yaw until slalom gate detected")
        # start yawing indefinitely
        self.node.publish_move(Direction.YAW, float('inf'))
        # wait for slalom detection
        while self.node.get_last_detection() != DetectedType.SLALOM:
            time.sleep(0.5)
            print(f" Seen so far: {self.node.get_last_detection()}")
        print("Slalom gate detected! Stopping yaw.")
        # stop yaw
        self.node.publish_move(Direction.YAW, 0.0)
        # wait until centered
        while not self.node.is_slalom_centered():
            time.sleep(0.1)
        print("Slalom centered: starting forward motion")
        # drive forward indefinitely
        self.node.publish_move(Direction.X, float('inf'))
        # continue until slalom gate leaves view
        while self.node.is_slalom_centered():
            time.sleep(0.1)
        print("Slalom out of view: stopping motion")
        # stop motion
        self.node.publish_move(Direction.X, 0.0)
        # mark complete
        self.mark_step_complete(3)

    # step4: continue forward while slalom gate in view, stop when clear
    def step4(self):
        print("Step 4: drive forward while slalom gate in view")
        # while slalom gate remains centered
        while self.node.is_slalom_centered():
            self.node.publish_move(Direction.X, float('inf'))
            time.sleep(0.1)
            print(f" Slalom still in view: continuing forward")
        # stop movement once clear
        print("Slalom cleared: stopping motion")
        self.node.publish_move(Direction.X, 0.0)
        self.mark_step_complete(4)

    #locate the bin with the bottom camera
    def step5(self):
        while not self.step_complete[5]:
            print("Performing step 5 logic...")
            self.wait_for_step_complete(5)

    #locate image that matches the image that is on the gate that we went through on the start
    def step6(self):
        while not self.step_complete[6]:
            print("Performing step 6 logic...")
            self.wait_for_step_complete(6)

    #orient/yaw with bin
    def step7(self):
        while not self.step_complete[7]:
            print("Performing step 7 logic...")
            self.wait_for_step_complete(7)

    #Drop the ball into the bin
    def step8(self):
        while not self.step_complete[8]:
            print("Performing step 8 logic...")
            self.wait_for_step_complete(8)

    #locate target for torpedo
    def step9(self):
        while not self.step_complete[9]:
            print("Performing step 9 logic...")
            self.wait_for_step_complete(9)

    #drive towards target
    def step10(self):
        while not self.step_complete[10]:
            print("Performing step 10 logic...")
            self.wait_for_step_complete(10)

    #orient with target
    def step11(self):
        while not self.step_complete[11]:
            print("Performing step 11 logic...")
            self.wait_for_step_complete(11)

    #launch torpedo
    def step12(self):
        while not self.step_complete[12]:
            print("Performing step 12 logic...")
            self.wait_for_step_complete(12)

    #find entrance gate
    def step13(self):
        while not self.step_complete[13]:
            print("Performing step 13 logic...")
            self.wait_for_step_complete(13)

    #go through entrance gate
    def step14(self):
        while not self.step_complete[13]:
            print("Performing step 14 logic...")
            self.wait_for_step_complete(14)
# Example/test usage
if __name__ == "__main__":
    import threading
    # initialize publisher and planner node
    publisher = PlannerPublisher(dummy_send_func)
    node = PlannerNode(publisher)
    fsm = PlannerFSM(node)

    def simulate_external_completion(fsm):
        for i in range(1, fsm.num_steps + 1):
            time.sleep(2)
            print(f"[External] Marking step{i} complete")
            fsm.mark_step_complete(i)

    threading.Thread(target=simulate_external_completion, args=(fsm,), daemon=True).start()
    fsm.run()
