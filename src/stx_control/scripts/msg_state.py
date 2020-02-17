import time


class MsgState:
    msgCounter = 1

    def __init__(self, msg_id, success, fail, immediate):
        self.msg_id = msg_id
        self.is_immediate = immediate
        self.is_differed = None
        self.err_code = None
        self.creation_time = int(round(time.time() * 1000))
        self.immediate_ack_time = None
        self.success_callback = success
        self.fail_callback = fail
        self.dead = False