#! /usr/bin/env python3

from enum import Enum
import enum
from typing import NamedTuple

class StateData(NamedTuple):
    name: object
    dataObject: object
    destinationState: object = 'MONITOR'

def sdataDecoder(state_queue, state):
    data = state_queue.get(False)
    if(data.destinationState == state):
        return data
    else:
        state_queue.put(data)
        return None

class AgentStates(Enum):
    IDL = 'IDLE'
    EXC = 'EXECUTION'
    MAP = 'MAPPING'
    ERR = 'ERROR'
    SDWN = 'SHUTDOWN'
    MON = 'MONITOR'

class AgentKeys(Enum):
    TRIGR_ACK = 'trigger_acknowledgement'
    TRIGR_M_RESET = 'trigger_motor_reset'
    TRIGR_STATE = 'trigger_state_change'
    TRIGR_STATE_EXTRA = 'trigger_state_change_with_data'
    SM_STATE = 'sm_state'
    ERR_OBJ = 'error_object'
    LNCH_OBJ = 'launch_object'
    MOD_STATES = 'module_states'
    IMG_STRM = 'image'
    VID_STRM = 'video'

class ErrCodes(Enum):
    BASE_OK = 5000
    BASE_EMERGENCY = 5001
    STATE_FALLTHROUGH = 5002

    SERVICE_NO_EXIST = 5020
    SERVICE_CALL_FAIL = 5021

class ShutdownAction(Enum):
    SHUTDOWN = 0
    RESTART = 1

class MapAction(Enum):
    NEW = 31
    CHANGE = 32

class MODE(Enum):
    MANUAL = 1
    AUTO = 2

class VIDACTION(Enum):
    START = 81
    SAVE = 82
    CANCEL = 83