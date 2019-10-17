#!/usr/bin/env python
import smach

from march_state_machine.states.GaitState import GaitState


def create():
    sm_sofa_stand = smach.StateMachine(outcomes=['succeeded', 'preempted', 'failed'])
    with sm_sofa_stand:
        smach.StateMachine.add('PREPARE STAND UP', GaitState("sofa_stand", "prepare_stand_up"),
                               transitions={'succeeded': 'STAND UP', 'preempted': 'failed', 'aborted': 'failed'})
        smach.StateMachine.add('STAND UP', GaitState("sofa_stand", "stand_up"),
                               transitions={'succeeded': 'succeeded', 'preempted': 'preempted', 'aborted': 'failed'})
    return sm_sofa_stand
