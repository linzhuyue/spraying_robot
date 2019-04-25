#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    if your os ubuntu 18.04:
        echo "deb http://archive.ubuntu.com/ubuntu trusty main universe" | sudo tee /etc/apt/sources.list.d/wily-copies.list
        sudo apt update

        sudo apt install python-wxgtk2.8

        sudo rm /etc/apt/sources.list.d/wily-copies.list
        sudo apt update
"""
import rospy, sys
import smach
import smach_ros
#define Foo
class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['outcome1','outcome2'])
        self.counter=0
    def execute(self,userdata):
        rospy.loginfo('Executing state Foo')
        if self.counter<3:
            self.counter+=1
            return 'outcome1'
        else:
            return 'outcome2'
#define Bar
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['outcome2'])
        self.counter=0
    def execute(self,userdata):
        rospy.loginfo('Executing state Bar')
        return 'outcome2'
def main():
    rospy.init_node('smach_example_state_machine')
    #create a smach matchine
    sm=smach.StateMachine(outcomes=['outcome4','outcome5'])
    #open smach state vector
    with sm:
        smach.StateMachine.add('Foo',Foo(),transitions={'outcome1':'Bar','outcome2':'outcome4'})
        smach.StateMachine.add('Bar',Bar(),transitions={'outcome2':'Foo'})
    #create and open internal monitor service 
    sis =smach_ros.IntrospectionServer('my_smach_introspection_server',sm,'/SM_ROOT')
    sis.start()
    #start smach
    outcome =sm.execute()
    rospy.spin()
    sis.stop()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
