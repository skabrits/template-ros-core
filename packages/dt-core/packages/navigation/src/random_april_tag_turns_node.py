#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy
from duckietown_msgs.msg import FSMState, AprilTagsWithInfos, BoolStamped, TurnIDandType
from std_msgs.msg import String, Int16 #Imports msg
import math
import sys
# insert at 1, 0 is the script path (or '' in REPL)
# sys.path.append('/../../../../circle_drive/scripts/Stem_graph.py')
# import Stem_graph as sg


class RandomAprilTagTurnsNode(object):
    def __init__(self):
        # self.Rmap, self.R1, self.r1, self.R2, self.r2, self.R3, self.r3, self.R4, self.r4, self.R5, self.r5,
        # self.R6, self.r6 = sg.init_lib() self.cur_road = self.R1
        self.route = [1, 2]
        # Save the name of the node
        self.node_name = rospy.get_name()
        self.turn_type = -1

        rospy.loginfo("[%s] Initializing." % (self.node_name))

        # Setup publishers
        # self.pub_topic_a = rospy.Publisher("~topic_a",String, queue_size=1)
        self.pub_turn_type = rospy.Publisher("~turn_type",Int16, queue_size=1, latch=True)
        self.pub_id_and_type = rospy.Publisher("~turn_id_and_type",TurnIDandType, queue_size=1, latch=True)
        self.pub_turn = rospy.Publisher("/turn", String, queue_size=5)

        # Setup subscribers
        # self.sub_topic_b = rospy.Subscriber("~topic_b", String, self.cbTopic)
        self.sub_topic_mode = rospy.Subscriber("~mode", FSMState, self.cbMode, queue_size=1)
        #self.fsm_mode = None #TODO what is this?
        self.sub_topic_tag = rospy.Subscriber("~tag", AprilTagsWithInfos, self.cbTag, queue_size=1)
        self.sub_route = rospy.Subscriber("/route", String, self.setRoute, queue_size=1)

        # Read parameters
        self.pub_timestep = self.setupParameter("~pub_timestep", 1.0)
        # Create a timer that calls the cbTimer function every 1.0 second
        # self.timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep),self.cbTimer)

        rospy.loginfo("[%s] Initialzed." % (self.node_name))

        self.rate = rospy.Rate(30)  # 10hz

    def setRoute(self, string):
        string = string.data
        r = [0] * len(string)
        for i in range(len(string)):
            r[i] = string[i]
        self.route = r

    def cbMode(self, mode_msg):
        #print mode_msg
        #TODO PUBLISH JUST ONCE
        self.fsm_mode = mode_msg.state
        if(self.fsm_mode != mode_msg.INTERSECTION_CONTROL):
            self.turn_type = -1
            self.pub_turn_type.publish(self.turn_type)
            #rospy.loginfo("Turn type now: %i" %(self.turn_type))
    def cbTag(self, tag_msgs):
        if self.fsm_mode == "INTERSECTION_CONTROL" or self.fsm_mode == "INTERSECTION_COORDINATION" or self.fsm_mode == "INTERSECTION_PLANNING":
            #loop through list of april tags

            # filter out the nearest apriltag
            dis_min = 999
            idx_min = -1
            for idx, taginfo in enumerate(tag_msgs.infos):
                if(taginfo.tag_type == taginfo.SIGN):
                    tag_det = (tag_msgs.detections)[idx]
                    pos = tag_det.pose.pose.position
                    distance = math.sqrt(pos.x**2 + pos.y**2 + pos.z**2)
                    if distance < dis_min:
                        dis_min = distance
                        idx_min = idx

            if idx_min != -1:
                taginfo = (tag_msgs.infos)[idx_min]

                availableTurns = []
                #go through possible intersection types
                signType = taginfo.traffic_sign_type
                if(signType == taginfo.NO_RIGHT_TURN or signType == taginfo.LEFT_T_INTERSECT):
                    rospy.loginfo("left T")
                    availableTurns = [0,1] # these mystical numbers correspond to the array ordering in open_loop_intersection_control_node (very bad)
                elif (signType == taginfo.NO_LEFT_TURN or signType == taginfo.RIGHT_T_INTERSECT):
                    rospy.loginfo("right T")
                    availableTurns = [1,2]
                elif (signType== taginfo.FOUR_WAY):
                    availableTurns = [0,1,2]
                elif (signType == taginfo.T_INTERSECTION):
                    rospy.loginfo("center T")
                    availableTurns = [0,2]

                    #now randomly choose a possible direction
                if(len(availableTurns)>0):
                    if len(self.route) == 0:
                        randomIndex = numpy.random.randint(len(availableTurns)) # по часовой
                        chosenTurn = availableTurns[randomIndex]
                        # if randomIndex == 1:
                        #     self.pub_turn.publish("1")
                        # if randomIndex == 0:
                        #     self.pub_turn.publish("2")
                    else:
                        chosenTurn = availableTurns[::-1][self.route[0]-1]
                        del self.route[0]
                    self.turn_type = chosenTurn
                    self.pub_turn_type.publish(self.turn_type)

                    id_and_type_msg = TurnIDandType()
                    id_and_type_msg.tag_id = taginfo.id
                    id_and_type_msg.turn_type = self.turn_type
                    self.pub_id_and_type.publish(id_and_type_msg)

                    rospy.loginfo("possible turns %s." %(availableTurns))
                    #rospy.loginfo("Turn type now: %i" %(self.turn_type))

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        #rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." % (self.node_name))


if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('random_april_tag_turns_node', anonymous=False)

    # Create the NodeName object
    node = RandomAprilTagTurnsNode()

    # Setup proper shutdown behavior
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
