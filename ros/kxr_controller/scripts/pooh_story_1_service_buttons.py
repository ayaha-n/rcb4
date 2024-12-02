#!/usr/bin/env python

import rospy

from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse
from std_srvs.srv import SetBool
from std_srvs.srv import SetBoolResponse
from std_srvs.srv import Trigger
from std_srvs.srv import TriggerResponse


class PoohStoryServiceButtons(object):
    def __init__(self):
        self.services = [
            rospy.Service('pooh_story_1/thanks', Trigger, self.trigger_thanks_cb),
            rospy.Service('pooh_story_1/wait', Trigger, self.trigger_wait_cb),
            rospy.Service('pooh_story_1/hoge', Trigger, self.trigger_hoge_cb),
            rospy.Service('pooh_story_1/fuga', Trigger, self.trigger_fuga_cb),
            rospy.Service('pooh_story_1/1_1', Trigger, self.trigger_1_1_cb),
            rospy.Service('pooh_story_1/1_2', Trigger, self.trigger_1_2_cb),
            rospy.Service('pooh_story_1/1_3', Trigger, self.trigger_1_3_cb),
            rospy.Service('pooh_story_1/1_4', Trigger, self.trigger_1_4_cb),
            rospy.Service('pooh_story_1/2_1', Trigger, self.trigger_2_1_cb),
            rospy.Service('pooh_story_1/2_2', Trigger, self.trigger_2_2_cb),
            rospy.Service('pooh_story_1/2_3', Trigger, self.trigger_2_3_cb),
            rospy.Service('pooh_story_1/2_4', Trigger, self.trigger_2_4_cb),
        ]
        self._name = rospy.get_name()

    def _set_bool_cb(self, req):
        rospy.loginfo('{} | SetBool service called: req.data={}'.format(self._name,req.data))
        return SetBoolResponse(success=True)

    def _trigger_cb(self, req):
        rospy.loginfo('{} | Trigger service called'.format(self._name))
        return TriggerResponse(success=True)

    def _empty_cb(self, req):
        rospy.loginfo('{} | Empty service called'.format(self._name))
        return EmptyResponse()

    def trigger_thanks_cb(self, req):
        rospy.loginfo('pooh_story_1 thanks | Trigger service called')
        rospy.loginfo('Pooh: ありがとう')
        ## TODO rosparamを設定して動きと音声を再生？
        return TriggerResponse(success=True)

    def trigger_wait_cb(self, req):
        rospy.loginfo('pooh_story_1 wait | Trigger service called')
        rospy.loginfo('Pooh: ちょっと待ってね')
        ## TODO rosparamを設定して動きと音声を再生？
        return TriggerResponse(success=True)

    def trigger_hoge_cb(self, req):
        rospy.loginfo('pooh_story_1 hoge | Trigger service called')
        rospy.loginfo('Pooh: hoge')
        ## TODO rosparamを設定して動きと音声を再生？
        return TriggerResponse(success=True)

    def trigger_fuga_cb(self, req):
        rospy.loginfo('pooh_story_1 fuga | Trigger service called')
        rospy.loginfo('Pooh: fuga')
        ## TODO rosparamを設定して動きと音声を再生？
        return TriggerResponse(success=True)

    def trigger_1_1_cb(self, req):
        rospy.loginfo('pooh_story_1 1_1 | Trigger service called')
        rospy.loginfo('Pooh: おはよう')
        rospy.loginfo('参加者: プー、おはよう')
        ## TODO rosparamを設定して動きと音声を再生？
        return TriggerResponse(success=True)

    def trigger_1_2_cb(self, req):
        rospy.loginfo('pooh_story_1 1_2 | Trigger service called')
        rospy.loginfo('Pooh: あのね、今日はきみにお願いがあるの。風船ちょうだい')
        rospy.loginfo('参加者: 風船なんてなんに使うの？')
        ## TODO rosparamを設定して動きと音声を再生？
        return TriggerResponse(success=True)

    def trigger_1_3_cb(self, req):
        rospy.loginfo('pooh_story_1 1_3 | Trigger service called')
        rospy.loginfo('Pooh: ハチミツ取るの！')
        rospy.loginfo('参加者: 風船じゃあハチミツは取れないよ')
        ## TODO rosparamを設定して動きと音声を再生？
        return TriggerResponse(success=True)

    def trigger_1_4_cb(self, req):
        rospy.loginfo('pooh_story_1 1_4 | Trigger service called')
        rospy.loginfo('Pooh: 取れるよ！')
        ## TODO rosparamを設定して動きと音声を再生？
        return TriggerResponse(success=True)

    def trigger_2_1_cb(self, req):
        rospy.loginfo('pooh_story_1 2_1 | Trigger service called')
        rospy.loginfo('参加者: 青い風船と緑の風船、どっちがいい？')
        rospy.loginfo('Pooh: どっちがいいかなあ')
        ## TODO rosparamを設定して動きと音声を再生？
        return TriggerResponse(success=True)

    def trigger_2_2_cb(self, req):
        rospy.loginfo('pooh_story_1 2_2 | Trigger service called')
        rospy.loginfo('参加者: 風船ではちみつを取るには、ミツバチに気づかれないようにしなくちゃいけないよね')
        rospy.loginfo('Pooh: うん。緑だったら木に見えるし、青だったら空に見えるから、どっちの方が気づかれにくいかが大事なんだ')
        ## TODO rosparamを設定して動きと音声を再生？
        return TriggerResponse(success=True)

    def trigger_2_3_cb(self, req):
        rospy.loginfo('pooh_story_1 2_3 | Trigger service called')
        rospy.loginfo('参加者: でも、ハチは、風船の下にいる、きみのこと、気がつくんじゃない？')
        rospy.loginfo('Pooh: じゃ、僕は黒い雲のフリをしようと思うんだ')
        ## TODO rosparamを設定して動きと音声を再生？
        return TriggerResponse(success=True)

    def trigger_2_4_cb(self, req):
        rospy.loginfo('pooh_story_1 2_4 | Trigger service called')
        rospy.loginfo('参加者: それなら、空と同じ、青い色の風船がいいね')
        ## TODO rosparamを設定して動きと音声を再生？
        return TriggerResponse(success=True)

if __name__ == '__main__':
    rospy.init_node('pooh_story_1_service_buttons')
    service_buttons = PoohStoryServiceButtons()
    rospy.spin()
