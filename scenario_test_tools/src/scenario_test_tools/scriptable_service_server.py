import rospy

from scenario_test_tools.scriptable_base import ScriptableBase
from scenario_test_tools.util import countdown_sleep


class ScriptableServiceServer(ScriptableBase):
    """
    ScriptableServiceServer allows its users to determine the Response to a service Request
    """
    def __init__(self, name, service_type, request_formatter=format, response_formatter=format, default_response=None, response_delay=0):
        """
        Set up a ScriptableServiceServer based on the name and the type ofService is should implement
        :param service_type: Service type (e.g. std_srvs/SetBool)
        """
        ScriptableBase.__init__(self, name,
                                goal_formatter=request_formatter,
                                reply_formatter=response_formatter,
                                default_reply=default_response,
                                reply_delay=response_delay)

        self._srv = rospy.Service(name, service_type, self._execute_cb)

    def stop(self):
        super(ScriptableServiceServer, self).stop()

    def _execute_cb(self, request):
        """
        Called when the underlying service receives a goal.
        If the default_result is None, it will wait for a custom result to be set via reply* otherwise
        return the default_result after the given result_delay

        In the reply-case,it then notifies self.reply* (which should be called by a test script outside this class),
        after which self.reply* determines the result to the goal.
        Then it notifies _execute_cb that the result has been determined so that _execute_cb can send it
        """

        self._current_goal = request
        try:
            request_str = self.goal_formatter(self._current_goal)
        except Exception as e:
            rospy.logerr("request_formatter of {} raised an exception: {}".format(self._name, e))
            request_str = self._current_goal
        print('{}.execute: Request: {}'.format(self._name, request_str))

        if self.default_reply is not None:
            result = self.default_reply
            countdown_sleep(self._reply_delay, text="{}.execute: Wait for {}s. ".format(self._name, self._reply_delay) + "Remaining {}s...")
        else:
            self._request.set()
            # Now, wait for  to be called, which sets the _reply event AND the _next_reply
            self._reply.wait()
            self._reply.clear()

            result = self._next_reply

            self._next_reply = None
            self._current_goal = None
            self._sent.set()

        self._call_pre_reply_callbacks(request, result)
        return result
