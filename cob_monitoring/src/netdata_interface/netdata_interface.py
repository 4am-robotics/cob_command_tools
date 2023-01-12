import rospy
import requests
import numpy


class NetdataInterface:
    def __init__(self, base_url="http://127.0.0.1:19999/api/v1"):
        """Create a new NetData interface.

        :param base_url: Base URL of the NetData interface
        """
        self._base_url = base_url
        self._failed_counter_dict = {}

    def _request_data(self, url):
        res = requests.get(url)

        if res.status_code != 200:
            msg = "NetData request not successful (url='{}', status_code={})".format(
                url, res.status_code
            )
            raise requests.ConnectionError(msg)

        return res.json()

    def _reset_failed_counter(self, chart, label):
        """Reset the failed counter for given label in chart.

        :param chart: Chart
        :param label: Label
        """
        if chart not in self._failed_counter_dict:
            self._failed_counter_dict[chart] = {}

        self._failed_counter_dict[chart][label] = 0

    def _increase_failed_counter(self, chart, label):
        """Increase the failed counter for given label in chart.

        Throws an exception if failed counter is greater or equal 5.

        :param chart: Chart
        :param label: Label
        """
        if chart not in self._failed_counter_dict:
            self._failed_counter_dict[chart] = {}

        if label not in self._failed_counter_dict[chart]:
            self._failed_counter_dict[chart][label] = 1
        else:
            self._failed_counter_dict[chart][label] += 1

        if self._failed_counter_dict[chart][label] >= 5:
            raise Exception(
                "Data from NetData was malformed {} times (chart='{}', label='{}')".format(
                    self._failed_counter_dict[chart][label], chart, label
                )
            )

    def query_netdata_info(self):
        """Get NetData information."""
        url = "{}/info".format(self._base_url)
        res = self._request_data(url)

        return res

    def query_netdata(self, chart, after):
        """Get data from NetData chart after a certain time.

        :param chart: Chart identifier
        :param after: Timedelta in seconds
        """
        url = "{}/data?chart={}&format=json&after=-{}".format(
            self._base_url, chart, after
        )
        res = self._request_data(url)

        sdata = list(zip(*res["data"]))
        data = dict()

        for idx, label in enumerate(res["labels"]):
            np_array = numpy.array(sdata[idx])
            if np_array.dtype == object or (np_array == None).any():
                msg = "Data from NetData malformed: {}".format(label)
                rospy.logwarn(msg)
                rospy.logwarn(
                    "... malformed data for Label <{}>: {}".format(label, np_array)
                )
                self._increase_failed_counter(chart, label)
                return None, msg
            data[label] = np_array
            self._reset_failed_counter(chart, label)

        return data, None
