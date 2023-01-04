import rospy
import requests
import numpy

_NETDATA_URL = 'http://127.0.0.1:19999/api/v1/'

def _request_data(url):
    r = requests.get(url)
    
    if r.status_code != 200:
        msg = "NetData request not successful (url: '{}', status_code {}".format(url, r.status_code)
        raise requests.ConnectionError(msg)

    return r.json()

def query_netdata_info():
    url = _NETDATA_URL + 'info'
    r = _request_data(url)

    return r

def query_netdata(chart, after):
    url = _NETDATA_URL + 'data?chart=%s&format=json&after=-%d' % (chart, int(after)) 
    rdata = _request_data(url)

    sdata = list(zip(*rdata['data']))
    d = dict()

    for idx, label in enumerate(rdata['labels']):
        np_array = numpy.array(sdata[idx])
        if np_array.dtype == object or (np_array == None).any():
            msg = "Data from NetData malformed: {}".format(label)
            rospy.logwarn(msg)
            rospy.logwarn('... malformed data for Label <{}>: {}'.format(label, np_array))
            return None, msg
        d[label] = np_array
    return d, None
