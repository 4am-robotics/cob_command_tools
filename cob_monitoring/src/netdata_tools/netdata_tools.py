import rospy
import requests
import numpy

def query_netdata_info():

    try:
        r = requests.get('http://127.0.0.1:19999/api/v1/info')
    except requests.ConnectionError as ex:
        rospy.logerr("NetData ConnectionError %r", ex)
        return None
    
    if r.status_code != 200:
        rospy.logerr("NetData request not successful with status_code %d", r.status_code)
        return None

    return r.json()

def query_netdata(chart, after):
    NETDATA_URI = 'http://127.0.0.1:19999/api/v1/data?chart=%s&format=json&after=-%d'
    url = NETDATA_URI % (chart, int(after))

    try:
        r = requests.get(url)
    except requests.ConnectionError as ex:
        rospy.logerr("NetData ConnectionError %r", ex)
        return None

    if r.status_code != 200:
        rospy.logerr("NetData request not successful with status_code %d", r.status_code)
        return None

    rdata = r.json()

    sdata = list(zip(*rdata['data']))
    d = dict()

    for idx, label in enumerate(rdata['labels']):
        np_array = numpy.array(sdata[idx])
        if np_array.dtype == object:
            rospy.logwarn("Data from NetData malformed")
            return None
        d[label] = np_array

    return d