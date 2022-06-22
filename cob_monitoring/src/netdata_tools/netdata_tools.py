import rospy
import requests
import numpy

def query_netdata_info():
    try:
        r = requests.get('http://127.0.0.1:19999/api/v1/info')
    except requests.ConnectionError as ex:
        msg = "NetData ConnectionError: {}".format(ex)
        rospy.logerr(msg)
        return None, msg
    
    if r.status_code != 200:
        msg = "NetData request not successful with status_code {}".format(r.status_code)
        rospy.logerr(msg)
        return None, msg

    return r.json(), None

def query_netdata(chart, after):
    NETDATA_URI = 'http://127.0.0.1:19999/api/v1/data?chart=%s&format=json&after=-%d'
    url = NETDATA_URI % (chart, int(after))

    try:
        r = requests.get(url)
    except requests.ConnectionError as ex:
        msg = "NetData ConnectionError: {}".format(ex)
        rospy.logerr(msg)
        return None, msg

    if r.status_code != 200:
        msg = "NetData request not successful with status_code {}".format(r.status_code)
        rospy.logerr(msg)
        return None, msg

    rdata = r.json()

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
