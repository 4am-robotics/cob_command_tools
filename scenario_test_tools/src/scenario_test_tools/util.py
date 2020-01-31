import sys

import rospy


def countdown_sleep(duration, stepsize=1, text="{}"):
    """
    Print the numbers of a countdown on the terminal in-place, without using a new line for each number

    :param duration: How long to count down
    :param stepsize: Time between steps
    :param text: What text to display each tick. Must include a format string, eg. 'launch in {}...'
    """
    step, substep = divmod(duration, stepsize)
    for i in range(int(step), 0, -stepsize):
        if not rospy.is_shutdown():
            sys.stdout.write("\r" + text.format(i))
            sys.stdout.flush()
            rospy.sleep(stepsize)
    rospy.sleep(substep)

    if step > 0: sys.stdout.write('\n')
    sys.stdout.flush()

def round_tuple(tup, decimals):
    """
    Rounding operation broadcast over a tuple

    :param tup: tuple to be rounded
    :param decimals: how many decimals to round to
    :return: tuple of same dimension but with lower precision
    """
    return tuple([round(elem, decimals) for elem in tup])