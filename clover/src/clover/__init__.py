import rospy
from threading import Thread, Event

def long_callback(fn):
    """
    Decorator fixing a rospy issue for long-running topic callbacks, primarily
    for image processing.

    See: https://github.com/ros/ros_comm/issues/1901.

    Usage example:

    @long_callback
    def image_callback(msg):
        # perform image processing
        # ...

    rospy.Subscriber('main_camera/image_raw', Image, image_callback, queue_size=1)
    """
    e = Event()

    def thread():
        while not rospy.is_shutdown():
            e.wait()
            e.clear()
            fn(thread.current_msg)

    thread.current_msg = None
    Thread(target=thread, daemon=True).start()

    def wrapper(msg):
        thread.current_msg = msg
        e.set()

    return wrapper
