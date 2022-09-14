import pytest
import subprocess

def test_no_tf_repeated_data():
    # `/rosout` acts weirdly inside rostest, so using a subprocess
    cmd = """python -c 'import rospy, tf; rospy.init_node("foo"); listener = tf.TransformListener(); rospy.sleep(2)'"""
    output = str(subprocess.check_output(cmd, shell=True, stderr=subprocess.STDOUT))
    assert 'TF_REPEATED_DATA' not in output, 'TF_REPEATED_DATA was logged on duplicate markers'
