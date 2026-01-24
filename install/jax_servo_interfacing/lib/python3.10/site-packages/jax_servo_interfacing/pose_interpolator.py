import time 

def lerp(a, b, t):
    return a + (b - a) * t


def interpolate_poses(start_pose, end_pose, duration, rate_hz=50):
    """
    Generator that yields interpolated joint poses.

    start_pose, end_pose: dict[str, float]
    duration: seconds
    rate_hz: update frequency

    yields: dict[str, float]
    """

    steps = max(1, int(duration * rate_hz))
    dt = 1.0 / rate_hz

    for i in range(steps + 1):
        t = i / steps

        pose = {}
        for joint in end_pose.keys():
            start = start_pose.get(joint, 0.0)
            end = end_pose[joint]
            pose[joint] = lerp(start, end, t)

        yield pose

        time.sleep(dt)
