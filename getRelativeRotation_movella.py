import numpy as np
# from scipy.spatial.transform import Rotation as R
from pyquaternion import Quaternion


def getRelativeRotation_movella(data):

    data = data.reshape(-1, 4)  # Make sure it's (N, 4)

    numTrackers = data.shape[0]
    qTrackers = [Quaternion(data[i]) for i in range(numTrackers)]
    qRelative = []

    if numTrackers > 1:
        for i in range(numTrackers - 1):
            qRel = qTrackers[i + 1].conjugate * qTrackers[i]
            qRelative.append(qRel)
    else:
        qRelative = [np.nan]  # or [Quaternion()] or whatever you want

    return qRelative
