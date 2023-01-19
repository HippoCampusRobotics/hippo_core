import numpy as np


def limit_to_shorter(a, b):
    if len(a) < len(b):
        return a, b[:len(a)]
    else:
        return a[:len(b)], b


def crop_data(data, time, t0, t1):
    tmp = np.abs(time - t0)
    a = tmp.argmin()
    tmp = np.abs(time - t1)
    b = tmp.argmin()
    return data[a:b], time[a:b]

def resample(data, time, time_sample):
    resampled_data = np.interp(time_sample, time, data)
    return resampled_data
