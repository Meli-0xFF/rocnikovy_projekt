#!/usr/bin/env python

import os
import pandas as pd
import numpy as np
import math
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_from_euler
from scipy.interpolate import interp1d
from scipy.signal import correlate, welch
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt
from shutil import copyfile

"""
Set of helper functions for IMU simulation framework ROS package.
2018 - 2019, David Cesenek, cesendav@fel.cvut.cz
"""

def quaternions_to_yaw(quaternions, axes='sxyz'):

    """ Compute yaw angle (z-axis rotation).
        Takes a numpy array of quaternions and returns numpy array of yaw angles in radians.
    """
    size = np.shape(quaternions)[1]
    yaw = np.zeros(size)
    for i in range(size):
        e = euler_from_quaternion(quaternions[:,i], axes)
        yaw[i] = e[2]
    return yaw

def euler_to_quaternions(roll,pitch,yaw):
    return quaternion_from_euler(roll, pitch, yaw, 'sxyz')

def get_time_in_seconds(time_nsecs, time_secs=None):
    if time_secs is None:
        return time_nsecs / 1e9
    else:
        return (time_secs * 1e9 + time_nsecs) / 1e9

def get_time_in_nanoseconds(time_sec, time_nsec):
    return time_sec * 1e9 + time_nsec

def normalize_array(array):
    assert (np.size(array) > 0)
    return array - array[0]

def convert_yaw_format(yaw):
    """
    Convert formalism of yaw angles from: <-pi, pi> to <0, 2pi>
    """
    array = np.zeros(np.shape(yaw))
    for i, val in enumerate(yaw):
        if (val < 0):
            array[i] = 2*np.pi + val
        elif(val > 2*math.pi):
            array[i] = val - 2*math.pi
        else:
            array[i] = val

    return array

def normalize_yaw_angle(yaw):
    """
    Convert the angle from arbitrary notation to the <-pi, pi> notation
    """
    return np.arctan2(np.sin(yaw), np.cos(yaw))

def compute_angle_difference(angle1, angle2):
    """
    For two given numpy angles in different notations (in radians), compute their difference.
    Returns angle difference in <-pi, pi>.
    """
    # convert arbitrary angle notations to the same <-pi, pi> notation
    a1 = np.arctan2(np.sin(angle1), np.cos(angle1))
    a2 = np.arctan2(np.sin(angle2), np.cos(angle2))

    if isinstance(angle1, np.ndarray) and isinstance(angle2, np.ndarray):
        N = np.size(a1)
        assert(np.size(a1) == np.size(a2))

        a1 = convert_yaw_format(a1)
        a2 = convert_yaw_format(a2)
        d_a = np.zeros(N)

        for i in range(N):
            d_a[i] = a1[i] - a2[i]
            if(d_a[i] < -np.pi):
                d_a[i] = (2*np.pi - a2[i]) + a1[i]  # the transition: 2pi >> 0
            elif(d_a[i] > np.pi):
                d_a[i] = -((2*np.pi - a1[i]) + a2[i]) # the transition: 0 >> 2pi
    else:
        # convert it again to <0, 2pi> notation
        angles = convert_yaw_format([a1,a2])
        a1 = angles[0]
        a2 = angles[1]
        d_a = a1 - a2
        if(d_a < -np.pi):
            d_a = (2*np.pi - a2) + a1  # the transition: 2pi >> 0
        elif(d_a > np.pi):
            d_a = -((2*np.pi - a1) + a2) # the transition: 0 >> 2pi

    return d_a

def compute_1l_difference(time1, time2, data1, data2, time_step=0.1, angle=False, abs=True):
    """
    Compute the 1D difference of two, differently sampled signals with approximately same lenght,
    return the signal difference; optionally al absolute value of difference (when abs==True), or perform angle normalization (when angle==True)
    """
    max_time1 = np.amax(time1)
    max_time2 = np.amax(time2)
    max_time = np.amin([max_time1, max_time2])
    time_samples = max_time / time_step
    time = np.linspace(0, max_time, time_samples)

    f1 = interp1d(time1, data1, kind='linear', fill_value="extrapolate")
    f2 = interp1d(time2, data2, kind='linear', fill_value="extrapolate")
    d1_int = f1(time)
    d2_int = f2(time)

    if angle is True and abs is False:
        return time, compute_angle_difference(d1_int, d2_int)
    if angle is True and abs is True:
        return time, np.abs(compute_angle_difference(d1_int, d2_int))

    if angle is False and abs is True:
        return time, np.abs(d1_int - d2_int)

    return time, d1_int - d2_int

def compute_2l_difference(time1, time2, x1, y1, x2, y2, time_step=0.1):
    """
    Compute Euclidian distance between two 2D signals (i.e. two set of waypoints).
    Returns the distances in 1D numpy array of the same length as provided vectors
    """
    max_time = np.amin([np.amax(time1), np.amax(time2)])
    time_samples = max_time / time_step
    time = np.linspace(0, max_time, time_samples)

    f_x1 = interp1d(time1, x1, kind='linear')
    f_y1 = interp1d(time1, y1, kind='linear')
    f_x2 = interp1d(time2, x2, kind='linear')
    f_y2 = interp1d(time2, y2, kind='linear')

    x1_int = f_x1(time)
    y1_int = f_y1(time)
    x2_int = f_x2(time)
    y2_int = f_y2(time)
    return np.sqrt((x1_int - x2_int)**2 + (y1_int - y2_int)**2)

def compute_position_difference(pose1, pose2):
    """
    For two sets of positions compute their difference
    """
    return np.abs(pose1 - pose2)

def interpolate_1d_data (time, data, time_step=0.1, time_new=None, interpolation_type='linear'):
    "interpolate arbitrary 1D data with desired time_step, or with give new time axis"
    if time_new is not None:
        assert(np.size(time_new > 0))
    else:
        time_max = np.max(time)
        time_min = np.min(time)
        number_of_timesteps = (time_max - time_min)/time_step
        time_new = np.linspace(start=time_min, stop=time_max, num=number_of_timesteps, endpoint=True)
        assert(number_of_timesteps > 0)

    f_x = interp1d(time, data, kind=interpolation_type)

    return time_new, f_x(time_new)

def interpolate_data(time, x, y, yaw, interpolation_type='linear', time_step=0.1):
    """
    Return the interpolated values of x,y and yaw parameters, with desired time slices.
    Assumes both time and time step given in seconds.
    """
    time_new, x_new = interpolate_1d_data(time, x, interpolation_type=interpolation_type, time_step=time_step)
    _, y_new = interpolate_1d_data(time, y, interpolation_type=interpolation_type, time_step=time_step)
    _, yaw_new = interpolate_1d_data(time, yaw, interpolation_type=interpolation_type, time_step=time_step)

    return time_new, x_new, y_new, yaw_new

def compute_2_signals_shift(time1, time2, signal1, signal2):
    """
    Estimate the time shift between two signals using cross-correlation function.
    Returns time shift in seconds.
    """

    # interpolate both signals in the same time staps
    time_step = 0.001
    max_time = np.amin([np.amax(time1), np.amax(time2)])
    min_time = np.amax([np.amin(time1), np.amin(time2)])
    time_samples = max_time / time_step
    time_new = np.linspace(min_time, max_time, time_samples)
    _, s1 = interpolate_1d_data(time1, signal1, time_new=time_new, interpolation_type='linear')
    _, s2 = interpolate_1d_data(time2, signal2, time_new=time_new, interpolation_type='linear')
    N = np.size(time_new)

    # regularize given signals
    # s1 -= np.mean(s1)
    # s1 /= s1.std()
    # s2 -= np.mean(s2)
    # s2 /= s2.std()
    # s1 = np.roll(s1,2)

    # find cross-correlation
    xcorr = correlate(s1, s2)
    dt = np.arange(1-N, N)

    index_shift = dt[xcorr.argmax()]
    time_shift = time_step * index_shift
    return time_shift

def compute_psd_welch(signal, Ts, nperseg=None):
    """
    Compute the PowerSpectralDensity function by Welch's method, which provides a good trade-off
    between frequency resolution and error caused by noise.
    Returns f, Pxx, nperseg, l;
    where:
    - f used frequencies
    - Pxx PSD function for these frequencies
    - nperseg - number of samples per segment
    - l total number of samples in the signal
    """
    # remove the mean, i.e. the frequency at 0 Hz
    signal = signal - np.mean(signal)
    l = np.size(signal)
    if nperseg is None:
        nperseg = 262144

        # get the biggest possible number of samples per segment to achive good frequency resolution
        while (nperseg > l):
            nperseg = nperseg / 2
    # rospy.loginfo("PSD welch method: selected nperseg: %d",nperseg)
    f, Pxx = welch(signal,fs=1/Ts, nperseg=nperseg)
    return f, Pxx, nperseg, l

def load_data_imu(reference_imu_file, start_rostime, stop_rostime):
    """
    Load the reference real IMU data from the csv file, crop it to match with the experiment boundaries.
    If zero times are given, it loads the whole raw data file.
    Return numpy arrays:
    time, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, qx, qy, qz, qw
    """
    try:
        assert(os.path.isfile(reference_imu_file))
        df = pd.read_csv(reference_imu_file, delimiter=';', low_memory=False)

    except(AssertionError):
        rospy.logerr("The IMU raw data file: %s was not found!")
        return None

    if start_rostime.secs == 0 and start_rostime.nsecs == 0 and stop_rostime.secs == 0 and stop_rostime.nsecs == 0:
        start_idx = 0
        stop_idx = np.size(df['secs'].values)
    else:
        start_idx_secs = find_nearest_index(df['secs'].values, value=start_rostime.secs, start_idx=0)
        start_idx = find_nearest_nsecs_index(df['nsecs'].values, value=start_rostime.nsecs, start_idx=start_idx_secs)
        stop_idx_secs = find_nearest_index(df['secs'].values, value=stop_rostime.secs, start_idx=start_idx)
        stop_idx = find_nearest_nsecs_index(df['nsecs'].values, value=stop_rostime.nsecs, start_idx=stop_idx_secs)

    rospy.loginfo("IMU raw data start_idx:%f", start_idx)
    rospy.loginfo("IMU raw data stop_idx:%f", stop_idx)

    try:
        assert(stop_idx >= start_idx)
    except (AssertionError):
        rospy.logerr("Error when founding the IMU raw data boundaries' indexes.")
        raise AssertionError

    rospy.loginfo("IMU raw data - duration:%f secs", df['secs'].values[stop_idx] - df['secs'].values[start_idx])

    # normalize the time and store it as seconds
    time = df['secs'].values[start_idx:stop_idx] + get_time_in_seconds(df['nsecs'].values[start_idx:stop_idx])
    time = normalize_array(time)

    # quaterions - reference position
    qx = df['x'].values[start_idx:stop_idx]
    qy = df['y'].values[start_idx:stop_idx]
    qz = df['z'].values[start_idx:stop_idx]
    qw = df['w'].values[start_idx:stop_idx]

    # linear accelerations:
    acc_x = df['x.2'].values[start_idx:stop_idx]
    acc_y = df['y.2'].values[start_idx:stop_idx]
    acc_z = df['z.2'].values[start_idx:stop_idx]

    # angular velocities
    gyro_x = df['x.1'].values[start_idx:stop_idx]
    gyro_y = df['y.1'].values[start_idx:stop_idx]
    gyro_z = df['z.1'].values[start_idx:stop_idx]

    return time, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, qx, qy, qz, qw

def normalize_yaw_angle(yaw):
    """
    Convert the angle from arbitrary notation to the <-pi, pi> notation
    """
    return np.arctan2(np.sin(yaw), np.cos(yaw))

def load_data_position(reference_pose_file, normalize):
    """
    Load csv file with provided reference positions and orientations (set of reference waypoints),
    assume that the position is provided as quaternions, with the rotation order: 'xyz' (with static frame)
    """
    rospy.loginfo("Trying to load csv file with raw position data:\n  ---> %s", os.path.join(os.path.abspath(os.path.curdir), reference_pose_file))
    df = pd.read_csv(reference_pose_file, delimiter=';')

    # normalize time -> start in time 0sec,0nsec, convert it to seconds:
    t = (df['rosbagTimestamp'].values - df['rosbagTimestamp'].values[0])

    if normalize: # normalize also postion (ie. should robot start at position [0,0]?)
        x = df['x'].values - df['x'].values[0]
        y = df['y'].values - df['y'].values[0]
    else:
        x = df['x'].values
        y = df['y'].values
    quaternions = np.array([df['x.1'].values, df['y.1'].values, df['z.1'].values, df['w'].values])
    yaw = quaternions_to_yaw(quaternions, axes='sxyz')

    return t, x, y, yaw

def generate_mock_data():
    """
    Generate simple trajectory with desired properties.
    """
    # stright simple path implemented only for now
    x_start = 0 #meters
    x_stop = 10
    y_start = 0
    y_stop = 0

    yaw_start = 0 #rad
    yaw_stop = 0

    duration = 30 #sec
    time_step = 0.2

    number_of_steps = duration/time_step
    time = np.linspace(0, duration, num=number_of_steps, endpoint=True)
    x = np.linspace(x_start, x_stop, num=number_of_steps, endpoint=True)
    y = np.linspace(y_start, y_stop, num=number_of_steps, endpoint=True)
    yaw = np.linspace(yaw_start, yaw_stop, num=number_of_steps, endpoint=True)

    return time, x, y, yaw

def save_trajectory_data(time, x, y, yaw, velocity_left, velocity_right, file_name):
    """
    Save the reference trajectory data (raw, or mocked) into csv file together with
    computed wheel velocities for the result comparison script.
    """
    array = np.vstack((time, x, y, yaw, velocity_left, velocity_right)).T
    df = pd.DataFrame(array)
    directory = os.path.dirname(file_name)

    if not os.path.exists(directory):
        os.makedirs(directory)
        rospy.loginfo("The output_dir: %s was not found, I've created it!", directory)

    df.to_csv(path_or_buf=file_name, header=['time', 'x', 'y','yaw','velocity_left','velocity_right'], index=False, sep=';')
    rospy.loginfo("Trajectory data saved in file:\n --->%s", file_name)

def load_trajectory_data(reference_pose_file):
    """
    Load reference trajectory data; assume that it was already stored in appropriate
    form in csv file with columns marked with headers: [time, x, y, yaw, velocity_left, velocity_right]
    Already normalized time is in seconds, yaw angles in radians.
    """
    rospy.loginfo("Trying to load reference trajectory produced by path_follower script")
    try:
        assert(os.path.isfile(reference_pose_file))
        df = pd.read_csv(reference_pose_file, delimiter=';')
        rospy.loginfo("Reference trajectory data successfuly loaded from csv file:\n --> %s", reference_pose_file)
    except (AssertionError):
        rospy.logerr("The csv file \"%s\" withReference trajectory data not found!",reference_pose_file)
        os._exit(1)

    return df['time'].values, df['x'].values, df['y'].values, df['yaw'].values, df['velocity_left'].values, df['velocity_right'].values

def find_nearest_index(array, value, start_idx=0):
    """
    Return the index of the numpy array field, whose value is closest to the given value.
    In case of multiple occurences, the first index is returned.
    """
    array = np.asarray(array)
    idx = start_idx + (np.abs(array[start_idx:] - value)).argmin()
    assert (idx >= start_idx)
    return idx

def find_nearest_nsecs_index(array, value, start_idx=0):
    """
    In a given numpy array of nanoseconds,
    find the first value that is greater than the given nsec value.
    Returns its index.
    """
    sum = 0

    for i in range(start_idx, np.size(array)):
        sum += array[i]
        if sum > value:
            return i

    return start_idx

def save_config_file(time_suffix, ini_file, output_dir):
    """
    Save the copy of ini file to store the config of executed experiment.
    """
    if not os.path.isfile(ini_file):
        rospy.logerr("The confing file: %s was not found when trying to copy it and store!",ini_file)
        raise IOError

    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        rospy.loginfo("The output_dir: %s was not found, I've created it!")
    new_filename = output_dir + time_suffix + '_config.ini'

    try:
        copyfile(ini_file, new_filename)
    except IOError:
        rospy.logerr("Error when saving the copy of config file with name:%s", new_filename)