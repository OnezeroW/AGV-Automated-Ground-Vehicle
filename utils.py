'''
Created on Jan 19, 2020

@author: duolu
'''

import math
import numpy as np


def wgs84_llh_to_ecef(latitude, longitude, height):
    
    lat = latitude / 180 * np.pi
    lon = longitude / 180 * np.pi
    
    a = 6378137.0
    b = 6356752.314245
    
    b2 = b * b
    a2 = a * a
    
    
    slat = math.sin(lat)
    clat = math.cos(lat)
    slon = math.sin(lon)
    clon = math.cos(lon)

    N = a2 / math.sqrt(a2 * clat * clat + b2 * slat * slat)

    x = (N + height) * clat * clon
    y = (N + height) * clat * slon
    z = (b2 / a2 * N + height) * slat

    return (x, y, z)

def wgs84_ecef_to_ned(x, y, z, lat0, lon0, height0):
    
    x0, y0, z0 = wgs84_llh_to_ecef(lat0, lon0, height0)

    # CAUTION: angles must be in radian before doing sin() and cos().
    lat0 = lat0 / 180 * np.pi
    lon0 = lon0 / 180 * np.pi
    
    slat = math.sin(lat0)
    clat = math.cos(lat0)
    slon = math.sin(lon0)
    clon = math.cos(lon0)
    
    R = np.zeros((3, 3))
    
    R[0, 0] = -slat * clon
    R[0, 1] = -slon
    R[0, 2] = -clat * clon
    R[1, 0] = -slat * slon
    R[1, 1] = clon
    R[1, 2] = -clat * slon
    R[2, 0] = clat
    R[2, 1] = 0
    R[2, 2] = -slat
    
    p = np.asarray((x, y, z))
    p = p.reshape((3, 1))
    
    p_ref = np.asarray((x0, y0, z0))
    p_ref = p_ref.reshape((3, 1))
    
    # CAUTION: R is transposed first
    ned = np.matmul(R.T, p - p_ref)
    
    return ned[0], ned[1], ned[2]

def wgs84_ecef_to_enu(x, y, z, lat0, lon0, height0):
    
    x0, y0, z0 = wgs84_llh_to_ecef(lat0, lon0, height0)

    # CAUTION: angles must be in radian before doing sin() and cos().
    lat0 = lat0 / 180 * np.pi
    lon0 = lon0 / 180 * np.pi
        
    slat = math.sin(lat0)
    clat = math.cos(lat0)
    slon = math.sin(lon0)
    clon = math.cos(lon0)
    
    R = np.zeros((3, 3))
    
    R[0, 0] = -slon
    R[0, 1] = clon
    R[0, 2] = 0
    R[1, 0] = -slat * clon
    R[1, 1] = -slat * slon
    R[1, 2] = clat
    R[2, 0] = clat * clon
    R[2, 1] = clat * slon
    R[2, 2] = slat
    
    p = np.asarray((x, y, z))
    p = p.reshape((3, 1))
    
    p_ref = np.asarray((x0, y0, z0))
    p_ref = p_ref.reshape((3, 1))
    
    
    enu = np.matmul(R, p - p_ref)
    
    return enu[0], enu[1], enu[2]

def wgs84_llh_to_ned(lat, lon, height, lat0, lon0, height0):

    x, y, z = wgs84_llh_to_ecef(lat, lon, height)
    
    n, e, d = wgs84_ecef_to_ned(x, y, z, lat0, lon0, height0)

    return n, e, d

def wgs84_llh_to_enu(lat, lon, height, lat0, lon0, height0):

    x, y, z = wgs84_llh_to_ecef(lat, lon, height)
    
    e, n, u = wgs84_ecef_to_enu(x, y, z, lat0, lon0, height0)

    return e, n, u



# ---------------------- flocking --------------------------


def sigma_norm(v, sigma):
    '''
    
    CAUTION: The input v is a numpy 1D array. 
    The output is a scalar.
    '''
    
    assert sigma > 1e-6
    
    ve = np.linalg.norm(v)
    ve_square = ve * ve
    
    ret = (math.sqrt(1 + sigma * ve_square) - 1) / sigma
    
    return ret

def sigma_norm_scalar(z, sigma):
    '''
    
    CAUTION: The input v is a numpy 1D array. 
    The output is a scalar.
    '''
    
    assert sigma > 1e-6
    
    z_square = z * z
    
    ret = (math.sqrt(1 + sigma * z_square) - 1) / sigma
    
    return ret

def sigma_1(v):
    '''
    
    CAUTION: The input v is a numpy 1D array. 
    The output is an numpy 1D array of the same size.
    
    '''
    
    nv = np.linalg.norm(v)
    
    n = math.sqrt(1 + nv * nv)
    
    sig_1 = v / n
    
    return sig_1


def sigma_s(v, sigma):
    '''
    
    CAUTION: The input v is a numpy 1D array. 
    The output is an numpy 1D array of the same size.
    '''

    nv = np.linalg.norm(v)
    
    n = math.sqrt(1 + sigma * nv * nv)
    
    sig_s = v / n
    
    return sig_s



def rou_h_func(z, h):

    assert h > 0 and h < 1 and z >= 0

    if z >= 0 and z < h:
        
        rou_h = 1;
        
    elif z >= h and z <= 1:
        
        c = math.cos(math.pi * (z - h) / (1 - h))
        rou_h = 0.5 * (1 + c)
        
    else:
    
        rou_h = 0
        
    
    return rou_h


def sigma_1_scalar(z):
    '''
    
    CAUTION: z is a scalar. 
    
    '''
    
    n = math.sqrt(1 + z * z)
    
    sig_1s = z / n
    
    return sig_1s


def phi_func(z, a, b):
    '''
    
    CAUTION: z is a scalar. 
    '''
    
    
    c = math.fabs(a - b) / math.sqrt(4 * a * b);
    phi = 0.5 * ((a + b) * sigma_1_scalar(z + c) + (a - b))
    
    return phi



def calculate_f_alpha_1_one_agent(
        z_pos, diff_pos, r_alpha, h_alpha, 
        d_alpha, sigma, a, b):

    s1 = rou_h_func(z_pos / r_alpha, h_alpha)
    s2 = phi_func(z_pos - d_alpha, a, b)
    t3 = sigma_s(diff_pos, sigma)

    return s1 * s2 * t3

def calculate_f_alpha_2_one_agent(
        z_pos, diff_vel, r_alpha, h_alpha):
    
    aij = rou_h_func(z_pos / r_alpha, h_alpha)
    
    t = aij * diff_vel
    
    return t


def calculate_f_beta_1_one_obstacle(
        z_pos, diff_pos, r_beta, h_beta, sigma):
    
    s1 = rou_h_func(z_pos / r_beta, h_beta)
    s2 = sigma_1_scalar(z_pos - r_beta) - 1
    t3 = sigma_s(diff_pos, sigma)
    
    t3n = np.linalg.norm(t3)

    return s1 * s2 * t3 / t3n

def calculate_f_beta_2_one_obstacle(
        z_pos, diff_vel, r_beta, h_beta):
    
    bik = rou_h_func(z_pos / r_beta, h_beta)
    
    t = bik * diff_vel
    
    return t



if __name__ == '__main__':
    pass
























