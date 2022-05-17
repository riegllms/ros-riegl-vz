import numpy as np
'''Quaternion 'lib' von Ul'''


def mult(q, p):
    '''returns product of two quaternions (q*p)'''
    r0 = q[0]*p[0]-q[1]*p[1]-q[2]*p[2]-q[3]*p[3]
    r1 = q[0]*p[1]+q[1]*p[0]+q[2]*p[3]-q[3]*p[2]
    r2 = q[0]*p[2]-q[1]*p[3]+q[2]*p[0]+q[3]*p[1]
    r3 = q[0]*p[3]+q[1]*p[2]-q[2]*p[1]+q[3]*p[0]
    return np.array([r0, r1, r2, r3])


def conj(q):
    '''returns conjugate quaternion'''
    r = q.copy()
    r[1:] *= -1
    return r


def inv(q):
    '''returns inverted quaternion'''
    return conj(q)/(q*q).sum()


def rotateVectorByQuaternion(v, q):
    '''returns q*v*q^-1 which rotates v by q
    v can be either a vector of a quaternion representing a vector'''
    if len(v)==3:
        v = np.hstack((0,v))
    return mult(mult(q, v), inv(q))


def angleVector2Quaternion(theta, rotAxis):
    '''converts rotation angle theta (in rad) and rotation axis r into quaternion'''
    if len(rotAxis)==4:
        rotAxis = rotAxis[1:]
    rotAxis = rotAxis / np.linalg.norm(rotAxis)
    return np.array([np.cos(theta/2),
                     rotAxis[0]*np.sin(theta/2),
                     rotAxis[1]*np.sin(theta/2),
                     rotAxis[2]*np.sin(theta/2)])


def vector2Quaternion(v):
    '''converts vector / point into quaternion'''
    return np.array([0, v[0], v[1], v[2]])


def eulerToQuaternion(r, p, y):
    '''constructs rotation quaternion from Euler angles'''
    aaX = angleVector2Quaternion(r, np.array([1.0, 0.0, 0.0]))
    aaY = angleVector2Quaternion(p, np.array([0.0, 1.0, 0.0]))
    aaZ = angleVector2Quaternion(y, np.array([0.0, 0.0, 1.0]))
    return mult(mult(aaZ, aaY), aaX)


def quaternion2Euler(q):
    '''converts rotation quaternion into Euler Angles'''
    w, x, y, z = q
    r = np.arctan2(2*(w*x+y*z), 1-2*(x**2+y**2))
    p = np.arcsin(2*(w*y-z*x))
    y = np.arctan2(2*(w*z+x*y), 1-2*(y**2+z**2))
    return np.array([r, p, y])


def matrix2quaternion(M, isPrecise=False):
    """Returns quaternion from rotation matrix.
    If isPrecise is True, the input matrix is assumed to be a precise rotation
    matrix and a faster algorithm is used.
    """
    if isPrecise:
        q = np.empty((4, ))
        t = np.trace(M) + 1
        if t > 1:
            q[0] = t
            q[3] = M[1, 0] - M[0, 1]
            q[2] = M[0, 2] - M[2, 0]
            q[1] = M[2, 1] - M[1, 2]
        else:
            i, j, k = 0, 1, 2
            if M[1, 1] > M[0, 0]:
                i, j, k = 1, 2, 0
            if M[2, 2] > M[i, i]:
                i, j, k = 2, 0, 1
            t = M[i, i] - (M[j, j] + M[k, k]) + 1
            q[i] = t
            q[j] = M[i, j] + M[j, i]
            q[k] = M[k, i] + M[i, k]
            q[3] = M[k, j] - M[j, k]
            q = q[[3, 0, 1, 2]]
        q *= 0.5 / np.sqrt(t)
    else:
        m00, m01, m02 = M[0, 0], M[0, 1], M[0, 2]
        m10, m11, m12 = M[1, 0], M[1, 1], M[1, 2]
        m20, m21, m22 = M[2, 0], M[2, 1], M[2, 2]
        # symmetric matrix K
        K = np.array([[m00-m11-m22, 0.0,         0.0,         0.0],
                      [m01+m10,     m11-m00-m22, 0.0,         0.0],
                      [m02+m20,     m12+m21,     m22-m00-m11, 0.0],
                      [m21-m12,     m02-m20,     m10-m01,     m00+m11+m22]])
        K /= 3.0
        # quaternion is eigenvector of K that corresponds to largest eigenvalue
        w, V = np.linalg.eigh(K)
        q = V[[3, 0, 1, 2], np.argmax(w)]
    if q[0] < 0.0:
        np.negative(q, q)
    return q
