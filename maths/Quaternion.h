/*
 * maths/Quaternion.h
 *
 * Copyright (C) 2007-2015 James Hogan <james@albanarts.com>
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details
 * (in the file called COPYING).
 *
 *
 * Quaternion class for orientations and rotations.
 *
 * Originally from Computer Graphics and Visualisation Open Assessment.
 *
 */

#ifndef _MATHS_QUATERNION_H_
#define _MATHS_QUATERNION_H_

#include "maths/TemplateMaths.h"
#include "maths/Vector.h"

#include <ostream>

namespace maths {

  // These aren't always used.
  template <int N, typename T>
      class Vector;
  template <int N, typename T>
      class Matrix;
  template <typename T>
      class Quaternion;

  /// Get a rotation quaterion between two vectors.
  template <typename T>
      Quaternion<T> rotationArc(const Vector<3,T> & a, const Vector<3,T> & b)
  {
    Vector<3,T> c;
    cross(c, a, b);
    T d = a * b;
    T s2 = (d+1)*2;
    if (s2) {
      T s = sqrt<T>(s2);
      return Quaternion<T>(c[0] / s,
                           c[1] / s,
                           c[2] / s,
                           s    / 2);
    } else {
      // 180 degrees, Result would ordinarily be not a number.
      return Quaternion<T>((T)1, (T)0, (T)0, (T)0);
    }
  }

  /// Get a rotation quaterion between two vectors.
  template <typename T>
      Quaternion<T> rotationArc(const Vector<3,T> & a, const Vector<3,T> & b, bool * pDeterminate)
  {
    Vector<3,T> c;
    cross(c, a, b);
    T d = a * b;
    T s2 = (d+1)*2;
    bool determinate = s2;
    if (pDeterminate) {
      *pDeterminate = determinate;
    }
    if (determinate) {
      T s = sqrt<T>(s2);
      return Quaternion<T>(c[0] / s,
                           c[1] / s,
                           c[2] / s,
                           s    / 2);
    } else {
      // 180 degrees, Result would ordinarily be not a number.
      return Quaternion<T>((T)1, (T)0, (T)0, (T)0);
    }
  }

  /// Quaternion orientation.
  /**
   * @param T typename Type of the components of the quaternion.
   */
  template <typename T>
      class Quaternion
  {
    public:
      T q[4];

    public:
      /// Default constructor (does not initialise values)
      inline Quaternion()
      {
      }

      inline explicit Quaternion(const T * quat)
      {
        for (int i = 0; i < 4; ++i) {
          q[i] = quat[i];
        }
      }
      inline explicit Quaternion(T x, T y, T z, T w)
      {
        q[0] = x;
        q[1] = y;
        q[2] = z;
        q[3] = w;
      }
      inline explicit Quaternion(const Vector<3, T> & vec)
      {
        for (int i = 0; i < 3; ++i) {
          q[i] = vec[i];
        }
        q[3] = (T)0;
      }

      template <typename U> inline explicit Quaternion(const Quaternion<U> & copy)
      {
        for (int i = 0; i < 4; ++i) {
          q[i] = (T)copy.q[i];
        }
      }

      /// Construct from a 3 by 3 matrix with the same component types.
      /**
       * Taken from game programming gems 1 i think
       */
      inline explicit Quaternion(const Matrix<3,T> & m)
      {
        T tr,s;
        tr = m[0][0] + m[1][1] + m[2][2];

        if (tr >= 0) {
          s = sqrt(tr+(T)1);
          q[3] = (T)0.5 * s;
          s = (T)0.5 / s;
          q[0] = (m[2][1] - m[1][2]) * s;
          q[1] = (m[0][2] - m[2][0]) * s;
          q[2] = (m[1][0] - m[0][1]) * s;
        } else {
          int i = 0;
          if (m[1][1] > m[0][0])
            i = 1;
          if (m[2][2] > m[i][i])
            i = 2;
          switch (i) {
            case 0:
              s = sqrt((m[0][0] - (m[1][1] + m[2][2])) + (T)1);
              q[0] = (T)0.5 * s;
              s = (T)0.5 / s;
              q[1] = (m[0][1] + m[1][0]) * s;
              q[2] = (m[2][0] + m[0][2]) * s;
              q[3] = (m[2][1] - m[1][2]) * s;
              break;
            case 1:
              s = sqrt((m[1][1] - (m[2][2] + m[0][0])) + (T)1);
              q[1] = (T)0.5 * s;
              s = (T)0.5 / s;
              q[2] = (m[1][2] + m[2][1]) * s;
              q[0] = (m[0][1] + m[1][0]) * s;
              q[3] = (m[0][2] - m[2][0]) * s;
              break;
            case 2:
              s = sqrt((m[2][2] - (m[0][0] + m[1][1])) + (T)1);
              q[2] = (T)0.5 * s;
              s = (T)0.5 / s;
              q[0] = (m[2][0] + m[0][2]) * s;
              q[1] = (m[1][2] + m[2][1]) * s;
              q[3] = (m[1][0] - m[0][1]) * s;
              break;
          }
        }
      }

      /// Convert to a 3 by 3 rotation matrix.
      inline Matrix<3, T> toMatrix3()
      {
        Matrix<3, T> mat;
        T xy = q[0]*q[1];
        T xz = q[0]*q[2];
        T yy = q[1]*q[1];
        T yw = q[1]*q[3];
        T zz = q[2]*q[2];
        T zw = q[2]*q[3];
        setVector3(mat[0].v, 1-2*(yy+zz),   2*(xy+zw),   2*(xz-yw));
        T xx = q[0]*q[0];
        T xw = q[0]*q[3];
        T yz = q[1]*q[2];
        setVector3(mat[1].v,  2*(xy-zw), 1-2*(xx+zz),   2*(yz+xw));
        setVector3(mat[2].v,  2*(xz+yw),   2*(yz-xw), 1-2*(xx+yy));
        return mat;
      }

      inline T & operator [] (int index)
      {
        return q[index];
      }
      inline const T & operator [] (int index) const
      {
        return q[index];
      }

      inline operator T * ()
      {
        return q;
      }
      inline operator const T * () const
      {
        return q;
      }

      // Construct a quaternion from a set of euler angles
      /*explicit Quaternion(const mlEulerf & E);
      // Construct a quaternion from a rotation matrix
            explicit Quaternion(const maths::Matrix<3,float> & M);
      // Construct a pure quaternion with V as {x,y,z}
            explicit inline Quaternion(const Vector3f & V)
            : x(V.q[0]), y(V.q[1]), z(V.q[2]), w((T)0)
            {
        }*/

      // Quick setter, returns self reference
      /*inline Quaternion & Set(R fX = (T)0, R fY = (T)0, R fZ = (T)0, R fW = (T)0)
            {
            x = fX;
            y = fY;
            z = fZ;
            w = fW;
            return *this;
        }*/


      inline T sqr() const
      {
        T ret = (T)0;
        for (int i = 0; i < 4; ++i)
          ret += q[i]*q[i];
        return ret;
      }

      inline T mag() const
      {
        return sqrt<T>(sqr());
      }


      inline Quaternion & normalize()
      {
        return operator /= (mag());
      }
      inline Quaternion normalized()
      {
        return operator / (this, mag());
      }

      inline Quaternion & resize(T tLength)
      {
        return operator *= (tLength / mag());
      }
      inline Quaternion resized(T tLength)
      {
        return operator * (this, tLength / mag());
      }

      inline Quaternion & conjugate()
      {
        for (int i = 0; i < 3; ++i)
          q[i] = -q[i];
        return *this;
      }
      inline Quaternion conjugated() const
      {
        Quaternion ret = *this;
        return ret.conjugate();
      }

      inline Quaternion & setIdentity()
      {
        for (int i = 0; i < 3; ++i)
          q[i] = (T)0;
        q[3] = (T)1;
        return *this;
      }
      inline static const Quaternion & identity()
      {
        static const Quaternion i;
        i.setIdentity();
        return i;
      }

      // Multiplication
      inline friend Quaternion operator * (const Quaternion & lhs, const Quaternion & rhs)
      {
        return Quaternion(lhs.q[3]*rhs.q[0] + lhs.q[0]*rhs.q[3] + lhs.q[1]*rhs.q[2] - lhs.q[2]*rhs.q[1],
                          lhs.q[3]*rhs.q[1] - lhs.q[0]*rhs.q[2] + lhs.q[1]*rhs.q[3] + lhs.q[2]*rhs.q[0],
                          lhs.q[3]*rhs.q[2] + lhs.q[0]*rhs.q[1] - lhs.q[1]*rhs.q[0] + lhs.q[2]*rhs.q[3],
                          lhs.q[3]*rhs.q[3] - lhs.q[0]*rhs.q[0] - lhs.q[1]*rhs.q[1] - lhs.q[2]*rhs.q[2]);
      }
      inline Quaternion & operator *=(const Quaternion & rhs)
      {
        q[0] = q[3]*rhs.q[0] + q[0]*rhs.q[3] + q[1]*rhs.q[2] - q[2]*rhs.q[1];
        q[1] = q[3]*rhs.q[1] - q[0]*rhs.q[2] + q[1]*rhs.q[3] + q[2]*rhs.q[0];
        q[2] = q[3]*rhs.q[2] + q[0]*rhs.q[1] - q[1]*rhs.q[0] + q[2]*rhs.q[3];
        q[3] = q[3]*rhs.q[3] - q[0]*rhs.q[0] - q[1]*rhs.q[1] - q[2]*rhs.q[2];
        return *this;
      }

      // Multiplications: Real * Quaternion or Quaternion * Real
      inline friend Quaternion operator *(const Quaternion & lhs, T rhs)
      {
        return Quaternion(lhs.q[0] * rhs,
                          lhs.q[1] * rhs,
                          lhs.q[2] * rhs,
                          lhs.q[3] * rhs);
      }
      inline friend Quaternion operator *(T lhs, const Quaternion & rhs)
      {
        return Quaternion(lhs * rhs.q[0],
                          lhs * rhs.q[1],
                          lhs * rhs.q[2],
                          lhs * rhs.q[3]);
      }
      inline Quaternion operator *=(T rhs)
      {
        q[0] *= rhs;
        q[1] *= rhs;
        q[2] *= rhs;
        q[3] *= rhs;
        return *this;
      }

      // Division
      inline friend Quaternion operator /(const Quaternion & lhs, const Quaternion & rhs)
      {
        T rhsSqr = rhs.sqr();
        return Quaternion((-lhs.q[3]*rhs.q[0] + lhs.q[0]*rhs.q[3] - lhs.q[1]*rhs.q[2] + lhs.q[2]*rhs.q[1]) / rhsSqr,
                          (-lhs.q[3]*rhs.q[1] + lhs.q[0]*rhs.q[2] + lhs.q[1]*rhs.q[3] - lhs.q[2]*rhs.q[0]) / rhsSqr,
                          (-lhs.q[3]*rhs.q[2] - lhs.q[0]*rhs.q[1] + lhs.q[1]*rhs.q[0] + lhs.q[2]*rhs.q[3]) / rhsSqr,
                          ( lhs.q[3]*rhs.q[3] + lhs.q[0]*rhs.q[0] + lhs.q[1]*rhs.q[1] + lhs.q[2]*rhs.q[2]) / rhsSqr);
      }
      inline Quaternion & operator /=(const Quaternion & den)
      {
        return *this = *this / den;
      }

      // Divisions: Quaternion / Real
      inline friend Quaternion operator /(const Quaternion & lhs, T rhs)
      {
        return Quaternion(lhs.q[0] / rhs,
                          lhs.q[1] / rhs,
                          lhs.q[2] / rhs,
                          lhs.q[3] / rhs);
      }
      inline Quaternion & operator /=(T rhs)
      {
        q[0] /= rhs;
        q[1] /= rhs;
        q[2] /= rhs;
        q[3] /= rhs;
        return *this;
      }

      // Negation
      inline Quaternion & negate()
      {
        q[0] = -q[0];
        q[1] = -q[1];
        q[2] = -q[2];
        q[3] = -q[3];
        return *this;
      }
      inline friend Quaternion operator - (const Quaternion & quat)
      {
        return Quaternion(-quat.q[0],
                           -quat.q[1],
                           -quat.q[2],
                           -quat.q[3]);
      }

      // Addition
      inline friend Quaternion operator +(const Quaternion & lhs, const Quaternion & rhs)
      {
        return Quaternion(lhs.q[0] + rhs.q[0],
                          lhs.q[1] + rhs.q[1],
                          lhs.q[2] + rhs.q[2],
                          lhs.q[3] + rhs.q[3]);
      }
      inline Quaternion & operator +=(const Quaternion & rhs)
      {
        q[0] = q[0] + rhs.q[0];
        q[1] = q[1] + rhs.q[1];
        q[2] = q[2] + rhs.q[2];
        q[3] = q[3] + rhs.q[3];
        return *this;
      }

      // Subtraction
      inline friend Quaternion operator -(const Quaternion & lhs, const Quaternion & rhs)
      {
        return Quaternion(lhs.q[0] - rhs.q[0],
                          lhs.q[1] - rhs.q[1],
                          lhs.q[2] - rhs.q[2],
                          lhs.q[3] - rhs.q[3]);
      }
      inline Quaternion & operator -=(const Quaternion & rhs)
      {
        q[0] = q[0] - rhs.q[0];
        q[1] = q[1] - rhs.q[1];
        q[2] = q[2] - rhs.q[2];
        q[3] = q[3] - rhs.q[3];
        return *this;
      }

      /*
          s/c = sin(Th/2)/cos(Th/2) = tan(Th/2)
          s = c.tan(Th/2)
          s = c.tan(arccos(c))
          s = c.sin(arccos(c))/cos(arccos(c))
          s = sin(arccos(c))
          VAxis = [x/s, y/s, z/s]
      */
      // Get the axis of rotation
      inline Vector<3,T> AxisOfRotation() const
      {
        T c = (q[3]>1) ? 1 : (q[3]<-1) ? -1 : q[3];
        T s = 1/sinf(acos(c));
        return Vector<3,T>(q[0]*s,q[1]*s,q[2]*s);
      }
      // Get the angle of rotation
      inline T AngleOfRotation() const
      {
        T c = (q[3]>1) ? 1 : (q[3]<-1) ? -1 : q[3];
        //T c = (q[3]>1) ? 1 : q[3];
        return 2 * acos(c);
      }

      // Get the axis and angle or rotation represented in the form of a non unit quaternion
      inline Quaternion ToAxisAngle() const
      {
        T c = (q[3]>1) ? 1 : (q[3]<-1) ? -1 : q[3];
        if (c >= (T)1) {
          return Quaternion((T)0, (T)0, (T)0, (T)0);
        }
        else if (c <= (T)-1) {
          return Quaternion((T)1, (T)0, (T)0, (T)M_PI);
        }
        else {
          T theta = acos(c);
          T s = (T)1/sin(theta);
          return Quaternion(q[0]*s,q[1]*s,q[2]*s,2*theta);
        }
      }

      // Exponentials / Logarithms
      // Logarithm of a quaternion, given as:
      // log(q) = v*a where q = [cos(a),v*sin(a)]
      inline friend Quaternion log(const Quaternion & q)
      {
        T a = acosf(q.q[3]);
        T sina = sinf(a);
        Quaternion ret;
        ret.q[3] = (T)0;
        if (sina > (T)0) {
          ret.q[0] = a*q.q[0]/sina;
          ret.q[1] = a*q.q[1]/sina;
          ret.q[2] = a*q.q[2]/sina;
        } else {
          ret.q[0] = ret.q[1] = ret.q[2] = (T)0;
        }
        return ret;
      }

      // e^quaternion given as:
      // exp(v*a) = [cos(a),vsin(a)]
      inline friend Quaternion exp(const Quaternion & q)
      {
        T a = q.mag();
        T sinaoa = sinf(a)/a;
        T cosa = cosf(a);
        Quaternion ret;

        ret.q[3] = cosa;
        if(a > (T)0) {
          ret.q[0] = sinaoa * q.q[0];
          ret.q[1] = sinaoa * q.q[1];
          ret.q[2] = sinaoa * q.q[2];
        } else {
          ret.q[0] = ret.q[1] = ret.q[2] = (T)0;
        }

        return ret;
      }

      // Linear interpolation between two quaternions
      inline friend Quaternion lerp(const Quaternion & q1, const Quaternion & q2, T t)
      {
        return Quaternion(q1.q[0]+t*(q2.q[0]-q1.q[0]),
                          q1.q[1]+t*(q2.q[1]-q1.q[1]),
                          q1.q[2]+t*(q2.q[2]-q1.q[2]),
                          q1.q[3]+t*(q2.q[3]-q1.q[3])).normalize();
      }

      // Spherical Linear interpolation between two quaternions
      inline friend Quaternion slerp(const Quaternion & q1, const Quaternion & q2, T t)
      {
        Quaternion q3;
        T dot = q1.q[0]*q2.q[0] + q1.q[1]*q2.q[1] + q1.q[2]*q2.q[2] + q1.q[3]*q2.q[3];

        // dot = cos(theta)
        // if (dot < 0), q1 and q2 are more than 90 degrees apart,
        // so we can invert one to reduce spinning
        if (dot < 0) {
          dot = -dot;
          q3 = -q2;
        } else {
          q3 = q2;
        }

        if (dot < (T)0.95) {
          T angle   = acosf(dot);
          T sina    = sinf(angle);
          T sinat   = sinf(angle*t);
          T sinaomt = sinf(angle*((T)1-t));
          return (q1*sinaomt+q3*sinat)/sina;
        } else {
          // if the angle is small, use linear interpolation
          return lerp(q1,q3,t);
        }
      }

      // This version of slerp, used by squad, does not check for theta > 90.
      inline friend Quaternion slerp_no_invert(const Quaternion & q1, const Quaternion & q2, T t)
      {
        T dot = q1.q[0]*q2.q[0] + q1.q[1]*q2.q[1] + q1.q[2]*q2.q[2] + q1.q[3]*q2.q[3];

        if (dot > (T)-0.95 && dot < (T)0.95) {
          T angle   = acos(dot),
            sina    = sin(angle),
            sinat   = sin(angle*t),
            sinaomt = sin(angle*((T)1-t));
          return (q1*sinaomt+q2*sinat)/sina;
        } else {
          // if the angle is small, use linear interpolation
          return lerp(q1,q2,t);
        }
      }

      // Spherical cubic interpolation
      inline friend Quaternion squad(const Quaternion & q1, const Quaternion & q2,
                                     const Quaternion & a,  const Quaternion & b,  T t)
      {
        Quaternion c(slerp_no_invert(q1,q2,t)),
                     d(slerp_no_invert(a, b, t));
                     return slerp_no_invert(c,d,2*t*((T)1-t));
      }

      // Given 3 quaternions, qn-1,qn and qn+1, calculate a control point to be used in spline interpolation
      inline friend Quaternion spline(const Quaternion & qnm1,const Quaternion & qn,const Quaternion & qnp1)
      {
        Quaternion qni(qn);
        qni.conjugate();
        return qn*exp((log(qni*qnm1)+log(qni*qnp1))*((T)-0.25));
      }
  }; // ::maths::Quaternion

  // Quick creation of quaternions
  template <typename T>
  inline const maths::Quaternion<T>& identityQuaternion()
  {
    return maths::Quaternion<T>::identity();
  }

  /// Write a vector to an output stream.
  template <typename T>
  inline std::ostream & operator << (std::ostream & out, const Quaternion<T> & v)
  {
    out << "(" << v[0];
    for (int i = 1; i < 4; ++i) {
      out << ", " << v[i];
    }
    return out << ")";
  }

} // ::maths

#endif // _MATHS_QUATERNION_H_
