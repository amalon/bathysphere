/*
 * maths/Spline.h
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
 * Cubic spline matrix.
 *
 * Originally from Computer Graphics and Visualisation Open Assessment.
 *
 */

#ifndef _MATHS_SPLINE_H_
#define _MATHS_SPLINE_H_

#include "maths/Matrix.h"

namespace maths
{

  /// Definition of a cubic spline type.
  /**
   * @param T The type to use for components.
   */
  template <typename T>
      class CubicSpline : public Matrix<4, T>
  {
    public:
      /// The value to divide all others by.
      T divisor;

    public:
      /// Copy the matrix from an array of vectors.
      /**
       * @param columns Pointer to array of column vectors.
       * @param newDivisor The value to divide all others by.
       * @pre @a newDivisor != 0
       */
      inline CubicSpline(const Vector<4, T> * columns, T newDivisor = (T)1)
      : Matrix<4, T>(columns), divisor(newDivisor)
      {
      }

      /// Get the columns of the matrix as vectors.
      /**
       * @param c1 First column.
       * @param c2 Second column.
       * @param c3 Third column.
       * @param c4 Forth column.
       * @param newDivisor The value to divide all others by.
       * @pre @a newDivisor != 0
       */
      inline CubicSpline(const Vector<4, T> & c1, const Vector<4, T> & c2,
                         const Vector<4, T> & c3, const Vector<4, T> & c4,
                         T newDivisor = (T)1)
      : Matrix<4, T>(c1, c2, c3, c4), divisor(newDivisor)
      {
      }

      /// Perform spline interpolation using the spline definition.
      /**
       * @param U Data type to interpolate, should have arithmetic operators defined.
       * @param F Floating point data type to use for interpolation values.
       * @param u Interpolation value.
       * @param u2 Interpolation value squared.
       * @param u3 Interpolation value cubed.
       * @param cp0 First control point.
       * @param cp1 Second control point.
       * @param cp2 Third control point.
       * @param cp3 Forth control point.
       */
      template <typename U, typename F>
          inline U operator () (F u, F u2, F u3,
                                U cp0, U cp1, U cp2, U cp3) const
      {
        return (cp0 * (u3*Matrix<4,T>::col[0][0] + u2*Matrix<4,T>::col[0][1] + u*Matrix<4,T>::col[0][2] + Matrix<4,T>::col[0][3]) +
                cp1 * (u3*Matrix<4,T>::col[1][0] + u2*Matrix<4,T>::col[1][1] + u*Matrix<4,T>::col[1][2] + Matrix<4,T>::col[1][3]) +
                cp2 * (u3*Matrix<4,T>::col[2][0] + u2*Matrix<4,T>::col[2][1] + u*Matrix<4,T>::col[2][2] + Matrix<4,T>::col[2][3]) +
                cp3 * (u3*Matrix<4,T>::col[3][0] + u2*Matrix<4,T>::col[3][1] + u*Matrix<4,T>::col[3][2] + Matrix<4,T>::col[3][3])) / divisor;
      }
  };

  /// Defines a method of cube spline.
  /**
   * This consists of the main spline, used for interpolating positions, and a
   *  tangent spline used for finding the tangent to the curve at a particular
   *  point on the curve.
   * @param T component type of the cubic splines.
   */
  template <typename T>
      class CubicSplineDefinition
  {
    public:
      /// Cubic spline for main curve position vectors.
      /**
       * @note This can be used as a function, see CubicSpline<T>::operator ()
       */
      CubicSpline<T> Spline;

      /// Cubic spline for the tangent to the curve.
      /**
       * @note This can be used as a function, see CubicSpline<T>::operator ()
       */
      CubicSpline<T> Tangent;

    public:
      /// Constructor
      /**
       * @param newSpline Main curve position spline.
       * @param newTangent Tangent to the curve spline.
       */
      inline CubicSplineDefinition(const CubicSpline<T> & spline, const CubicSpline<T> & tangent)
      : Spline(spline), Tangent(tangent)
      {}
  };

}

#endif // _MATHS_SPLINE_H_
