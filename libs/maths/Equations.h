/*
 * maths/Equations.h
 *
 * Copyright (C) 2004-2015 James Hogan <james@albanarts.com>
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
 * Solving of certain classes of equation.
 *
 */

#ifndef _MATHS_EQUATIONS_H_
#define _MATHS_EQUATIONS_H_

#include <maths/TemplateMaths.h>

namespace maths {

  /*
   * Find the real roots of a quadratic equation
   * Return the number of roots
   * Output the roots in s1 and s2
   */
  template <typename T, typename TO>
  unsigned int QuadraticRoots(T a, T b, T c, TO &s1, TO &s2)
  {
    // Find the determinant
    TO determ = (TO)b*b - (TO)4.0*a*c;
    if (determ < (TO)0) {
      // No real roots
      return 0;
    } else if (determ == (TO)0) {
      // One real root
      s1 = -(TO)b / ((TO)2*a);
      s2 = s1;
      return 1;
    } else {
      // Two real roots, lesser one first if a >= 0
      TO sqrt_det = maths::sqrt(determ);
      s1 = (-(TO)b - sqrt_det) / ((TO)2*a);
      s2 = (-(TO)b + sqrt_det) / ((TO)2*a);
      return 2;
    }
  }

};

#endif  // _MATHS_EQUATIONS_H_
