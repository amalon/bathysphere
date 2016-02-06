/*
 * maths/SplineDefinitions.h
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
 * Spline types.
 *
 * Originally from Computer Graphics and Visualisation Open Assessment.
 *
 */

#ifndef _MATHS_SPLINEDEFINITIONS_H_
#define _MATHS_SPLINEDEFINITIONS_H_

#include "maths/Spline.h"

namespace maths {

  /// B-Spline.
  /**
   * - Doesn't intersect control points.
   * - Good continuity (nice and smooooth).
   */
  extern const maths::CubicSplineDefinition<char> bSpline;

  /// Catmull-Rom Spline.
  /**
   * - Intersects control points.
   * - Not as continuous as B-Spline.
   */
  extern const maths::CubicSplineDefinition<char> catmullRomSpline;

}

#endif // _MATHS_SPLINEDEFINITIONS_H_
