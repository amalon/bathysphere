/*
 * maths/SplineDefinitions.cpp
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

#include "maths/SplineDefinitions.h"

// B-Spline
// Doesn't intersect control points
// Nice and smoooooooth
const maths::CubicSplineDefinition<char> maths::bSpline
    ( // Spline matrix and divisor
      maths::CubicSpline<char>( maths::Vector<4, char>( -1,  3, -3,  1),
                                maths::Vector<4, char>(  3, -6,  0,  4),
                                maths::Vector<4, char>( -3,  3,  3,  1),
                                maths::Vector<4, char>(  1,  0,  0,  0), 6),
      // Tangent matrix and divisor
      maths::CubicSpline<char>( maths::Vector<4, char>(  0, -1,  2, -1),
                                maths::Vector<4, char>(  0,  3, -4,  0),
                                maths::Vector<4, char>(  0, -3,  2,  1),
                                maths::Vector<4, char>(  0,  1,  0,  0), 2)
    );

// Catmull-Rom Spline
// Intersects control points
// Not as smoooooth as B-Spline
const maths:: CubicSplineDefinition<char> maths::catmullRomSpline
    ( // Spline matrix and divisor
      maths::CubicSpline<char>( maths::Vector<4, char>( -1,  2, -1,  0),
                                maths::Vector<4, char>(  3, -5,  0,  2),
                                maths::Vector<4, char>( -3,  4,  1,  0),
                                maths::Vector<4, char>(  1, -1,  0,  0), 2),
      // Tangent matrix and divisor
      maths::CubicSpline<char>( maths::Vector<4, char>( -3, -1,  4, -1),
                                maths::Vector<4, char>(  9,  3,-10,  0),
                                maths::Vector<4, char>( -9, -3,  8,  1),
                                maths::Vector<4, char>(  3,  1, -2,  0), 2)
    );
