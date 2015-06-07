/*
 * collision/Collision.cpp
 *
 * Copyright (C) 2007-2014 James Hogan <james@albanarts.com>
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
 * Collision detection functions.
 *
 * Originally from Computer Graphics and Visualisation Open Assessment.
 *
 */

#include "collision/Collision.h"

#include <limits>
#include <cassert>

/// Find the closest point between two lines
/**
 * @See Got idea from http://www.geometryalgorithms.com/Archive/algorithm_0106/algorithm_0106.htm
 */
float Collision::ClosestPointBetweenLines(const maths::Vector<3, float> & a1, const maths::Vector<3, float> & a2,
                                          const maths::Vector<3, float> & b1, const maths::Vector<3, float> & b2,
                                          float * otherResult)
{
  // Quick calculate the normals of the lines.
  maths::Vector<3, float> norm1 = a2 - a1;
  maths::Vector<3, float> norm2 = b2 - b1;
  maths::Vector<3, float> startDiff = a1 - b1;
  
  float a = norm1*norm1;
  float b = norm1*norm2;
  float c = norm2*norm2;
  float d = norm1*startDiff;
  float e = norm2*startDiff;
  
  // Return NAN's if the lines are paralele
  float denominator = a*c - b*b;
  if (denominator == 0.0f) {
    if (otherResult) {
      *otherResult = std::numeric_limits<float>::quiet_NaN();
    }
    return std::numeric_limits<float>::quiet_NaN();
  }
  
  // Otherwise do normal calculation
  if (otherResult) {
    *otherResult = (a*e - b*d) / denominator;
  }
  return (b*e - c*d) / denominator;
}

namespace Collision {
  
  /// Find collisions between a point and a moving sphere.
  float DetectPointSphere(Interaction & result,
                          const maths::Vector<3, float> & point, const Sphere & sphere,
                          const maths::Vector<3, float> & spherePosition,
                          const maths::Vector<3, float> & sphereMovement)
  {
    // Find the fraction along the path of the sphere that the point is closest
    float frac = ClosestPointOnLine(spherePosition, spherePosition + sphereMovement, point);
    
    // Find the shortest distance from that point
    float distSq = (spherePosition + sphereMovement*frac - point).sqr();
    
    // If no collision took place, don't do anything else
    if (distSq >= sphere.radius*sphere.radius) {
      return NO_COLLISION;
    }
    
    // Backtrack along the path until the point is just touching the sphere
    float adjustmentSq = (sphere.radius*sphere.radius - distSq) / sphereMovement.sqr();
    frac -= sqrt(adjustmentSq);
    if (frac < 0.0f) {
   // if (adjustmentSq > frac*frac) {
      // this would make the collision happen in the past
      return NO_COLLISION;
    }
    if (frac > 1.0f) {
    //float frac1minus = 1.0f - frac;
    //if (adjustmentSq > frac1minus*frac1minus) {
      // this would make the collision happen sometime in the future
      return NO_COLLISION;
    }
    
    // Now find the collision point, normal
    result.location = point;
    // The vector from the sphere to the point is of length sphere.radius :)
    result.normal = ((spherePosition + sphereMovement*frac) - point) / sphere.radius;
    return frac;
  }
  
  /// Find collisions between a line and a moving sphere.
  float DetectLineSphere(Interaction & result,
                         const maths::Vector<3, float> & point1, const maths::Vector<3, float> & point2, const Sphere & sphere,
                         const maths::Vector<3, float> & spherePosition,
                         const maths::Vector<3, float> & sphereMovement)
  {
    // Get closest point between path and edge as fraction of P1->P2
    float lineFrac;
    float frac = ClosestPointBetweenLines(spherePosition, spherePosition + sphereMovement,
                                          point1, point2,
                                          &lineFrac);
    
    // Find the shortest distance from the line of the path of the sphere
    maths::Vector<3, float> lineDirection = point2 - point1;
    float distSq = ((spherePosition + sphereMovement*frac) - (point1 + lineDirection*lineFrac)).sqr();
    
    // If no collision took place, don't do anything else
    if (distSq >= sphere.radius*sphere.radius) {
      return NO_COLLISION;
    }
    
    // Backtrack along the path until the point is just touching the sphere
    float effectiveRadiusSq = sphere.radius*sphere.radius - distSq;
    float cosAngleSq = (sphereMovement*lineDirection);
    cosAngleSq *= cosAngleSq;
    cosAngleSq /= sphereMovement.sqr()*lineDirection.sqr();
    float lineAdjustmentSq = effectiveRadiusSq * cosAngleSq / lineDirection.sqr();
    frac -= sqrt((lineAdjustmentSq*lineDirection.sqr() + effectiveRadiusSq) / sphereMovement.sqr());
    
    // Find the position of the sphere at the point of collision
    maths::Vector<3, float> sphereAtCollision = spherePosition + sphereMovement*frac;
    // The closest point on the line is the location of collision
    lineFrac = ClosestPointOnLine(point1, point2, sphereAtCollision);
    if (lineFrac < 0.0f) {
      // this would make the collision happen beyond point1
      return DetectPointSphere(result, point1, sphere, spherePosition, sphereMovement);
    }
    if (lineFrac > 1.0f) {
      // this would make the collision happen beyond point2
      return DetectPointSphere(result, point2, sphere, spherePosition, sphereMovement);
    }
    result.location = point1 + lineDirection * lineFrac;
    // The vector between these points is the normal of length sphere.radius
    result.normal = (sphereAtCollision - result.location) / sphere.radius;
    return frac;
  }
  
  /// Find collisions between a plane and a moving sphere.
  float DetectPlaneSphere(Interaction & result, const maths::Vector<4, float> & plane, const Sphere & sphere,
                          const maths::Vector<3, float> & spherePosition,
                          const maths::Vector<3, float> & sphereMovement)
  {
    // First evaluate the two points with the plane equation
    float dot1 = maths::plane(plane, spherePosition);
    float dot2 = maths::plane(plane, spherePosition + sphereMovement);
    
    // If both the points are on one side, then there hasn't been an intersection
    if (dot1 >= sphere.radius && dot2 >= sphere.radius) {
      result.backface = false;
      return NO_COLLISION;
    }
    if (dot1 <= -sphere.radius && dot2 <= -sphere.radius) {
      result.backface = true;
      return NO_COLLISION;
    }
    
    // If both dots are the equal, then we're moving parallel, but through it, so we're colliding.
    if (dot1 == dot2) {
      //result.backface = false;
      //result.normal = (maths::Vector<3,float>)plane;
      //result.location = spherePosition;
      return NO_COLLISION;
    }
  
    // if dot1 is positive, we want where the dot is radius (at the point of collision)
    // if dot1 is negative, we want where the dot is -radius (at the point of collision)
    // Scale so that a plane equation result of 0 is where it first touches
    result.backface = (dot1 < 0.0f);
    if (result.backface) {
      dot1 = -dot1-sphere.radius;
      dot2 = -dot2-sphere.radius;
    } else {
      dot1 -= sphere.radius;
      dot2 -= sphere.radius;
    }
  
    // Now return the time at which the collision took place.
    result.normal = (maths::Vector<3, float>)plane * (result.backface ? -1.0f : 1.0f);
    float factor = dot1 / (dot1-dot2);
    result.location = spherePosition + sphereMovement*factor - result.normal*sphere.radius;
    return factor;
  }
  
  /// Find collisions between a triangle and a moving sphere.
  float DetectTriangleSphere(Interaction & result,
                             const Triangle & triangle, const Sphere & sphere,
                             const maths::Vector<3, float> & spherePosition,
                             const maths::Vector<3, float> & sphereMovement)
  {
    // First check that the path actually intersects the plane
    float fraction = DetectPlaneSphere(result, triangle.facePlane, sphere,
                            spherePosition, sphereMovement);
    if (fraction == NO_COLLISION || result.backface || fraction < 0.0f || fraction > 1.0f) {
      return NO_COLLISION;
    }
    
    // Path intersects plane, now check if it is within the main polygon
    // In this case the sphere hit the polygon on its main face
    if (triangle.PointInside(result.location)) {
      // The normal will have been set by Detect<plane, sphere>
      return fraction;
    }
  
    // Now check whether it touches any of the edges, it might have just missed.
    fraction = NO_COLLISION;
    for (int i = 0; i < 3; i++) {
      Interaction interaction;
      float frac = DetectLineSphere(interaction, triangle.vertices[i], triangle.vertices[(i+1)%3], sphere,
                                    spherePosition, sphereMovement);
      if (frac != NO_COLLISION && (fraction == NO_COLLISION || frac < fraction) && (frac >= 0.0f && frac <= 1.0f)) {
        fraction = frac;
        result = interaction;
      }
    }
    
    return fraction;
  }
}
