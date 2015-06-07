/*
 * util/TemplateUtils.h
 *
 * Copyright (C) 2011-2014 James Hogan <james@albanarts.com>
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
 * Template util functions.
 *
 */

#ifndef _FILE_util_templateutils_h
#define _FILE_util_templateutils_h

#include <limits>

namespace util {

  /// Size attribute
  struct sizeOfAttribute
  {
    template <typename T>
    struct type
    {
      enum {
        value = sizeof(T),
      };
    };
  };

  /** Empty type to be used as placeholder.
   * This can be used as a placeholder when a data member may not be required.
   * @see condType
   */
  typedef struct EmptyStruct
  {
    /// Default constructor.
    EmptyStruct()
    {
    }

    /// Construct from anything.
    template <typename T>
    EmptyStruct(const T& o)
    {
    }

    /// Convert to anything.
    template <typename T>
    operator T() const
    {
      return T();
    }
  } EmptyType;

  /// Internal conditional type
  template <bool COND>
  struct _ifType;

  template <>
  struct _ifType<true>
  {
    template <typename T, typename F>
    struct t
    {
      typedef T type;
    };
  };

  template <>
  struct _ifType<false>
  {
    template <typename T, typename F>
    struct t
    {
      typedef F type;
    };
  };

  /** Conditional type between two choices.
   * @param COND  Boolean condition.
   * @param T     First type.
   * @param F     Second type.
   * The typedef type will be T if COND, and F if !COND.
   */
  template <bool COND, typename T, typename F>
  struct ifType
  {
    typedef typename _ifType<COND>::template t<T, F>::type type;
  };

  /** Conditional existence type.
   * @param COND  Boolean condition.
   * @param T     Type.
   * The typedef type will be T if COND, and EmptyTyp if !COND.
   */
  template <bool COND, typename T>
  struct condType
  {
    typedef typename ifType<COND, T, EmptyType>::type type;
  };

  /// Type with greater attribute
  template <typename ATTR, typename T, typename U>
    struct greaterType
    {
      enum {
        val_t = ATTR::template type<T>::value,
        val_u = ATTR::template type<U>::value,
      };
      typedef typename ifType<(val_t > val_u), T, U>::type type;
    };

  /// Type with lesser attribute
  template <typename ATTR, typename T, typename U>
    struct lesserType
    {
      enum {
        val_t = ATTR::template type<T>::value,
        val_u = ATTR::template type<U>::value,
      };
      typedef typename ifType<(val_t < val_u), T, U>::type type;
    };

};

#endif /* _FILE_maths_templateutils_h */
