/*
 * random.c
 *
 * Copyright (C) 2003 Robin Hogan <r.j.hogan@reading.ac.uk>
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
 * Various random number generating functions. Only hash_32 is used.
 *
 * This file has been cut down to only the functions used in the open assessment.
 *
 */

#include "random.h"

/**
 * @todo Make this endian safe
 */
unsigned int
hash_32(int n, unsigned char *sequence, unsigned int hash)
{
  int i;
  for (i = 0; i < n; ++i) {
    hash = FNV_32_HASH(hash, sequence[i]);
  }
  return hash;
}
