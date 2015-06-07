/*
 * util/assert.cpp
 *
 * Copyright (C) 2003-2014 James Hogan <james@albanarts.com>
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
 * Assertion handlers
 *
 */

#ifndef NDEBUG

#include "util/assert.h"

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>

bool handleFailedAssertion(const char* cond, const char* function,
			   const char* file, const int line, unsigned int hits,
			   int *ignore, const char* name, ...)
{
  const unsigned int limit = 10;
  static bool ignore_all = false;
  if (ignore_all) {
    *ignore = true;
    return false;
  }
  if (!*ignore && hits <= limit) {
    va_list ap;
    static bool prompt = true;

    fprintf(stderr, "Assertion \"%s\" failed (", cond);
    va_start(ap, name);
    vfprintf(stderr, name, ap);
    va_end(ap);
    fprintf(stderr, ") in \"%s\" at %s:%d", function, file, line);
    if (hits > 1)
      fprintf(stderr, " again (hits: %u)", hits);
    if (hits == limit)
      fprintf(stderr, " [last display]");
    fprintf(stderr, "\n");

    while (prompt) {
      printf("Enter command: (C)ontinue, (B)reak, Continue (F)orever, (I)gnore, Ignore (A)ll, (Q)uit\n? ");
#define BUF_LEN 10
      char buf[BUF_LEN];
      if (!fgets(buf, BUF_LEN, stdin)) {
        printf("Bad command\n");
        continue;
      }
      size_t last = strlen(buf) - 1;
      if (buf[last] == '\n')
        buf[last--] = '\0';
      if (last > 0) {
        printf("Bad command\n");
        continue;
      }
      switch (buf[0]) {
        case 'a':
        case 'A':
          ignore_all = true;
          /* fall through */
        case 'i':
        case 'I':
          *ignore = true;
          /* fall through */
        case 'c':
        case 'C':
          return false;

        case 'f':
        case 'F':
          prompt = false;
          return false;
        case 'b':
        case 'B':
          return true;
        case 'q':
        case 'Q':
          exit(1);
      };
    }
  }
  return false;
}

#endif
