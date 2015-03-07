#include "util/assert.h"

#define TEN 10

#ifdef COMPILE
#define MAYBE_TEN 10
#else
#define MAYBE_TEN 9
#endif

COMPILETIME_ASSERT(5+5==TEN, a__five_plus_five_should_be_ten);
COMPILETIME_ASSERT(5+5==MAYBE_TEN, a__five_plus_five_should_be_nine);
#ifndef COMPILE
COMPILETIME_WARNING(0, always);
#endif
COMPILETIME_WARNING(1, never);

class temp
{
  int data;
  COMPILETIME_ASSERT(5+5==TEN, b__five_plus_five_should_be_ten);
  COMPILETIME_ASSERT(5+5==MAYBE_TEN, b__five_plus_five_should_be_nine);
#ifndef COMPILE
  COMPILETIME_WARNING(0, always);
#endif
  COMPILETIME_WARNING(1, never);

  temp()
  {
    COMPILETIME_ASSERT(5+5==TEN, c__five_plus_five_should_be_ten);
    COMPILETIME_ASSERT(5+5==MAYBE_TEN, c__five_plus_five_should_be_nine);
#ifndef COMPILE
    COMPILETIME_WARNING(0, always);
#endif
    COMPILETIME_WARNING(1, never);
  }
};

template <typename T>
class template_temp
{
  int data;
  COMPILETIME_ASSERT(5+5==TEN, d__five_plus_five_should_be_ten);
  COMPILETIME_ASSERT(5+5==MAYBE_TEN, d__five_plus_five_should_be_nine);
  COMPILETIME_WARNING(0, always);
  COMPILETIME_WARNING(1, never);
  public:
  void unused_func()
  {
    COMPILETIME_ASSERT(5+5==TEN, e__five_plus_five_should_be_ten);
    COMPILETIME_ASSERT(5+5==MAYBE_TEN, e__five_plus_five_should_be_nine);
    COMPILETIME_WARNING(0, always);
    COMPILETIME_WARNING(1, never);
  }
};

class empty_class
{
  int data;
};

unsigned int global_val;

COMPILETIME_ASSERT(sizeof(temp) == sizeof(empty_class),
                   temp_should_be_same_size_as_empty_class);

inline static void never_used(void)
{
  LINKTIME_ASSERT(1, uh_oh_never_used_still_compiled);
}

int main(int argc, char** argv)
{
  template_temp<int> d_and_e;
  d_and_e.unused_func();

  COMPILETIME_ASSERT(5+5==TEN, f__five_plus_five_should_be_ten);
  COMPILETIME_ASSERT(5+5==MAYBE_TEN, f__five_plus_five_should_be_nine);
#ifndef COMPILE
  COMPILETIME_WARNING(0, always);
#endif
  COMPILETIME_WARNING(1, never);

  for (int i = 0; i < 50; ++i)
  {
    int number = 5;
    RUNTIME_ASSERT(number == 5, "number should be 5");
    RUNTIME_ASSERT(number != 5, "number shouldn't be 5");
  }

#ifndef COMPILE
  if (0)
    LINKTIME_ASSERT(global_val, should_fail_not_const);
#endif
  if (1)
    LINKTIME_ASSERT(1, should_never_fail);
  if (0)
    LINKTIME_ASSERT(0, should_never_fail);
#ifndef LINK
  if (1)
    LINKTIME_ASSERT(0, should_fail_at_link);
#endif
}
